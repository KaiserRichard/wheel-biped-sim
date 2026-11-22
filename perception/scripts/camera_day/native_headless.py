#!/usr/bin/env python3
"""Prepare/build isolated native examples. NEVER runs an executable or opens SDK.

Run on the Pi against its existing ORB source/build and saved SDK evidence.
Only the destination is writable; all compiler/linker commands are serialized.
"""
from __future__ import annotations

import argparse
import os
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shlex
import shutil
import subprocess

MODES = {"stereo": "Stereo", "rgbd": "RGB-D", "stereo_inertial": "Stereo-Inertial"}
ASSETS = Path(__file__).with_name("native")
SERIAL = os.environ.get("CAMERA_SERIAL", "")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def extrinsic(text: str, source: str, target: str) -> list[float]:
    pattern = rf'Extrinsic from "{re.escape(source)}"\s+To\s+"{re.escape(target)}"\s*:'
    found = list(re.finditer(pattern, text))
    if len(found) != 1:
        raise ValueError(f"need exactly one saved SDK {source} -> {target} extrinsic")
    block = text[found[0].end():].split("Extrinsic from", 1)[0]
    match = re.search(r"Rotation Matrix\x3a\s*(.*?)\s*Translation Vector\x3a\s*([^\n]+)", block, re.S)
    if not match:
        raise ValueError("incomplete extrinsic")
    from math import isfinite
    rotation, translation = ([float(x) for x in part.split()] for part in match.groups())
    if len(rotation) != 9 or len(translation) != 3 or not all(map(isfinite, rotation + translation)):
        raise ValueError("invalid extrinsic numbers")
    # SDK tool prints matrix rows (SDK memory array itself is column major).
    for i in range(3):
        for j in range(3):
            if abs(sum(rotation[3*k+i] * rotation[3*k+j] for k in range(3)) - (i == j)) > 1e-5:
                raise ValueError("extrinsic rotation is not orthonormal")
    return [v for i in range(3) for v in rotation[i*3:i*3+3] + [translation[i]]] + [0., 0., 0., 1.]


def settings(evidence: Path, upstream: Path, destination: Path, fps: int, features: int) -> None:
    if not SERIAL.isdigit():
        raise ValueError("set CAMERA_SERIAL to the intended camera's SDK serial")
    text = (evidence / "rs-enumerate.txt").read_text()
    serials = re.findall(r"(?m)^\s*Serial Number\s*:\s*(\d+)\s*$", text)
    if serials != [SERIAL]:
        raise ValueError(f"readback must contain only the requested device {SERIAL}, got {serials}")
    tbc = extrinsic(text, "Infrared 1", "Gyro")
    if any(abs(a-b) > 1e-7 for a, b in zip(tbc, extrinsic(text, "Infrared 1", "Accel"))):
        raise ValueError("gyro and acceleration frames differ; native unrotated XYZ is insufficient")
    upstream_yaml = (upstream / "Examples/Stereo-Inertial/RealSense_D435i.yaml").read_text()
    for mode in MODES:
        base = evidence / ("rgbd-ros.yaml" if mode == "rgbd" else "stereo.yaml")
        yaml = base.read_text()
        if f"SDK serial: {SERIAL}" not in yaml or f"input SHA256: {sha(evidence / 'rs-enumerate.txt')}" not in yaml:
            raise ValueError(f"{base}: settings provenance does not match SDK readback")
        for key, value in (("Camera.fps", fps), ("ORBextractor.nFeatures", features)):
            yaml, n = re.subn(rf"(?m)^{re.escape(key)}:.*$", f"{key}: {value}", yaml)
            if n != 1:
                raise ValueError(f"expected one {key}")
        yaml += f'\nWBR.serial: "{SERIAL}"\n'
        if mode == "rgbd":
            yaml = yaml.replace("# d435_node normalizes ROS 16UC1 millimetres to float metres; not native SDK Z16.",
                                "# Native helper reads depth_frame.get_units(), converts Z16 to float metres.")
            if not re.search(r"(?m)^RGBD.DepthMapFactor: 1\.0$", yaml):
                raise ValueError("expected RGBD factor 1 for SDK units conversion")
        if mode == "stereo_inertial":
            yaml += "\n# SDK IR1 -> Gyro: p_b = R_bc p_c1 + t_bc (metres); XYZ remains in SDK axes.\n"
            yaml += "IMU.T_b_c1: !!opencv-matrix\n   rows: 4\n   cols: 4\n   dt: f\n   data: ["
            yaml += ",\n          ".join(", ".join(format(x, ".17g") for x in tbc[i:i+4]) for i in range(0, 16, 4)) + "]\n"
            yaml += "# ASSUMED noise: upstream RealSense_D435i.yaml defaults; NOT measured on this device.\n"
            for key in ("NoiseGyro", "NoiseAcc", "GyroWalk", "AccWalk"):
                value = re.search(rf"(?m)^IMU\.{key}:\s*([^\s#]+)", upstream_yaml)
                if not value:
                    raise ValueError(f"missing upstream IMU.{key}")
                yaml += f"IMU.{key}: {value.group(1)}\n"
            yaml += "IMU.Frequency: 200.0 # Both streams explicitly requested at supported 200 Hz\nIMU.InsertKFsWhenLost: 0\n"
        with (destination / f"{mode}-{fps}fps-{features}features.yaml").open("x") as stream:
            stream.write(yaml)


def prepare(orb: Path, destination: Path, evidence: Path) -> None:
    destination.mkdir()  # exclusive, including on the first invocation
    for folder in ("src", "include", "bin", "obj", "logs", "metadata", "settings"):
        (destination / folder).mkdir()
    manifest = {"orb_root": str(orb.resolve()), "source_sha256": {}, "evidence_sha256": {}}
    for header in ASSETS.glob("*.hpp"):
        shutil.copy2(header, destination / "include" / header.name)
    for mode, folder in MODES.items():
        target = f"{mode}_realsense_D435i"
        source = orb / "Examples" / folder / f"{target}.cc"
        copied = destination / "src" / source.name
        manifest["source_sha256"][str(source)] = sha(source)
        shutil.copy2(source, copied)
        # difflib patches operate on normalized terminal newline only; notices
        # and all source text are otherwise preserved before applying changes.
        copied.write_text(copied.read_text().rstrip("\n") + "\n")
        log = destination / "logs" / f"patch-{mode}.log"
        with log.open("x") as output:
            subprocess.run(["patch", "--batch", "--fuzz=0", str(copied), str(ASSETS / f"{mode}.patch")],
                           stdout=output, stderr=subprocess.STDOUT, check=True)
            if mode == "stereo_inertial":
                subprocess.run(["patch", "--batch", "--fuzz=0", str(copied),
                                str(ASSETS / "stereo_inertial_unified.patch")],
                               stdout=output, stderr=subprocess.STDOUT, check=True)
        for name in ("flags.make", "link.txt"):
            metadata = orb / "build/CMakeFiles" / f"{target}.dir" / name
            shutil.copy2(metadata, destination / "metadata" / f"{mode}-{name}")
    for relative in ("include/System.h", "src/System.cc", "src/Settings.cc", "src/ImuTypes.cc", "lib/libORB_SLAM3.so"):
        manifest["source_sha256"][str(orb / relative)] = sha(orb / relative)
    for name in ("rs-enumerate.txt", "stereo.yaml", "rgbd-ros.yaml"):
        manifest["evidence_sha256"][str(evidence / name)] = sha(evidence / name)
    for fps, features in ((30, 1250), (15, 800)):
        settings(evidence, orb, destination / "settings", fps, features)
    (destination / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(destination / "manifest.json")


def compile_commands(destination: Path, mode: str) -> list[list[str]]:
    flags_text = (destination / "metadata" / f"{mode}-flags.make").read_text()
    variables = dict(re.findall(r"(?m)^(CXX_\w+) = (.*)$", flags_text))
    flags = [part for key in ("CXX_DEFINES", "CXX_INCLUDES", "CXX_FLAGS")
             for part in shlex.split(variables[key])]
    target = f"{mode}_realsense_D435i"
    source = destination / "src" / f"{target}.cc"
    obj = destination / "obj" / f"{target}.o"
    link = shlex.split((destination / "metadata" / f"{mode}-link.txt").read_text())
    compiler = link[0]
    old_object = f"CMakeFiles/{target}.dir/Examples/{MODES[mode]}/{target}.cc.o"
    if link.count(old_object) != 1 or link.count("-o") != 1:
        raise ValueError("unrecognized original link command")
    link[link.index(old_object)] = str(obj)
    link[link.index("-o") + 1] = str(destination / "bin" / target)
    # All original library/rpath arguments are retained; no ORB source rebuilt.
    compile_one = [compiler, *flags, "-pthread", "-I" + str(destination / "include"),
                   "-c", str(source), "-o", str(obj)]
    return [compile_one, [*link, "-pthread"]]


def build(destination: Path) -> None:
    # flock prevents concurrent invocations of THIS builder; no make job pool.
    import fcntl
    with (destination / "build.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        for mode in MODES:
            log = destination / "logs" / f"build-{stamp}-{mode}.log"
            print(log, flush=True)
            with log.open("x") as output:
                for command in compile_commands(destination, mode):
                    output.write(shlex.join(command) + "\n")
                    output.flush()
                    subprocess.run(command, cwd=destination, stdout=output, stderr=subprocess.STDOUT, check=True)
        manifest = json.loads((destination / "manifest.json").read_text())
        unchanged = all(sha(Path(path)) == expected for path, expected in manifest["source_sha256"].items())
        receipt = {
            "built_utc": stamp, "all_original_sources_and_library_unchanged": unchanged,
            "launched_native_executables": False,
            "artifacts_sha256": {str(path.relative_to(destination)): sha(path)
                                 for folder in ("src", "include", "bin", "settings", "metadata")
                                 for path in sorted((destination / folder).iterdir()) if path.is_file()},
        }
        (destination / "logs" / f"build-{stamp}-receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
        if not unchanged:
            raise RuntimeError("original ORB sources/library changed since prepare; inspect receipt")
        print("All three examples compiled/linked sequentially. No executable was launched.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("prepare", "build"))
    parser.add_argument("--orb-root", type=Path, default=Path("./orbslam3/ORB_SLAM3"))
    parser.add_argument("--destination", type=Path, default=Path("./wbr-camera-native-20261021"))
    parser.add_argument("--evidence", type=Path, default=Path("./wbr-evidence/camera-day-20261021-01"))
    args = parser.parse_args()
    if args.action == "prepare":
        prepare(args.orb_root.resolve(), args.destination.resolve(), args.evidence.resolve())
    else:
        build(args.destination.resolve())


if __name__ == "__main__":
    main()
