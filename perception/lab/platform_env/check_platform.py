#!/usr/bin/env python3
"""Read-only, camera-free Pi platform check with offline re-analysis."""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import time

VERSION = "1.0"
HERE = Path(__file__).resolve().parent
MANIFEST = HERE / "manifest.yaml"  # JSON is a valid YAML subset; stdlib only.
ORB_ROOTS = [Path.home() / "orbslam3/ORB_SLAM3", Path.home() / "ORB_SLAM3"]
ORB_SOURCE_ROOT = Path.home() / "wbr-perception-phase1-runtime/third_party/ORB_SLAM3"


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def run_command(name: str, argv: list[str], raw: Path, env: dict[str, str] | None = None) -> dict:
    started = time.monotonic()
    try:
        proc = subprocess.run(argv, text=True, capture_output=True, timeout=30, env=env)
        stdout, stderr, rc, error = proc.stdout, proc.stderr, proc.returncode, None
    except (OSError, subprocess.TimeoutExpired) as exc:
        stdout, stderr, rc, error = "", "", None, str(exc)
    (raw / f"{name}.stdout.txt").write_text(stdout, encoding="utf-8")
    (raw / f"{name}.stderr.txt").write_text(stderr, encoding="utf-8")
    return {"argv": argv, "returncode": rc, "error": error,
            "stdout_file": f"raw/{name}.stdout.txt", "stderr_file": f"raw/{name}.stderr.txt",
            "duration_seconds": round(time.monotonic() - started, 6)}


def read_text(path: str) -> str | None:
    try:
        return Path(path).read_text(encoding="utf-8", errors="replace").strip().replace("\x00", "")
    except OSError:
        return None


def sha256(path: Path) -> str | None:
    try:
        h = hashlib.sha256()
        with path.open("rb") as f:
            for block in iter(lambda: f.read(1024 * 1024), b""):
                h.update(block)
        return h.hexdigest()
    except OSError:
        return None


def device_tree_ram_bytes() -> int | None:
    """Sum physical memory ranges from DT, excluding MemTotal's GPU reservation."""
    root = Path("/proc/device-tree")
    try:
        ac = int.from_bytes((root / "#address-cells").read_bytes(), "big")
        sc = int.from_bytes((root / "#size-cells").read_bytes(), "big")
        if ac not in (1, 2) or sc not in (1, 2):
            return None
        cell = 4
        total = 0
        found = False
        for reg in root.glob("memory*/reg"):
            data = reg.read_bytes()
            stride = (ac + sc) * cell
            if not data or len(data) % stride:
                continue
            found = True
            for offset in range(0, len(data), stride):
                size_offset = offset + ac * cell
                total += int.from_bytes(data[size_offset:size_offset + sc * cell], "big")
        return total if found and total else None
    except (OSError, ValueError):
        return None


def collect(outdir: Path, manifest: dict) -> dict:
    raw = outdir / "raw"
    raw.mkdir(parents=True, exist_ok=True)
    commands: dict[str, dict] = {}

    def cmd(key: str, argv: list[str], *, env: dict[str, str] | None = None) -> str:
        commands[key] = run_command(key, argv, raw, env)
        entry = commands[key]
        if entry["returncode"] != 0:
            return ""
        return (raw / entry["stdout_file"].split("/", 1)[1]).read_text(encoding="utf-8", errors="replace").strip()

    def shell_env() -> dict[str, str]:
        env = os.environ.copy()
        env["PATH"] = "/usr/local/sbin\x3a/usr/local/bin\x3a/usr/sbin\x3a/usr/bin\x3a/sbin\x3a/bin"
        return env

    facts: dict[str, object] = {}
    model = read_text("/proc/device-tree/model")
    facts["pi_model"] = model
    facts["board_ram_bytes"] = device_tree_ram_bytes()
    facts["kernel_release"] = platform.release() or None
    os_release = read_text("/etc/os-release") or ""
    release = re.search(r'^VERSION_ID="?([^"\n]+)', os_release, re.M)
    facts["ubuntu_version"] = release.group(1) if release else None
    facts["ros_distro"] = os.environ.get("ROS_DISTRO")

    package_text = cmd("ros_packages", ["dpkg-query", "-W", "-f=${binary:Package}\t${Version}\t${db:Status-Status}\n",
                                         "ros-jazzy-realsense2-camera", "ros-jazzy-librealsense2"])
    packages = {}
    for line in package_text.splitlines():
        cols = line.split("\t")
        if len(cols) >= 3 and cols[2] == "installed":
            packages[cols[0]] = cols[1]
    facts["ros_jazzy_realsense2_camera_package"] = packages.get("ros-jazzy-realsense2-camera")
    facts["ros_jazzy_librealsense2_package"] = packages.get("ros-jazzy-librealsense2")
    sdk = re.search(r"(?:^|\s)(\d+\.\d+\.\d+)(?:[-+\s]|$)", str(facts["ros_jazzy_librealsense2_package"] or ""))
    facts["librealsense_sdk_version"] = sdk.group(1) if sdk else None
    facts["d435i_firmware_expected"] = "not_probed_camera_access_prohibited"

    orb = next((p for p in ORB_ROOTS if p.is_dir()), ORB_ROOTS[0])
    source_root = ORB_SOURCE_ROOT if ORB_SOURCE_ROOT.is_dir() else orb
    git_head = cmd("orb_git_head", ["git", "-C", str(source_root), "rev-parse", "HEAD"])
    facts["orbslam3_source_commit"] = "4452a3c" if git_head.startswith("4452a3c") else None
    patch_text = cmd("orb_git_patch", ["git", "-C", str(orb), "log", "--format=%s", "-5"])
    # The installed copy's history is a synthetic build repository; extract patch intent from commit subjects.
    subjects = [line.lower() for line in patch_text.splitlines() if line.strip()]
    patches = []
    if any("c++17" in line for line in subjects):
        patches.append("C++11/C++14 to C++17; remove -march=native")
    cache_paths = [orb / "Thirdparty/Sophus/build/CMakeCache.txt"]
    cache = next((read_text(str(p)) or "" for p in cache_paths if p.is_file()), "")
    if "BUILD_TESTS:BOOL=OFF" in cache and "BUILD_EXAMPLES:BOOL=OFF" in cache:
        patches.append("disable Sophus tests/examples")
    if any("bool++" in line or "mnfullbaidx" in line for line in subjects):
        patches.append("mnFullBAIdx++ changed to mnFullBAIdx = true")
    facts["orbslam3_applied_patches"] = patches if patches else None
    lib_path = orb / "lib/libORB_SLAM3.so"
    exe_path = orb / "Examples/Stereo-Inertial/stereo_inertial_euroc"
    facts["liborbslam3_sha256"] = sha256(lib_path)
    facts["stereo_inertial_euroc_sha256"] = sha256(exe_path)

    pangolin = cmd("pangolin_version", ["pkg-config", "--modversion", "pangolin"], env=shell_env())
    if not pangolin:
        source_pangolin = Path.home() / "orbslam3/Pangolin"
        if source_pangolin.is_dir():
            pangolin = cmd("pangolin_source_tag", ["git", "-C", str(source_pangolin), "describe", "--tags", "--exact-match"])
            pangolin = pangolin.removeprefix("v")
    if not pangolin:
        cmake = read_text("/usr/local/lib/cmake/Pangolin/PangolinConfig.cmake") or ""
        ver = re.search(r"(?:VERSION|version)\s+[\"']?([0-9]+\.[0-9]+\.[0-9]+)", cmake)
        pangolin = ver.group(1) if ver else ""
    if not pangolin:
        header = read_text("/usr/local/include/pangolin/version.h") or ""
        parts = [re.search(rf"^#define\s+PANGOLIN_VERSION_{key}\s+(\d+)", header, re.M) for key in ("MAJOR", "MINOR", "PATCH")]
        if all(parts):
            pangolin = ".".join(m.group(1) for m in parts if m)
    facts["pangolin_version"] = pangolin or None
    facts["opencv_version"] = cmd("opencv_version", ["pkg-config", "--modversion", "opencv4"], env=shell_env()) or None
    eigen = cmd("eigen_version", ["dpkg-query", "-W", "-f=${Version}", "libeigen3-dev"], env=shell_env())
    facts["eigen_version"] = eigen or None
    facts["python_version"] = platform.python_version()

    facts["clocksource"] = read_text("/sys/devices/system/clocksource/clocksource0/current_clocksource")
    facts["cpu_governor"] = read_text("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
    swap = read_text("/proc/swaps") or ""
    swap_lines = [line for line in swap.splitlines()[1:] if line.strip()]
    facts["swap_bytes"] = sum(int(cols[2]) * 1024 for line in swap_lines if len(cols := line.split()) >= 3)
    timedate = cmd("timedatectl", ["timedatectl", "show", "-p", "NTPSynchronized", "-p", "NTP"])
    sync = re.search(r"^NTPSynchronized=(yes|no)$", timedate, re.M)
    facts["ntp_synchronized"] = (sync.group(1) == "yes") if sync else None
    findmnt = cmd("recording_mount", ["findmnt", "-J", "-T", str(Path.home() / "wbr-evidence"), "-o", "SOURCE,FSTYPE,TARGET"])
    mount = None
    try:
        fs = json.loads(findmnt)["filesystems"][0]
        mount = {"mountpoint": fs.get("target"), "source": fs.get("source"), "fstype": fs.get("fstype")}
    except (ValueError, KeyError, IndexError, TypeError):
        pass
    facts["recording_mount"] = mount
    df = cmd("recording_free", ["df", "-B1", "--output=avail", str(Path.home() / "wbr-evidence")])
    try:
        facts["recording_free_bytes"] = int(df.splitlines()[-1].strip())
    except (ValueError, IndexError):
        facts["recording_free_bytes"] = None
    udev_path = Path("/etc/udev/rules.d/99-wbr-d435i.rules")
    udev = read_text(str(udev_path)) or ""
    active_serial = str(manifest["fields"]["d435i_udev_rule_present"]["expected"])
    facts["d435i_udev_rule_present"] = active_serial if (
        "8086" in udev and "0b3a" in udev and active_serial in udev
    ) else None

    collection = {"collected_utc": utc_now(), "host": {"hostname": platform.node(), "platform": platform.platform(),
                  "python": platform.python_version(), "architecture": platform.machine(), "uid": os.getuid()},
                  "clock_note": "collected_utc uses host CLOCK_REALTIME; command durations use CLOCK_MONOTONIC and are not cross-compared",
                  "commands": commands, "facts": facts}
    (outdir / "collection.json").write_text(json.dumps(collection, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return collection


def compare_version(value: str, minimum: str) -> bool:
    def parts(v: str) -> tuple[int, ...]:
        m = re.match(r"^(\d+(?:\.\d+)*)", v)
        return tuple(map(int, m.group(1).split("."))) if m else ()
    left, right = parts(value), parts(minimum)
    if not left or not right:
        return False
    width = max(len(left), len(right))
    return left + (0,) * (width - len(left)) >= right + (0,) * (width - len(right))


def evaluate(collection: dict, manifest: dict, outdir: Path) -> dict:
    fields = manifest["fields"]
    facts = collection.get("facts", {})
    criteria = []
    for field, spec in fields.items():
        measured = facts.get(field)
        policy, severity, expected = spec["policy"], spec["severity"], spec.get("expected")
        if severity == "INFO" or policy == "info":
            status = "INFO"
        elif measured is None or measured == "":
            status = "BLOCKED"
        elif policy == "exact":
            status = "PASS" if measured == expected else "FAIL"
        elif policy == "present":
            status = "PASS" if expected in str(measured) else "FAIL"
        elif policy == "min-version":
            status = "PASS" if compare_version(str(measured), str(expected)) else "FAIL"
        else:
            status = "BLOCKED"
        criteria.append({"id": field, "description": f"{severity}: {spec.get('source', 'No source declared')}",
                         "measured": measured, "unit": "version/string/bytes/boolean as applicable",
                         "threshold": json.dumps(expected, sort_keys=True), "status": status,
                         "evidence": f"collection.json#facts.{field}",
                         "threshold_source": spec.get("source", "UNSPECIFIED")})
    statuses = [c["status"] for c in criteria if c["status"] != "INFO"]
    overall = "FAIL" if "FAIL" in statuses else "BLOCKED" if "BLOCKED" in statuses else "PASS"
    snapshot = {"schema_version": 1, "generated_utc": utc_now(), "evidence_label": "MEASURED candidate; review before pinning",
                "facts": facts, "source_collection": "collection.json"}
    (outdir / "snapshot.json").write_text(json.dumps(snapshot, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    hashes = []
    for path in sorted(outdir.rglob("*")):
        if path.is_file() and path.parent == outdir / "raw":
            hashes.append({"path": str(path.relative_to(outdir)), "sha256": sha256(path)})
    result = {"check": "platform_environment", "tool_version": VERSION, "status": overall,
              "started_utc": collection.get("collected_utc", utc_now()), "ended_utc": utc_now(),
              "host": collection.get("host", {}), "criteria": criteria, "raw_files": hashes,
              "notes": ["Camera-free check: firmware is recorded as not probed; no camera was opened.",
                        "Fixture PASS validates checker behavior only; it is not a platform gate PASS.",
                        "INFO fields are recorded but do not affect overall status."]}
    (outdir / "result.json").write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--outdir", type=Path, default=Path("platform-env-output"))
    parser.add_argument("--analyze-only", type=Path, metavar="DIR", help="re-evaluate saved collection.json; no commands run")
    parser.add_argument("--manifest", type=Path, default=MANIFEST)
    parser.add_argument("--write-manifest-candidate", action="store_true", help="write candidate-manifest.yaml in output directory; never replaces manifest")
    args = parser.parse_args(argv)
    try:
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        outdir = args.analyze_only if args.analyze_only else args.outdir
        outdir.mkdir(parents=True, exist_ok=True)
        if args.analyze_only:
            collection = json.loads((args.analyze_only / "collection.json").read_text(encoding="utf-8"))
        else:
            collection = collect(outdir, manifest)
        result = evaluate(collection, manifest, outdir)
        if args.write_manifest_candidate:
            candidate = {"schema_version": 1, "description": "Review before use; candidate generated from snapshot.json; does not replace manifest.yaml.",
                         "fields": {key: {"policy": spec["policy"], "severity": spec["severity"],
                                          "expected": collection.get("facts", {}).get(key),
                                          "source": f"{outdir}/snapshot.json (MEASURED candidate; review required)"}
                                    for key, spec in manifest["fields"].items()}}
            (outdir / "candidate-manifest.yaml").write_text(json.dumps(candidate, indent=2) + "\n", encoding="utf-8")
        print(f"Platform environment check: {result['status']}")
        print(f"{'FIELD':42} {'STATUS':8} MEASURED")
        for criterion in result["criteria"]:
            print(f"{criterion['id'][:42]:42} {criterion['status']:8} {criterion['measured']}")
        return {"PASS": 0, "FAIL": 1, "BLOCKED": 2}[result["status"]]
    except Exception as exc:
        print(f"platform check internal/usage error: {exc}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
