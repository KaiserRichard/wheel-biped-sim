#!/usr/bin/env python3
"""Convert one device/profile from rs-enumerate-devices -c to ORB Settings 1.0.

Stdlib only. Rectified stereo or color with aligned ROS depth; never infer an
IMU, a missing baseline, native SDK depth units or another device's calibration.
"""
from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
import re


def numbers(value: str, count: int) -> list[float]:
    result = [float(part) for part in value.split()]
    if len(result) != count or not all(math.isfinite(x) for x in result):
        raise ValueError(f"expected {count} finite calibration values: {value!r}")
    return result


def device_text(text: str, serial: str) -> str:
    devices = re.split(r"(?m)^\s*Device info\x3a\s*$", text)
    matches = [
        block for block in devices
        if re.search(r"(?m)^\s*Serial Number\s*:\s*" + re.escape(serial) + r"\s*$", block)
    ]
    if len(matches) != 1:
        raise ValueError(f"expected exactly one device with serial {serial}")
    return matches[0]


def intrinsic(text: str, stream: str, width: int, height: int, pixel_format: str) -> dict:
    header = re.compile(
        r'^\s*Intrinsic of "([^"]+)"\s*/\s*(\d+)x(\d+)\s*/\s*\{([^}]+)\}\s*$',
        re.MULTILINE,
    )
    headers = list(header.finditer(text))
    found = []
    for index, match in enumerate(headers):
        name, w, h, formats = match.groups()
        if (name, int(w), int(h)) != (stream, width, height):
            continue
        if pixel_format not in formats.split("/"):
            continue
        body = text[match.end():headers[index + 1].start() if index + 1 < len(headers) else len(text)]
        fields = dict(re.findall(r"(?m)^\s*(Width|Height|PPX|PPY|Fx|Fy|Distortion|Coeffs):[ \t]*(.+)$", body))
        if set(fields) != {"Width", "Height", "PPX", "PPY", "Fx", "Fy", "Distortion", "Coeffs"}:
            raise ValueError(f"incomplete {stream} intrinsics")
        if int(fields["Width"]) != width or int(fields["Height"]) != height:
            raise ValueError(f"{stream} dimensions disagree with profile header")
        values = {key: numbers(fields[source], 1)[0]
                  for key, source in (("fx", "Fx"), ("fy", "Fy"), ("cx", "PPX"), ("cy", "PPY"))}
        if values["fx"] <= 0 or values["fy"] <= 0:
            raise ValueError(f"{stream} focal lengths must be positive")
        values["coeffs"] = numbers(fields["Coeffs"], 5)
        values["distortion"] = fields["Distortion"].strip()
        found.append(values)
    if len(found) != 1:
        raise ValueError(f"expected one {stream} {width}x{height} {pixel_format} profile, got {len(found)}")
    return found[0]


def stereo_baseline(text: str) -> float:
    pattern = r'Extrinsic from "Infrared 1"\s+To\s+"Infrared 2"\s*:'
    matches = list(re.finditer(pattern, text))
    if len(matches) != 1:
        raise ValueError("expected one Infrared 1 -> Infrared 2 extrinsic")
    block = text[matches[0].end():].split("Extrinsic from", 1)[0]
    rotation = re.search(r"Rotation Matrix\x3a\s*(.*?)\s*Translation Vector:", block, re.DOTALL)
    translation = re.search(r"Translation Vector:[ \t]*([^\n]+)", block)
    if not rotation or not translation:
        raise ValueError("incomplete IR stereo extrinsic")
    matrix = numbers(rotation.group(1), 9)  # printed rows, not SDK's column-major array
    xyz = numbers(translation.group(1), 3)  # SDK metres, p_ir2 = R * p_ir1 + t
    if any(abs(v - (1.0 if i in (0, 4, 8) else 0.0)) > 1e-6
           for i, v in enumerate(matrix)) or abs(xyz[1]) > 1e-6 or abs(xyz[2]) > 1e-6:
        raise ValueError("IR pair is not rectified along x; explicit rectification is required")
    if xyz[0] >= 0:
        raise ValueError("IR1 -> IR2 translation x must be negative (positive left-to-right baseline)")
    return -xyz[0]


def convert(text: str, *, serial: str, mode: str, width: int, height: int, fps: int) -> str:
    if not serial.isdecimal():
        raise ValueError("serial must contain only decimal digits")
    if mode not in ("stereo", "rgbd") or min(width, height, fps) <= 0:
        raise ValueError("mode must be stereo/rgbd and width/height/fps must be positive")
    device = device_text(text, serial)
    baseline = stereo_baseline(device)
    if mode == "stereo":
        camera = intrinsic(device, "Infrared 1", width, height, "Y8")
        right = intrinsic(device, "Infrared 2", width, height, "Y8")
        for profile in (camera, right):
            if any(profile["coeffs"]) or profile["distortion"] not in ("None", "Brown Conrady"):
                raise ValueError("stereo requires zero-distortion rectified IR profiles")
        if any(abs(camera[key] - right[key]) > 1e-5 for key in ("fx", "fy", "cx", "cy")):
            raise ValueError("rectified stereo requires matching left/right intrinsics")
    else:
        camera = intrinsic(device, "Color", width, height, "RGB8")
        # Inverse/modified Brown coefficients cannot be copied into OpenCV Brown.
        if camera["distortion"] not in ("None", "Brown Conrady", "Inverse Brown Conrady", "Modified Brown Conrady"):
            raise ValueError("unsupported color distortion model")
        if camera["distortion"] != "Brown Conrady" and any(camera["coeffs"]):
            raise ValueError("nonzero color distortion requires an explicit model conversion")
        intrinsic(device, "Depth", width, height, "Z16")
    entries = {
        "File.version": '"1.0"',
        "Camera.type": '"Rectified"' if mode == "stereo" else '"PinHole"',
        **{f"Camera1.{key}": camera[key] for key in ("fx", "fy", "cx", "cy")},
        "Camera.width": width, "Camera.height": height, "Camera.fps": fps,
        "Camera.RGB": 1, "Stereo.b": baseline, "Stereo.ThDepth": 40.0,
    }
    if mode == "rgbd":
        entries.update({f"Camera1.{key}": value for key, value in
                        zip(("k1", "k2", "p1", "p2", "k3"), camera["coeffs"])})
        entries["RGBD.DepthMapFactor"] = 1.0
    entries.update({
        "ORBextractor.nFeatures": 1250, "ORBextractor.scaleFactor": 1.2,
        "ORBextractor.nLevels": 8, "ORBextractor.iniThFAST": 20, "ORBextractor.minThFAST": 7,
        "Viewer.KeyFrameSize": 0.05, "Viewer.KeyFrameLineWidth": 1.0,
        "Viewer.GraphLineWidth": 0.9, "Viewer.PointSize": 2.0,
        "Viewer.CameraSize": 0.08, "Viewer.CameraLineWidth": 3.0,
        "Viewer.ViewpointX": 0.0, "Viewer.ViewpointY": -0.7,
        "Viewer.ViewpointZ": -3.5, "Viewer.ViewpointF": 500.0,
    })
    header = [
        "%YAML:1.0", "---",
        f"# SDK serial: {serial}; mode: {mode}; input SHA256: {hashlib.sha256(text.encode()).hexdigest()}",
        "# Intrinsics/baseline: selected SDK readback, not independent calibration.",
        "# FPS, extractor, ThDepth, viewer: operating choices, not measured performance.",
        "# Stereo.b uses -t_x of IR1 -> IR2, in metres.",
    ]
    if mode == "rgbd":
        header.append("# d435_node normalizes ROS 16UC1 millimetres to float metres; not native SDK Z16.")
    return "\n".join(header + [f"{key}: {value}" for key, value in entries.items()]) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="saved rs-enumerate-devices -c stdout")
    parser.add_argument("output", type=Path, help="new YAML file; never overwritten")
    parser.add_argument("--serial", required=True)
    parser.add_argument("--mode", choices=("stereo", "rgbd"), required=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    args = parser.parse_args()
    try:
        output = convert(args.input.read_text(), serial=args.serial, mode=args.mode,
                         width=args.width, height=args.height, fps=args.fps)
        with args.output.open("x") as stream:
            stream.write(output)
    except (ValueError, OSError) as error:
        parser.exit(2, f"calibration: {error}\n")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
