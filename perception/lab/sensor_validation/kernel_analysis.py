#!/usr/bin/env python3
"""Classify saved kernel/SDK logs; raw-log human review remains mandatory."""
import argparse
import json
from pathlib import Path
import re

RULES = {
    "usb_disconnect": r"USB disconnect",
    "usb_reset": r"reset .*USB device",
    "usb_enumeration": r"new (?:low|full|high|super)[ -]speed.*USB device|New USB device found",
    "xhci_error": r"xhci.*(?:error|fail|dead|timeout|not responding|WARN)",
    "uvc_error": r"(?:uvcvideo|\bUVC\b).*(?:fail|error|timeout|unknown.*format)",
    "hid_iio_error": r"(?:\bhid\b|hidraw|hid-sensor|\biio\b).*(?:fail|error|denied|timeout)",
    "frame_corruption": r"Frame Corrupted|Incomplete video frame|corrupt(?:ed|ion).*frame|frame.*corrupt|dropped.*frame",
    "permission_error": r"Permission denied|Operation not permitted|access denied",
    "power_thermal_error": r"under.?voltage|over.?current|throttl(?:ed|ing)|over.?temperature",
    "process_error": r"segfault|terminate called|unhandled exception|failed to (?:start|open)|No device connected|Connect Realsense Camera",
    "profile_error": r"not supported by the device|Invalid syntax in configuration",
    "kernel_monitor_error": r"No journal files were (?:found|opened)|not seeing messages|Failed to (?:open|seek).*journal",
}
PATTERNS = {key: re.compile(value, re.I) for key, value in RULES.items()}
CALIBRATION = re.compile(r"IMU Calibration is not available.*", re.I)


def classify(text):
    """Conservative matching includes unrelated USB faults for human attribution."""
    matches = []
    warnings = []
    for number, line in enumerate(text.splitlines(), 1):
        categories = [key for key, pattern in PATTERNS.items() if pattern.search(line)]
        if categories:
            matches.append({"line": number, "categories": categories, "text": line})
        if CALIBRATION.search(line):
            warnings.append({"line": number, "text": line})
    return {"faults": matches, "calibration_warnings": warnings,
            "automatic_checks_pass": not matches,
            "raw_log_review_required": True,
            "vio_calibration_qualified": False}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = {str(path): classify(path.read_text(errors="replace")) for path in args.logs}
    with args.output.open("x") as handle:
        json.dump(result, handle, indent=2)
        handle.write("\n")
    return 0 if all(r["automatic_checks_pass"] for r in result.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
