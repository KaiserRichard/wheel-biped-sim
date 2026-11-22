#!/usr/bin/env python3
"""Fail-closed offline analysis of native rs-data-collect 2.58.4 CSV files.

The SDK timer starts before sensors are opened. Whole-request rates include
that setup time. Initial arrival offsets are reported for mandatory review;
they are not mislabeled as gaps between delivered samples. Interior windows
cannot rescue a failed whole-request rate, internal gap, or trailing silence.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

EXPECTED = {"Infrared": {"index": 1, "rate": (14.25, 15.75), "gap": .20},
            "Gyro": {"index": 0, "rate": (190., 210.), "gap": .05},
            "Accel": {"index": 0, "rate": (95., 105.), "gap": .05}}
# The SDK header labels its first columns backwards; emitted rows contain
# stream_name followed by stream_type (confirmed in the v2.58.4 source).
PROFILES = {("Infrared 1", "Infrared", "Y8", "15", "640", "480"),
            ("Gyro", "Gyro", "MOTION_XYZ32F", "200"),
            ("Accel", "Accel", "MOTION_XYZ32F", "100")}
CONFIG_HEADER = ["Stream Type", "Stream Name", "Format", "FPS", "Width", "Height"]
SAMPLE_HEADER = ["Stream Type", "Index", "F#", "HW Timestamp (ms)", "Host Timestamp(ms)"]


def read_capture(path):
    path = Path(path)
    if not path.stat().st_size:
        raise ValueError("CSV is empty/unfinalized; no timing evidence exists")
    data = {name: [] for name in EXPECTED}
    profiles = []
    state = None
    saw_header = False
    with path.open(newline="") as handle:
        for number, raw in enumerate(csv.reader(handle), 1):
            row = [value.strip() for value in raw]
            if not row or not any(row):
                continue
            if row == ["Configuration:"]:
                continue
            if row == CONFIG_HEADER:
                if profiles or state is not None:
                    raise ValueError("Duplicate/misplaced configuration header")
                state = "profiles"
                continue
            if row[:5] == SAMPLE_HEADER:
                state = "samples"
                saw_header = True
                continue
            if state == "profiles":
                profiles.append(tuple(row))
                continue
            if state != "samples":
                raise ValueError(f"Unexpected CSV row at line {number}")
            name = row[0]
            if name not in EXPECTED:
                raise ValueError(f"Unexpected stream {name!r} at line {number}")
            if len(row) != (5 if name == "Infrared" else 8):
                raise ValueError(f"Malformed {name} sample at line {number}")
            try:
                index, frame = int(row[1]), int(row[2])
                stamp, arrival = float(row[3]) / 1000, float(row[4]) / 1000
                vector = [float(value) for value in row[5:]]
            except ValueError as error:
                raise ValueError(f"Invalid number at line {number}") from error
            if index != EXPECTED[name]["index"]:
                raise ValueError(f"Unexpected {name} index at line {number}")
            if frame < 0 or arrival < 0 or not all(math.isfinite(v) for v in [stamp, arrival, *vector]):
                raise ValueError(f"Nonfinite/invalid sample at line {number}")
            data[name].append({"frame": frame, "stamp": stamp, "arrival": arrival, "vector": vector})
    if not saw_header or len(profiles) != 3 or set(profiles) != PROFILES:
        raise ValueError(f"Opened profiles do not exactly match baseline: {profiles!r}")
    return {"profiles": [list(profile) for profile in profiles], "streams": data}


def differences(values):
    return [b - a for a, b in zip(values, values[1:])]


def summarize(rows):
    stamps = differences([r["stamp"] for r in rows])
    arrivals = differences([r["arrival"] for r in rows])
    frames = differences([r["frame"] for r in rows])
    return {"count": len(rows),
            "first_stamp_seconds": rows[0]["stamp"] if rows else None,
            "last_stamp_seconds": rows[-1]["stamp"] if rows else None,
            "first_arrival_seconds": rows[0]["arrival"] if rows else None,
            "last_arrival_seconds": rows[-1]["arrival"] if rows else None,
            "nonpositive_stamp_steps": sum(v <= 0 for v in stamps),
            "repeated_stamp_steps": sum(v == 0 for v in stamps),
            "backward_stamp_steps": sum(v < 0 for v in stamps),
            "nonpositive_arrival_steps": sum(v <= 0 for v in arrivals),
            "repeated_arrival_steps": sum(v == 0 for v in arrivals),
            "backward_arrival_steps": sum(v < 0 for v in arrivals),
            "max_stamp_gap_seconds": max(stamps) if stamps else None,
            "max_arrival_gap_seconds": max(arrivals) if arrivals else None,
            "frame_number_discontinuities": sum(v != 1 for v in frames),
            "nonpositive_frame_steps": sum(v <= 0 for v in frames),
            "missing_frame_numbers": sum(max(0, v - 1) for v in frames)}


def internal_pass(stats, limits):
    return (stats["count"] >= 2 and stats["nonpositive_stamp_steps"] == 0
            and stats["nonpositive_arrival_steps"] == 0
            and stats["frame_number_discontinuities"] == 0
            and stats["max_stamp_gap_seconds"] <= limits["gap"]
            and stats["max_arrival_gap_seconds"] <= limits["gap"])


def window(rows, start, duration, limits, setup_boundary=False):
    selected = [row for row in rows if start <= row["arrival"] < start + duration]
    result = summarize(selected)
    rate = len(selected) / duration
    result.update(window_start_seconds=start, window_duration_seconds=duration,
                  whole_window_hz=rate, setup_boundary=setup_boundary,
                  start_boundary_gap_seconds=selected[0]["arrival"] - start if selected else None,
                  end_boundary_gap_seconds=start + duration - selected[-1]["arrival"] if selected else None)
    rate_pass = limits["rate"][0] <= rate <= limits["rate"][1]
    boundaries_pass = bool(selected) and result["end_boundary_gap_seconds"] <= limits["gap"]
    if not setup_boundary and selected:
        boundaries_pass = boundaries_pass and result["start_boundary_gap_seconds"] <= limits["gap"]
    result["setup_boundary_review_required"] = bool(setup_boundary and selected
                                                     and result["start_boundary_gap_seconds"] > limits["gap"])
    result["automatic_checks_pass"] = rate_pass and boundaries_pass and internal_pass(result, limits)
    return result


def analyze(path, duration, interior_window=30.):
    if not math.isfinite(duration) or duration <= 0 or not math.isfinite(interior_window) or interior_window <= 0:
        raise ValueError("Duration and interior window must be positive finite seconds")
    capture = read_capture(path)
    out = {"source": str(path), "source_sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest(),
           "duration_requested_seconds": duration, "opened_profiles": capture["profiles"],
           "profile_check_pass": True, "whole_request": {}, "all_received_samples": {}, "interior_windows": [],
           "timestamp_domain": "Not serialized by vendor CSV; do not infer camera/IMU synchronization",
           "arrival_clock": "Vendor high_resolution_clock relative to pre-open timer; not guaranteed monotonic by C++ API",
           "setup_boundary_policy": "Report initial opening offsets separately; mandatory human review if beyond stream gap target. Internal/end gaps and all rates remain strict.",
           "imu_precision": "Vendor CSV quantizes vectors to 0.001; retain raw file",
           "N2": "NOT DECIDED; requires 300-second soak plus image/raw-log/power review"}
    for name, rows in capture["streams"].items():
        stats = summarize(rows)
        stats["automatic_checks_pass"] = internal_pass(stats, EXPECTED[name])
        stats["samples_after_requested_interval"] = sum(row["arrival"] >= duration for row in rows)
        out["all_received_samples"][name] = stats
        out["whole_request"][name] = window(rows, 0., duration, EXPECTED[name], setup_boundary=True)
    start = 5.
    while start + interior_window <= duration - 1.:
        out["interior_windows"].append({name: window(rows, start, interior_window, EXPECTED[name])
                                        for name, rows in capture["streams"].items()})
        start += interior_window
    out["automatic_checks_pass"] = all(stats["automatic_checks_pass"] for group in
                                         [out["all_received_samples"], out["whole_request"], *out["interior_windows"]]
                                         for stats in group.values())
    out["setup_boundary_review_required"] = any(s["setup_boundary_review_required"] for s in out["whole_request"].values())
    out["verdict"] = "REVIEW_REQUIRED" if out["automatic_checks_pass"] else "FAIL"
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--interior-window", type=float, default=30.)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        result = analyze(args.input, args.duration, args.interior_window)
    except (ValueError, OSError) as error:
        result = {"source": str(args.input), "verdict": "FAIL", "automatic_checks_pass": False,
                  "failures": [str(error)], "N2": "NOT PASSED"}
    with args.output.open("x") as handle:
        json.dump(result, handle, indent=2, allow_nan=False)
        handle.write("\n")
    print(json.dumps({"verdict": result["verdict"], "output": str(args.output)}))
    return 0 if result["automatic_checks_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
