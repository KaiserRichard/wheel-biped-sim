"""Reuse tracking CSV and stdout counts emitted by stereo_inertial_d435i_file.patch."""

import argparse
import csv
from decimal import Decimal, InvalidOperation, ROUND_HALF_EVEN
import hashlib
import io
import json
from pathlib import Path
import re
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from tools.runtime_observer.stats import UNAVAILABLE, distribution

HEADER = ["image_s", "state", "imu_initialized", "imu_samples", "track_ms"]
SUMMARY_FIELDS = ("received", "processed", "prefix_images", "prefix_gyro",
                  "uncovered_tail_images", "pending_images")
SUMMARY_RE = re.compile(r"^" + r"\s+".join(field + r"=([0-9]+)" for field in SUMMARY_FIELDS) + r"\s*$")


def duration_ns(text, scale):
    try:
        value = Decimal(text) * scale
        if not value.is_finite() or value < 0:
            raise ValueError("timing values must be finite and nonnegative")
        return int(value.to_integral_value(rounding=ROUND_HALF_EVEN))
    except InvalidOperation as exc:
        raise ValueError("invalid decimal timing value") from exc


def read_tracking(text):
    reader = csv.DictReader(io.StringIO(text))
    if reader.fieldnames != HEADER:
        raise ValueError("expected the patch's image_s,state,imu_initialized,imu_samples,track_ms header")
    records = []
    for row in reader:
        if None in row or any(value is None for value in row.values()):
            raise ValueError("malformed tracking row")
        if row["imu_initialized"] not in ("0", "1"):
            raise ValueError("imu_initialized must be 0 or 1")
        if not re.fullmatch(r"-?[0-9]+", row["state"]) or not re.fullmatch(r"[0-9]+", row["imu_samples"]):
            raise ValueError("state/imu_samples must be integers")
        image_ns = duration_ns(row["image_s"], 1_000_000_000)
        if records and image_ns <= records[-1]["image_ns"]:
            raise ValueError("nonincreasing image stamps in tracking CSV")
        records.append({"image_ns": image_ns, "image_s_printed": row["image_s"],
                        "state": int(row["state"]), "imu_initialized": int(row["imu_initialized"]),
                        "imu_samples": int(row["imu_samples"]),
                        "track_ns": duration_ns(row["track_ms"], 1_000_000)})
    return records


def read_stdout(text):
    summaries = []
    for line in text.splitlines():
        if line.startswith("received="):
            match = SUMMARY_RE.fullmatch(line)
            if match is None:
                raise ValueError("malformed patch count summary")
            summary = dict(zip(SUMMARY_FIELDS, map(int, match.groups())))
            if summary["processed"] + summary["prefix_images"] + summary["uncovered_tail_images"] + summary["pending_images"] > summary["received"]:
                raise ValueError("patch image counts exceed received frames")
            summaries.append(summary)
    return summaries


def summarize(records, stdout=None, source_domain=None, source_session=None):
    if bool(source_domain) != bool(source_session):
        raise ValueError("provide source domain and session together")
    summaries = read_stdout(stdout) if stdout is not None else []
    if len(summaries) > 1:
        raise ValueError("multiple run summaries; separate runs before aggregation")
    if summaries and summaries[0]["processed"] != len(records):
        raise ValueError("processed count differs from tracking row count")
    span = records[-1]["image_ns"] - records[0]["image_ns"] if records else 0
    return {"evidence": "MEASURED parsed patch numbers; hardware timing UNKNOWN",
            "track_duration_ns": distribution([row["track_ns"] for row in records]),
            "track_duration_domain": "host_steady (duration only; absolute host epoch absent)",
            "timestamp_conversion": "printed decimal seconds/ms to ns, half-even rounding",
            "source_domain": source_domain or "UNKNOWN", "source_session": source_session or "UNKNOWN",
            "source_interval_processed_hz": ((len(records) - 1) * 1e9 / span
                                              if source_domain and source_domain != "UNKNOWN" and span else UNAVAILABLE),
            "host_throughput_per_s": UNAVAILABLE,
            "source_ages_ns": UNAVAILABLE, "queue_wait_ns": UNAVAILABLE,
            "queue_depth": UNAVAILABLE, "drops_by_reason": UNAVAILABLE,
            "count_summary": summaries[0] if summaries else UNAVAILABLE,
            "imu_samples": distribution([row["imu_samples"] for row in records]),
            "initialized_rows": sum(row["imu_initialized"] for row in records),
            "states": {str(state): sum(row["state"] == state for row in records)
                       for state in sorted({row["state"] for row in records})}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("tracking", type=Path)
    parser.add_argument("--stdout", type=Path)
    parser.add_argument("--source-domain")
    parser.add_argument("--source-session")
    args = parser.parse_args(argv)
    try:
        raw = args.tracking.read_bytes()
        output = args.stdout.read_bytes() if args.stdout else None
        report = summarize(read_tracking(raw.decode()), output.decode() if output is not None else None,
                           args.source_domain, args.source_session)
        report["input"] = {"path": str(args.tracking), "sha256": hashlib.sha256(raw).hexdigest()}
        if output is not None:
            report["stdout_input"] = {"path": str(args.stdout), "sha256": hashlib.sha256(output).hexdigest()}
        print(json.dumps(report, indent=2, allow_nan=False))
        return 0
    except (OSError, ValueError, TypeError) as exc:
        print(f"patch timing: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
