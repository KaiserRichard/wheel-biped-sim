#!/usr/bin/env python3
"""Offline recorder timestamp diagnostics; never a qualified online clock map."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from decimal import Decimal, DecimalException, ROUND_HALF_EVEN, localcontext
import hashlib
import io
import json
from pathlib import Path
import sys

import numpy as np

if __package__:
    from .clockmap import _fit_affine
else:
    from clockmap import _fit_affine

VERSION = "0.1.0"
PERCENTILE_METHOD = "numpy linear interpolation"
REQUIRED_COLUMNS = {
    "stream", "index", "frame_number", "sdk_ms", "domain",
    "host_steady_ns", "host_utc_ns",
}
LATENCY_LABEL = (
    "Arrival latency includes USB/driver delay and recorder callback/lock delay; "
    "it is not exposure-to-arrival. This is a proxy assuming SDK Global Time "
    "and host UTC share an epoch; SDK mapping error and clock sampling skew are unknown."
)


def _sdk_ns(value: str) -> int:
    try:
        with localcontext() as context:
            context.prec = max(60, len(value) + 20)
            number = Decimal(value)
            if not number.is_finite():
                raise ValueError("sdk_ms must be finite")
            return int((number * 1_000_000).to_integral_value(rounding=ROUND_HALF_EVEN))
    except DecimalException as exc:
        raise ValueError("invalid sdk_ms") from exc


def _domain(value: str) -> str:
    normalized = value.strip().lower().replace(" ", "_")
    return normalized.removeprefix("rs2_timestamp_domain_")


def _percentiles(values: list[int] | tuple[float, ...]) -> dict:
    return {
        "count": len(values),
        "percentile_method": PERCENTILE_METHOD,
        **{name: float(np.percentile(values, p)) for name, p in
           (("min", 0), ("p50", 50), ("p90", 90), ("p95", 95),
            ("p99", 99), ("max", 100))},
    }


@dataclass(frozen=True)
class _Row:
    line: int
    stream: str
    index: int
    frame_number: int
    sdk_ns: int
    domain: str
    steady_ns: int
    utc_ns: int

    @property
    def arrival_latency_ns(self) -> int:
        # Explicit per-record Global Time -> host steady bridge:
        # mapped_sdk_steady = steady + (sdk_global - utc).
        # Thus steady - mapped_sdk_steady = utc - sdk_global. This comparison
        # uses the declared common-epoch assumption, not an online ClockMap.
        mapped_sdk_steady = self.steady_ns + (self.sdk_ns - self.utc_ns)
        return self.steady_ns - mapped_sdk_steady


def analyze_timestamps(path: str | Path, *, utc_step_threshold_ns: int = 1_000_000) -> dict:
    """Fit only global_time; split domain changes, reversals and UTC steps.

    The CSV contains no boot IDs: file identity is diagnostic provenance only.
    It cannot authorize source-age arithmetic or detect a hidden reboot.
    """
    if type(utc_step_threshold_ns) is not int or utc_step_threshold_ns < 0:
        raise ValueError("utc_step_threshold_ns must be a nonnegative integer")
    path = Path(path)
    raw = path.read_bytes()
    reader = csv.DictReader(io.StringIO(raw.decode("utf-8-sig")))
    if reader.fieldnames is None or len(set(reader.fieldnames)) != len(reader.fieldnames):
        raise ValueError("missing or duplicate CSV header")
    missing = REQUIRED_COLUMNS - set(reader.fieldnames)
    if missing:
        raise ValueError(f"missing CSV columns: {', '.join(sorted(missing))}")
    streams: dict[tuple[str, int], list[_Row]] = {}
    for fields in reader:
        line = reader.line_num
        try:
            if None in fields or any(fields[key] is None for key in REQUIRED_COLUMNS):
                raise ValueError("incorrect CSV field count")
            row = _Row(line, fields["stream"], int(fields["index"]),
                       int(fields["frame_number"]), _sdk_ns(fields["sdk_ms"]),
                       _domain(fields["domain"]), int(fields["host_steady_ns"]),
                       int(fields["host_utc_ns"]))
            if not row.stream.strip() or not row.domain:
                raise ValueError("empty stream/domain")
            if min(row.index, row.frame_number, row.sdk_ns, row.steady_ns, row.utc_ns) < 0:
                raise ValueError("negative recorder index/frame/epoch")
        except (ValueError, TypeError) as exc:
            raise ValueError(f"CSV line {line}: {exc}") from exc
        streams.setdefault((row.stream, row.index), []).append(row)
    if not streams:
        raise ValueError("CSV has no timestamp rows")

    reports = []
    for (stream, index), rows in sorted(streams.items()):
        segments: list[tuple[str, list[_Row]]] = []
        previous: _Row | None = None
        current: list[_Row] | None = None
        domains: dict[str, int] = {}
        for row in rows:
            domains[row.domain] = domains.get(row.domain, 0) + 1
            if row.domain != "global_time":
                previous, current = row, None
                continue
            reason = None
            if previous is None:
                reason = "start of stream"
            elif previous.domain != row.domain:
                reason = "domain transition"
            elif row.sdk_ns <= previous.sdk_ns:
                reason = "nonincreasing SDK epoch"
            elif row.steady_ns <= previous.steady_ns:
                reason = "nonincreasing host steady epoch"
            elif row.frame_number <= previous.frame_number:
                reason = "nonincreasing frame number"
            elif abs((row.utc_ns - previous.utc_ns) -
                     (row.steady_ns - previous.steady_ns)) > utc_step_threshold_ns:
                reason = "host UTC step or clock sampling skew"
            if current is None or reason:
                current = []
                segments.append((reason or "start of global_time", current))
            current.append(row)
            previous = row
        segment_reports = []
        for reason, segment in segments:
            entry = {
                "start_reason": reason,
                "first_csv_line": segment[0].line,
                "last_csv_line": segment[-1].line,
                "sample_count": len(segment),
                "arrival_latency_proxy_ns": _percentiles([r.arrival_latency_ns for r in segment]),
                "negative_arrival_latency_count": sum(r.arrival_latency_ns < 0 for r in segment),
            }
            if len(segment) < 3:
                entry["fit_status"] = "UNAVAILABLE: need at least three ordered global_time rows"
            else:
                fit = _fit_affine([r.sdk_ns for r in segment], [r.steady_ns for r in segment])
                entry.update({
                    "fit_status": "DIAGNOSTIC",
                    "sdk_to_host_steady_fit": {
                        "source_domain": "sdk_global_time",
                        "target_domain": "host_steady",
                        "source_reference_ns": fit.source_ref_ns,
                        "target_reference_ns": fit.target_ref_ns,
                        "slope": fit.slope,
                        "drift_ppm": (fit.slope - 1) * 1_000_000,
                        "offset_at_reference_ns": fit.map_ns(fit.source_ref_ns) - fit.source_ref_ns,
                        "relative_intercept_ns": fit.intercept_ns,
                        "residual_ns": _percentiles(fit.residuals_ns),
                        "absolute_residual_ns": _percentiles(tuple(abs(r) for r in fit.residuals_ns)),
                    },
                })
            segment_reports.append(entry)
        global_rows = [row for row in rows if row.domain == "global_time"]
        reports.append({
            "stream": stream,
            "index": index,
            "row_count": len(rows),
            "domain_counts": domains,
            "skipped_non_global_time_count": len(rows) - len(global_rows),
            "arrival_latency_proxy_ns": (_percentiles([r.arrival_latency_ns for r in global_rows])
                                         if global_rows else None),
            "segments": segment_reports,
        })
    return {
        "tool": "clockmap",
        "version": VERSION,
        "percentile_method": PERCENTILE_METHOD,
        "evidence": "INFERRED diagnostic method; real recorder files NOT verified",
        "input": {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest()},
        "utc_step_threshold_ns": utc_step_threshold_ns,
        "arrival_latency_label": LATENCY_LABEL,
        "fit_label": (
            "SDK global_time -> host_steady arrival-pair least squares. Offset includes "
            "mean arrival delay; changing transport delay can bias drift. Residuals "
            "are centered jitter, not absolute arrival latency or mapping uncertainty."
        ),
        "streams": reports,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("timestamps_csv", type=Path)
    parser.add_argument("--utc-step-threshold-ns", type=int, default=1_000_000)
    args = parser.parse_args(argv)
    try:
        report = analyze_timestamps(args.timestamps_csv,
                                    utc_step_threshold_ns=args.utc_step_threshold_ns)
    except (OSError, ValueError, UnicodeError, csv.Error) as exc:
        print(f"clockmap: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
