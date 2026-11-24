#!/usr/bin/env python3
"""Summarize advisory_link output directories and plot Pi-monotonic run data."""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


ADVISORY_NAMES = {
    "0": "NONE",
    "1": "JUMP",
    "2": "STOP",
    "3": "UNCERTAIN",
    "NONE": "NONE",
    "JUMP": "JUMP",
    "JUMP_CANDIDATE": "JUMP",
    "STOP": "STOP",
    "STOP_CANDIDATE": "STOP",
    "UNCERTAIN": "UNCERTAIN",
}
ADVISORY_COLORS = {
    "NONE": "#9e9e9e",
    "JUMP": "#2ca02c",
    "STOP": "#d62728",
    "UNCERTAIN": "#ffbf00",
    "UNKNOWN": "#7f7f7f",
}


def _number(row, *names):
    for name in names:
        value = row.get(name)
        if value in (None, ""):
            continue
        try:
            value = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            return value
    return None


def _advisory(value):
    if value is None or str(value).strip() == "":
        return "UNKNOWN"
    token = str(value).strip().upper()
    try:
        numeric = float(token)
        if numeric.is_integer():
            token = str(int(numeric))
    except ValueError:
        pass
    return ADVISORY_NAMES.get(token, token)


def _percentiles_ms(values, percentiles):
    valid = [v / 1e6 for v in values if v is not None and math.isfinite(v)]
    if not valid:
        return {p: None for p in percentiles}
    result = np.percentile(np.asarray(valid, dtype=float), percentiles)
    return {p: float(v) for p, v in zip(percentiles, np.atleast_1d(result))}


def _format(value, digits=3):
    return "—" if value is None else f"{value:.{digits}f}"


def _time_ns(row):
    return _number(row, "frame_receipt_pi_ns", "detector_before_ns",
                   "detector_after_ns", "before_write_ns")


def plot_run(out_dir, rows, run_name):
    times = [_time_ns(r) for r in rows]
    valid_times = [t for t in times if t is not None]
    if not valid_times:
        raise ValueError(f"{out_dir}: packets.csv has no usable Pi-monotonic time column")
    origin = valid_times[0]
    x = np.asarray([(t - origin) / 1e9 if t is not None else np.nan for t in times])
    labels = [_advisory(r.get("advisory")) for r in rows]

    fig, (band_ax, data_ax) = plt.subplots(
        2, 1, figsize=(11, 6), sharex=True,
        gridspec_kw={"height_ratios": [1, 4]}, constrained_layout=True)
    band_ax.set_ylim(0, 1)
    band_ax.set_yticks([])
    band_ax.set_ylabel("Advisory")
    band_ax.grid(False)
    for i, (time_s, label) in enumerate(zip(x, labels)):
        if not np.isfinite(time_s):
            continue
        if i + 1 < len(x) and np.isfinite(x[i + 1]) and x[i + 1] > time_s:
            right = x[i + 1]
        else:
            right = time_s + (np.nanmedian(np.diff(x[np.isfinite(x)]))
                              if np.count_nonzero(np.isfinite(x)) > 1 else 0.05)
        band_ax.axvspan(time_s, right, color=ADVISORY_COLORS.get(label, ADVISORY_COLORS["UNKNOWN"]),
                        linewidth=0)

    distance = [_number(r, "distance_m") for r in rows]
    height = [_number(r, "height_m") for r in rows]
    data_ax.plot(x, distance, label="distance_m", color="#1f77b4", linewidth=1.2)
    data_ax.plot(x, height, label="height_m", color="#ff7f0e", linewidth=1.2)
    data_ax.set_xlabel("Pi monotonic time since first frame (s)")
    data_ax.set_ylabel("Distance / height (m)")
    data_ax.set_title(run_name)
    data_ax.grid(True, alpha=0.25)
    data_ax.legend(loc="best")

    # Include a compact advisory key even when one class is absent in this run.
    handles = [plt.Line2D([0], [0], color=c, linewidth=6, label=name)
               for name, c in ADVISORY_COLORS.items() if name != "UNKNOWN"]
    band_ax.legend(handles=handles, ncol=4, loc="upper center", fontsize="small",
                   frameon=False, bbox_to_anchor=(0.5, 1.35))
    path = out_dir / "advisory_summary.png"
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def summarize_run(out_dir):
    out_dir = Path(out_dir)
    with (out_dir / "summary.json").open() as f:
        source_summary = json.load(f)
    with (out_dir / "packets.csv").open(newline="") as f:
        rows = list(csv.DictReader(f))

    labels = [_advisory(r.get("advisory")) for r in rows]
    histogram = Counter(labels)
    confirmed = [r for r, label in zip(rows, labels) if label in ("JUMP", "STOP")]
    first_confirmed_pair = next(((r, label) for r, label in zip(rows, labels)
                                 if label in ("JUMP", "STOP")), None)
    first_confirmed = first_confirmed_pair[0] if first_confirmed_pair else None
    detector = [_number(r, "detector_ns") for r in rows]
    receipt_to_send = [
        _number(r, "receipt_to_before_write_ns")
        if _number(r, "receipt_to_before_write_ns") is not None
        else ((before - receipt) if before is not None and receipt is not None else None)
        for r in rows
        for before, receipt in [(_number(r, "before_write_ns"),
                                 _number(r, "frame_receipt_pi_ns"))]
    ]
    rtt = [_number(r, "rtt_ns") for r in rows]

    echo_rejects = source_summary.get("rejected_echoes")
    if echo_rejects is None:
        echo_rejects = sum(1 for r in rows if (_number(r, "echo_result") or 0) != 0)
    stale_events = source_summary.get("stale_events")
    if stale_events is None:
        stale_events = 0
        previously_stale = False
        for row in rows:
            stale = (_number(row, "stale") or 0) != 0
            if stale and not previously_stale:
                stale_events += 1
            previously_stale = stale

    plot_path = plot_run(out_dir, rows, out_dir.name)
    det_pct = _percentiles_ms(detector, [50, 95])
    send_pct = _percentiles_ms(receipt_to_send, [50, 95])
    rtt_pct = _percentiles_ms(rtt, [50, 99])
    confirmed_height = [v for r in confirmed if (v := _number(r, "height_m")) is not None]
    confirmed_distance = [v for r in confirmed if (v := _number(r, "distance_m")) is not None]
    first_seq = _number(first_confirmed, "seq") if first_confirmed else None
    first_distance = _number(first_confirmed, "distance_m") if first_confirmed else None
    result = {
        "run": out_dir.name,
        "frames": source_summary.get("frames", len(rows)),
        "dropped_frames": source_summary.get("dropped_frames", 0),
        "advisory_histogram": dict(sorted(histogram.items())),
        "first_confirmed_seq": first_seq,
        "first_confirmed_advisory": first_confirmed_pair[1] if first_confirmed_pair else None,
        "first_confirmed_distance_m": first_distance,
        "confirmed_median_height_m": float(np.median(confirmed_height)) if confirmed_height else None,
        "confirmed_median_distance_m": float(np.median(confirmed_distance)) if confirmed_distance else None,
        "detector_p50_ms": det_pct[50], "detector_p95_ms": det_pct[95],
        "receipt_to_send_p50_ms": send_pct[50], "receipt_to_send_p95_ms": send_pct[95],
        "rtt_p50_ms": rtt_pct[50], "rtt_p99_ms": rtt_pct[99],
        "echo_rejects": echo_rejects, "stale_events": stale_events,
        "plot": str(plot_path),
    }
    result["advisory_histogram_text"] = ", ".join(
        f"{name}:{count}" for name, count in sorted(histogram.items())) or "—"
    return result


def print_table(results):
    columns = [
        ("run", "run"), ("frames", "frames"), ("dropped_frames", "dropped"),
        ("advisory_histogram_text", "advisory histogram"),
        ("first_confirmed_seq", "first JUMP/STOP seq"),
        ("first_confirmed_distance_m", "distance at first"),
        ("confirmed_median_height_m", "confirmed median height"),
        ("confirmed_median_distance_m", "confirmed median distance"),
        ("detector_p50_ms", "det P50 ms"), ("detector_p95_ms", "det P95 ms"),
        ("receipt_to_send_p50_ms", "receipt→send P50 ms"),
        ("receipt_to_send_p95_ms", "receipt→send P95 ms"),
        ("rtt_p50_ms", "RTT P50 ms"), ("rtt_p99_ms", "RTT P99 ms"),
        ("echo_rejects", "echo rejects"), ("stale_events", "stale events"),
    ]
    data = []
    for result in results:
        row = []
        for key, _ in columns:
            value = result[key]
            row.append(_format(value) if key.endswith("_ms") or key.endswith("_m") else str(value))
        data.append(row)
    widths = [max(len(title), *(len(row[i]) for row in data)) for i, (_, title) in enumerate(columns)]
    print(" | ".join(title.ljust(widths[i]) for i, (_, title) in enumerate(columns)))
    print("-+-".join("-" * width for width in widths))
    for row in data:
        print(" | ".join(cell.ljust(widths[i]) for i, cell in enumerate(row)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dirs", nargs="+", type=Path,
                        help="advisory_link output directories containing summary.json and packets.csv")
    args = parser.parse_args(argv)
    results = [summarize_run(path) for path in args.output_dirs]
    print_table(results)
    for result in results:
        print(f"{result['run']}: wrote {result['plot']}")


if __name__ == "__main__":
    main()
