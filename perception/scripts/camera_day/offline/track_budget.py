#!/usr/bin/env python3
"""Summarize per-call TrackStereo compute time from archived frames.csv files."""

import argparse
import csv
import json
import math
from pathlib import Path


REQUIRED_COLUMNS = (
    "source_ns", "tracking_state", "pose_published", "processed",
    "application_dropped", "processing_start_steady_ns",
    "processing_end_steady_ns", "map_id",
)
NOMINAL_BUDGET_NS = 66_700_000
EXACT_15HZ_NUMERATOR_NS = 1_000_000_000
HISTOGRAM_EDGES_MS = (0, 10, 20, 30, 40, 50, 60, 66.7, 75, 100, 150, 200, math.inf)


def _integer(row, field, line_number):
    value = row[field]
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"line {line_number}: {field} must be an integer") from exc
    if str(parsed) != value.strip():
        raise ValueError(f"line {line_number}: {field} must be an integer")
    return parsed


def read_frames(path):
    """Read and validate one runner frames.csv without changing its contents."""
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None or tuple(reader.fieldnames) != REQUIRED_COLUMNS:
            raise ValueError(f"{path}: expected CSV columns {','.join(REQUIRED_COLUMNS)}")
        records = []
        previous_source = None
        previous_end = None
        expected_processed = 1
        for line_number, row in enumerate(reader, start=2):
            if None in row or any(row.get(field) is None or not row[field].strip()
                                  for field in REQUIRED_COLUMNS):
                raise ValueError(f"line {line_number}: malformed or incomplete row")
            source_ns = _integer(row, "source_ns", line_number)
            processed = _integer(row, "processed", line_number)
            start_ns = _integer(row, "processing_start_steady_ns", line_number)
            end_ns = _integer(row, "processing_end_steady_ns", line_number)
            state = _integer(row, "tracking_state", line_number)
            dropped = _integer(row, "application_dropped", line_number)
            _integer(row, "map_id", line_number)
            pose_text = row["pose_published"].lower()
            if pose_text not in ("true", "false"):
                raise ValueError(f"line {line_number}: pose_published must be true or false")
            if previous_source is not None and source_ns <= previous_source:
                raise ValueError(f"line {line_number}: source_ns must be strictly increasing")
            if processed != expected_processed:
                raise ValueError(f"line {line_number}: processed must be contiguous; expected {expected_processed}")
            if start_ns <= 0 or end_ns <= start_ns or (previous_end is not None and start_ns < previous_end):
                raise ValueError(f"line {line_number}: TrackStereo interval must be positive and serial")
            if dropped < 0:
                raise ValueError(f"line {line_number}: application_dropped cannot be negative")
            records.append({
                "source_ns": source_ns,
                "processed": processed,
                "duration_ns": end_ns - start_ns,
                "tracking_state": state,
                "pose_published": pose_text == "true",
            })
            previous_source = source_ns
            previous_end = end_ns
            expected_processed += 1
    if not records:
        raise ValueError(f"{path}: no frame rows")
    return records


def over_budget(duration_ns, threshold_ns):
    """Strict overrun comparison; equality is within budget."""
    return duration_ns > threshold_ns


def over_exact_15hz_budget(duration_ns):
    """Compare against 1/15 s exactly without rounding it to integer ns."""
    return duration_ns * 15 > EXACT_15HZ_NUMERATOR_NS


def _percentile_nearest_rank(values, percentile):
    ordered = sorted(values)
    rank = max(1, math.ceil(percentile * len(ordered)))
    return ordered[rank - 1]


def _threshold_summary(records, threshold_ns, label, exact_15hz=False):
    compare = over_exact_15hz_budget if exact_15hz else lambda value: over_budget(value, threshold_ns)
    overruns = [r for r in records if compare(r["duration_ns"])]
    tracked = [r for r in records if r["pose_published"]]
    tracked_overruns = [r for r in tracked if compare(r["duration_ns"])]
    return {
        "label": label,
        "threshold_ns": threshold_ns,
        "overrun_count": len(overruns),
        "overrun_percent_all_calls": 100.0 * len(overruns) / len(records),
        "tracked_call_count": len(tracked),
        "tracked_overrun_count": len(tracked_overruns),
        "tracked_overrun_percent_tracked_calls": (
            100.0 * len(tracked_overruns) / len(tracked) if tracked else 0.0
        ),
        "over_budget_excess_seconds": (
            sum(max(0, r["duration_ns"] * 15 - EXACT_15HZ_NUMERATOR_NS) for r in records)
            / (15 * 1e9) if exact_15hz else
            sum(max(0, r["duration_ns"] - threshold_ns) for r in records) / 1e9
        ),
    }


def summarize(records, replay):
    durations = [r["duration_ns"] for r in records]
    return {
        "replay": replay,
        "call_count": len(records),
        "tracked_call_count": sum(r["pose_published"] for r in records),
        "duration_percentiles": "nearest-rank",
        "duration_ms": {
            "p50": _percentile_nearest_rank(durations, 0.50) / 1e6,
            "p95": _percentile_nearest_rank(durations, 0.95) / 1e6,
            "max": max(durations) / 1e6,
        },
        "sum_duration_seconds": sum(durations) / 1e9,
        "thresholds": [
            _threshold_summary(records, NOMINAL_BUDGET_NS, "strictly greater than 66.7 ms"),
            _threshold_summary(records, EXACT_15HZ_NUMERATOR_NS // 15,
                               "strictly greater than exact 1/15 s (integer compare: 15*duration_ns > 1e9)",
                               exact_15hz=True),
        ],
        "interpretation": [
            "These are compute-budget misses in a no-drop 1x offline queue, not observed live camera drops or a direct forecast of latest-image queue drops.",
            "Measurements are from the archived Intel Mac x86_64 Docker replay and do not transfer to the Raspberry Pi.",
        ],
    }


def _histogram_counts(records):
    counts = [0] * (len(HISTOGRAM_EDGES_MS) - 1)
    for record in records:
        duration_ms = record["duration_ns"] / 1e6
        for index, (lower, upper) in enumerate(zip(HISTOGRAM_EDGES_MS, HISTOGRAM_EDGES_MS[1:])):
            if lower <= duration_ms < upper:
                counts[index] += 1
                break
    if sum(counts) != len(records):
        raise AssertionError("histogram failed to account for every TrackStereo call")
    return counts


def write_histogram_csv(path, records):
    counts = _histogram_counts(records)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("lower_ms_inclusive", "upper_ms_exclusive", "count", "denominator_calls", "unit"))
        for lower, upper, count in zip(HISTOGRAM_EDGES_MS, HISTOGRAM_EDGES_MS[1:], counts):
            writer.writerow((lower, "inf" if math.isinf(upper) else upper, count, len(records), "ms"))


def write_histogram_svg(path, records):
    counts = _histogram_counts(records)
    width, height = 900, 360
    left, right, top, bottom = 58, 20, 32, 82
    plot_w, plot_h = width - left - right, height - top - bottom
    peak = max(counts) or 1
    slot = plot_w / len(counts)
    bars = []
    labels = []
    for i, count in enumerate(counts):
        bar_h = plot_h * count / peak
        x = left + i * slot + slot * 0.12
        y = top + plot_h - bar_h
        bars.append(f'<rect x="{x:.2f}" y="{y:.2f}" width="{slot * .76:.2f}" height="{bar_h:.2f}" fill="#3569a8"/>')
        upper = HISTOGRAM_EDGES_MS[i + 1]
        label = "200+" if math.isinf(upper) else str(upper)
        labels.append(f'<text x="{left + (i + .5) * slot:.2f}" y="{height - 48}" text-anchor="middle" font-size="10">{label}</text>')
        if count:
            labels.append(f'<text x="{left + (i + .5) * slot:.2f}" y="{max(top + 12, y - 4):.2f}" text-anchor="middle" font-size="10">{count}</text>')
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
           '<rect width="100%" height="100%" fill="white"/>',
           f'<text x="{left}" y="20" font-family="sans-serif" font-size="14">TrackStereo duration histogram — n={len(records)} calls, units ms</text>',
           f'<line x1="{left}" y1="{top + plot_h}" x2="{width-right}" y2="{top + plot_h}" stroke="#333"/>',
           f'<text transform="translate(16 {top + plot_h/2}) rotate(-90)" text-anchor="middle" font-family="sans-serif" font-size="12">calls per bin</text>',
           *bars, *labels,
           f'<text x="{left + plot_w/2}" y="{height - 16}" text-anchor="middle" font-family="sans-serif" font-size="12">Bin upper edge (ms); bins are lower-inclusive and upper-exclusive</text>',
           '</svg>', '']
    path.write_text("\n".join(svg), encoding="utf-8")


def _markdown_table(results):
    lines = [
        "# TrackStereo frame budget — archived three-replay batch", "",
        "All calls are retained in the offline queue. An overrun is a compute-budget miss, not an observed live camera drop or a direct forecast of latest-image queue drops. These Intel Mac x86_64 Docker measurements do not transfer to the Pi.", "",
        "| Replay | Calls | Tracked calls | >66.7 ms all (%, tracked count) | >1/15 s all (%, tracked count) | p50 / p95 / max (ms) | Sum duration (s) | Excess over 66.7 ms (s) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in results:
        a, b = item["thresholds"]
        d = item["duration_ms"]
        lines.append(f"| {item['replay']} | {item['call_count']} | {item['tracked_call_count']} | {a['overrun_count']} ({a['overrun_percent_all_calls']:.3f}%, {a['tracked_overrun_count']}) | {b['overrun_count']} ({b['overrun_percent_all_calls']:.3f}%, {b['tracked_overrun_count']}) | {d['p50']:.3f} / {d['p95']:.3f} / {d['max']:.3f} | {item['sum_duration_seconds']:.3f} | {a['over_budget_excess_seconds']:.3f} |")
    lines.extend(["", "Overrun counts use strict integer comparisons: `duration_ns > 66,700,000` and `15 × duration_ns > 1,000,000,000`. Percentages are in each run's JSON. Histogram bins are identical across runs, lower-inclusive/upper-exclusive, in milliseconds; each CSV/SVG accounts for all calls in its run.", ""])
    return "\n".join(lines)


def run_batch(input_root, output_root):
    output_root.mkdir(parents=True, exist_ok=False)
    results = []
    for number in range(1, 4):
        replay = f"replay-{number}"
        records = read_frames(input_root / replay / "frames.csv")
        summary = summarize(records, replay)
        run_dir = output_root / replay
        run_dir.mkdir()
        (run_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        write_histogram_csv(run_dir / "histogram.csv", records)
        write_histogram_svg(run_dir / "histogram.svg", records)
        results.append(summary)
    (output_root / "summary.json").write_text(json.dumps({"results": results}, indent=2) + "\n", encoding="utf-8")
    report = _markdown_table(results)
    (output_root / "results.md").write_text(report, encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_root", type=Path, help="directory containing replay-1/frames.csv through replay-3/frames.csv")
    parser.add_argument("output_dir", type=Path, help="new, exclusive-created evidence output directory")
    args = parser.parse_args()
    print(run_batch(args.input_root, args.output_dir))


if __name__ == "__main__":
    main()
