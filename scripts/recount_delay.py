#!/usr/bin/env python3
"""Recount pass flags in archived CAD obstacle receipts."""
import argparse
import csv
import json
from pathlib import Path


SEEDS = range(5, 10)
DELAYS_MS = (0, 2, 4, 6, 8)


def recount(results_dir, output):
    counts = []
    for delay in DELAYS_MS:
        passes = 0
        for seed in SEEDS:
            source = results_dir / f"built_obstacle_s{seed}_d{delay}_c0_baseline.json"
            if not source.is_file():
                raise FileNotFoundError(source.name)
            receipt = json.loads(source.read_text())
            if receipt.get("status") != "MEASURED":
                raise ValueError(f"unmeasured input: {source.name}")
            passes += bool(receipt["metrics"]["pass"])
        counts.append((delay, passes, len(SEEDS), "MEASURED"))
    with output.open("w", newline="") as handle:
        handle.write(
            "# MEASURED by scripts/recount_delay.py from port.py receipts; "
            "source files are built_obstacle_s{5..9}_d{0,2,4,6,8}_c0_baseline.json. "
            "Delay settings are ASSUMED.\n"
        )
        writer = csv.writer(handle)
        writer.writerow(["delay_ms", "passes", "trials", "status"])
        writer.writerows(counts)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results_dir", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    recount(args.results_dir, args.output)
