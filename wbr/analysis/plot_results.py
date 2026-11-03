"""Plot labeled scenario CSV output as a small SVG figure."""
import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def read_trace(path):
    with Path(path).open(newline="") as handle:
        rows = (line for line in handle if not line.startswith("#"))
        reader = csv.DictReader(rows)
        values = {key: [] for key in reader.fieldnames}
        for row in reader:
            for key in values:
                values[key].append(float(row[key]))
    return {key: np.asarray(value) for key, value in values.items()}


def plot(path, output):
    trace = read_trace(path)
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 5))
    axes[0].plot(trace["t"], np.degrees(trace["phi"]))
    axes[0].set_ylabel("Pitch (deg)")
    axes[1].plot(trace["t"], trace["x"], label="forward position")
    axes[1].plot(trace["t"], trace["z"], label="body height")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Position (m)")
    axes[1].legend()
    fig.suptitle(f"MEASURED simulation trace: {Path(path).stem}")
    fig.tight_layout()
    fig.savefig(output, format="svg")
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    plot(args.trace, args.output)
