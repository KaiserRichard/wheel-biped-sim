#!/usr/bin/env python3
"""Measure a straight-line ORB trajectory against a tape distance."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from orb_checks import POSE_TOPIC, read_bag, l1_metrics, tracking_metrics, points


def run(bag: Path, output: Path, tape: float) -> dict:
    data = read_bag(bag)
    poses = data["poses"]
    metrics = l1_metrics(poses, tape)
    metrics["input_bag"] = str(bag)
    metrics["pose_count"] = len(poses)
    metrics["tracking"] = tracking_metrics(data["statuses"])
    output.mkdir(parents=True, exist_ok=True)
    (output / "l1_line.json").write_text(json.dumps(metrics, indent=2, allow_nan=False) + "\n")
    xyz = points(poses)
    fig, ax = plt.subplots(figsize=(8, 6), dpi=180)
    if len(xyz):
        ax.plot(xyz[:, 0], xyz[:, 2], color="#2468a2", linewidth=1.3, label="camera trajectory")
        start, end = np.asarray(metrics["start_position_m"]), np.asarray(metrics["end_position_m"])
        ax.scatter(start[0], start[2], marker="o", s=48, color="#2ca02c", edgecolor="black", zorder=3, label="stationary start median")
        ax.scatter(end[0], end[2], marker="X", s=54, color="#d62728", edgecolor="black", zorder=3, label="stationary end median")
        ax.plot([start[0], end[0]], [start[2], end[2]], linestyle="--", color="black", alpha=.6, label="measured line")
    ax.set_xlabel("slam_world x (m, optical right)")
    ax.set_ylabel("slam_world z (m, optical forward)")
    ax.set_title(f"L1 line: {metrics['start_end_distance_m']:.4f} m; tape {tape:.3f} m")
    ax.grid(True, alpha=.25); ax.axis("equal"); ax.legend(loc="best", fontsize=8)
    fig.tight_layout(); fig.savefig(output / "l1_line.png"); plt.close(fig)
    return metrics


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--tape", type=float, default=3.00)
    parser.add_argument("--output", type=Path, default=Path("orb_checks_l1"))
    args = parser.parse_args()
    if args.tape <= 0:
        parser.error("--tape must be positive")
    result = run(args.bag, args.output, args.tape)
    print(json.dumps({k: result[k] for k in ("start_end_distance_m", "error_vs_tape_m", "path_length_m", "lateral_deviation_max_m", "pose_jumps")}, indent=2))

if __name__ == "__main__":
    main()
