#!/usr/bin/env python3
"""Summarize tracking and trajectory metrics for an M1 lab map bag."""
from __future__ import annotations
import argparse
import importlib.util
import json
from pathlib import Path
from orb_checks import read_bag, tracking_metrics, map_change_metrics, trajectory_metrics


def _exporter():
    path = Path(__file__).resolve().parents[1] / "map_export" / "export_map.py"
    spec = importlib.util.spec_from_file_location("orb_map_export", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load map exporter at {path}")
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module


def run(bag: Path, output: Path) -> dict:
    data = read_bag(bag)
    tracking = tracking_metrics(data["statuses"])
    map_changes = map_change_metrics(data["infos"])
    traj = trajectory_metrics(data["poses"])
    summary = {"input_bag": str(bag), "pose_count": len(data["poses"]), "tracking": tracking,
               "map_changes": map_changes, "trajectory_length_m": traj["path_length_m"],
               "start_to_end_return_distance_m": traj["return_distance_m"],
               "pose_jumps": traj["pose_jumps"],
               "map_change_limitation": None if map_changes["available"] else
               "No /d435_orb/map_points_info messages; map changes cannot be counted from pose/status alone."}
    output.mkdir(parents=True, exist_ok=True)
    exported = _exporter().export(bag, output, 0.02)
    summary["map_export"] = {"raw_snapshot_points": exported["raw_snapshot_points"],
                             "exported_points": exported["exported_points"],
                             "snapshot_info": exported["snapshot_info"],
                             "figures": ["topdown.png", "side.png"]}
    (output / "m1_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, default=Path("orb_checks_m1"))
    args = parser.parse_args()
    result = run(args.bag, args.output)
    print(json.dumps({"tracking": result["tracking"], "map_changes": result["map_changes"],
                      "trajectory_length_m": result["trajectory_length_m"],
                      "start_to_end_return_distance_m": result["start_to_end_return_distance_m"],
                      "pose_jumps": result["pose_jumps"]}, indent=2))

if __name__ == "__main__":
    main()
