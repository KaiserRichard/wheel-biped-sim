"""Evaluate box, empty-floor, and wall-target runs from a mobile-demo run sheet."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import re
import sys

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

import matplotlib.pyplot as plt
import numpy as np

from tools.obstacle_decision.replay_bag import replay_bag
from tools.obstacle_decision.decide import DecisionParams

NUMERIC_COLUMNS = ("box_height_m", "box_front_distance_m", "box_width_m", "lateral_offset_m",
                   "camera_height_m", "camera_tilt_deg", "wall_distance_m")
CAMERA_HEIGHT_STD_LIMIT_M = 0.005
CAMERA_PITCH_STD_LIMIT_DEG = 0.3
WALL_RELATIVE_ERROR_LIMIT = 0.02


def read_run_sheet(path):
    """Read rows; only run_id and bag_path are required, and numeric cells may be blank."""
    with Path(path).open(newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        columns = reader.fieldnames or []
        missing = [c for c in ("run_id", "bag_path") if c not in columns]
        if missing:
            raise ValueError(f"run sheet missing required columns: {', '.join(missing)}")
        rows, seen = [], set()
        for line, row in enumerate(reader, 2):
            rid, bag = (row.get("run_id") or "").strip(), (row.get("bag_path") or "").strip()
            if not rid or not bag:
                raise ValueError(f"run sheet line {line}: run_id and bag_path are required")
            if rid in seen:
                raise ValueError(f"run sheet line {line}: duplicate run_id {rid!r}")
            seen.add(rid)
            parsed = dict(row)
            parsed["run_id"], parsed["bag_path"] = rid, bag
            for col in NUMERIC_COLUMNS:
                value = (row.get(col) or "").strip()
                if not value:
                    parsed[col] = None
                    continue
                try:
                    parsed[col] = float(value)
                except ValueError as exc:
                    raise ValueError(f"run sheet line {line}: {col} must be numeric or empty") from exc
                if not math.isfinite(parsed[col]):
                    raise ValueError(f"run sheet line {line}: {col} must be finite")
            rows.append(parsed)
    return rows


def _classify(row):
    if row["wall_distance_m"] is not None:
        if row["wall_distance_m"] <= 0:
            raise ValueError(f"run {row['run_id']}: wall_distance_m must be positive")
        return "wall"
    if row["box_height_m"] is None and row["box_front_distance_m"] is None:
        return "empty_floor"
    return "box"


def evaluate_runs(sheet, out=".", *, stride=1, depth_topic=None, info_topic=None, wall_roi_size=80):
    rows = read_run_sheet(sheet)
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    base = Path(sheet).resolve().parent
    policy = DecisionParams()
    results = []
    for row in rows:
        bag = Path(row["bag_path"])
        if not bag.is_absolute():
            bag = base / bag
        kind = _classify(row)
        run_dir = out / re.sub(r"[^A-Za-z0-9_.-]+", "_", row["run_id"])
        summary = replay_bag(bag, depth_topic=depth_topic, info_topic=info_topic,
                             stride=stride, out=run_dir, wall_roi_size=wall_roi_size)
        result = {"run_id": row["run_id"], "run_type": kind, "bag_path": str(bag),
                  "run_sheet": {k: v for k, v in row.items() if k not in ("run_id", "bag_path")},
                  "frame_count": summary["frame_count"], "plane_valid_rate": summary["plane_valid_rate"],
                  "depth_valid_fraction": summary["depth_valid_fraction"],
                  "advisory_histogram": summary["advisory_histogram"]}
        if kind == "box":
            measured_h, measured_d = row["box_height_m"], row["box_front_distance_m"]
            est_h, est_d = summary["obstacle_height_m"]["median"], summary["obstacle_distance_m"]["median"]
            est_w = summary["obstacle_width_m"]["median"]
            expected = None if measured_h is None else (
                "STOP_CANDIDATE" if measured_h > policy.jump_max_h else
                "JUMP_CANDIDATE" if policy.jump_min_h <= measured_h <= policy.jump_max_h else "UNCERTAIN")
            estimated = max(summary["advisory_histogram"], key=summary["advisory_histogram"].get) if summary["advisory_histogram"] else "UNCERTAIN"
            h_err, d_err = _err(est_h, measured_h), _err(est_d, measured_d)
            passed = None
            if h_err is not None and d_err is not None and measured_d <= 1.5:
                passed = abs(h_err) <= 0.015 and abs(d_err) <= 0.05
            result.update({"measured_height_m": measured_h, "estimated_height_m": est_h,
                           "height_error_m": h_err, "measured_distance_m": measured_d,
                           "estimated_distance_m": est_d, "distance_error_m": d_err,
                           "measured_width_m": row["box_width_m"], "estimated_width_m": est_w,
                           "width_error_m": _err(est_w, row["box_width_m"]),
                           "expected_advisory": expected, "estimated_advisory": estimated,
                           "advisory_correct": None if expected is None else estimated == expected,
                           "pass": passed})
        elif kind == "empty_floor":
            camera = summary["camera_height_m"]["median"]
            camerr = _err(camera, row["camera_height_m"])
            hstd, pstd = summary["plane_height_std_m"], summary["plane_pitch_std_deg"]
            passed = (hstd is not None and pstd is not None and
                      hstd <= CAMERA_HEIGHT_STD_LIMIT_M and pstd <= CAMERA_PITCH_STD_LIMIT_DEG)
            result.update({"expected_advisory": "NONE", "estimated_advisory":
                           max(summary["advisory_histogram"], key=summary["advisory_histogram"].get)
                           if summary["advisory_histogram"] else "UNCERTAIN",
                           "false_detection_rate": summary["false_detection_rate"],
                           "measured_camera_height_m": row["camera_height_m"],
                           "estimated_camera_height_m": camera, "camera_height_error_m": camerr,
                           "plane_height_std_m": hstd, "plane_pitch_std_deg": pstd,
                           "pass": passed})
        else:
            measured = row["wall_distance_m"]
            estimate = summary["wall_forward_m"]["median"]
            error = _err(estimate, measured)
            relative = None if error is None else error / measured
            passed = relative is not None and abs(relative) <= WALL_RELATIVE_ERROR_LIMIT
            result.update({"measured_wall_distance_m": measured, "estimated_wall_distance_m": estimate,
                           "wall_error_m": error, "wall_relative_error": relative, "pass": passed,
                           "central_roi_depth_m": summary["central_roi_depth_m"]["median"],
                           "wall_roi_size_px": wall_roi_size})
        results.append(result)
    thresholds = {"box_height_error_m": 0.015, "box_distance_error_m": 0.05,
                  "box_distance_pass_limit_m": 1.5, "empty_floor_height_std_m": CAMERA_HEIGHT_STD_LIMIT_M,
                  "empty_floor_pitch_std_deg": CAMERA_PITCH_STD_LIMIT_DEG,
                  "wall_relative_error": WALL_RELATIVE_ERROR_LIMIT, "wall_roi_size_px": wall_roi_size}
    (out / "evaluation.json").write_text(json.dumps({"thresholds": thresholds, "runs": results}, indent=2) + "\n")
    _write_markdown(out / "evaluation.md", results, policy, thresholds)
    _plots(out, [r for r in results if r["run_type"] == "box"])
    return results


def _err(est, measured):
    return None if est is None or measured is None else float(est - measured)


def _write_markdown(path, rows, policy, thresholds):
    lines = ["# RGB-D bag evaluation", "",
             f"Advisory thresholds: jump height {policy.jump_min_h:.3f}–{policy.jump_max_h:.3f} m; near range {policy.near_m:.2f} m.",
             "Relative bag paths are resolved from the run-sheet directory. All reported statistics use processed frames.",
             "", "## Box runs", "",
             "PASS requires height error ≤ 0.015 m and distance error ≤ 0.05 m when measured distance is ≤ 1.5 m; farther runs have no PASS/FAIL.", "",
             "| run_id | height measured/estimated (m) | height error (m) | distance measured/estimated (m) | distance error (m) | width measured/estimated (m) | width error (m) | advisory expected/estimated | PASS |",
             "|---|---:|---:|---:|---:|---:|---:|---|---|"]
    for r in (x for x in rows if x["run_type"] == "box"):
        lines.append(f"| {r['run_id']} | {_pair(r['measured_height_m'], r['estimated_height_m'])} | {_fmt(r['height_error_m'])} | {_pair(r['measured_distance_m'], r['estimated_distance_m'])} | {_fmt(r['distance_error_m'])} | {_pair(r['measured_width_m'], r['estimated_width_m'])} | {_fmt(r['width_error_m'])} | {r['expected_advisory'] or 'n/a'} / {r['estimated_advisory']} | {_pass(r['pass'])} |")
    lines += ["", "## Empty-floor runs", "", "PASS requires plane height std ≤ 5 mm and plane pitch std ≤ 0.3°.", "",
              "| run_id | false-detection rate | camera-height error (m) | plane height std (m) | plane pitch std (°) | expected/estimated advisory | PASS |",
              "|---|---:|---:|---:|---:|---|---|"]
    for r in (x for x in rows if x["run_type"] == "empty_floor"):
        lines.append(f"| {r['run_id']} | {_fmt(r['false_detection_rate'])} | {_fmt(r['camera_height_error_m'])} | {_fmt(r['plane_height_std_m'])} | {_fmt(r['plane_pitch_std_deg'])} | NONE / {r['estimated_advisory']} | {_pass(r['pass'])} |")
    lines += ["", "## Wall-target runs", "", "Wall forward = horizontal ground-frame distance (mode of corridor points ≥ 10 cm above the plane). The central-ROI axial depth is shown for reference only. PASS requires absolute relative error ≤ 2%.", "",
              "| run_id | tape distance (m) | wall forward (m) | error (m) | relative error | central ROI axial (m) | PASS |", "|---|---:|---:|---:|---:|---:|---|"]
    for r in (x for x in rows if x["run_type"] == "wall"):
        lines.append(f"| {r['run_id']} | {_fmt(r['measured_wall_distance_m'])} | {_fmt(r['estimated_wall_distance_m'])} | {_fmt(r['wall_error_m'])} | {_fmt(r['wall_relative_error'])} | {_fmt(r['central_roi_depth_m'])} | {_pass(r['pass'])} |")
    path.write_text("\n".join(lines) + "\n")


def _plots(out, rows):
    valid = [r for r in rows if r["estimated_height_m"] is not None and r["measured_height_m"] is not None]
    fig, ax = plt.subplots(figsize=(6, 5))
    if valid:
        x = [r["measured_height_m"] for r in valid]; y = [r["estimated_height_m"] for r in valid]
        ax.scatter(x, y); lo=min(x+y); hi=max(x+y); ax.plot([lo,hi],[lo,hi],"k--",linewidth=1)
    ax.set(xlabel="Measured box height (m)", ylabel="Estimated box height (m)", title="Height: estimated vs measured"); ax.grid(True,alpha=.3)
    fig.tight_layout(); fig.savefig(out / "height_estimated_vs_measured.png", dpi=150); plt.close(fig)
    valid = [r for r in rows if r["distance_error_m"] is not None]
    fig, ax = plt.subplots(figsize=(6, 5))
    if valid: ax.scatter([r["measured_distance_m"] for r in valid], [r["distance_error_m"] for r in valid])
    ax.axhline(0,color="k",linewidth=1); ax.set(xlabel="Measured box front distance (m)", ylabel="Distance error, estimate − measured (m)", title="Distance error vs measured distance"); ax.grid(True,alpha=.3)
    fig.tight_layout(); fig.savefig(out / "distance_error_vs_distance.png", dpi=150); plt.close(fig)


def _fmt(v): return "n/a" if v is None else f"{v:.3f}"
def _pair(a,b): return f"{_fmt(a)} / {_fmt(b)}"
def _pass(v): return "n/a" if v is None else "PASS" if v else "FAIL"


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__); p.add_argument("run_sheet"); p.add_argument("--out",default=".")
    p.add_argument("--stride",type=int,default=1); p.add_argument("--depth-topic"); p.add_argument("--info-topic")
    p.add_argument("--wall-roi-size",type=int,default=80,help="central square ROI width/height in pixels")
    a=p.parse_args(argv)
    try: rows=evaluate_runs(a.run_sheet,a.out,stride=a.stride,depth_topic=a.depth_topic,info_topic=a.info_topic,wall_roi_size=a.wall_roi_size)
    except (ValueError,FileNotFoundError) as exc: p.error(str(exc))
    print(json.dumps({"run_count":len(rows),"out":str(Path(a.out))},indent=2))

if __name__ == "__main__": main()
