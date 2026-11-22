#!/usr/bin/env python3
"""Evaluate repeated ORB-SLAM3 EuRoC runs using the repository trajectory evaluator."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
from decimal import Decimal, InvalidOperation

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
EVALUATOR = REPO / "tools/trajectory_eval/traj_eval.py"
VERSION = "1.0.0"
GT_CLOCK = ("EuRoC camera, IMU, and state_groundtruth_estimate0 timestamps are in the "
            "same dataset time base (SOURCE-SUPPORTED: EuRoC dataset documentation and "
            "Burri et al., The EuRoC micro aerial vehicle datasets, IJRR 2016, "
            "https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). "
            "Estimate offset is explicitly zero; this is not a live-device clock mapping.")

def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()

def pose_file(run: Path) -> Path | None:
    # ORB-SLAM3 stereo-inertial EuRoC output is f_*.txt (all frame poses).
    candidates = sorted(run.glob("f_*.txt"))
    candidates.extend(p for p in (run / "CameraTrajectory.txt", run / "KeyFrameTrajectory.txt") if p.is_file())
    return candidates[0] if len(candidates) == 1 else None

def run_exit_code(run: Path) -> tuple[int | None, str | None]:
    """Read this run's exit code, including the legacy saved runs.log format."""
    code_file = run / "exit_code.txt"
    if code_file.is_file():
        try:
            return int(code_file.read_text(encoding="utf-8").strip()), str(code_file)
        except ValueError as exc:
            raise ValueError(f"invalid engine exit code in {code_file}") from exc
    legacy_log = run.parent / "runs.log"
    if legacy_log.is_file():
        match = re.search(rf"(?m)^{re.escape(run.name)} end \S+ rc=(-?\d+)(?:\s|$)",
                          legacy_log.read_text(encoding="utf-8", errors="replace"))
        if match:
            return int(match.group(1)), str(legacy_log)
    return None, None

def detect_timestamp_format(path: Path, config: dict) -> tuple[str, str]:
    first = None
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            row = line.strip()
            if row and not row.startswith("#"):
                first = row.split()[0]
                break
    if first is None:
        raise ValueError(f"empty trajectory: {path}")
    try:
        stamp = abs(Decimal(first))
    except InvalidOperation as exc:
        raise ValueError(f"invalid first trajectory timestamp in {path}: {first}") from exc
    cutoff = Decimal(config["evaluation"]["timestamp_unit_detection"]["ns_magnitude_cutoff"]["value"])
    return ("orbslam3", "ns") if stamp >= cutoff else ("tum", "s")

def count_rows(path: Path) -> int:
    return sum(1 for line in path.open(encoding="utf-8", errors="replace") if line.strip() and not line.lstrip().startswith("#"))

def run_diagnostics(run: Path, trajectory: Path, expected_frames: int | None) -> dict:
    log = run / "stdout.log"
    text = log.read_text(encoding="utf-8", errors="replace") if log.is_file() else ""
    poses = count_rows(trajectory)
    initialized = bool(re.search(r"(?:IMU initialized|end VIBA\s+2|IMU is initialized)", text, re.I))
    loss = len(re.findall(r"Fail to track|Tracking lost|lost tracking", text, re.I))
    resets = len(re.findall(r"Reseting|Resetting|Reset map|Map reset|resetting map", text, re.I))
    return {"initialization_success": initialized, "tracking_loss_log_count": loss,
            "map_reset_log_count": resets, "pose_count": poses,
            "expected_frame_count": expected_frames,
            "pose_fraction": poses / expected_frames if expected_frames else None,
            "pose_fraction_evidence": "CALCULATED: emitted pose rows / non-comment times-file rows" if expected_frames else "UNKNOWN: provide --times"}

def aggregate(values: list[float | None]) -> dict:
    finite = [float(v) for v in values if v is not None]
    if not finite:
        return {"n": 0, "mean": None, "std_population": None, "min": None, "max": None}
    mean = sum(finite) / len(finite)
    return {"n": len(finite), "mean": mean,
            "std_population": (sum((v - mean) ** 2 for v in finite) / len(finite)) ** 0.5,
            "min": min(finite), "max": max(finite)}

def overall_status(criteria: list[dict], runs: list[dict]) -> str:
    if any(c["status"] == "FAIL" for c in criteria) or any(r.get("status") == "FAIL" for r in runs):
        return "FAIL"
    if any(c["status"] == "BLOCKED" for c in criteria) or any(r.get("status") == "BLOCKED" for r in runs):
        return "BLOCKED"
    return "PASS"

def pose_fraction_status(runs: list[dict], minimum: float, maximum: float) -> str:
    fractions = [r.get("diagnostics", {}).get("pose_fraction") for r in runs]
    if not runs or any(value is None for value in fractions):
        return "BLOCKED"
    if any(value < minimum or value > maximum for value in fractions):
        return "FAIL"
    return "PASS"

def evaluator_result(run_name: str, trajectory: Path, gt: Path, out: Path, config: dict, delta: str, unit: str, est_format: str, est_unit: str) -> dict:
    out.parent.mkdir(parents=True, exist_ok=True)
    cmd = [sys.executable, str(EVALUATOR), "single", "--est", str(trajectory), "--ref", str(gt),
           "--est-format", est_format, "--est-time-unit", est_unit, "--ref-format", "euroc_gt", "--ref-time-unit", "ns",
           "--clock-assumption", GT_CLOCK, "--est-frame", "imu_camera (ORB-SLAM3 camera IMU body)",
           "--ref-frame", "body (EuRoC state_groundtruth_estimate0)", "--t-offset", str(config["evaluation"]["estimate_offset_s"]["value"]),
           "--t-max-diff", str(config["evaluation"]["tolerance_s"]["value"]), "--align-both", "--rotation-ate",
           "--rpe-delta", str(delta), "--rpe-unit", unit, "--no-plots", "--out", str(out)]
    if getattr(evaluator_result, "allow_overwrite", False):
        cmd.append("--overwrite")
    proc = subprocess.run(cmd, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
    (out / "console.log").write_text(proc.stdout, encoding="utf-8")
    if proc.returncode != 0:
        raise RuntimeError(f"trajectory evaluator failed for {run_name} ({proc.returncode}): {proc.stdout[-2000:]}")
    return json.loads((out / "result.json").read_text(encoding="utf-8"))

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", nargs="+", type=Path, required=True, help="per-run directories")
    parser.add_argument("--ground-truth", type=Path, required=True, help="EuRoC state_groundtruth_estimate0/data.csv")
    parser.add_argument("--times", type=Path, help="EuRoC timestamp file used to calculate output-pose fraction")
    parser.add_argument("--expected-frames", type=int, help="frame count from the exact sequence timestamps file when that file is unavailable")
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--overwrite", action="store_true", help="replace pipeline-owned report/evaluation outputs")
    parser.add_argument("--config", type=Path, default=HERE / "thresholds.json")
    args = parser.parse_args()
    try:
        config = json.loads(args.config.read_text(encoding="utf-8"))
        gt = args.ground_truth.resolve()
        times = args.times.resolve() if args.times else None
        if args.expected_frames is not None and args.expected_frames <= 0:
            raise ValueError("--expected-frames must be positive")
        frame_count = count_rows(times) if times else args.expected_frames
        if args.out.exists() and not args.overwrite:
            raise ValueError(f"output exists: {args.out}")
        args.out.mkdir(parents=True, exist_ok=True)
        evaluator_result.allow_overwrite = args.overwrite
        runs = []
        for index, run_arg in enumerate(args.runs, 1):
            run = run_arg.resolve()
            trajectory = pose_file(run)
            if trajectory is None:
                runs.append({"run": run.name, "status": "BLOCKED", "reason": "expected exactly one f_*.txt, CameraTrajectory.txt, or KeyFrameTrajectory.txt"})
                continue
            est_format, est_unit = detect_timestamp_format(trajectory, config)
            diag = run_diagnostics(run, trajectory, frame_count)
            exit_code, exit_code_source = run_exit_code(run)
            run_out = args.out / f"run{index:02d}"
            meter = evaluator_result(run.name, trajectory, gt, run_out / "rpe_1m", config,
                                     config["evaluation"]["rpe_delta_m"]["value"], "m", est_format, est_unit)
            frames = evaluator_result(run.name, trajectory, gt, run_out / "rpe_10frames", config,
                                      config["evaluation"]["rpe_delta_frames"]["value"], "frames", est_format, est_unit)
            status = ("BLOCKED" if exit_code is None else
                      ("FAIL" if exit_code != 0 or not diag["initialization_success"] else "PASS"))
            provenance_names = ("start_utc.txt", "end_utc.txt", "exit_code.txt", "throttled_before.txt",
                                "throttled_after.txt", "ldd.txt", "input.sha256", "stderr.log")
            capture = {name: {"path":str(run / name), "sha256":sha(run / name),
                              "text":(run / name).read_text(encoding="utf-8", errors="replace").strip() if name in {"start_utc.txt", "end_utc.txt", "exit_code.txt", "throttled_before.txt", "throttled_after.txt"} else None}
                       for name in provenance_names if (run / name).is_file()}
            runs.append({"run": run.name, "status": status, "trajectory": str(trajectory),
                         "trajectory_sha256": sha(trajectory), "stdout_sha256": sha(run / "stdout.log") if (run / "stdout.log").is_file() else None,
                         "trajectory_format": {"format":est_format,"time_unit":est_unit,"detection":"first non-comment timestamp magnitude; see thresholds.json"},
                         "capture_provenance":capture,
                         "exit_code": exit_code, "exit_code_evidence": exit_code_source,
                         "diagnostics": diag, "ate_rpe_1m": meter["metrics"], "ate_rpe_10frames": frames["metrics"],
                         "trajectory_evaluator_provenance": {"version": meter["tool"]["version"], "result_path": str(run_out / "rpe_1m/result.json")}})
        valid = [r for r in runs if r.get("status") != "BLOCKED"]
        vals = [r["ate_rpe_1m"]["se3"]["ate_translation_m"]["rmse"] for r in valid]
        stats = aggregate(vals)
        metric_aggregates = {"ate_rmse_se3_m": stats}
        for key, block in (("rpe_1m_translation_rmse_m", ("ate_rpe_1m", "rpe_translation_m")),
                           ("rpe_1m_rotation_rmse_deg", ("ate_rpe_1m", "rpe_rotation_deg")),
                           ("rpe_10frames_translation_rmse_m", ("ate_rpe_10frames", "rpe_translation_m")),
                           ("rpe_10frames_rotation_rmse_deg", ("ate_rpe_10frames", "rpe_rotation_deg"))):
            metric_aggregates[key] = aggregate([r[block[0]]["se3"][block[1]]["rmse"] for r in valid])
        max_ate = config["acceptance"]["ate_rmse_se3_max_m"]["value"]
        min_fraction = config["acceptance"]["min_pose_fraction"]["value"]
        max_fraction = config["acceptance"]["max_pose_fraction"]["value"]
        criteria = [{"id":"all_engine_runs_exit_zero", "status":"BLOCKED" if not valid or any(r.get("exit_code") is None for r in valid) else ("PASS" if all(r["exit_code"] == 0 for r in valid) else "FAIL"), "measured":[r.get("exit_code") for r in valid], "threshold":"every evaluable run exits 0", "threshold_source":"SOURCE-SUPPORTED: ORB-SLAM3 process exit status; nonzero exits are not accuracy evidence"},
                    {"id":"all_runs_initialized", "status":"BLOCKED" if not valid else ("PASS" if all(r["diagnostics"]["initialization_success"] for r in valid) else "FAIL"), "measured":sum(bool(r.get("diagnostics",{}).get("initialization_success")) for r in valid), "threshold":"every evaluable run", "threshold_source":"PROPOSED project acceptance; initialization log phrase detector documented in README"},
                    {"id":"se3_ate_rmse_each_run", "status":"BLOCKED" if not vals else ("PASS" if all(v <= max_ate for v in vals) else "FAIL"), "measured":vals, "unit":"m", "threshold":f"<= {max_ate}", "threshold_source":config["acceptance"]["ate_rmse_se3_max_m"]["source"]},
                    {"id":"pose_fraction_each_run", "status":pose_fraction_status(valid, min_fraction, max_fraction), "measured":[r["diagnostics"].get("pose_fraction") for r in valid], "threshold":f">= {min_fraction} and <= {max_fraction}", "threshold_source":config["acceptance"]["min_pose_fraction"]["source"] + "; " + config["acceptance"]["max_pose_fraction"]["source"]}]
        overall = overall_status(criteria, runs)
        result = {"pipeline":"euroc_accuracy", "version":VERSION, "status":overall, "evidence_label":"CALCULATED from saved run files",
                  "generated_utc":datetime.now(timezone.utc).isoformat(), "sequence":args.sequence,
                  "ground_truth":{"path":str(gt),"sha256":sha(gt),"clock_domain":"EuRoC dataset time base"},
                  "times_file":{"path":str(times),"sha256":sha(times)} if times else None,
                  "expected_frame_count_override":{"value":args.expected_frames,"evidence":"caller-supplied sequence timestamp row count"} if args.expected_frames is not None else None,
                  "clock_assumption":GT_CLOCK, "alignment":"SE3 primary; Sim3 reported as labelled secondary",
                  "acceptance":config["acceptance"], "criteria":criteria,"aggregate_metrics_se3":metric_aggregates,"aggregate_ate_rmse_se3_m":stats,"runs":runs}
        (args.out / "summary.json").write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
        rows=["# EuRoC accuracy evaluation", "", f"Status: **{overall}**", "", f"Sequence: {args.sequence}", "",
              "EuRoC benchmark accuracy is not a D435i result. Primary ATE is rigid SE3 aligned; Sim3 is secondary and can absorb scale error.", "",
              "| Run | Init | Poses | Pose fraction | ATE RMSE SE3 (m) | ATE RMSE Sim3 (m) | RPE 1 m translation RMSE (m) | RPE 10 frames translation RMSE (m) |", "|---|---:|---:|---:|---:|---:|---:|---:|"]
        for r in runs:
            if r["status"] == "BLOCKED": rows.append(f"| {r['run']} | BLOCKED | — | — | — | — | — | — |")
            else:
                m=r["ate_rpe_1m"]; f=r["ate_rpe_10frames"]
                rows.append(f"| {r['run']} | {r['diagnostics']['initialization_success']} | {r['diagnostics']['pose_count']} | {r['diagnostics']['pose_fraction']} | {m['se3']['ate_translation_m']['rmse']:.6f} | {m['sim3']['ate_translation_m']['rmse']:.6f} | {m['se3']['rpe_translation_m']['rmse']} | {f['se3']['rpe_translation_m']['rmse']} |")
        rows += ["", f"Across-run SE3 ATE RMSE [CALCULATED]: mean {stats['mean']}, population std {stats['std_population']}, min {stats['min']}, max {stats['max']} m.", "",
                 "Clock: estimate offset is explicitly zero. The dataset time-base assumption is source-supported for EuRoC files and does not map live camera clocks.",
                 "Tracking loss and map reset counts are stdout phrase counts; missing phrases do not prove no loss or resets. Pose fraction is an output row-count proxy.",
                 "This report does not prove hardware performance, calibration, D435i accuracy, or a project gate PASS. See summary.json for hashes, thresholds and evaluator provenance."]
        (args.out / "report.md").write_text("\n".join(rows)+"\n", encoding="utf-8")
        print(f"status={overall}; runs={len(runs)}; SE3 ATE RMSE mean={stats['mean']}")
        return {"PASS":0,"FAIL":1,"BLOCKED":2}[overall]
    except (OSError, ValueError, KeyError, json.JSONDecodeError, RuntimeError, ZeroDivisionError) as exc:
        print(f"evaluate.py: {exc}", file=sys.stderr)
        return 3

if __name__ == "__main__":
    raise SystemExit(main())
