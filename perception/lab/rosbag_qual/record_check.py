#!/usr/bin/env python3
"""Record a bounded explicit-topic rosbag2 session from a live ROS camera."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import shutil
import math
import subprocess
import sys
import time

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from bag_core import analyze_database, evaluate_report, storage_files
from qual_common import (base_result, command_text, criterion, file_record,
                         finalize_result, load_config, run_command, utc, write_json, UsageParser)


def _qos_path(config_path: Path, config: dict) -> Path:
    configured = Path(config["qos_overrides_file"])
    if not configured.is_absolute():
        configured = config_path.parent / configured
    return configured


def _stop(process: subprocess.Popen[bytes], grace: float) -> str:
    if process.poll() is not None:
        return "already_exited"
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=grace)
        return "SIGINT"
    except ProcessLookupError:
        return "already_exited"
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return "already_exited"
        try:
            process.wait(timeout=grace)
            return "SIGTERM"
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                return "already_exited"
            process.wait(timeout=grace)
            return "SIGKILL"


def _save_command_result(output_dir: Path, name: str, data: dict) -> Path:
    path = output_dir / f"{name}.json"
    write_json(path, data)
    (output_dir / f"{name}.stdout").write_text(data.get("stdout", ""))
    (output_dir / f"{name}.stderr").write_text(data.get("stderr", ""))
    return path


def _blocked(output_dir: Path, result: dict, message: str, raw: list[Path], *, error_code: int | None = None) -> int:
    path = output_dir / "error.json"
    write_json(path, {"error": message})
    raw.append(path)
    return finalize_result(result, output_dir, [criterion("record_check", message, None, "", "ROS available and command succeeds",
                                                         "BLOCKED", "error.json", "UNKNOWN")],
                           [file_record(item, output_dir) for item in raw if item.is_file()], [message], error_code=error_code)


def _record_criteria(raw: dict, config: dict) -> list[dict]:
    source = config["record"]["threshold_source"]
    commands = raw.get("commands", {})
    stop = raw.get("stop_reason")
    elapsed = raw.get("elapsed_monotonic_s")
    duration = raw.get("duration_s")
    bounded = (stop == "SIGINT" or (stop == "self_terminated" and elapsed is not None and duration is not None
                                   and elapsed >= duration * config["record"]["min_elapsed_fraction"]))
    criteria = [criterion("record:bounded", "Recorder completed the requested CLOCK_MONOTONIC interval",
                           elapsed, "s", f"requested {duration}; orderly shutdown", "BLOCKED" if stop is None or elapsed is None or duration is None else (
                               "PASS" if bounded else "FAIL"), "record_raw.json", source)]
    for key in ("bag_info", "parameter_dump"):
        command = commands.get(key, {})
        code = command.get("returncode")
        status = "BLOCKED" if code is None else ("PASS" if code == 0 else "FAIL")
        criteria.append(criterion("record:" + key, f"ros2 {key} completed", code, "exit_code", "0", status,
                                  key + ".stdout", source))
    topic_results = commands.get("topic_info", {})
    codes = [topic_results.get(spec["name"], {}).get("returncode") for spec in config["topics"]]
    status = "FAIL" if any(code is not None and code != 0 for code in codes) else (
        "BLOCKED" if any(code is None for code in codes) else "PASS")
    criteria.append(criterion("record:topic_qos", "Every configured topic has ros2 topic info -v evidence",
                              sum(code == 0 for code in codes), "topics", str(len(codes)), status, "topic_info.json", source))
    code = raw.get("record_returncode")
    criteria.append(criterion("record:exit", "Recorder exited successfully or by SIGINT", code, "exit_code",
                              "0, -2, or 130", "BLOCKED" if code is None else (
                                  "PASS" if code in (0, -2, 130) else "FAIL"), "record_raw.json", source))
    receive_duration = raw.get("bag_receive_duration_s")
    target = duration * config["record"]["min_recorded_duration_fraction"] if duration is not None else None
    criteria.append(criterion("record:capture_window", "Bag receive-time span covers the requested recording interval",
                              receive_duration, "s", f">= {target}", "BLOCKED" if receive_duration is None or target is None else (
                                  "PASS" if receive_duration >= target else "FAIL"), "analysis.json", source))
    return criteria


def run_record(args: argparse.Namespace) -> int:
    output_dir = Path(args.analyze_only or args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    result = base_result("record_check", utc())
    try:
        config_path = Path(args.config) if args.config else (output_dir / "config.json" if args.analyze_only else Path(__file__).with_name("config.json"))
        config = load_config(config_path)
        qos_path = _qos_path(config_path, config)
    except Exception as error:
        return _blocked(output_dir, result, f"config: {type(error).__name__}: {error}", [], error_code=2 if args.analyze_only else 3)
    if args.analyze_only:
        saved = output_dir / "record_raw.json"
        if not saved.is_file():
            return _blocked(output_dir, result, "record --analyze-only requires record_raw.json", [output_dir / "config.json"])
        criteria = []
        try:
            raw = json.loads(saved.read_text())
            criteria = _record_criteria(raw, config)
            report = json.loads((output_dir / "analysis.json").read_text())
            raw["bag_receive_duration_s"] = report.get("duration_s")
            criteria = _record_criteria(raw, config) + evaluate_report(report, config)
            for name in raw.get("required_raw_files", []):
                if not (output_dir / name).is_file():
                    raise FileNotFoundError(name)
            if not raw.get("required_raw_files"):
                raise ValueError("record raw-file inventory is missing")
        except Exception as error:
            criteria.append(criterion("record:saved_input", "Complete saved recording evidence is readable", None, "",
                                      str(error), "BLOCKED", "record_raw.json", "UNKNOWN"))
            return finalize_result(result, output_dir, criteria, [file_record(saved, output_dir)], [str(error)])
        return finalize_result(result, output_dir, criteria,
                               [file_record(item, output_dir) for item in output_dir.iterdir() if item.is_file() and item.name != "result.json"],
                               ["ANALYZE_ONLY: re-evaluated saved record outputs and offline measurements."])

    duration = float(args.seconds if args.seconds is not None else config["record"]["duration_s"])
    if not math.isfinite(duration) or duration <= 0:
        return _blocked(output_dir, result, "duration must be finite and positive", [], error_code=3)
    if any((output_dir / name).exists() for name in ("record.log", "record_raw.json", "bag")):
        return _blocked(output_dir, result, "refusing to overwrite existing recording evidence", [])

    write_json(output_dir / "config.json", config)
    raw_paths: list[Path] = [output_dir / "config.json"]
    if qos_path.is_file():
        captured_qos = output_dir / "qos_overrides.yaml"
        if qos_path.resolve() != captured_qos.resolve():
            shutil.copy2(qos_path, captured_qos)
        qos_path = captured_qos.resolve()
        raw_paths.append(captured_qos)
    else:
        return _blocked(output_dir, result, f"configured QoS overrides file is missing: {qos_path}", raw_paths)
    node_list = run_command(["ros2", "node", "list"], timeout=config["record"]["node_list_timeout_s"],
                             env=dict(os.environ, ROS_LOCALHOST_ONLY="1"))
    node_path = _save_command_result(output_dir, "node_list", node_list)
    raw_paths += [node_path, output_dir / "node_list.stdout", output_dir / "node_list.stderr"]
    if node_list.get("missing_command"):
        return _blocked(output_dir, result, "ros2 is unavailable; native ROS checks are BLOCKED", raw_paths)
    if node_list["returncode"] != 0:
        return _blocked(output_dir, result, "ros2 node list failed", raw_paths)
    if config.get("camera_node") not in node_list.get("stdout", "").splitlines():
        return _blocked(output_dir, result, f"configured camera node {config.get('camera_node')} is not running", raw_paths)

    bag = output_dir / "bag"
    if bag.exists():
        return _blocked(output_dir, result, f"refusing to overwrite existing bag {bag}", raw_paths)
    topics = [item["name"] for item in config["topics"]]
    command = ["ros2", "bag", "record", "--storage", config["storage"]["storage_id"], "--output", str(bag),
               "--qos-profile-overrides-path", str(qos_path), "--topics", *topics]
    write_json(output_dir / "command.json", {"argv": command, "shell": command_text(command), "duration_s": duration,
                                               "qos_file": str(qos_path), "topics": topics, "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
                                               "clock_policy": "rosbag2 receive clock and original ROS header.stamp retained; no epoch mapping assumed"})
    raw_paths.append(output_dir / "command.json")
    record_log = (output_dir / "record.log").open("x")
    process = None
    stop_reason = "not_started"
    record_started = time.monotonic()
    try:
        try:
            process = subprocess.Popen(command, stdout=record_log, stderr=subprocess.STDOUT,
                                       env=dict(os.environ, ROS_LOCALHOST_ONLY="1"), start_new_session=True)
        except FileNotFoundError:
            return _blocked(output_dir, result, "ros2 is unavailable; recording is BLOCKED", raw_paths)
        try:
            process.wait(timeout=duration)
            stop_reason = "self_terminated"
        except subprocess.TimeoutExpired:
            stop_reason = _stop(process, config["record"]["shutdown_grace_s"])
    finally:
        if process is not None and process.poll() is None:
            _stop(process, config["record"]["shutdown_grace_s"])
        record_log.close()
    elapsed = time.monotonic() - record_started
    raw_paths.append(output_dir / "record.log")
    bag_info = run_command(["ros2", "bag", "info", str(bag)], timeout=config["record"]["bag_info_timeout_s"])
    bag_info_path = _save_command_result(output_dir, "bag_info", bag_info)
    raw_paths += [bag_info_path, output_dir / "bag_info.stdout", output_dir / "bag_info.stderr"]
    params = run_command(["ros2", "param", "dump", config["camera_node"]], timeout=config["record"]["parameter_timeout_s"])
    params_path = _save_command_result(output_dir, "parameter_dump", params)
    raw_paths += [params_path, output_dir / "parameter_dump.stdout", output_dir / "parameter_dump.stderr"]
    topic_results = {}
    for index, topic in enumerate(topics):
        topic_results[topic] = run_command(["ros2", "topic", "info", "-v", topic],
                                           timeout=config["record"]["topic_info_timeout_s"])
        topic_name = f"topic_{index:03d}"
        _save_command_result(output_dir, topic_name, topic_results[topic])
        raw_paths += [output_dir / f"{topic_name}.{suffix}" for suffix in ("json", "stdout", "stderr")]
    write_json(output_dir / "topic_info.json", topic_results)
    raw_paths.append(output_dir / "topic_info.json")
    raw_snapshot = {"commands": {"record": command, "bag_info": bag_info,
                                  "parameter_dump": params, "topic_info": topic_results},
                    "stop_reason": stop_reason, "record_returncode": process.returncode if process else None,
                    "elapsed_monotonic_s": elapsed, "duration_s": duration}
    criteria = []
    if bag.exists():
        try:
            report, bag_criteria = analyze_database(bag, config)
            raw_snapshot["bag_receive_duration_s"] = report.get("duration_s")
            report["analyzed_utc"] = utc()
            write_json(output_dir / "analysis.json", report)
            criteria.extend(bag_criteria)
            raw_paths.append(output_dir / "analysis.json")
            raw_paths.extend(storage_files(bag))
        except Exception as error:
            criteria.append(criterion("record:offline_analysis", "Recorded bag can be analyzed offline", None, "", str(error), "BLOCKED",
                                      "record.log", "UNKNOWN"))
    else:
        criteria.append(criterion("record:bag_exists", "Bounded recording created a bag", 0, "bags", ">= 1", "FAIL",
                                  "record.log", config["record"]["threshold_source"]))
    criteria = _record_criteria(raw_snapshot, config) + criteria
    raw = [file_record(item, output_dir) for item in raw_paths if item.is_file()]
    raw_snapshot["required_raw_files"] = [str(path.relative_to(output_dir)) for path in raw_paths
                                          if path.is_relative_to(output_dir)]
    write_json(output_dir / "record_raw.json", raw_snapshot)
    raw.append(file_record(output_dir / "record_raw.json", output_dir))
    return finalize_result(result, output_dir, criteria, raw,
                           ["Physical acceptance still requires review of bag metadata, logs and current Pi power/thermal evidence."])


def main(argv: list[str] | None = None) -> int:
    parser = UsageParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--seconds", type=float)
    parser.add_argument("--analyze-only", type=Path)
    args = parser.parse_args(argv)
    if bool(args.analyze_only) == bool(args.output_dir):
        parser.error("provide exactly one of --output-dir or --analyze-only")
    return run_record(args)


if __name__ == "__main__":
    raise SystemExit(main())

