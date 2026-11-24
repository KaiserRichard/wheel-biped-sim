#!/usr/bin/env python3
"""Replay a bag in a separate ROS graph and compare delivery to offline evidence."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from bag_core import analyze_database, evaluate_report, storage_files, _digest_stamps
from qual_common import (base_result, criterion, file_record, finalize_result,
                         load_config, run_command, utc, write_json, UsageParser, overall_status, command_text)
from record_check import _stop, _save_command_result


def _qos_path(config_path: Path, config: dict) -> Path:
    qos = Path(config["qos_overrides_file"])
    return qos if qos.is_absolute() else config_path.parent / qos


def _header_stamp(message) -> int | None:
    header = getattr(message, "header", None)
    if header is None or not hasattr(header, "stamp"):
        return None
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def _message_stamps(message) -> list[int]:
    if hasattr(message, "transforms"):
        return [stamp for transform in message.transforms if (stamp := _header_stamp(transform)) is not None]
    stamp = _header_stamp(message)
    return [] if stamp is None else [stamp]


def _observe_replay(bag: Path, config: dict, qos_path: Path, output_dir: Path, replay_domain: int, bag_duration_s: float | None = None) -> tuple[dict, int, str]:
    """Run rclpy subscribers and ros2 bag play; imports are intentionally lazy."""
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rosidl_runtime_py.utilities import get_message
    except ImportError as error:
        raise RuntimeError(f"ROS Python packages unavailable: {error}") from error

    topics = config["topics"]
    observations = {spec["name"]: {"count": 0, "ros_header_stamps_ns": [], "received_monotonic_ns": [], "errors": []}
                    for spec in topics}
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init(args=None, domain_id=replay_domain)
    node = rclpy.create_node("wbr_rosbag_qual_replay_observer")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    subscriptions = []
    player = None
    try:
        for spec in topics:
            try:
                msg_class = get_message(spec["type"])
                if spec.get("role") == "tf_static":
                    qos = QoSProfile(depth=config["replay"]["static_subscriber_depth"], reliability=ReliabilityPolicy.RELIABLE,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
                else:
                    qos = QoSProfile(depth=config["replay"]["subscriber_depth"], reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.VOLATILE)
                def callback(message, name=spec["name"]):
                    item = observations[name]
                    item["count"] += 1
                    item["ros_header_stamps_ns"].extend(_message_stamps(message))
                    item["received_monotonic_ns"].append(time.monotonic_ns())
                subscriptions.append(node.create_subscription(msg_class, spec["name"], callback, qos))
            except Exception as error:
                observations[spec["name"]]["errors"].append(f"subscription: {type(error).__name__}: {error}")
        if any(node.get_publishers_info_by_topic(spec["name"]) for spec in topics):
            raise RuntimeError("replay domain has existing publishers on configured topics")
        command = ["ros2", "bag", "play", str(bag.resolve()), "--rate", str(config["replay"]["playback_rate"]),
                   "--delay", str(config["replay"]["startup_delay_s"]),
                   "--qos-profile-overrides-path", str(qos_path)]
        write_json(output_dir / "player_command.json", {"argv": command, "shell": command_text(command),
                   "replay_domain": replay_domain, "clock_policy": "no /clock; preserve message header.stamp; arrival uses Python time.monotonic_ns"})
        log_path = output_dir / "player.log"
        with log_path.open("w") as log:
            try:
                player = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                           env=dict(os.environ, ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID=str(replay_domain)), start_new_session=True)
            except FileNotFoundError as error:
                raise RuntimeError(f"ros2 is unavailable: {error}") from error
            timeout = float((bag_duration_s if bag_duration_s is not None else config["record"]["duration_s"]) / config["replay"]["playback_rate"] + config["record"]["command_timeout_s"] + config["replay"]["startup_delay_s"])
            deadline = time.monotonic() + timeout
            while player.poll() is None:
                for spec in topics:
                    if any(endpoint.node_name != "rosbag2_player" for endpoint in node.get_publishers_info_by_topic(spec["name"])):
                        raise RuntimeError("unexpected publisher appeared during replay")
                executor.spin_once(timeout_sec=config["replay"]["spin_timeout_s"])
                if time.monotonic() >= deadline:
                    _stop(player, config["record"]["shutdown_grace_s"])
                    raise RuntimeError("ros2 bag play exceeded bounded timeout")
            drain_until = time.monotonic() + config["replay"]["drain_duration_s"]
            while time.monotonic() < drain_until:
                executor.spin_once(timeout_sec=config["replay"]["spin_timeout_s"])
            for spec in topics:
                if any(endpoint.node_name != "rosbag2_player" for endpoint in node.get_publishers_info_by_topic(spec["name"])):
                    observations[spec["name"]]["errors"].append("publisher other than rosbag2_player")
        observations["_command"] = command
        return observations, player.returncode, str(log_path)
    finally:
        if player is not None:
            _stop(player, config["record"]["shutdown_grace_s"])
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


def _replay_report(observations: dict, config: dict) -> dict:
    report = {"topics": {}, "evidence_label": "MEASURED", "receive_clock": "Python time.monotonic_ns (CLOCK_MONOTONIC)", "stamp_clock": "original ROS header.stamp; mapping UNKNOWN"}
    for spec in config["topics"]:
        item = observations.get(spec["name"], {})
        if not all(key in item for key in ("count", "ros_header_stamps_ns", "received_monotonic_ns", "errors")):
            report["topics"][spec["name"]] = {"missing_observation": True}
            continue
        stamps = item.get("ros_header_stamps_ns", [])
        received = item.get("received_monotonic_ns", [])
        header_rate = ((len(stamps) - 1) / ((stamps[-1] - stamps[0]) / 1e9)
                       if len(stamps) > 1 and stamps[-1] > stamps[0] else None)
        receive_rate = ((len(received) - 1) / ((received[-1] - received[0]) / 1e9)
                        if len(received) > 1 and received[-1] > received[0] else None)
        report["topics"][spec["name"]] = {
            "count": item.get("count", 0), "header_count": len(stamps),
            "header_rate_hz": header_rate, "replay_receive_rate_hz": receive_rate,
            "first_header_stamp_ns": stamps[0] if stamps else None,
            "last_header_stamp_ns": stamps[-1] if stamps else None,
            "header_stamp_sha256": _digest_stamps(stamps), "errors": item.get("errors", [])}
    return report


def _criteria(stored: dict, replay: dict, config: dict, evidence: str) -> list[dict]:
    source = config["replay"]["threshold_source"]
    code = replay.get("player_returncode")
    criteria = [criterion("replay:player", "ros2 bag play exited successfully", code, "exit_code", "0",
                           "BLOCKED" if code is None else ("PASS" if code == 0 else "FAIL"), evidence, source)]
    tolerance = config["replay"]["rate_relative_tolerance"]
    fraction_limit = config["replay"]["min_count_fraction"]
    for spec in config["topics"]:
        name = spec["name"]
        expected = stored.get("topics", {}).get(name, {})
        actual = replay.get("topics", {}).get(name, {})
        if not expected or "header_count" not in expected or not actual or actual.get("missing_observation") or actual.get("errors"):
            criteria.append(criterion(f"replay:{name}:evidence", "Stored and replay observations are available without subscriber errors",
                                      None, "", "available", "BLOCKED", evidence, source))
            continue
        expected_count, observed_count = expected.get("count"), actual.get("count")
        fraction = observed_count / expected_count if expected_count and observed_count is not None else None
        count_status = "BLOCKED" if fraction is None else ("PASS" if fraction >= fraction_limit else "FAIL")
        if expected_count == 0 and not spec.get("required", True):
            count_status = "PASS"
        criteria.append(criterion(f"replay:{name}:count", "Replay count fraction versus stored bag", fraction,
                                  "fraction", f">= {fraction_limit}", count_status, evidence, source))
        if expected.get("header_count", 0):
            expected_hash, actual_hash = expected.get("header_stamp_sha256"), actual.get("header_stamp_sha256")
            stamp_status = "BLOCKED" if not expected_hash or not actual_hash else ("PASS" if expected_hash == actual_hash else "FAIL")
            criteria.append(criterion(f"replay:{name}:stamps", "Replay preserves every header.stamp sequence (including static TF)",
                                      None if stamp_status == "BLOCKED" else stamp_status == "PASS", "exact", "true", stamp_status, evidence, source))
        if "rate_min_hz" in spec:
            # Compare durations/rates, never subtract epochs across the two clocks.
            # Playback scheduling follows bag receive timestamps, not header.stamp.
            recorded_rate = expected.get("bag_receive_rate_hz")
            replay_rate = actual.get("replay_receive_rate_hz")
            if recorded_rate and replay_rate:
                target_rate = recorded_rate * config["replay"]["playback_rate"]
                relative_error = abs(replay_rate - target_rate) / target_rate
                rate_status = "PASS" if relative_error <= tolerance else "FAIL"
            else:
                relative_error, rate_status = None, "BLOCKED"
            criteria.append(criterion(f"replay:{name}:rate", "Python CLOCK_MONOTONIC arrival rate versus recorded bag receive rate scaled by playback rate",
                                      relative_error, "relative_error", f"<= {tolerance}", rate_status, evidence, source))
    return criteria


def _graph_criteria(data: dict, config: dict, evidence: str) -> list[dict]:
    names = data.get("stdout", "").splitlines()
    forbidden = [name for name in names if "realsense" in name.lower() or name.strip() == config.get("camera_node")]
    status = "PASS" if data.get("returncode") == 0 and not forbidden else "BLOCKED"
    return [criterion("replay:quiet:" + evidence, "ROS node list contains no configured camera or RealSense node",
                      forbidden if forbidden else data.get("returncode"), "", "successful list and camera stopped", status,
                      evidence, "SOURCE-SUPPORTED: phase-4 requires live-camera shutdown before replay")]


def _blocked(output_dir: Path, result: dict, message: str, raw: list[Path], *, error_code: int | None = None) -> int:
    error_path = output_dir / "error.json"
    write_json(error_path, {"error": message})
    raw.append(error_path)
    return finalize_result(result, output_dir, [criterion("replay_check", message, None, "", "ROS graph stopped and ROS Python available",
                                                         "BLOCKED", "error.json", "UNKNOWN")],
                           [file_record(item, output_dir) for item in raw if item.is_file()], [message], error_code=error_code)


def run_replay(args: argparse.Namespace) -> int:
    output_dir = Path(args.analyze_only or args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    result = base_result("replay_check", utc())
    config_path = Path(args.config) if args.config else (output_dir / "config.json" if args.analyze_only else Path(__file__).with_name("config.json"))
    try:
        config = load_config(config_path)
    except Exception as error:
        return _blocked(output_dir, result, f"config: {type(error).__name__}: {error}", [], error_code=2 if args.analyze_only else 3)

    if args.analyze_only:
        stored_path, observed_path = output_dir / "stored_analysis.json", output_dir / "replay_observations.json"
        if not stored_path.is_file() or not observed_path.is_file():
            return _blocked(output_dir, result, "replay --analyze-only requires stored_analysis.json and replay_observations.json", [])
        criteria = []
        try:
            stored = json.loads(stored_path.read_text())
            observations = json.loads(observed_path.read_text())
            replay = _replay_report(observations, config) | {"player_returncode": observations.get("player_returncode")}
            criteria = _criteria(stored, replay, config, "replay_observations.json")
            criteria.extend(evaluate_report(stored, config, "stored_analysis.json"))
            for name in ("player.log", "player_command.json", "qos_overrides.yaml",
                         "node_list.stdout", "node_list.stderr", "replay_node_list.stdout", "replay_node_list.stderr"):
                if not (output_dir / name).is_file():
                    raise FileNotFoundError(name)
            for name in ("node_list.json", "replay_node_list.json"):
                criteria.extend(_graph_criteria(json.loads((output_dir / name).read_text()), config, name))
        except Exception as error:
            criteria.append(criterion("replay:saved_input", "Complete saved replay evidence is readable", None, "",
                                      str(error), "BLOCKED", "replay_observations.json", "UNKNOWN"))
            return finalize_result(result, output_dir, criteria,
                                   [file_record(path, output_dir) for path in (stored_path, observed_path) if path.is_file()], [str(error)])
        return finalize_result(result, output_dir, criteria,
                               [file_record(item, output_dir) for item in output_dir.iterdir() if item.is_file() and item.name != "result.json"],
                               ["ANALYZE_ONLY: re-evaluated saved replay observations; no ROS graph used."])

    if not args.bag:
        return _blocked(output_dir, result, "a bag path is required unless --analyze-only is used", [])
    if args.live_domain == args.replay_domain or not all(0 <= value <= 232 for value in (args.live_domain, args.replay_domain)):
        return _blocked(output_dir, result, "live/replay domains must differ and be valid ROS domain IDs", [], error_code=3)
    write_json(output_dir / "config.json", config)
    qos_path = _qos_path(config_path, config).resolve()
    raw: list[Path] = [output_dir / "config.json"]
    if qos_path.is_file():
        import shutil
        captured_qos = output_dir / "qos_overrides.yaml"
        if qos_path.resolve() != captured_qos.resolve():
            shutil.copy2(qos_path, captured_qos)
        qos_path = captured_qos.resolve()
        raw.append(captured_qos)
    if not qos_path.is_file():
        return _blocked(output_dir, result, f"configured QoS overrides file is missing: {qos_path}", raw)
    node_list = run_command(["ros2", "node", "list"], timeout=config["record"]["node_list_timeout_s"],
                             env=dict(os.environ, ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID=str(args.live_domain)))
    _save_command_result(output_dir, "node_list", node_list)
    raw.extend(output_dir / ("node_list." + suffix) for suffix in ("json", "stdout", "stderr"))
    if node_list.get("missing_command"):
        return _blocked(output_dir, result, "ros2 is unavailable; replay is BLOCKED", raw)
    names = node_list.get("stdout", "").splitlines()
    forbidden = [name for name in names if "realsense" in name.lower() or name.strip() == config.get("camera_node")]
    if node_list.get("returncode") != 0:
        return _blocked(output_dir, result, "ros2 node list failed", raw)
    if forbidden:
        return _blocked(output_dir, result, "live camera/realsense node is still running: " + ", ".join(forbidden), raw)
    replay_nodes = run_command(["ros2", "node", "list"], timeout=config["record"]["node_list_timeout_s"],
                               env=dict(os.environ, ROS_LOCALHOST_ONLY="1", ROS_DOMAIN_ID=str(args.replay_domain)))
    _save_command_result(output_dir, "replay_node_list", replay_nodes)
    raw.extend(output_dir / ("replay_node_list." + suffix) for suffix in ("json", "stdout", "stderr"))
    graph_criteria = _graph_criteria(node_list, config, "node_list.json") + _graph_criteria(replay_nodes, config, "replay_node_list.json")
    if overall_status(graph_criteria) != "PASS":
        return _blocked(output_dir, result, "replay domain camera shutdown cannot be established", raw)
    try:
        stored, _ = analyze_database(Path(args.bag), config)
        stored["criteria"] = evaluate_report(stored, config, "stored_analysis.json")
        write_json(output_dir / "stored_analysis.json", stored)
        raw += [output_dir / "stored_analysis.json", *storage_files(Path(args.bag))]
        if overall_status(stored["criteria"]) != "PASS":
            return finalize_result(result, output_dir, stored["criteria"],
                                   [file_record(item, output_dir) for item in raw if item.is_file()],
                                   ["Stored bag did not pass offline qualification; player was not started."])
        observations, player_returncode, player_log = _observe_replay(Path(args.bag), config, qos_path, output_dir,
                                                                      args.replay_domain, stored["duration_s"])
        observations["player_returncode"] = player_returncode
        observations["observed_utc"] = utc()
        write_json(output_dir / "replay_observations.json", observations)
        raw += [output_dir / "replay_observations.json", Path(player_log), output_dir / "player_command.json", output_dir / "config.json"]
        criteria = _criteria(stored, _replay_report(observations, config) | {"player_returncode": player_returncode}, config,
                             "replay_observations.json")
        criteria.extend(stored["criteria"])
        criteria.extend(graph_criteria)
        return finalize_result(result, output_dir, criteria,
                               [file_record(item, output_dir) for item in raw if item.is_file()],
                               ["Replay evidence is separate from offline bag evidence.",
                                "No physical camera node was allowed in the replay ROS domain."])
    except Exception as error:
        return _blocked(output_dir, result, f"replay: {type(error).__name__}: {error}", raw)


def main(argv: list[str] | None = None) -> int:
    parser = UsageParser(description=__doc__)
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--live-domain", type=int, default=42)
    parser.add_argument("--replay-domain", type=int, default=43)
    parser.add_argument("--analyze-only", type=Path)
    args = parser.parse_args(argv)
    if bool(args.analyze_only) == bool(args.output_dir):
        parser.error("provide exactly one of --output-dir or --analyze-only")
    return run_replay(args)


if __name__ == "__main__":
    raise SystemExit(main())

