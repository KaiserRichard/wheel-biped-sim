"""Shared bag reading and trajectory metrics for offline ORB-SLAM3 checks."""
from __future__ import annotations

import json
import math
from collections import Counter
from pathlib import Path
from typing import Any

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

POSE_TOPIC = "/d435_orb/pose"
STATUS_TOPIC = "/d435_orb/status"
INFO_TOPIC = "/d435_orb/map_points_info"
STATE_NAMES = {1: "NOT_INITIALIZED", 2: "OK", 3: "RECENTLY_LOST", 4: "LOST"}


def _stamp_ns(msg: Any, fallback: int) -> int:
    h = getattr(msg, "header", None)
    if h is None:
        return fallback
    value = int(h.stamp.sec) * 1_000_000_000 + int(h.stamp.nanosec)
    return value or fallback


def _status_payload(text: str) -> dict | None:
    try:
        value = json.loads(text)
    except (ValueError, TypeError):
        return None
    return value if isinstance(value, dict) else None


def classify_tracking(text: str) -> str:
    payload = _status_payload(text)
    if payload is not None:
        try:
            state = payload.get("tracking_state")
            if isinstance(state, str) and not state.lstrip("-").isdigit():
                return state.upper()
            return STATE_NAMES.get(int(state), "UNKNOWN_STATE")
        except (TypeError, ValueError):
            return "UNKNOWN_STATE"
    upper = text.upper()
    if "RECENTLY_LOST" in upper:
        return "RECENTLY_LOST"
    if '"LOST"' in upper or upper.strip() == "LOST":
        return "LOST"
    if '"OK"' in upper or upper.strip() in {"OK", "TRACKING"}:
        return "OK"
    return "OTHER"


def read_bag(path: Path) -> dict:
    typestore = get_typestore(Stores.ROS2_JAZZY)
    poses: list[dict] = []
    statuses: list[dict] = []
    infos: list[dict] = []
    with AnyReader([path], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic in {POSE_TOPIC, STATUS_TOPIC, INFO_TOPIC}]
        for conn, log_ns, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == POSE_TOPIC:
                p = msg.pose.position
                poses.append({"t_ns": _stamp_ns(msg, int(log_ns)), "log_ns": int(log_ns),
                              "x": float(p.x), "y": float(p.y), "z": float(p.z)})
            elif conn.topic == STATUS_TOPIC:
                text = str(msg.data)
                payload = _status_payload(text)
                event_ns = int(payload.get("source_ns", log_ns)) if payload else int(log_ns)
                statuses.append({"t_ns": event_ns, "state": classify_tracking(text), "raw": text,
                                 "payload": payload})
            elif conn.topic == INFO_TOPIC:
                text = str(msg.data)
                try:
                    payload = json.loads(text)
                except (ValueError, TypeError):
                    payload = {}
                if isinstance(payload, dict):
                    event_ns = int(payload.get("source_ns", log_ns))
                    infos.append({"t_ns": event_ns, "payload": payload})
    poses.sort(key=lambda p: p["t_ns"])
    statuses.sort(key=lambda p: p["t_ns"])
    infos.sort(key=lambda p: p["t_ns"])
    return {"poses": poses, "statuses": statuses, "infos": infos}


def points(poses: list[dict]) -> np.ndarray:
    if not poses:
        return np.empty((0, 3), dtype=float)
    return np.asarray([[p["x"], p["y"], p["z"]] for p in poses], dtype=float)


def trajectory_metrics(poses: list[dict], jump_threshold_mps: float = 1.0) -> dict:
    xyz = points(poses)
    if len(xyz) < 2:
        return {"path_length_m": 0.0, "pose_jumps": [], "return_distance_m": None}
    dt = np.diff([p["t_ns"] for p in poses]).astype(float) / 1e9
    dist = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    speed = np.divide(dist, dt, out=np.full_like(dist, np.inf), where=dt > 0)
    jumps = [{"from_t_ns": poses[i]["t_ns"], "to_t_ns": poses[i + 1]["t_ns"],
              "distance_m": float(dist[i]), "speed_mps": float(speed[i])}
             for i in range(len(speed)) if speed[i] > jump_threshold_mps]
    return {"path_length_m": float(dist.sum()), "pose_jumps": jumps,
            "return_distance_m": float(np.linalg.norm(xyz[-1] - xyz[0]))}


def tracking_metrics(statuses: list[dict]) -> dict:
    counts = Counter(s["state"] for s in statuses)
    n = len(statuses)
    events = []
    for i, status in enumerate(statuses):
        if status["state"] in {"LOST", "RECENTLY_LOST"}:
            events.append({"time_ns": status["t_ns"], "state": status["state"],
                           "transition": i == 0 or statuses[i - 1]["state"] != status["state"]})
    return {"frame_count": n, "tracking_ok_count": counts["OK"],
            "tracking_ok_percent": (100.0 * counts["OK"] / n) if n else None,
            "state_counts": dict(counts), "lost_count": counts["LOST"],
            "recently_lost_count": counts["RECENTLY_LOST"], "lost_events": events,
            "lost_transition_count": sum(e["transition"] for e in events)}


def map_change_metrics(infos: list[dict]) -> dict:
    changes = []
    prior: dict | None = None
    for item in infos:
        p = item["payload"]
        fields = {k: p.get(k) for k in ("map_id", "init_kf_id", "map_change", "big_change")}
        changed = False
        if prior is not None:
            changed = any(fields[k] != prior[k] for k in fields if fields[k] is not None and prior[k] is not None)
        if changed:
            changes.append({"time_ns": item["t_ns"],
                            "changed_fields": [k for k in fields if fields[k] is not None and prior[k] is not None and fields[k] != prior[k]],
                            **fields})
        prior = fields
    return {"sample_count": len(infos), "change_count": len(changes), "changes": changes,
            "available": bool(infos)}


def stationary_segments(poses: list[dict], speed_limit_mps: float = 0.02,
                        minimum_duration_s: float = 1.0) -> list[tuple[int, int]]:
    """Return inclusive pose index ranges whose every interval is below speed limit."""
    if len(poses) < 2:
        return []
    xyz = points(poses)
    t = np.asarray([p["t_ns"] for p in poses], dtype=np.int64)
    dt = np.diff(t).astype(float) / 1e9
    d = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
    speed = np.divide(d, dt, out=np.full_like(d, np.inf), where=dt > 0)
    quiet = speed < speed_limit_mps
    result = []
    i = 0
    while i < len(quiet):
        if not quiet[i]:
            i += 1
            continue
        first = i
        while i + 1 < len(quiet) and quiet[i + 1]:
            i += 1
        last = i + 1
        if (t[last] - t[first]) / 1e9 >= minimum_duration_s:
            result.append((first, last))
        i += 1
    return result


def l1_metrics(poses: list[dict], tape_m: float = 3.0, speed_limit_mps: float = 0.02,
               minimum_duration_s: float = 1.0) -> dict:
    xyz = points(poses)
    segments = stationary_segments(poses, speed_limit_mps, minimum_duration_s)
    if len(segments) < 2:
        raise ValueError("could not find stationary start and end segments (speed < 0.02 m/s for >= 1 s)")
    start_range, end_range = segments[0], segments[-1]
    if start_range[1] >= end_range[0]:
        raise ValueError("stationary start and end segments overlap")
    start = np.median(xyz[start_range[0]:start_range[1] + 1], axis=0)
    end = np.median(xyz[end_range[0]:end_range[1] + 1], axis=0)
    vector = end - start
    length = float(np.linalg.norm(vector))
    if length <= 0:
        raise ValueError("start and end medians are coincident; trajectory line is undefined")
    unit = vector / length
    rel = xyz - start
    lateral = np.linalg.norm(rel - np.outer(rel @ unit, unit), axis=1)
    metrics = trajectory_metrics(poses)
    return {"tape_distance_m": float(tape_m), "start_end_distance_m": length,
            "error_vs_tape_m": length - tape_m, "absolute_error_vs_tape_m": abs(length - tape_m),
            "path_length_m": metrics["path_length_m"], "lateral_deviation_max_m": float(lateral.max()),
            "lateral_deviation_rms_m": float(np.sqrt(np.mean(lateral ** 2))),
            "start_position_m": start.tolist(), "end_position_m": end.tolist(),
            "start_segment": {"first_pose": start_range[0], "last_pose": start_range[1],
                               "duration_s": (poses[start_range[1]]["t_ns"] - poses[start_range[0]]["t_ns"]) / 1e9},
            "end_segment": {"first_pose": end_range[0], "last_pose": end_range[1],
                             "duration_s": (poses[end_range[1]]["t_ns"] - poses[end_range[0]]["t_ns"]) / 1e9},
            "pose_jumps": metrics["pose_jumps"]}
