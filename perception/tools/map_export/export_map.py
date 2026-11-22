#!/usr/bin/env python3
"""Export ORB-SLAM3 PointCloud2 snapshots and poses from a rosbag2 directory."""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
import struct
from collections import Counter
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

MAP_TOPIC = "/d435_orb/map_points"
POSE_TOPIC = "/d435_orb/pose"
STATUS_TOPIC = "/d435_orb/status"
INFO_TOPIC = "/d435_orb/map_points_info"


def decode_pointcloud2(msg: Any) -> np.ndarray:
    """Decode float32 xyz fields from a ROS PointCloud2-like object."""
    fields = {f.name: f for f in msg.fields}
    if not all(k in fields for k in ("x", "y", "z")):
        raise ValueError("PointCloud2 must contain x, y and z fields")
    fs = [fields[k] for k in ("x", "y", "z")]
    if any(f.datatype != 7 or f.count < 1 for f in fs):  # sensor_msgs/PointField FLOAT32
        raise ValueError("xyz fields must be FLOAT32")
    n = int(msg.width) * int(msg.height)
    if n == 0:
        return np.empty((0, 3), dtype=np.float64)
    step = int(msg.point_step)
    row_step = int(msg.row_step)
    data = bytes(msg.data)
    endian = ">" if msg.is_bigendian else "<"
    xyz = np.empty((n, 3), dtype=np.float64)
    idx = 0
    for row in range(int(msg.height)):
        for col in range(int(msg.width)):
            base = row * row_step + col * step
            for axis, f in enumerate(fs):
                off = base + int(f.offset)
                if off + 4 > len(data):
                    raise ValueError("PointCloud2 data is shorter than its layout")
                xyz[idx, axis] = struct.unpack_from(endian + "f", data, off)[0]
            idx += 1
    return xyz[np.isfinite(xyz).all(axis=1)]


def voxel_downsample(points: np.ndarray, voxel_m: float) -> np.ndarray:
    if voxel_m <= 0 or len(points) == 0:
        return points.copy()
    keys = np.floor(points / voxel_m).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first)]


def _pose_ns(msg: Any, fallback: int) -> int:
    return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec) or fallback


def read_bag(path: Path) -> tuple[np.ndarray, list[dict], Counter, dict]:
    typestore = get_typestore(Stores.ROS2_JAZZY)
    latest: tuple[int, np.ndarray] | None = None
    poses: list[dict] = []
    statuses: Counter = Counter()
    info: dict = {}
    with AnyReader([path], default_typestore=typestore) as reader:
        conns = [c for c in reader.connections if c.topic in {MAP_TOPIC, POSE_TOPIC, STATUS_TOPIC, INFO_TOPIC}]
        for conn, log_ns, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic == MAP_TOPIC:
                cloud = decode_pointcloud2(msg)
                # Replacement semantics: an empty newest snapshot intentionally clears older geometry.
                latest = (int(log_ns), cloud)
            elif conn.topic == POSE_TOPIC:
                p, q = msg.pose.position, msg.pose.orientation
                poses.append({"t_ns": _pose_ns(msg, int(log_ns)), "x": float(p.x), "y": float(p.y), "z": float(p.z),
                              "qx": float(q.x), "qy": float(q.y), "qz": float(q.z), "qw": float(q.w)})
            elif conn.topic == STATUS_TOPIC:
                text = str(msg.data)
                try:
                    payload = json.loads(text)
                except (ValueError, TypeError):
                    payload = None
                statuses[classify_status(text, payload)] += 1
            elif conn.topic == INFO_TOPIC:
                try:
                    info = json.loads(msg.data)
                except (ValueError, TypeError):
                    info = {"raw": str(msg.data)}
    return (latest[1] if latest else np.empty((0, 3))), poses, statuses, info


def classify_status(text: str, payload: dict | None = None) -> str:
    if payload and "tracking_state" in payload:
        value = payload["tracking_state"]
        if isinstance(value, str) and not value.lstrip("-").isdigit():
            return value.upper()  # node.cpp publishes state names, d435_node raw ints
        try:
            # ORB_SLAM3 include/Tracking.h eTrackingState (checked against
            # python3/ORB_SLAM3 on 2026-10-23).
            return {-1: "SYSTEM_NOT_READY", 0: "NO_IMAGES_YET", 1: "NOT_INITIALIZED",
                    2: "OK", 3: "RECENTLY_LOST", 4: "LOST", 5: "OK_KLT"}.get(int(value), "UNKNOWN_STATE")
        except (TypeError, ValueError):
            return "UNKNOWN_STATE"
    s = text.upper()
    if re.search(r"\b(LOST|RECENTLY_LOST|TRACKING_LOST)\b", s):
        return "LOST"
    if re.search(r"\b(TRACKING|OK|OK_KLT)\b", s):
        return "TRACKING"
    if re.search(r"\b(INITIALIZING|NOT_INITIALIZED|STARTING)\b", s):
        return "INITIALIZING"
    if re.search(r"\b(ERROR|FAULT|FAIL)\b", s):
        return "ERROR"
    return "OTHER"


def write_ply(path: Path, points: np.ndarray) -> None:
    with path.open("w", encoding="ascii") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
        for x, y, z in points:
            f.write(f"{x:.9g} {y:.9g} {z:.9g}\n")


def read_ply(path: Path) -> np.ndarray:
    with path.open(encoding="ascii") as f:
        while f.readline().strip() != "end_header":
            pass
        rows = [[float(v) for v in line.split()[:3]] for line in f if line.strip()]
    return np.asarray(rows, dtype=float).reshape((-1, 3))


def _scale_bar(ax, points: np.ndarray, axis: str) -> None:
    if len(points) == 0:
        return
    coords = points[:, 0] if axis == "x" else points[:, 2]
    span = float(np.ptp(coords))
    target = span / 4 if span > 0 else 0.1
    power = 10 ** math.floor(math.log10(target))
    length = max(v * power for v in (1, 2, 5) if v * power <= target)
    x0, x1 = ax.get_xlim(); y0, y1 = ax.get_ylim()
    bx = x0 + .06 * (x1 - x0); by = y0 + .07 * (y1 - y0)
    ax.plot([bx, bx + length], [by, by], color="black", linewidth=3)
    ax.text(bx + length / 2, by, f"{length:g} m", ha="center", va="bottom", fontsize=8)


def plot_views(out: Path, points: np.ndarray, poses: list[dict]) -> None:
    traj = np.array([[p["x"], p["y"], p["z"]] for p in poses], dtype=float).reshape((-1, 3))
    # In ORB stereo slam_world, x is optical right, y optical down, z forward.
    for name, horizontal, vertical, xlabel, ylabel, invert in [
        ("topdown.png", 0, 2, "slam_world x (m, optical right)", "slam_world z (m, optical forward)", False),
        ("side.png", 2, 1, "slam_world z (m, optical forward)", "slam_world y (m, optical down)", True),
    ]:
        fig, ax = plt.subplots(figsize=(8, 6), dpi=180)
        if len(points):
            sc = ax.scatter(points[:, horizontal], points[:, vertical], c=points[:, 1], s=3, cmap="viridis", alpha=.78, linewidths=0)
            fig.colorbar(sc, ax=ax, label="slam_world y (m, optical down)", shrink=.78)
        if len(traj):
            ax.plot(traj[:, horizontal], traj[:, vertical], color="#d62728", linewidth=1.2, label="camera trajectory")
            ax.scatter(traj[0, horizontal], traj[0, vertical], marker="o", s=24, color="limegreen", edgecolor="black", label="start")
            ax.scatter(traj[-1, horizontal], traj[-1, vertical], marker="x", s=28, color="red", label="end")
            ax.legend(loc="best", fontsize=8)
        ax.set_xlabel(xlabel); ax.set_ylabel(ylabel)
        ax.set_title("ORB-SLAM3 sparse map and camera trajectory")
        ax.grid(True, alpha=.22); ax.set_aspect("equal", adjustable="datalim")
        if invert:
            ax.invert_yaxis()
        _scale_bar(ax, points, "x" if horizontal == 0 else "z")
        fig.tight_layout(); fig.savefig(out / name); plt.close(fig)


def export(bag: Path, out: Path, voxel_m: float) -> dict:
    out.mkdir(parents=True, exist_ok=True)
    raw, poses, statuses, info = read_bag(bag)
    points = voxel_downsample(raw, voxel_m)
    write_ply(out / "map.ply", points)
    with (out / "trajectory.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["t_ns", "x", "y", "z", "qx", "qy", "qz", "qw"])
        writer.writeheader(); writer.writerows(poses)
    plot_views(out, points, poses)
    if len(points):
        mins, maxs = points.min(axis=0), points.max(axis=0)
        extent = (maxs - mins).tolist()
        volume = float(np.prod(maxs - mins))
        density = float(len(points) / volume) if volume > 0 else None
    else:
        mins = maxs = np.full(3, np.nan); extent = [None, None, None]; volume = 0.0; density = None
    summary = {"input_bag": str(bag), "snapshot_policy": "latest PointCloud2 replacement snapshot; empty latest snapshot clears previous geometry",
               "snapshot_info": info, "frame": "slam_world", "axes": {"x": "optical right", "y": "optical down", "z": "optical forward"},
               "raw_snapshot_points": int(len(raw)), "exported_points": int(len(points)), "voxel_size_m": float(voxel_m),
               "extent_m": {"x": extent[0], "y": extent[1], "z": extent[2]},
               "bounds_m": {"min": mins.tolist() if len(points) else None, "max": maxs.tolist() if len(points) else None},
               "bounding_box_volume_m3": volume if len(points) else None, "point_density_points_per_m3": density,
               "pose_count": len(poses), "status_counts": dict(statuses)}
    (out / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return summary


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("bag", type=Path, help="rosbag2 directory containing MCAP or db3 storage")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--voxel", type=float, default=0.02, help="voxel width in metres; 0 disables downsampling (default: 0.02)")
    a = p.parse_args()
    if a.voxel < 0:
        p.error("--voxel must be >= 0")
    summary = export(a.bag, a.output, a.voxel)
    print(json.dumps({k: summary[k] for k in ("raw_snapshot_points", "exported_points", "pose_count", "status_counts")}, indent=2))

if __name__ == "__main__":
    main()
