"""Stream RGB-D messages from ROS 2 MCAP or rosbag2 recordings."""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
from collections import Counter, deque

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[2]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

from tools.obstacle_decision.detect import Params, ground_coords, process_frame
from tools.obstacle_decision.decide import DecisionParams, update

DEPTH_TOPICS = (
    "/camera/camera/aligned_depth_to_color/image_raw",
    "/camera/camera/depth/image_rect_raw",
)
INFO_TOPICS = (
    "/camera/camera/color/camera_info",
    "/camera/camera/depth/camera_info",
)


def _stamp_ns(msg):
    stamp = msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _message_type(msg):
    return getattr(msg, "encoding", "")


def decode_depth(msg):
    """Decode a ROS Image into metres, honoring row step and common depth encodings."""
    encoding = _message_type(msg).upper()
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    raw = memoryview(msg.data)
    if encoding in ("16UC1", "MONO16"):
        dtype = np.dtype("<u2" if getattr(msg, "is_bigendian", 0) == 0 else ">u2")
        row = np.frombuffer(raw, dtype=dtype, count=(step // 2) * h).reshape(h, step // 2)[:, :w]
        return row.astype(np.float32) * 0.001
    if encoding == "32FC1":
        dtype = np.dtype("<f4" if getattr(msg, "is_bigendian", 0) == 0 else ">f4")
        row = np.frombuffer(raw, dtype=dtype, count=(step // 4) * h).reshape(h, step // 4)[:, :w]
        return row.astype(np.float32)
    raise ValueError(f"unsupported depth image encoding {encoding!r}; expected 16UC1 or 32FC1")


def _intrinsics(msg):
    k = np.asarray(msg.k, dtype=float).reshape(3, 3)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError("camera_info has invalid pinhole intrinsics")
    return k


def _quantiles(vals):
    a = np.asarray(vals, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return {"median": None, "iqr": None}
    q1, med, q3 = np.percentile(a, [25, 50, 75])
    return {"median": float(med), "iqr": float(q3 - q1)}


def _reader_paths(path):
    """Return paths accepted by AnyReader for a bag directory or one bag file."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"bag not found: {path}")
    if path.is_dir():
        if not (path / "metadata.yaml").is_file():
            raise ValueError(f"bag directory has no metadata.yaml: {path}")
        files = sorted((*path.glob("*.db3"), *path.glob("*.mcap")))
        if not files:
            raise ValueError(f"bag directory contains no .db3 or .mcap files: {path}")
        return files
    if path.suffix.lower() not in (".mcap", ".db3"):
        raise ValueError(f"unsupported bag file {path}; expected .mcap, .db3, or a rosbag2 directory")
    return [path]


def _resolve_topics(reader, path, depth_topic=None, info_topic=None):
    found = {}
    for connection in reader.connections:
        found[connection.topic] = connection.msgtype
    depths = [t for t in DEPTH_TOPICS if t in found]
    infos = [t for t in INFO_TOPICS if t in found]
    dt = depth_topic or (depths[0] if depths else None)
    it = info_topic or ("/camera/camera/color/camera_info" if dt and "aligned_depth_to_color" in dt and "/camera/camera/color/camera_info" in found else None)
    if not it:
        it = infos[0] if infos else None
    if not dt or not it or dt not in found or it not in found:
        available = ", ".join(f"{t} ({ty})" for t, ty in sorted(found.items())) or "none"
        missing = "depth topic" if not dt or dt not in found else "camera_info topic"
        raise ValueError(f"missing {missing} in {path}; available topics: {available}")
    if not found[dt].endswith("/Image") or not found[it].endswith("/CameraInfo"):
        raise ValueError(f"topic type mismatch: {dt} is {found[dt]}, {it} is {found[it]}")
    return dt, it


def replay_bag(bag, *, depth_topic=None, info_topic=None, stride=1, out=".", wall_roi_size=80):
    from rosbags.highlevel import AnyReader
    from rosbags.rosbag2 import Reader as Rosbag2Reader
    from rosbags.typesys import Stores, get_typestore
    bag = Path(bag)
    if stride < 1:
        raise ValueError("stride must be >= 1")
    if wall_roi_size < 1:
        raise ValueError("wall_roi_size must be >= 1")
    paths = _reader_paths(bag)
    typestore = get_typestore(Stores.ROS2_JAZZY)
    # A rosbag2 directory is read with the low-level reader, which does not
    # parse every stored message definition: realsense2_camera_msgs/Metadata
    # ships a definition with CRLF line endings that AnyReader rejects.
    # Image/CameraInfo are decoded with the Jazzy typestore either way.
    reader = Rosbag2Reader(bag) if bag.is_dir() else AnyReader(paths, default_typestore=typestore)
    reader.open()
    try:
        dt, it = _resolve_topics(reader, bag, depth_topic, info_topic)
    except Exception:
        reader.close()
        raise
    selected = [c for c in reader.connections if c.topic in (dt, it)]
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    stem = bag.stem
    csv_path, json_path = out / f"{stem}_frames.csv", out / f"{stem}_summary.json"
    params = Params()
    decision_params = DecisionParams()
    plane_state, decision_state = {}, {}
    counts, depths_valid = Counter(), []
    plane_h, plane_pitch, obs_d, obs_h, obs_w, wall_depths, wall_fwd = [], [], [], [], [], [], []
    obstacle_frames = 0
    pending = deque()
    latest_k = None
    written = 0
    depth_seen = 0

    fields = ["stamp", "plane_valid", "plane_height_m", "plane_pitch_deg", "plane_roll_deg",
              "nearest_obstacle_distance_m", "nearest_obstacle_height_m", "nearest_obstacle_width_m",
              "advisory", "ms", "depth_valid_fraction", "any_corridor_obstacle", "central_roi_depth_m", "wall_forward_m"]

    def process_depth(msg, k, writer):
        nonlocal written, plane_state, decision_state, obstacle_frames
        depth = decode_depth(msg)
        valid = np.isfinite(depth) & (depth > 0)
        frac = float(valid.mean()) if depth.size else 0.0
        depths_valid.append(frac)
        fr, plane_state = process_frame(depth, k, params, plane_state)
        decision, decision_state = update(fr, decision_state, decision_params)
        counts[decision.advisory.name] += 1
        if fr.nearest is not None:
            obstacle_frames += 1
        if fr.plane_valid:
            plane_h.append(fr.plane_height_m)
            plane_pitch.append(float(np.rad2deg(fr.plane_pitch_rad)))
        if fr.nearest is not None:
            obs_d.append(fr.nearest.distance_m); obs_h.append(fr.nearest.height_m); obs_w.append(fr.nearest.width_m)
        y0 = max(0, depth.shape[0] // 2 - wall_roi_size // 2)
        x0 = max(0, depth.shape[1] // 2 - wall_roi_size // 2)
        roi = depth[y0:min(depth.shape[0], y0 + wall_roi_size), x0:min(depth.shape[1], x0 + wall_roi_size)]
        roi = roi[np.isfinite(roi) & (roi > 0)]
        roi_med = float(np.median(roi)) if roi.size else math.nan
        if math.isfinite(roi_med):
            wall_depths.append(roi_med)
        wall_m = _wall_forward(depth, k, plane_state.get("plane"), params)
        if math.isfinite(wall_m):
            wall_fwd.append(wall_m)
        writer.writerow({
            "stamp": f"{_stamp_ns(msg) / 1e9:.9f}", "plane_valid": int(fr.plane_valid),
            "plane_height_m": _finite(fr.plane_height_m), "plane_pitch_deg": _finite(np.rad2deg(fr.plane_pitch_rad)),
            "plane_roll_deg": _finite(np.rad2deg(fr.plane_roll_rad)),
            "nearest_obstacle_distance_m": _finite(fr.nearest.distance_m if fr.nearest else math.nan),
            "nearest_obstacle_height_m": _finite(fr.nearest.height_m if fr.nearest else math.nan),
            "nearest_obstacle_width_m": _finite(fr.nearest.width_m if fr.nearest else math.nan),
            "advisory": decision.advisory.name, "ms": f"{fr.timing_ms:.6f}", "depth_valid_fraction": f"{frac:.8f}",
            "any_corridor_obstacle": int(fr.nearest is not None), "central_roi_depth_m": _finite(roi_med),
            "wall_forward_m": _finite(wall_m),
        })
        written += 1

    try:
        with csv_path.open("w", newline="") as fcsv:
            writer = csv.DictWriter(fcsv, fieldnames=fields); writer.writeheader()
            for connection, _, rawdata in reader.messages(connections=selected):
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
                if connection.topic == it:
                    latest_k = _intrinsics(msg)
                    # CameraInfo may precede images; apply its static calibration to queued frames.
                    while pending:
                        process_depth(pending.popleft(), latest_k, writer)
                elif connection.topic == dt:
                    depth_seen += 1
                    if (depth_seen - 1) % stride:
                        continue
                    if latest_k is None:
                        pending.append(msg)
                    else:
                        process_depth(msg, latest_k, writer)
        if pending:
            raise ValueError(f"no {it} message available to decode {len(pending)} depth frame(s)")
    finally:
        reader.close()
    summary = {
        "bag_path": str(bag), "depth_topic": dt, "info_topic": it,
        "frame_count": written, "plane_valid_rate": float(len(plane_h) / written) if written else 0.0,
        "static_segment": "all replayed frames (no motion/static metadata in bag)",
        "camera_height_m": _quantiles(plane_h), "plane_pitch_deg": _quantiles(plane_pitch),
        "plane_height_std_m": float(np.std(plane_h)) if plane_h else None,
        "plane_pitch_std_deg": float(np.std(plane_pitch)) if plane_pitch else None,
        "false_detection_rate": float(obstacle_frames / written) if written else None,
        "central_roi_depth_m": _quantiles(wall_depths), "wall_roi_size_px": wall_roi_size,
        "wall_forward_m": _quantiles(wall_fwd),
        "obstacle_distance_m": _quantiles(obs_d),
        "obstacle_height_m": _quantiles(obs_h), "obstacle_width_m": _quantiles(obs_w),
        "advisory_histogram": dict(sorted(counts.items())),
        "depth_valid_fraction": float(np.mean(depths_valid)) if depths_valid else 0.0,
        "frames_csv": str(csv_path),
    }
    json_path.write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def _wall_forward(depth, k, plane, params):
    """Horizontal distance to the dominant vertical surface ahead.

    Mode (1 cm bins) of the ground-frame forward coordinate of corridor points
    10 cm or more above the plane: a wall contributes many points at one
    forward distance, while a thin pole in front contributes few. The central
    ROI depth is axial range along a pitched optical axis and can land on the
    floor, so it is not a horizontal wall distance.
    """
    if plane is None:
        return math.nan
    _, _, h, fwd, lat = ground_coords(depth, k, plane, max(1, params.stride))
    m = np.isfinite(h) & (h >= 0.10) & (np.abs(lat) <= params.corridor_half_width) & (fwd > params.min_forward)
    if m.sum() < 50:
        return math.nan
    hist, edges = np.histogram(fwd[m], bins=np.arange(params.min_forward, params.max_forward + 0.01, 0.01))
    i = int(np.argmax(hist))
    sel = fwd[m][(fwd[m] >= edges[i] - 0.01) & (fwd[m] < edges[i + 1] + 0.01)]
    return float(np.median(sel))


def _finite(x):
    return float(x) if math.isfinite(float(x)) else ""


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("bag"); p.add_argument("--depth-topic"); p.add_argument("--info-topic")
    p.add_argument("--stride", type=int, default=1); p.add_argument("--out", default=".")
    p.add_argument("--wall-roi-size", type=int, default=80)
    a = p.parse_args(argv)
    try:
        result = replay_bag(a.bag, depth_topic=a.depth_topic, info_topic=a.info_topic, stride=a.stride,
                            out=a.out, wall_roi_size=a.wall_roi_size)
    except (ValueError, FileNotFoundError) as exc:
        p.error(str(exc))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
