"""Build a gravity-aligned local elevation map from an RGB-D rosbag2."""
from __future__ import annotations

import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path
import sys
import time

import cv2
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.local_map.fuse import (  # noqa: E402
    depth_points,
    gravity_frame_from_first_frame,
    transform_points,
)
from tools.local_map.grid import ElevationGrid  # noqa: E402

DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
INFO_TOPIC = "/camera/camera/color/camera_info"
POSE_TOPIC = "/d435_orb/pose"


def stamp_ns(msg):
    stamp = msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def decode_depth(msg):
    """Decode RealSense 16UC1 millimetre depth, respecting row stride."""
    if msg.encoding.upper() not in ("16UC1", "MONO16"):
        raise ValueError(f"depth encoding must be 16UC1 millimetres, got {msg.encoding!r}")
    h, w, step = int(msg.height), int(msg.width), int(msg.step)
    dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
    raw = np.frombuffer(memoryview(msg.data), dtype=dtype, count=(step // 2) * h)
    return raw.reshape(h, step // 2)[:, :w].astype(np.float32) * 0.001


def _pose_matrix(msg):
    """Convert the documented ORB PoseStamped T_wc into a 4x4 matrix."""
    p, q = msg.pose.position, msg.pose.orientation
    qx, qy, qz, qw = map(float, (q.x, q.y, q.z, q.w))
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("pose quaternion is invalid")
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    R = np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ])
    t = np.array([p.x, p.y, p.z], dtype=float)
    if not np.isfinite(t).all():
        raise ValueError("pose translation is invalid")
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def _decode_info(msg):
    k = np.asarray(msg.k, dtype=float).reshape(3, 3)
    if not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError("camera_info has invalid pinhole intrinsics")
    return (float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2]))


def _topics(reader):
    found = {c.topic: c.msgtype for c in reader.connections}
    required = (DEPTH_TOPIC, INFO_TOPIC, POSE_TOPIC)
    missing = [topic for topic in required if topic not in found]
    if missing:
        raise ValueError("missing required topic(s): " + ", ".join(missing))
    return {c.topic: c for c in reader.connections if c.topic in required}


def _read_poses(reader, typestore, connections):
    poses = []
    for conn, _, raw in reader.messages(connections=[connections[POSE_TOPIC]]):
        msg = typestore.deserialize_cdr(raw, conn.msgtype)
        poses.append((stamp_ns(msg), _pose_matrix(msg)))
    poses.sort(key=lambda item: item[0])
    if not poses:
        raise ValueError(f"no poses found on {POSE_TOPIC}")
    return poses


def _nearest_pose(stamp, pose_stamps, poses, max_delta_ns):
    i = bisect_left(pose_stamps, stamp)
    candidates = poses[max(0, i-1):min(len(poses), i+1)]
    if not candidates:
        return None
    delta, matrix = min(candidates, key=lambda item: abs(item[0] - stamp))
    return matrix if abs(delta-stamp) <= max_delta_ns else None


def _rotation_delta(a, b):
    relative = a[:3, :3].T @ b[:3, :3]
    return math.acos(float(np.clip((np.trace(relative)-1.0)*0.5, -1.0, 1.0)))


def _save_maps(grid, out, *, obstacle_threshold_m):
    latest = float(np.max(grid.last_seen[np.isfinite(grid.last_seen)]))
    result = grid.outputs(time_s=latest, obstacle_threshold_m=obstacle_threshold_m)
    height = result["height"]
    finite = np.isfinite(height)
    height_u8 = np.zeros(height.shape, dtype=np.uint8)
    if finite.any():
        lo, hi = float(np.nanmin(height)), float(np.nanmax(height))
        span = max(hi-lo, 1e-6)
        height_u8[finite] = np.clip((height[finite]-lo) * (254.0/span) + 1.0, 1, 255).astype(np.uint8)
    cv2.imwrite(str(out / "height_map.png"), height_u8)
    # 0 unknown=black, 1 free=white, 2 obstacle=red.
    classes = np.zeros((*result["classes"].shape, 3), dtype=np.uint8)
    classes[result["classes"] == 1] = (255, 255, 255)
    classes[result["classes"] == 2] = (0, 0, 255)
    cv2.imwrite(str(out / "obstacle_classes.png"), classes)
    return result


def build_map(bag, out, *, max_pose_delta_s=0.050, stride=4,
              size_m=4.0, resolution_m=0.05, obstacle_threshold_m=0.05,
              expected_box=None):
    bag, out = Path(bag), Path(out)
    if not bag.is_dir():
        raise ValueError("BAG must be a rosbag2 directory")
    if not (bag / "metadata.yaml").is_file():
        raise ValueError(f"bag directory has no metadata.yaml: {bag}")
    if max_pose_delta_s < 0 or stride < 1:
        raise ValueError("pose tolerance must be nonnegative and stride >= 1")
    out.mkdir(parents=True, exist_ok=True)
    typestore = get_typestore(Stores.ROS2_JAZZY)
    reader = Reader(bag)
    reader.open()
    try:
        connections = _topics(reader)
        poses = _read_poses(reader, typestore, connections)
        pose_stamps = [p[0] for p in poses]
        # Second pass: depth images are decoded and fused one at a time.
        previous_pose = None
        previous_stamp = None
        intrinsics = None
        T_map_world = origin = plane = grid = None
        counts = {"depth_frames": 0, "frames_used": 0, "frames_skipped_jump": 0,
                  "frames_no_pose": 0, "frames_without_valid_floor": 0}
        frame_ms = []
        for conn, _, raw in reader.messages(connections=[connections[DEPTH_TOPIC], connections[INFO_TOPIC]]):
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            if conn.topic == INFO_TOPIC:
                intrinsics = _decode_info(msg)
                continue
            counts["depth_frames"] += 1
            start = time.perf_counter()
            depth = decode_depth(msg)
            stamp = stamp_ns(msg)
            pose = _nearest_pose(stamp, pose_stamps, poses, int(max_pose_delta_s*1e9))
            if pose is None:
                counts["frames_no_pose"] += 1
                continue
            if intrinsics is None:
                raise ValueError("depth arrived before camera_info")
            if previous_pose is not None:
                dt = (stamp-previous_stamp) * 1e-9
                if dt <= 0:
                    counts["frames_skipped_jump"] += 1
                    continue
                speed = np.linalg.norm(pose[:3, 3]-previous_pose[:3, 3]) / dt
                angular_speed = _rotation_delta(previous_pose, pose) / dt
                if speed > 1.0 or angular_speed > math.radians(60):
                    counts["frames_skipped_jump"] += 1
                    continue
            if T_map_world is None:
                try:
                    T_map_world, origin, plane = gravity_frame_from_first_frame(
                        depth, pose, intrinsics=intrinsics, stride=stride)
                except ValueError:
                    counts["frames_without_valid_floor"] += 1
                    continue
                grid = ElevationGrid(size_m, resolution_m)
            points_cam = depth_points(depth, intrinsics=intrinsics, stride=stride)
            points_world = transform_points(pose, points_cam)
            points_map = transform_points(T_map_world, points_world)
            robot_map = transform_points(T_map_world, pose[None, :3, 3])[0]
            grid.scroll(robot_map[:2])
            grid.update(points_map, time_s=stamp*1e-9, floor_z=0.0)
            previous_pose, previous_stamp = pose, stamp
            counts["frames_used"] += 1
            frame_ms.append((time.perf_counter()-start)*1000.0)
        if grid is None:
            raise ValueError("no depth frame with a matched pose and valid floor plane")
    finally:
        reader.close()

    result = _save_maps(grid, out, obstacle_threshold_m=obstacle_threshold_m)
    obstacle = result["classes"] == 2
    obstacle_cells = np.argwhere(obstacle)
    if len(obstacle_cells):
        # Grid row decreases with map +y; cell centres are reported in map coordinates.
        rows, cols = obstacle_cells[:, 0], obstacle_cells[:, 1]
        x0, y0 = grid.center_xy
        xs = x0-grid.size_m/2+(cols+0.5)*grid.resolution_m
        ys = y0+grid.size_m/2-(rows+0.5)*grid.resolution_m
        centroid = [float(xs.mean()), float(ys.mean()), float(np.nanmean(result["height"][obstacle]))]
    else:
        centroid = None
    expected_distance = None
    if expected_box is not None and centroid is not None:
        expected = np.asarray(expected_box, dtype=float)
        expected_distance = float(np.linalg.norm(np.asarray(centroid)-expected))
    extent = {
        "x_min_m": float(grid.center_xy[0]-grid.size_m/2),
        "x_max_m": float(grid.center_xy[0]+grid.size_m/2),
        "y_min_m": float(grid.center_xy[1]-grid.size_m/2),
        "y_max_m": float(grid.center_xy[1]+grid.size_m/2),
    }
    summary = {
        **counts,
        "map_frame": "gravity_aligned_local_map",
        "map_extent_m": extent,
        "grid_resolution_m": grid.resolution_m,
        "obstacle_cell_count": int(len(obstacle_cells)),
        "obstacle_centroid_map_xyz_m": centroid,
        "expected_box_map_xyz_m": list(map(float, expected_box)) if expected_box is not None else None,
        "expected_box_centroid_distance_m": expected_distance,
        "mean_ms_per_frame": float(np.mean(frame_ms)) if frame_ms else None,
        "median_ms_per_frame": float(np.median(frame_ms)) if frame_ms else None,
        "max_ms_per_frame": float(np.max(frame_ms)) if frame_ms else None,
        "pose_tolerance_ms": max_pose_delta_s*1000.0,
        "pose_convention": "T_wc: camera pose in slam_world, per ros2/wbr_orb_node/MAP_POINTS.md",
        "floor_plane_camera_height_m": float(plane.camera_height),
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="rosbag2 directory (SQLite or MCAP)")
    parser.add_argument("--out", required=True, help="output directory")
    parser.add_argument("--expected-box", type=lambda s: tuple(map(float, s.split(","))),
                        metavar="X,Y,H", help="expected obstacle centroid in map coordinates")
    args = parser.parse_args(argv)
    if args.expected_box is not None and len(args.expected_box) != 3:
        parser.error("--expected-box requires x,y,h")
    try:
        summary = build_map(args.bag, args.out, expected_box=args.expected_box)
    except (ValueError, FileNotFoundError) as exc:
        parser.error(str(exc))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
