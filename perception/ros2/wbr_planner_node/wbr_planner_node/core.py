"""NumPy-only live lateral path suggestion core; ROS-independent."""
from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np

import os

# After `colcon build` this file lives under install/, so the repo root comes
# from WBR_REPO_ROOT (set by scripts/camera_day/path_live.sh); the fallback
# works when run from the source tree.
ROOT = Path(os.environ.get("WBR_REPO_ROOT") or Path(__file__).resolve().parents[3])
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.local_map.fuse import depth_points, gravity_frame_from_first_frame, transform_points
from tools.local_map.grid import ElevationGrid


class PlannerCore:
    def __init__(self, *, goal_forward_m=3.0, robot_half_width_m=0.25,
                 grid_size_m=6.0, resolution_m=0.05, stride=4,
                 pose_tolerance_ns=50_000_000, obstacle_height_m=0.03):
        self.goal_forward_m = float(goal_forward_m)
        self.robot_half_width_m = float(robot_half_width_m)
        self.stride = int(stride)
        self.pose_tolerance_ns = int(pose_tolerance_ns)
        self.obstacle_height_m = float(obstacle_height_m)
        self.grid_size_m = float(grid_size_m)
        self.resolution_m = float(resolution_m)
        self.grid = None
        self.T_map_world = None
        self.previous_pose = None
        self.previous_stamp_ns = None
        self.start_position = None
        self.start_stamp_ns = None
        self.discontinuity = False
        self.recovery_pose = None
        self.recovery_stamp_ns = None
        self.recovery_count = 0

    def update(self, depth_m, K, T_wc, stamp_ns):
        """Fuse one depth frame and return a JSON-safe advisory dictionary.

        T_wc is a 4x4 camera-to-SLAM-world pose, or None when no pose is within
        the configured source-stamp tolerance.
        """
        started = time.perf_counter()
        timings = {}
        stamp_ns = int(stamp_ns)
        depth = np.asarray(depth_m, dtype=np.float32)
        if T_wc is None:
            return self._result("NO_POSE", stamp_ns, started, [], None, timings)
        intrinsics = self._intrinsics(K)
        pose = np.asarray(T_wc, dtype=float)
        if pose.shape != (4, 4) or not np.isfinite(pose).all():
            return self._result("NO_POSE", stamp_ns, started, [], None)
        if depth.ndim != 2 or depth.size == 0:
            raise ValueError("depth must be a nonempty 2D array")
        if self.previous_pose is not None and not self.discontinuity:
            dt = (stamp_ns - self.previous_stamp_ns) * 1e-9
            if dt <= 0 or np.linalg.norm(pose[:3, 3] - self.previous_pose[:3, 3]) / dt > 1.0 or self._rotation_delta(self.previous_pose, pose) / dt > math.radians(60):
                self.discontinuity = True
                self.recovery_pose = pose.copy()
                self.recovery_stamp_ns = stamp_ns
                self.recovery_count = 1
                return self._result("NO_POSE", stamp_ns, started, [], None, timings)
        if self.discontinuity:
            dt = (stamp_ns - self.recovery_stamp_ns) * 1e-9
            valid = (dt > 0 and
                     np.linalg.norm(pose[:3, 3] - self.recovery_pose[:3, 3]) / dt <= 1.0 and
                     self._rotation_delta(self.recovery_pose, pose) / dt <= math.radians(60))
            self.recovery_count = self.recovery_count + 1 if valid else 1
            self.recovery_pose = pose.copy()
            self.recovery_stamp_ns = stamp_ns
            if self.recovery_count < 3:
                return self._result("NO_POSE", stamp_ns, started, [], None, timings)
            # Three consecutive locally consistent poses establish a new map epoch.
            self.grid = None
            self.T_map_world = None
            self.previous_pose = None
            self.previous_stamp_ns = None
            self.start_position = None
            self.start_stamp_ns = None
            self.discontinuity = False
            self.recovery_pose = None
            self.recovery_stamp_ns = None
            self.recovery_count = 0

        if self.T_map_world is None:
            try:
                T_right_forward_up, _, _ = gravity_frame_from_first_frame(
                    depth, pose, intrinsics=intrinsics, stride=self.stride,
                    min_inliers=60)
            except ValueError:
                return self._result("NO_PLANE", stamp_ns, started, [], None, timings)
            # Existing helper returns [camera-right, forward, up]. The path
            # convention is [forward, left, up].
            rotate = np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])
            self.T_map_world = np.eye(4)
            self.T_map_world[:3, :3] = rotate @ T_right_forward_up[:3, :3]
            self.T_map_world[:3, 3] = rotate @ T_right_forward_up[:3, 3]
            self.grid = ElevationGrid(self.grid_size_m, self.resolution_m)
            self.start_position = self._camera_map_position(pose)
            self.start_stamp_ns = stamp_ns

        fuse_started = time.perf_counter()
        points_camera = depth_points(depth, intrinsics=intrinsics, stride=self.stride)
        points_world = transform_points(pose, points_camera)
        points_map = transform_points(self.T_map_world, points_world)
        camera_map = self._camera_map_position(pose)
        self.grid.scroll(camera_map[:2])
        self.grid.update(points_map, time_s=stamp_ns * 1e-9, floor_z=0.0)
        self.previous_pose = pose.copy()
        self.previous_stamp_ns = stamp_ns
        timings["fusion"] = float((time.perf_counter()-fuse_started)*1000.)
        outputs = self.grid.outputs(time_s=stamp_ns * 1e-9,
                                    obstacle_threshold_m=self.obstacle_height_m)
        obstacle_rc = np.argwhere(outputs["obstacle"])
        if len(obstacle_rc):
            xy = [self._cell_center(int(r), int(c), camera_map[:2]) for r, c in obstacle_rc]
            xy.sort(key=lambda cell: (cell["x"]-float(camera_map[0]))**2 + (cell["y"]-float(camera_map[1]))**2)
            obstacle_cells = xy[:100]
        else:
            obstacle_cells = []
        plan_started = time.perf_counter()
        advice = self._choose(camera_map, outputs)
        timings["planning"] = float((time.perf_counter()-plan_started)*1000.)
        return self._result(advice["action"], stamp_ns, started,
                            obstacle_cells, advice, timings)

    @staticmethod
    def _intrinsics(K):
        a = np.asarray(K, dtype=float)
        if a.shape == (3, 3):
            vals = (a[0, 0], a[1, 1], a[0, 2], a[1, 2])
        elif a.size == 4:
            vals = a.reshape(-1)
        else:
            raise ValueError("K must be a 3x3 matrix or (fx, fy, cx, cy)")
        if not np.isfinite(vals).all() or vals[0] <= 0 or vals[1] <= 0:
            raise ValueError("invalid pinhole intrinsics")
        return tuple(float(v) for v in vals)

    @staticmethod
    def _rotation_delta(a, b):
        rel = a[:3, :3].T @ b[:3, :3]
        return math.acos(float(np.clip((np.trace(rel) - 1.) * .5, -1., 1.)))

    def _camera_map_position(self, pose):
        return transform_points(self.T_map_world, pose[None, :3, 3])[0]

    def _cell_center(self, row, col, camera_xy):
        x = self.grid.center_xy[0] - self.grid.size_m/2 + (col + .5)*self.grid.resolution_m
        y = self.grid.center_xy[1] + self.grid.size_m/2 - (row + .5)*self.grid.resolution_m
        return {"x": float(x), "y": float(y)}

    def _choose(self, camera, outputs):
        current_x, current_y = float(camera[0]), float(camera[1])
        if current_x >= self.goal_forward_m:
            return {"action": "GOAL_REACHED", "target_offset_m": current_y,
                    "current_lateral_m": current_y, "shift_needed_m": 0.0,
                    "distance_to_goal_m": 0.0, "unknown_fraction": 0.0,
                    "cost": 0.0, "goal_x_m": self.goal_forward_m}
        x0 = max(0.0, current_x)
        x1 = self.goal_forward_m
        candidates = np.round(np.arange(-0.6, 0.6001, 0.05), 2)
        best = None
        grid_x0 = self.grid.center_xy[0] - self.grid.size_m/2
        grid_y0 = self.grid.center_xy[1] - self.grid.size_m/2
        grid_x1 = grid_x0 + self.grid.size_m
        grid_y1 = grid_y0 + self.grid.size_m
        # Model the lateral transition close to the robot, then check the
        # target corridor for the remaining route. This allows an early shift
        # around an obstacle farther ahead while still checking the crossing.
        transition_end = min(x1, x0 + max(2*self.robot_half_width_m, self.resolution_m))
        for target_y in candidates:
            xlo = max(x0, grid_x0)
            xhi = min(x1, grid_x1)
            lane_xlo = max(transition_end, xlo)
            lane_xhi = xhi
            # Check the lateral transition footprint at the near-field segment.
            sweep_ylo = min(target_y, current_y) - self.robot_half_width_m
            sweep_yhi = max(target_y, current_y) + self.robot_half_width_m
            ylo = target_y - self.robot_half_width_m
            yhi = target_y + self.robot_half_width_m
            # Grid columns are map x; grid rows are reversed map y.
            sweep_col0 = max(0, int(math.floor((xlo-grid_x0)/self.grid.resolution_m)))
            sweep_col1 = min(self.grid.n, int(math.ceil((min(transition_end, xhi)-grid_x0)/self.grid.resolution_m)))
            lane_col0 = max(0, int(math.floor((lane_xlo-grid_x0)/self.grid.resolution_m)))
            lane_col1 = min(self.grid.n, int(math.ceil((lane_xhi-grid_x0)/self.grid.resolution_m)))
            sweep_row0 = max(0, int(math.floor((grid_y1-sweep_yhi)/self.grid.resolution_m)))
            sweep_row1 = min(self.grid.n, int(math.ceil((grid_y1-sweep_ylo)/self.grid.resolution_m)))
            row0 = max(0, int(math.floor((grid_y1-yhi)/self.grid.resolution_m)))
            row1 = min(self.grid.n, int(math.ceil((grid_y1-ylo)/self.grid.resolution_m)))
            # Occupancy grid arrays are [row(y), col(x)].
            sweep_obs = outputs["obstacle"][sweep_row0:sweep_row1, sweep_col0:sweep_col1]
            lane_obs = outputs["obstacle"][row0:row1, lane_col0:lane_col1]
            region_known = outputs["known"][row0:row1, lane_col0:lane_col1]
            sweep_valid = transition_end > xlo and sweep_row1 > sweep_row0 and sweep_col1 > sweep_col0
            lane_valid = xhi > lane_xlo and row1 > row0 and lane_col1 > lane_col0
            valid_region = sweep_valid or lane_valid
            blocked = (not valid_region) or bool(sweep_obs.any()) or bool(lane_obs.any())
            unknown_fraction = 1.0-float(region_known.mean()) if lane_valid else 1.0
            cost = (1e6 if blocked else 0.) + abs(float(target_y)) + .5*unknown_fraction
            item = (cost, float(target_y), blocked, unknown_fraction)
            if best is None or item[0] < best[0]:
                best = item
        cost, target_y, blocked, unknown = best
        dist = max(0.0, self.goal_forward_m - current_x)
        if blocked:
            action = "BLOCKED"
        elif abs(target_y - current_y) <= .05:
            action = "GO_STRAIGHT"
        elif target_y > current_y:
            action = "SHIFT_LEFT"
        else:
            action = "SHIFT_RIGHT"
        return {"action": action, "target_offset_m": target_y,
                "current_lateral_m": current_y,
                "shift_needed_m": target_y-current_y,
                "distance_to_goal_m": dist, "unknown_fraction": unknown,
                "cost": cost, "goal_x_m": self.goal_forward_m}

    def _result(self, action, stamp_ns, started, obstacles, advice, timings=None):
        data = {"action": action, "target_offset_m": None,
                "current_lateral_m": None, "shift_needed_m": None,
                "distance_to_goal_m": None, "nearest_obstacle_cells": obstacles,
                "stamp_ns": int(stamp_ns), "timings_ms": {}}
        if advice is not None:
            for key in ("target_offset_m", "current_lateral_m", "shift_needed_m", "distance_to_goal_m", "unknown_fraction", "cost", "goal_x_m"):
                data[key] = float(advice[key])
        data["timings_ms"].update(timings or {})
        data["timings_ms"]["total"] = float((time.perf_counter()-started)*1000.)
        return data

    def occupancy(self, stamp_ns=0):
        """Return map metadata and ROS-ready int8 occupancy values."""
        if self.grid is None:
            return None
        outputs = self.grid.outputs(time_s=int(stamp_ns)*1e-9,
                                    obstacle_threshold_m=self.obstacle_height_m)
        # Nav occupancy: unknown=-1, free=0, obstacle=100.
        cells = np.full(outputs["classes"].shape, -1, dtype=np.int8)
        cells[outputs["known"]] = 0
        cells[outputs["obstacle"]] = 100
        return {"resolution": float(self.grid.resolution_m),
                "width": int(self.grid.n), "height": int(self.grid.n),
                "origin_x": float(self.grid.center_xy[0]-self.grid.size_m/2),
                "origin_y": float(self.grid.center_xy[1]-self.grid.size_m/2),
                "data": cells[::-1, :].reshape(-1).tolist()}
