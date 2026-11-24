"""Depth/pose fusion into a gravity-aligned robot-centric elevation grid."""
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from tools.ground_plane.plane import backproject, estimate_plane
try:
    from .grid import ElevationGrid, StereoNoise
except ImportError:
    from grid import ElevationGrid, StereoNoise


def gravity_frame_from_first_frame(depth, T_world_camera, *, intrinsics,
                                   stride=4, min_inliers=60):
    """Return T_map_world and the map-frame origin using the first valid floor.

    The fitted camera-facing floor normal, transformed through the first pose,
    defines map +z. Camera right projected onto the floor defines +x; +y=+z×+x.
    The map origin is the projection of the first camera centre onto that floor.
    """
    fx, fy, cx, cy = intrinsics
    plane = estimate_plane(depth, fx=fx, fy=fy, cx=cx, cy=cy, stride=stride,
                           min_inliers=min_inliers, seed=17)
    if not plane.valid:
        raise ValueError("could not estimate first-frame floor plane")
    pose = np.asarray(T_world_camera, dtype=float)
    R, t = pose[:3, :3], pose[:3, 3]
    up = R @ plane.normal; up /= np.linalg.norm(up)
    right = R[:, 0] - up * np.dot(up, R[:, 0]); right /= np.linalg.norm(right)
    forward = np.cross(up, right); forward /= np.linalg.norm(forward)
    R_map_world = np.vstack((right, forward, up))
    origin_world = t - up * plane.camera_height
    T_map_world = np.eye(4); T_map_world[:3, :3] = R_map_world
    T_map_world[:3, 3] = -R_map_world @ origin_world
    return T_map_world, origin_world, plane


def depth_points(depth, *, intrinsics, stride=4):
    fx, fy, cx, cy = intrinsics
    pts, _ = backproject(depth, fx=fx, fy=fy, cx=cx, cy=cy, stride=stride)
    return pts


def transform_points(T, points):
    return np.asarray(points) @ T[:3, :3].T + T[:3, 3]


def fuse_sequence(frames, poses, *, intrinsics, stride=4, size_m=4.0,
                  resolution_m=0.05, noise_model=None, pose_bias=None,
                  obstacle_threshold_m=0.05, gravity_align=True):
    """Fuse list of axial depth frames and T_world_camera poses.

    pose_bias is a fixed map-frame translation (3,) and/or yaw/pitch/roll
    rotation vector (3,) applied to every pose, useful for sensitivity studies.
    """
    if len(frames) != len(poses) or not frames:
        raise ValueError("frames and poses must have equal nonzero length")
    biased = [np.array(p, dtype=float, copy=True) for p in poses]
    if pose_bias:
        trans = np.asarray(pose_bias.get("translation", (0, 0, 0)), dtype=float)
        angles = np.asarray(pose_bias.get("rotation_rad", (0, 0, 0)), dtype=float)
        a, b, c = angles
        Rx = np.array([[1,0,0],[0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)]])
        Ry = np.array([[np.cos(b),0,np.sin(b)],[0,1,0],[-np.sin(b),0,np.cos(b)]])
        Rz = np.array([[np.cos(c),-np.sin(c),0],[np.sin(c),np.cos(c),0],[0,0,1]])
        for i, p in enumerate(biased):
            # Interpret the supplied error as accumulated end-of-sequence drift;
            # the first pose remains the gravity-frame anchor.
            fraction = i / max(len(biased) - 1, 1)
            aa, bb, cc = angles * fraction
            Rx_i = np.array([[1,0,0],[0,np.cos(aa),-np.sin(aa)],[0,np.sin(aa),np.cos(aa)]])
            Ry_i = np.array([[np.cos(bb),0,np.sin(bb)],[0,1,0],[-np.sin(bb),0,np.cos(bb)]])
            Rz_i = np.array([[np.cos(cc),-np.sin(cc),0],[np.sin(cc),np.cos(cc),0],[0,0,1]])
            p[:3, :3] = p[:3, :3] @ Rz_i @ Ry_i @ Rx_i
            p[:3, 3] += trans * fraction
    T_gravity, origin, plane = gravity_frame_from_first_frame(
        frames[0], biased[0], intrinsics=intrinsics, stride=stride)
    if gravity_align:
        T_map_world, floor_z = T_gravity, 0.0
    else:
        # Fixed nominal optical-to-level map axes; no depth-derived tilt correction.
        optical_to_map = np.array([[1.,0.,0.],[0.,0.,1.],[0.,-1.,0.]])
        T_map_world = np.eye(4)
        T_map_world[:3,:3] = optical_to_map @ biased[0][:3,:3].T
        T_map_world[:3,3] = -T_map_world[:3,:3] @ biased[0][:3,3]
        floor_z = -plane.camera_height
    grid = ElevationGrid(size_m, resolution_m)
    updates = []
    for i, (depth, pose) in enumerate(zip(frames, biased)):
        pts_cam = depth_points(depth, intrinsics=intrinsics, stride=stride)
        pts_world = transform_points(pose, pts_cam)
        pts_map = transform_points(T_map_world, pts_world)
        robot_map = transform_points(T_map_world, pose[None, :3, 3])[0]
        grid.scroll(robot_map[:2])
        grid.update(pts_map, noise_model, time_s=float(i) / 10.0, floor_z=floor_z,
                    sensor_range_m=pts_cam[:, 2])
        updates.append(len(pts_map))
    return grid, T_map_world, origin, plane, updates

