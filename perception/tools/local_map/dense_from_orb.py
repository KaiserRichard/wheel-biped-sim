#!/usr/bin/env python3
"""Fuse rectified D435 stereo with ORB-SLAM3 poses into a Foxglove-ready cloud."""
from __future__ import annotations

import argparse
import bisect
import json
import math
import re
import sys
import time
from pathlib import Path

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import Decoder
from mcap.writer import Writer
from rosbags.rosbag2 import Reader as BagReader
from rosbags.typesys import Stores, get_typestore

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.stereo_depth.mcap_stereo import (
    LEFT_IMAGE, RIGHT_IMAGE, LEFT_INFO, RIGHT_INFO,
    image_to_array, stamp_ns, calibration_from_camera_info, read_calibration,
)

POSE_TOPIC = "/d435_orb/pose"
MAP_POINTS_TOPIC = "/d435_orb/map_points"
MAP_FRAME = "map"
WORLD_FRAME = "slam_world"
MAP_Q = (-0.5, 0.5, -0.5, 0.5)


def _pose_matrix(msg):
    p, q = msg.pose.position, msg.pose.orientation
    qx, qy, qz, qw = map(float, (q.x, q.y, q.z, q.w))
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid ORB pose quaternion")
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    R = np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ], dtype=np.float64)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = (p.x, p.y, p.z)
    return T


def read_poses(bag: Path):
    ts = get_typestore(Stores.ROS2_JAZZY)
    with BagReader(bag) as reader:
        conns = [c for c in reader.connections if c.topic == POSE_TOPIC]
        poses = []
        for conn, _, raw in reader.messages(connections=conns):
            msg = ts.deserialize_cdr(raw, conn.msgtype)
            poses.append((stamp_ns(msg), _pose_matrix(msg), msg))
    poses.sort(key=lambda x: x[0])
    if not poses:
        raise ValueError(f"no {POSE_TOPIC} messages in {bag}")
    return poses


def _nearest_pose(stamp, stamps, poses, max_delta_ns):
    i = bisect.bisect_left(stamps, stamp)
    candidates = poses[max(0, i-1):min(len(poses), i+1)]
    if not candidates:
        return None
    result = min(candidates, key=lambda p: abs(p[0]-stamp))
    return result if abs(result[0]-stamp) <= max_delta_ns else None


def _box_cost(left, right, min_disp, disparities, radius=2):
    """5x5 SAD cost volume for integer disparities (SGBM matching cost)."""
    h, w = left.shape
    cost = np.full((h, w, disparities), 255*25, dtype=np.int16)
    l = left.astype(np.int16, copy=False)
    r = right.astype(np.int16, copy=False)
    for k in range(disparities):
        d = min_disp + k
        x0, x1 = max(0, d), min(w, w+d)
        # left[x] matches right[x-d].
        if x1 <= x0:
            continue
        diff = np.zeros((h, w), dtype=np.int16)
        diff[:, x0:x1] = np.abs(l[:, x0:x1] - r[:, x0-d:x1-d])
        integ = np.pad(diff, ((1, 0), (1, 0))).cumsum(0, dtype=np.int32).cumsum(1, dtype=np.int32)
        block = (integ[2*radius+1:, 2*radius+1:] - integ[:-2*radius-1, 2*radius+1:]
                 - integ[2*radius+1:, :-2*radius-1] + integ[:-2*radius-1, :-2*radius-1])
        cost[radius:h-radius, radius:w-radius, k] = block
    return cost


def _aggregate_path(cost, reverse=False, vertical=False, p1=200, p2=800):
    """One semi-global path; scan lines are parallelized in NumPy."""
    h, w, nd = cost.shape
    out = np.zeros((h, w, nd), dtype=np.uint16)
    inf = np.int32(1 << 27)
    length = h if vertical else w
    for j in range(length):
        i = length-1-j if reverse else j
        current_cost = cost[i, :, :].astype(np.int32) if vertical else cost[:, i, :].astype(np.int32)
        if j == 0:
            current = current_cost
        else:
            prev_min = prev.min(axis=1, keepdims=True)
            lo = np.full_like(prev, inf); lo[:, 1:] = prev[:, :-1] + p1
            hi = np.full_like(prev, inf); hi[:, :-1] = prev[:, 1:] + p1
            best = np.minimum(np.minimum(prev, lo), np.minimum(hi, prev_min+p2))
            current = current_cost + best - prev_min
        if vertical:
            out[i, :, :] = np.minimum(current, 65535).astype(np.uint16)
        else:
            out[:, i, :] = np.minimum(current, 65535).astype(np.uint16)
        prev = current
    return out


def sgbm_disparity(left, right, min_disp, num_disp):
    """SGBM-style 4-path aggregation fallback when OpenCV is absent.

    The repository's OpenCV helper is used when cv2 is installed. This backend
    retains semi-global smoothness and left-right checking without installing
    packages; it uses a 5x5 SAD cost and four scan directions.
    """
    cost = _box_cost(left, right, min_disp, num_disp)
    aggregate = np.zeros(cost.shape, dtype=np.uint32)
    for reverse, vertical in ((False, False), (True, False), (False, True), (True, True)):
        aggregate += _aggregate_path(cost, reverse=reverse, vertical=vertical)
    disparity = (np.argmin(aggregate, axis=2).astype(np.float32) + min_disp)
    # Parabolic subpixel correction around the minimum.
    best = np.argmin(aggregate, axis=2)
    valid = (best > 0) & (best < num_disp-1)
    rows, cols = np.nonzero(valid)
    d = best[rows, cols]
    a = aggregate[rows, cols, d-1].astype(np.float32)
    b = aggregate[rows, cols, d].astype(np.float32)
    c = aggregate[rows, cols, d+1].astype(np.float32)
    denom = a - 2*b + c
    offset = np.zeros_like(b)
    good = np.abs(denom) > 1e-5
    offset[good] = np.clip(0.5*(a[good]-c[good])/denom[good], -1, 1)
    disparity[rows, cols] += offset
    return disparity


def stereo_depth(left, right, fx, baseline, min_depth=0.3, max_depth=3.0):
    min_disp = max(1, int(math.floor(fx*baseline/max_depth)))
    max_disp = int(math.ceil(fx*baseline/min_depth))
    nd = max_disp-min_disp+1
    try:
        import cv2
    except ImportError:
        # The NumPy fallback works at quarter linear resolution to keep this
        # 45-second recording tractable without installing OpenCV. Disparity
        # and intrinsics scale together; depth is invariant under this change.
        factor = 4
        h, w = left.shape
        small_left, small_right = left[::factor, ::factor], right[::factor, ::factor]
        small_min = max(1, int(math.floor(min_disp/factor)))
        small_max = max(small_min+1, int(math.ceil(max_disp/factor)))
        small_nd = small_max-small_min+1
        disp = sgbm_disparity(small_left, small_right, small_min, small_nd)
        reverse = sgbm_disparity(small_right, small_left, -small_max, small_nd)
        hs, ws = small_left.shape
        xs = np.broadcast_to(np.arange(ws), (hs, ws))
        xr = np.rint(xs-disp).astype(np.int32)
        bounds = (xr >= 0) & (xr < ws)
        sampled = reverse[np.arange(hs)[:, None], np.clip(xr, 0, ws-1)]
        small_valid = bounds & (disp > 0) & (np.abs(disp + sampled) <= 1.5)
        disp = np.repeat(np.repeat(disp*factor, factor, axis=0), factor, axis=1)[:h, :w]
        valid = np.repeat(np.repeat(small_valid, factor, axis=0), factor, axis=1)[:h, :w]
    else:
        block = 5
        common = dict(blockSize=block, P1=8*block*block, P2=32*block*block,
                      disp12MaxDiff=1, preFilterCap=31, uniquenessRatio=10,
                      speckleWindowSize=100, speckleRange=2,
                      mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        ml = cv2.StereoSGBM_create(minDisparity=min_disp, numDisparities=int(math.ceil(nd/16)*16), **common)
        mr = cv2.StereoSGBM_create(minDisparity=-int(math.ceil(nd/16)*16),
                                   numDisparities=int(math.ceil(nd/16)*16), **common)
        disp = ml.compute(left, right).astype(np.float32)/16
        rev = mr.compute(right, left).astype(np.float32)/16
        h, w = left.shape
        xs = np.broadcast_to(np.arange(w), (h, w))
        xr = np.rint(xs-disp).astype(np.int32)
        bounds = (xr >= 0) & (xr < w)
        sampled = rev[np.arange(h)[:, None], np.clip(xr, 0, w-1)]
        valid = bounds & (disp > 0) & (np.abs(disp + sampled) <= 1.5)
    depth = np.zeros(left.shape, dtype=np.float32)
    depth[valid] = fx*baseline/disp[valid]
    valid &= np.isfinite(depth) & (depth >= min_depth) & (depth <= max_depth)
    depth[~valid] = 0
    return depth, valid


def map_transform():
    x, y, z, w = MAP_Q
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]], dtype=float)
    T = np.eye(4); T[:3, :3] = R
    return T


def voxel_filter(points, voxel, min_frames):
    keys = np.floor(points[:, :3]/voxel).astype(np.int64)
    uniq, inv = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inv)
    sums = np.zeros((len(uniq), 4), dtype=np.float64)
    np.add.at(sums, inv, points)
    frame_counts = np.zeros(len(uniq), dtype=np.int32)
    # Each frame's points are contiguous and tracked separately by caller.
    return uniq, sums/counts[:, None], counts, frame_counts


def _plane_rms(points):
    """Deterministic dominant-plane RANSAC followed by orthogonal least squares."""
    if len(points) < 3:
        return None, 0
    rng = np.random.default_rng(314159)
    sample_n = min(1200, len(points))
    samples = points[rng.choice(len(points), sample_n, replace=False)]
    best = np.empty(0, dtype=np.int64)
    for _ in range(400):
        ids = rng.choice(len(samples), 3, replace=False)
        a, b, c = samples[ids]
        n = np.cross(b-a, c-a); norm = np.linalg.norm(n)
        if norm < 1e-8: continue
        n /= norm
        # A table top should be approximately horizontal in the requested map
        # frame. Reject wall-like planes so the sanity metric targets that face.
        if abs(n[2]) < 0.85: continue
        residual = np.abs((points-a) @ n)
        inliers = np.flatnonzero(residual < 0.01)
        if len(inliers) > len(best): best = inliers
    if len(best) < max(50, int(0.03*len(points))):
        return None, len(best)
    subset = points[best]
    center = subset.mean(axis=0)
    _, _, vh = np.linalg.svd(subset-center, full_matrices=False)
    residual = (subset-center) @ vh[-1]
    return float(np.sqrt(np.mean(residual**2))), len(best)


def _write_ply(path, points):
    with path.open("w", encoding="ascii") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\nproperty float x\nproperty float y\nproperty float z\nproperty float intensity\nend_header\n")
        np.savetxt(f, points, fmt="%.6g")


def _make_ros_messages(ts, cloud, poses, stamp):
    t = ts.types
    def stamp_obj(ns): return t["builtin_interfaces/msg/Time"](int(ns//1_000_000_000), int(ns%1_000_000_000))
    header = t["std_msgs/msg/Header"](stamp_obj(stamp), MAP_FRAME)
    fields = [t["sensor_msgs/msg/PointField"](name, off, 7, 1)
              for name, off in (("x",0),("y",4),("z",8),("intensity",12))]
    packed = np.asarray(cloud, dtype="<f4").reshape(-1,4).tobytes()
    pc = t["sensor_msgs/msg/PointCloud2"](header, 1, len(cloud), fields, False, 16,
        16*len(cloud), np.frombuffer(packed,dtype=np.uint8).copy(), True)
    qmap = t["geometry_msgs/msg/Quaternion"](*MAP_Q)
    pos0 = t["geometry_msgs/msg/Point"](0.,0.,0.)
    tf = t["geometry_msgs/msg/Transform"](t["geometry_msgs/msg/Vector3"](0.,0.,0.),qmap)
    tfst = t["geometry_msgs/msg/TransformStamped"](
        t["std_msgs/msg/Header"](stamp_obj(0), MAP_FRAME), WORLD_FRAME, tf)
    tfmsg = t["tf2_msgs/msg/TFMessage"]([tfst])
    path_poses=[]
    Tmw=map_transform()
    for ns, T, msg in poses:
        p=msg.pose.position; q=msg.pose.orientation
        xyz=Tmw[:3,:3]@np.array([p.x,p.y,p.z])
        # q_map_world * q_camera_world
        ax,ay,az,aw=MAP_Q; bx,by,bz,bw=map(float,(q.x,q.y,q.z,q.w))
        qm=(aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz)
        ps=t["geometry_msgs/msg/PoseStamped"](
            t["std_msgs/msg/Header"](stamp_obj(ns),MAP_FRAME),
            t["geometry_msgs/msg/Pose"](t["geometry_msgs/msg/Point"](*map(float,xyz)),
                                         t["geometry_msgs/msg/Quaternion"](*qm)))
        path_poses.append(ps)
    path=t["nav_msgs/msg/Path"](header,path_poses)
    return pc, path, tfmsg


def _layout(path, growing=False):
    cloud_topic = "/dense_map_growing" if growing else "/dense_map"
    image_panel = {"topicPath": LEFT_IMAGE, "imageMode": "fit", "foxglovePanelTitle": "Infrared left"}
    cloud_panel = {"topics": {cloud_topic: {"visible": True, "colorField": "intensity",
        "colorMode": "colormap", "colorMap": "turbo", "pointSize": 2},
        "/orb/path": {"visible": True}},
        "cameraState": {"distance": 3, "perspective": True, "phi": 60, "target": [0,0,0],
        "targetOffset": [0,0,0], "targetOrientation": [0,0,0,1], "thetaOffset": 0, "fovy": 45,
        "near": 0.01, "far": 100}, "followMode": "follow-none", "fixedFrame": "map", "scene": {},
        "transforms": {"showLabel": False, "axisScale": 0.1}, "imageMode": {}, "synchronize": False,
        "foxglovePanelTitle": "Dense stereo map"}
    doc={"configById":{"3D!dense-map": cloud_panel, "Image!infra1": image_panel,
        "RawMessages!path":{"topicPath":"/orb/path","foxglovePanelTitle":"ORB trajectory"}},
        "layout":{"first":"Image!infra1","second":"3D!dense-map","direction":"row","splitPercentage":30},
        "globalVariables":{},"playbackConfig":{"speed":1},"userNodes":{}}
    path.write_text(json.dumps(doc,indent=2)+"\n")


def _write_mcap(path, cloud, poses, stamp):
    ts=get_typestore(Stores.ROS2_JAZZY)
    pc, orb_path, tfmsg=_make_ros_messages(ts,cloud,poses,stamp)
    with path.open("wb") as stream:
        writer=Writer(stream); writer.start(profile="ros2")
        channels={}
        for topic,kind in (("/dense_map","sensor_msgs/msg/PointCloud2"),
                            ("/orb/path","nav_msgs/msg/Path"),
                            ("/tf_static","tf2_msgs/msg/TFMessage")):
            definition,_=ts.generate_msgdef(kind,ros_version=2)
            schema=writer.register_schema(kind,"ros2msg",definition.encode())
            channels[topic]=writer.register_channel(topic,"cdr",schema)
        for topic,kind,msg in (("/dense_map","sensor_msgs/msg/PointCloud2",pc),
                               ("/orb/path","nav_msgs/msg/Path",orb_path),
                               ("/tf_static","tf2_msgs/msg/TFMessage",tfmsg)):
            writer.add_message(channel_id=channels[topic], log_time=int(stamp),
                publish_time=int(stamp), sequence=0, data=ts.serialize_cdr(msg,kind))
        writer.finish()


def _limited_voxel_cloud(points, voxel, cap=150_000):
    if not len(points):
        return np.empty((0, 4), dtype=np.float32)
    keys = np.floor(points[:, :3] / voxel).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    sums = np.zeros((len(counts), 4), dtype=np.float64)
    np.add.at(sums, inverse, points)
    cloud = (sums / counts[:, None]).astype(np.float32)
    if len(cloud) > cap:
        # Deterministic spatially distributed thinning after voxel reduction.
        cloud = cloud[np.linspace(0, len(cloud)-1, cap, dtype=np.int64)]
    return cloud


def _write_growing_mcap(path, events, poses, image_records, camera_info_records, voxel):
    """Write time-aligned cloud snapshots, trajectory, transforms and source imagery."""
    ts = get_typestore(Stores.ROS2_JAZZY)
    t = ts.types
    channels = {}
    with path.open("wb") as stream:
        writer = Writer(stream); writer.start(profile="ros2")
        kinds = {"/dense_map_growing": "sensor_msgs/msg/PointCloud2", "/orb/path": "nav_msgs/msg/Path",
                 "/tf": "tf2_msgs/msg/TFMessage", "/camera/camera/infra1/image_rect_raw": "sensor_msgs/msg/Image",
                 "/camera/camera/infra1/camera_info": "sensor_msgs/msg/CameraInfo"}
        for topic, kind in kinds.items():
            definition, _ = ts.generate_msgdef(kind, ros_version=2)
            schema = writer.register_schema(kind, "ros2msg", definition.encode())
            channels[topic] = writer.register_channel(topic, "cdr", schema)
        # Camera info is static but carries the sensor's original header stamp.
        for stamp, raw in camera_info_records:
            writer.add_message(channel_id=channels["/camera/camera/infra1/camera_info"], log_time=stamp,
                               publish_time=stamp, sequence=0, data=raw)
        accumulated = []
        next_snapshot = None
        for stamp, frame_points, pose_index in events:
            accumulated.append(frame_points)
            if next_snapshot is None:
                next_snapshot = stamp
            if stamp < next_snapshot:
                continue
            next_snapshot = stamp + 500_000_000
            partial_poses = poses[:pose_index+1]
            cloud = _limited_voxel_cloud(np.concatenate(accumulated), voxel)
            pc, orb_path, _ = _make_ros_messages(ts, cloud, partial_poses, stamp)
            # map -> slam_world and slam_world -> orb_camera at the corresponding pose.
            _, _, base_tf = _make_ros_messages(ts, cloud, [], stamp)
            base_tf.transforms[0].header.stamp = t["builtin_interfaces/msg/Time"](
                stamp//1_000_000_000, stamp%1_000_000_000)
            pmsg = poses[pose_index][2]
            q = pmsg.pose.orientation; p = pmsg.pose.position
            world_camera = t["geometry_msgs/msg/Transform"](
                t["geometry_msgs/msg/Vector3"](float(p.x), float(p.y), float(p.z)),
                t["geometry_msgs/msg/Quaternion"](float(q.x), float(q.y), float(q.z), float(q.w)))
            tf_camera = t["geometry_msgs/msg/TransformStamped"](
                t["std_msgs/msg/Header"](t["builtin_interfaces/msg/Time"](stamp//1_000_000_000, stamp%1_000_000_000), WORLD_FRAME),
                "orb_camera", world_camera)
            tfmsg = t["tf2_msgs/msg/TFMessage"](base_tf.transforms + [tf_camera])
            for topic, kind, msg in (("/dense_map_growing", "sensor_msgs/msg/PointCloud2", pc),
                                     ("/orb/path", "nav_msgs/msg/Path", orb_path),
                                     ("/tf", "tf2_msgs/msg/TFMessage", tfmsg)):
                writer.add_message(channel_id=channels[topic], log_time=stamp, publish_time=stamp,
                                   sequence=0, data=ts.serialize_cdr(msg, kind))
        # Preserve every left infrared frame at its source header time.
        for stamp, raw in image_records:
            writer.add_message(channel_id=channels["/camera/camera/infra1/image_rect_raw"], log_time=stamp,
                               publish_time=stamp, sequence=0, data=raw)
        writer.finish()


def run(sensor_bag, pose_bag, stereo_yaml, out, *, every=3, stride=3,
        voxel=0.01, min_frames=2, min_depth=0.3, max_depth=3.0, pose_tol_ms=20, growing=False):
    sensor_bag,pose_bag,stereo_yaml,out=map(Path,(sensor_bag,pose_bag,stereo_yaml,out))
    if every<1 or stride<1 or voxel<=0: raise ValueError("every/stride/voxel must be positive")
    yaml=stereo_yaml.read_text(); m=re.search(r"^Stereo\.b\x3a\s*([0-9.eE+-]+)",yaml,re.M)
    if not m: raise ValueError("Stereo.b missing from stereo.yaml")
    yaml_baseline=float(m.group(1))
    calibration=read_calibration(sensor_bag/"sensor-bag_0.mcap")
    if abs(calibration.baseline_m-yaml_baseline)>0.002:
        raise ValueError(f"camera_info baseline {calibration.baseline_m:.6f} and stereo.yaml {yaml_baseline:.6f} disagree")
    poses=read_poses(pose_bag); pose_stamps=[p[0] for p in poses]
    decoder=Decoder(); infos={}; pairs={}; pending={}; growing_events=[]; image_records=[]; camera_info_records=[]
    # The MCAP reader returns source records in log order; CameraInfo is cached,
    # and images are paired strictly by their ROS header stamps.
    with open(sensor_bag/"sensor-bag_0.mcap","rb") as stream:
        reader=make_reader(stream)
        pair_index=0; frames_used=0; skipped_pose=0; raw_xyz=[]; raw_i=[]; frame_ids=[]
        counts={"left":0,"right":0,"pairs":0}
        for schema,channel,raw in reader.iter_messages(topics=[LEFT_INFO,RIGHT_INFO,LEFT_IMAGE,RIGHT_IMAGE]):
            msg=decoder.decode(schema,raw)
            if channel.topic in (LEFT_INFO,RIGHT_INFO):
                infos[channel.topic]=msg
                if growing and channel.topic == LEFT_INFO:
                    camera_info_records.append((stamp_ns(msg), raw.data))
                continue
            side="left" if channel.topic==LEFT_IMAGE else "right"
            counts[side]+=1; stamp=stamp_ns(msg)
            slot=pending.setdefault(stamp,{})
            slot[side]=image_to_array(msg)
            if growing and side == "left":
                image_records.append((stamp, raw.data))
            if "left" not in slot or "right" not in slot: continue
            left,right=slot["left"],slot["right"]; del pending[stamp]
            index=pair_index; pair_index+=1; counts["pairs"]+=1
            if index%every: continue
            pose=_nearest_pose(stamp,pose_stamps,poses,int(pose_tol_ms*1e6))
            if pose is None: skipped_pose+=1; continue
            # camera_info P[3] supplies fx*baseline; stereo.yaml is cross-checked above.
            cal=calibration
            depth,valid=stereo_depth(left,right,cal.fx,cal.baseline_m,min_depth,max_depth)
            rows,cols=np.mgrid[0:depth.shape[0]:stride,0:depth.shape[1]:stride]
            dep=depth[::stride,::stride]; inten=left[::stride,::stride].astype(np.float32)/255.0
            mask=(dep>0)&valid[::stride,::stride]
            z=dep[mask]; x=(cols[mask]-cal.cx)*z/cal.fx; y=(rows[mask]-cal.cy)*z/cal.fx
            camera=np.column_stack((x,y,z))
            T=pose[1]
            world=camera@T[:3,:3].T+T[:3,3]
            Tmw=map_transform(); mapped=world@Tmw[:3,:3].T
            points=np.column_stack((mapped,inten[mask]))
            if growing:
                growing_events.append((stamp, points.astype(np.float32), pose_stamps.index(pose[0])))
            raw_xyz.append(points); frame_ids.append(np.full(len(points),frames_used,dtype=np.int32))
            frames_used+=1
    if frames_used==0: raise RuntimeError("no stereo pairs matched an ORB pose within tolerance")
    raw=np.concatenate(raw_xyz,axis=0).astype(np.float32)
    ids=np.concatenate(frame_ids)
    before=len(raw)
    keys=np.floor(raw[:,:3]/voxel).astype(np.int64)
    uniq,inverse=np.unique(keys,axis=0,return_inverse=True)
    nvox=len(uniq); totals=np.zeros((nvox,4),dtype=np.float64); np.add.at(totals,inverse,raw)
    counts=np.bincount(inverse).astype(np.int64)
    # Number of distinct contributing frames per voxel.
    pairs=np.column_stack((inverse,ids)); order=np.lexsort((pairs[:,1],pairs[:,0])); pairs=pairs[order]
    change=np.r_[True,np.any(pairs[1:]!=pairs[:-1],axis=1)]
    seen=np.bincount(pairs[change,0],minlength=nvox).astype(np.int32)
    cloud=(totals/counts[:,None]).astype(np.float32)
    cloud=cloud[seen>=min_frames]
    if len(cloud)==0: raise RuntimeError("all voxels were observed in fewer than the required frame count")
    out.mkdir(parents=True,exist_ok=True)
    _write_ply(out/"map_dense.ply",cloud)
    stamp=poses[-1][0]
    _write_mcap(out/"map_dense.mcap",cloud,poses,stamp)
    _layout(out/"foxglove_layout.json", growing=growing)
    if growing:
        _write_growing_mcap(out/"map_growing.mcap", growing_events, poses, image_records, camera_info_records, voxel)
    # Top-down in map (x forward, y left).
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(9,7),dpi=160)
    sc=ax.scatter(cloud[:,0],cloud[:,1],c=cloud[:,2],s=0.25,cmap="viridis",linewidths=0,rasterized=True)
    fig.colorbar(sc,ax=ax,label="map z (m)"); ax.set_xlabel("map x (forward, m)"); ax.set_ylabel("map y (left, m)")
    ax.set_title("D435 stereo dense map, top-down"); ax.axis("equal"); ax.grid(alpha=.2); fig.tight_layout(); fig.savefig(out/"topdown.png"); plt.close(fig)
    rms,nplane=_plane_rms(cloud[:,:3])
    mins=cloud[:,:3].min(axis=0); maxs=cloud[:,:3].max(axis=0)
    extent=maxs-mins
    stats=(f"frames used: {frames_used}\npoints before/after voxel: {before} / {len(cloud)}\nextent: x={extent[0]:.3f} m, y={extent[1]:.3f} m, z={extent[2]:.3f} m\n")
    (out/"stats.txt").write_text(stats)
    return {"frames_used":frames_used,"pairs":pair_index,"skipped_pose":skipped_pose,
            "points_before_voxel":before,"points_after_voxel":len(cloud),"extent_m":extent.tolist(),
            "table_plane_rms_m":rms,"table_plane_inliers":nplane}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sensor-bag",required=True); p.add_argument("--pose-bag",required=True)
    p.add_argument("--stereo-yaml",required=True); p.add_argument("--out",default="dense-out")
    p.add_argument("--every",type=int,default=3); p.add_argument("--stride",type=int,default=3)
    p.add_argument("--voxel",type=float,default=.01); p.add_argument("--min-frames",type=int,default=2)
    p.add_argument("--min-depth",type=float,default=.3); p.add_argument("--max-depth",type=float,default=3.)
    p.add_argument("--pose-tol-ms",type=float,default=20.)
    p.add_argument("--growing",action="store_true",help="also write a time-ordered growing map MCAP for Foxglove")
    a=p.parse_args()
    result=run(a.sensor_bag,a.pose_bag,a.stereo_yaml,a.out,every=a.every,stride=a.stride,
        voxel=a.voxel,min_frames=a.min_frames,min_depth=a.min_depth,max_depth=a.max_depth,pose_tol_ms=a.pose_tol_ms,growing=a.growing)
    print(json.dumps(result,indent=2))


if __name__=="__main__": main()
