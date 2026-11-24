"""Create a Foxglove MCAP with offline WBR obstacle-detector overlays.

Usage: foxglove_demo.py BAG_DIR OUT.mcap
The bag must be a rosbag2 SQLite/MCAP directory (or an MCAP file) containing
RealSense color and aligned-depth topics. RealSense Metadata connections are
deliberately skipped.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np
from mcap.writer import Writer
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from ros2.wbr_obstacle_node.wbr_obstacle_node.core import AdvisoryCore, CoreParams, depth_to_metres
from tools.bag_tools.mcap_recovery import messages as bag_messages, readable_mcap

COLOR = '/camera/camera/color/image_raw'
DEPTH = '/camera/camera/aligned_depth_to_color/image_raw'
COLOR_INFO = '/camera/camera/color/camera_info'
DEPTH_INFO = '/camera/camera/aligned_depth_to_color/camera_info'
TF_STATIC = '/tf_static'
OUT_TOPICS = {
    'markers': '/wbr_obstacle_node/markers',
    'advisory': '/wbr_obstacle_node/advisory',
    'points': '/wbr_obstacle_node/obstacle_points',
    'depth_points': '/wbr_obstacle_node/depth_points',
    'tf': '/tf',
}


def _image_array(msg):
    """Decode packed ROS depth image respecting step, endianness, and padding."""
    encoding = msg.encoding.upper()
    dtype = np.dtype(('>' if msg.is_bigendian else '<') + ('u2' if encoding == '16UC1' else 'f4'))
    if encoding not in ('16UC1', '32FC1'):
        raise ValueError(f'unsupported aligned depth encoding: {msg.encoding}')
    itemsize = dtype.itemsize
    row = np.frombuffer(msg.data, dtype=dtype, count=(msg.step // itemsize) * msg.height)
    return row.reshape(msg.height, msg.step // itemsize)[:, :msg.width]


def _create_messages(ts):
    t = ts.types
    return {
        'Header': t['std_msgs/msg/Header'], 'Point': t['geometry_msgs/msg/Point'],
        'Quaternion': t['geometry_msgs/msg/Quaternion'], 'Pose': t['geometry_msgs/msg/Pose'],
        'Vector3': t['geometry_msgs/msg/Vector3'], 'Duration': t['builtin_interfaces/msg/Duration'],
        'Color': t['std_msgs/msg/ColorRGBA'], 'Marker': t['visualization_msgs/msg/Marker'],
        'MarkerArray': t['visualization_msgs/msg/MarkerArray'], 'String': t['std_msgs/msg/String'],
        'PointField': t['sensor_msgs/msg/PointField'], 'PointCloud2': t['sensor_msgs/msg/PointCloud2'],
        'CompressedImage': t['sensor_msgs/msg/CompressedImage'], 'MeshFile': t['visualization_msgs/msg/MeshFile'],
        'Transform': t['geometry_msgs/msg/Transform'], 'TransformStamped': t['geometry_msgs/msg/TransformStamped'],
        'TFMessage': t['tf2_msgs/msg/TFMessage'],
    }


def _marker_array(specs, header, m):
    enums = {'CUBE': 1, 'LINE_LIST': 5, 'TRIANGLE_LIST': 11, 'TEXT_VIEW_FACING': 9}
    markers = []
    for spec in specs:
        xyz = spec.get('position', spec.get('center', [0.0, 0.0, 0.0]))
        scale = spec['scale']
        color = m['Color'](*map(float, spec['color']))
        pose = m['Pose'](m['Point'](*map(float, xyz)), m['Quaternion'](0.0, 0.0, 0.0, 1.0))
        points = [m['Point'](*map(float, p)) for p in spec.get('points', [])]
        marker_header = m['Header'](header.stamp, spec.get('frame_id', header.frame_id))
        markers.append(m['Marker'](
            marker_header, spec['ns'], int(spec['id']), enums[spec['type']], 0, pose,
            m['Vector3'](*map(float, scale)), color, m['Duration'](0, 0), False,
            points, [], '', m['CompressedImage'](header, '', np.empty(0, dtype=np.uint8)),
            [], spec.get('text', ''), '', m['MeshFile']('', np.empty(0, dtype=np.uint8)), False,
        ))
    return m['MarkerArray'](markers)


def _pointcloud(points, header, m):
    fields = [m['PointField'](name, offset, 7, 1) for name, offset in (('x', 0), ('y', 4), ('z', 8))]
    data = np.asarray(points, dtype='<f4').reshape(-1, 3).tobytes()
    return m['PointCloud2'](header, 1, len(points), fields, False, 12, len(points) * 12,
                            np.frombuffer(data, dtype=np.uint8).copy(), True)


def _height_pointcloud(depth_m, K, plane, header, m, stride=4):
    """Camera-frame XYZ plus signed height above the fitted floor plane."""
    stride = max(1, int(stride))
    dep = np.asarray(depth_m, dtype=np.float32)[::stride, ::stride]
    rows, cols = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    valid = np.isfinite(dep) & (dep > 0)
    z = dep[valid]
    x = (cols[valid] - cx) * z / fx
    y = (rows[valid] - cy) * z / fy
    n = np.asarray(plane.normal, dtype=np.float32)
    height = x * n[0] + y * n[1] + z * n[2] + float(plane.camera_height)
    packed = np.column_stack((x, y, z, height)).astype('<f4', copy=False)
    fields = [m['PointField'](name, offset, 7, 1)
              for name, offset in (('x', 0), ('y', 4), ('z', 8), ('height', 12))]
    data = packed.tobytes()
    return m['PointCloud2'](header, 1, len(packed), fields, False, 16, len(data),
                            np.frombuffer(data, dtype=np.uint8).copy(), True)


def _quaternion_matrix(q):
    x, y, z, w = map(float, q)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=np.float64)


def _transform_matrix(transform):
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = _quaternion_matrix((transform.rotation.x, transform.rotation.y,
                                        transform.rotation.z, transform.rotation.w))
    result[:3, 3] = (transform.translation.x, transform.translation.y, transform.translation.z)
    return result


def _matrix_quaternion(rotation):
    trace = float(np.trace(rotation))
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2.0
        q = ((rotation[2, 1] - rotation[1, 2]) / s,
             (rotation[0, 2] - rotation[2, 0]) / s,
             (rotation[1, 0] - rotation[0, 1]) / s, 0.25 * s)
    else:
        i = int(np.argmax(np.diag(rotation)))
        if i == 0:
            s = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            q = (0.25 * s, (rotation[0, 1] + rotation[1, 0]) / s,
                 (rotation[0, 2] + rotation[2, 0]) / s,
                 (rotation[2, 1] - rotation[1, 2]) / s)
        elif i == 1:
            s = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            q = ((rotation[0, 1] + rotation[1, 0]) / s, 0.25 * s,
                 (rotation[1, 2] + rotation[2, 1]) / s,
                 (rotation[0, 2] - rotation[2, 0]) / s)
        else:
            s = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            q = ((rotation[0, 2] + rotation[2, 0]) / s,
                 (rotation[1, 2] + rotation[2, 1]) / s, 0.25 * s,
                 (rotation[1, 0] - rotation[0, 1]) / s)
    return q


def _static_transform_to_root(transforms, root, child):
    """Compose T(root<-child) from the bag's static TF tree."""
    parents = {}
    for stamped in transforms:
        parent, frame = stamped.header.frame_id, stamped.child_frame_id
        if frame in parents and parents[frame][0] != parent:
            raise ValueError(f'static TF child {frame} has multiple parents')
        parents[frame] = (parent, _transform_matrix(stamped.transform))
    matrix = np.eye(4, dtype=np.float64)
    frame = child
    visited = set()
    while frame != root:
        if frame in visited or frame not in parents:
            raise ValueError(f'no static TF path from {root} to {child}')
        visited.add(frame)
        parent, parent_from_frame = parents[frame]
        matrix = parent_from_frame @ matrix
        frame = parent
    return matrix


def _ground_transform(plane, stamp, m, camera_link_from_depth):
    """Return TFMessage for wbr_ground -> camera_link, preserving static camera TF."""
    n = np.asarray(plane.normal, dtype=np.float64).copy()
    n /= np.linalg.norm(n)
    forward = np.array([0.0, 0.0, 1.0]) - n[2] * n
    forward /= np.linalg.norm(forward)
    left = np.cross(n, forward)
    # Columns are ground axes expressed in camera coordinates. TF needs the
    # inverse rotation: camera orientation expressed in the ground frame.
    ground_in_camera = np.column_stack((forward, left, n))
    rotation = ground_in_camera.T
    ground_from_depth = np.eye(4, dtype=np.float64)
    ground_from_depth[:3, :3] = rotation
    ground_from_depth[:3, 3] = (0.0, 0.0, float(plane.camera_height))
    ground_from_link = ground_from_depth @ np.linalg.inv(camera_link_from_depth)
    q = _matrix_quaternion(ground_from_link[:3, :3])
    t = m['Transform'](m['Vector3'](*map(float, ground_from_link[:3, 3])),
                      m['Quaternion'](*map(float, q)))
    stamped = m['TransformStamped'](
        m['Header'](stamp, 'wbr_ground'), 'camera_link', t)
    return m['TFMessage']([stamped])


def _demo_obstacle_markers(advisory, frame_result):
    """Ground-frame solid obstacle and readable status label."""
    cluster = frame_result.nearest
    if cluster is None:
        return []
    name = str(advisory['advisory'])
    if name.startswith('JUMP'):
        label, color = 'JUMP', [0.05, 0.85, 0.2, 1.0]
    elif name.startswith('STOP'):
        label, color = 'STOP', [0.95, 0.05, 0.05, 1.0]
    else:
        label, color = 'UNCERTAIN', [1.0, 0.75, 0.0, 1.0]
    width = max(float(cluster.width_m), 0.04)
    depth = max(float(cluster.depth_m), 0.04)
    height = max(float(cluster.height_m), 0.01)
    distance = float(cluster.distance_m)
    center = [distance + depth / 2.0, 0.0, height / 2.0]
    text = f"{label} {height * 100:.1f} cm @ {distance:.2f} m"
    return [
        {'ns': 'demo_obstacle', 'id': 100, 'type': 'CUBE', 'frame_id': 'wbr_ground',
         'center': center, 'scale': [depth, width, height], 'color': color, 'text': ''},
        {'ns': 'demo_obstacle_label', 'id': 101, 'type': 'TEXT_VIEW_FACING',
         'frame_id': 'wbr_ground', 'position': [center[0], center[1], height + 0.12],
         'scale': [0.0, 0.0, 0.22], 'color': color, 'text': text},
    ]


def _bag_reader_path(bag_dir):
    if bag_dir.is_file():
        if bag_dir.suffix.lower() not in ('.mcap', '.db3'):
            raise ValueError(f'unsupported rosbag2 file: {bag_dir}')
        return bag_dir
    if not bag_dir.is_dir():
        raise ValueError(f'bag path does not exist: {bag_dir}')
    if (bag_dir / 'metadata.yaml').is_file():
        return bag_dir
    mcap_files = sorted(bag_dir.glob('*.mcap'))
    if len(mcap_files) == 1:
        return mcap_files[0]
    raise ValueError(f'expected rosbag2 metadata.yaml or one MCAP file in {bag_dir}')


def convert(bag_dir: str | Path, output: str | Path, *, max_forward=1.3,
            range_hysteresis=0.10):
    bag_dir, output = Path(bag_dir), Path(output)
    reader_path = _bag_reader_path(bag_dir)
    output.parent.mkdir(parents=True, exist_ok=True)
    ts = get_typestore(Stores.ROS2_JAZZY)
    m = _create_messages(ts)
    core = AdvisoryCore(str(REPO_ROOT), CoreParams())
    with readable_mcap(reader_path) as (readable_path, recovered_tail_bytes), \
            Reader(readable_path) as reader, output.open('wb') as stream:
        writer = Writer(stream)
        writer.start(profile='ros2')
        channels, schemas = {}, {}

        def register(topic, msgtype):
            if msgtype not in schemas:
                definition, _ = ts.generate_msgdef(msgtype, ros_version=2)
                schemas[msgtype] = writer.register_schema(msgtype, 'ros2msg', definition.encode())
            channels[topic] = writer.register_channel(topic, 'cdr', schemas[msgtype])

        conns = {c.topic: c for c in reader.connections}
        required = (COLOR, DEPTH, COLOR_INFO, DEPTH_INFO, TF_STATIC)
        missing = [topic for topic in required if topic not in conns]
        if missing:
            raise ValueError(f'missing required topics: {missing}')
        for topic in required:
            conn = conns[topic]
            if conn.msgtype not in ts.types:
                raise ValueError(f'unsupported type for required topic {topic}: {conn.msgtype}')
            register(topic, conn.msgtype)
        static_transforms = []
        for tf_conn, _, tf_raw in reader.messages(connections=[conns[TF_STATIC]]):
            static_transforms.extend(ts.deserialize_cdr(tf_raw, tf_conn.msgtype).transforms)
        camera_link_from_depth = _static_transform_to_root(
            static_transforms, 'camera_link', 'camera_depth_optical_frame')
        register(OUT_TOPICS['markers'], 'visualization_msgs/msg/MarkerArray')
        register(OUT_TOPICS['advisory'], 'std_msgs/msg/String')
        register(OUT_TOPICS['points'], 'sensor_msgs/msg/PointCloud2')
        register(OUT_TOPICS['depth_points'], 'sensor_msgs/msg/PointCloud2')
        register(OUT_TOPICS['tf'], 'tf2_msgs/msg/TFMessage')
        selected = [c for c in reader.connections if c.topic in required]
        # Camera calibration is static in these recordings. Seed it before the
        # time-ordered pass because some bags begin recording images before the
        # first CameraInfo sample.
        latest_k = None
        for info_conn, _, info_raw in reader.messages(connections=[conns[COLOR_INFO]]):
            info = ts.deserialize_cdr(info_raw, info_conn.msgtype)
            candidate = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
            if np.isfinite(candidate).all() and candidate[0, 0] > 0 and candidate[1, 1] > 0:
                latest_k = candidate
                break
        if latest_k is None:
            raise ValueError(f'no valid CameraInfo on {COLOR_INFO}')
        counts, histogram = Counter(), Counter()
        last_plane = type('Plane', (), {'normal': np.array([0.0, -1.0, 0.0]), 'camera_height': 0.30})()

        for conn, log_time, raw in bag_messages(reader, readable_path, selected):
            log_time = int(log_time)
            msg = ts.deserialize_cdr(raw, conn.msgtype)
            writer.add_message(channels[conn.topic], log_time, raw, publish_time=log_time)
            counts[conn.topic] += 1
            if conn.topic == COLOR_INFO:
                k = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
                if np.isfinite(k).all() and k[0, 0] > 0 and k[1, 1] > 0:
                    latest_k = k
            elif conn.topic == DEPTH:
                if latest_k is None:
                    raise ValueError('aligned depth arrived before valid color CameraInfo')
                arr = _image_array(msg)
                depth_m = depth_to_metres(arr, msg.encoding)
                stamp = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
                out = core.process(depth_m, latest_k, frame_stamp_ns=stamp,
                                   frame_id=msg.header.frame_id, range_limit_m=max_forward,
                                   range_hysteresis=range_hysteresis)
                header = msg.header
                plane = core.detect_state.get('plane')
                if plane is not None:
                    last_plane = plane
                marker_array = _marker_array(
                    out['markers'] + _demo_obstacle_markers(out['advisory'], out['frame_result']), header, m)
                tf_message = _ground_transform(last_plane, header.stamp, m, camera_link_from_depth)
                advisory = {key: out['advisory'][key] for key in (
                    'advisory', 'height_m', 'distance_m', 'width_m', 'confidence', 'plane_valid')}
                generated = (
                    (OUT_TOPICS['markers'], 'visualization_msgs/msg/MarkerArray', marker_array),
                    (OUT_TOPICS['advisory'], 'std_msgs/msg/String', m['String'](json.dumps(advisory, separators=(',', ':'), allow_nan=False))),
                    (OUT_TOPICS['points'], 'sensor_msgs/msg/PointCloud2', _pointcloud(out['points'], header, m)),
                    (OUT_TOPICS['depth_points'], 'sensor_msgs/msg/PointCloud2',
                     _height_pointcloud(depth_m, latest_k, last_plane, header, m, stride=4)),
                    (OUT_TOPICS['tf'], 'tf2_msgs/msg/TFMessage', tf_message),
                )
                for topic, msgtype, output_msg in generated:
                    writer.add_message(channels[topic], log_time,
                                       ts.serialize_cdr(output_msg, msgtype), publish_time=log_time)
                    counts[topic] += 1
                histogram[advisory['advisory']] += 1
        writer.finish()
    return {'output': str(output), 'source_recovered_tail_bytes': recovered_tail_bytes,
            'max_forward_m': max_forward,
            'range_hysteresis_m': range_hysteresis,
            'message_counts': dict(sorted(counts.items())),
            'advisory_histogram': dict(sorted(histogram.items())), 'size_bytes': output.stat().st_size}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag_dir')
    parser.add_argument('output_mcap')
    parser.add_argument('--max-forward', type=float, default=1.3)
    parser.add_argument('--range-hysteresis', type=float, default=0.10)
    args = parser.parse_args(argv)
    if args.max_forward < 0 or args.range_hysteresis < 0:
        parser.error('--max-forward and --range-hysteresis must be non-negative')
    print(json.dumps(convert(args.bag_dir, args.output_mcap,
                             max_forward=args.max_forward,
                             range_hysteresis=args.range_hysteresis), indent=2))


if __name__ == '__main__':
    main()
