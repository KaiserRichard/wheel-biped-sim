#!/usr/bin/env python3
"""Check a separate sparse-map MCAP after all writers have stopped (ROS Jazzy)."""
import argparse
import json
import math
import struct
from pathlib import Path


def verify(bag, source_bag=None):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    def messages(path, topics=None):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
        if topics:
            reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            yield topic, deserialize_message(data, types[topic]), timestamp

    def stamp(message):
        return message.header.stamp.sec * 10**9 + message.header.stamp.nanosec

    poses, statuses, infos, clouds = set(), {}, {}, []
    for topic, msg, received in messages(bag):
        if topic == '/d435_orb/pose':
            assert msg.header.frame_id == 'slam_world'
            poses.add(stamp(msg))
        elif topic == '/d435_orb/status':
            status = json.loads(msg.data)
            statuses[status['source_ns']] = status
        elif topic == '/d435_orb/map_points_info':
            info = json.loads(msg.data)
            infos[info['source_ns']] = info
        elif topic == '/d435_orb/map_points':
            assert msg.header.frame_id == 'slam_world'
            assert 0 <= msg.header.stamp.nanosec < 10**9
            assert [(f.name, f.offset, f.datatype, f.count) for f in msg.fields] == [
                ('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1)]
            assert msg.height == 1 and msg.point_step == 12 and msg.row_step == msg.width * 12
            assert len(msg.data) == msg.row_step and msg.width <= 50000
            assert msg.is_dense and not msg.is_bigendian
            points = list(struct.iter_unpack('<fff', bytes(msg.data)))
            assert all(math.isfinite(v) for point in points for v in point)
            clouds.append({'source_ns': stamp(msg), 'record_ns': received, 'points': msg.width,
                           'sample_xyz': points[:3], 'frame': msg.header.frame_id,
                           'bounds_xyz': [[min(p[i] for p in points), max(p[i] for p in points)]
                                          for i in range(3)] if points else []})
    assert clouds and any(c['points'] > 0 for c in clouds), 'no actual nonempty map cloud'
    for cloud in clouds:
        ns = cloud['source_ns']
        assert ns in infos and ns in statuses, 'missing matching metadata/status stamp'
        assert cloud['points'] == infos[ns]['points']
        if cloud['points']:
            assert ns in poses and statuses[ns]['pose_published'] and infos[ns]['tracked']
    intervals = [b['source_ns'] - a['source_ns'] for a, b in zip(clouds, clouds[1:])]
    assert all(ns >= 10**9 for ns in intervals), 'source snapshot rate exceeds 1 Hz'
    source_match = None
    if source_bag:
        wanted = set(statuses)
        matched = set()
        for _, msg, _ in messages(source_bag, ['/camera/camera/infra1/image_rect_raw']):
            ns = stamp(msg)
            if ns in wanted:
                matched.add(ns)
            if ns > max(wanted):
                break
        assert matched == wanted, 'output timestamps absent from input recording'
        source_match = True
    return {'passed': True, 'clouds': len(clouds), 'nonempty_clouds': sum(c['points'] > 0 for c in clouds),
            'max_points': max(c['points'] for c in clouds), 'poses': len(poses),
            'processed_statuses': len(statuses), 'last_status': statuses[max(statuses)],
            'source_span_s': (max(statuses) - min(statuses)) / 1e9,
            'all_finite': True, 'fields': ['x:FLOAT32@0', 'y:FLOAT32@4', 'z:FLOAT32@8'],
            'point_step': 12, 'frame': 'slam_world', 'source_stamps_match_input': source_match,
            'snapshots': [dict(c, info=infos[c['source_ns']]) for c in clouds]}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--source-bag', type=Path)
    args = parser.parse_args()
    print(json.dumps(verify(args.bag, args.source_bag), indent=2))
