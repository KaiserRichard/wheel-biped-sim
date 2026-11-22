"""Small rosbag2-to-MCAP integration test for the offline demo converter."""
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from mcap.reader import make_reader
from rosbags.rosbag2 import Writer as RosbagWriter
from rosbags.typesys import Stores, get_typestore

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.bag_tools.foxglove_demo import convert
from tools.bag_tools.foxglove_demo import _transform_matrix


def _stamp(header):
    return (header.stamp.sec, header.stamp.nanosec, header.frame_id)


def _tiny_rosbag_gets_stamped_overlay(tmp_path):
    ts = get_typestore(Stores.ROS2_JAZZY)
    t = ts.types
    bag = tmp_path / 'tiny'
    writer = RosbagWriter(bag, version=9)
    writer.open()
    conns = {}
    for topic, kind in (
        ('/camera/camera/color/image_raw', 'sensor_msgs/msg/Image'),
        ('/camera/camera/aligned_depth_to_color/image_raw', 'sensor_msgs/msg/Image'),
        ('/camera/camera/color/camera_info', 'sensor_msgs/msg/CameraInfo'),
        ('/camera/camera/aligned_depth_to_color/camera_info', 'sensor_msgs/msg/CameraInfo'),
        ('/tf_static', 'tf2_msgs/msg/TFMessage'),
    ):
        conns[topic] = writer.add_connection(topic, kind, typestore=ts)

    time_msg = t['builtin_interfaces/msg/Time'](1, 234)
    hdr = t['std_msgs/msg/Header'](time_msg, 'camera_color_optical_frame')
    cam_hdr = t['std_msgs/msg/Header'](time_msg, 'camera_color_optical_frame')
    camera_info = t['sensor_msgs/msg/CameraInfo'](
        cam_hdr, 4, 4, 'plumb_bob', np.zeros(5, dtype=np.float64),
        np.array([100., 0., 1.5, 0., 100., 1.5, 0., 0., 1.]),
        np.eye(3).reshape(-1), np.zeros(12), 0, 0,
        t['sensor_msgs/msg/RegionOfInterest'](0, 0, 0, 0, False),
    )
    color = t['sensor_msgs/msg/Image'](hdr, 4, 4, 'rgb8', 0, 12,
                                       np.zeros(48, dtype=np.uint8))
    depth_bytes = np.full((4, 4), 1000, dtype='<u2').view(np.uint8).reshape(-1)
    depth = t['sensor_msgs/msg/Image'](hdr, 4, 4, '16UC1', 0, 8, depth_bytes)
    now = 1_000_000_000
    writer.write(conns['/camera/camera/color/camera_info'], now, ts.serialize_cdr(camera_info, 'sensor_msgs/msg/CameraInfo'))
    writer.write(conns['/camera/camera/aligned_depth_to_color/camera_info'], now, ts.serialize_cdr(camera_info, 'sensor_msgs/msg/CameraInfo'))
    writer.write(conns['/camera/camera/color/image_raw'], now + 1, ts.serialize_cdr(color, 'sensor_msgs/msg/Image'))
    writer.write(conns['/camera/camera/aligned_depth_to_color/image_raw'], now + 2, ts.serialize_cdr(depth, 'sensor_msgs/msg/Image'))
    def stamped(parent, child, xyz=(0., 0., 0.), q=(0., 0., 0., 1.)):
        transform = t['geometry_msgs/msg/Transform'](
            t['geometry_msgs/msg/Vector3'](*xyz), t['geometry_msgs/msg/Quaternion'](*q))
        return t['geometry_msgs/msg/TransformStamped'](
            t['std_msgs/msg/Header'](time_msg, parent), child, transform)

    tf = t['tf2_msgs/msg/TFMessage']([
        stamped('camera_link', 'camera_depth_frame'),
        stamped('camera_depth_frame', 'camera_depth_optical_frame',
                q=(-.5, .5, -.5, .5)),
        stamped('camera_link', 'camera_color_frame', (-.0002249468, .01472403, -.0001190947),
                (.0032448, .0036862, -.0030603, .99998325)),
        stamped('camera_color_frame', 'camera_color_optical_frame', q=(-.5, .5, -.5, .5)),
    ])
    writer.write(conns['/tf_static'], now + 3, ts.serialize_cdr(tf, 'tf2_msgs/msg/TFMessage'))
    writer.close()

    output = tmp_path / 'tiny.mcap'
    result = convert(bag, output)
    assert result['message_counts']['/wbr_obstacle_node/advisory'] == 1
    assert result['message_counts']['/wbr_obstacle_node/markers'] == 1
    assert result['message_counts']['/wbr_obstacle_node/obstacle_points'] == 1
    assert result['message_counts']['/wbr_obstacle_node/depth_points'] == 1
    assert result['message_counts']['/tf'] == 1
    assert output.stat().st_size > 0

    observed = {}
    static_transforms = []
    with output.open('rb') as stream:
        for schema, channel, message in make_reader(stream).iter_messages():
            if channel.topic in ('/wbr_obstacle_node/markers', '/wbr_obstacle_node/obstacle_points',
                                  '/wbr_obstacle_node/depth_points'):
                decoded = ts.deserialize_cdr(message.data, schema.name)
                if channel.topic.endswith('/markers'):
                    observed['markers'] = _stamp(decoded.markers[0].header)
                elif channel.topic.endswith('/obstacle_points'):
                    observed['points'] = _stamp(decoded.header)
                else:
                    observed['depth_points'] = _stamp(decoded.header)
                    observed['depth_fields'] = [field.name for field in decoded.fields]
            elif channel.topic == '/wbr_obstacle_node/advisory':
                observed['advisory'] = ts.deserialize_cdr(message.data, schema.name).data
            elif channel.topic == '/tf':
                tf = ts.deserialize_cdr(message.data, schema.name)
                observed['tf'] = tf.transforms[0]
            elif channel.topic == '/tf_static':
                static_transforms.extend(ts.deserialize_cdr(message.data, schema.name).transforms)
    assert observed['markers'] == (1, 234, 'camera_color_optical_frame')
    assert observed['points'] == observed['markers']
    assert observed['depth_points'] == observed['markers']
    assert observed['depth_fields'] == ['x', 'y', 'z', 'height']
    assert (observed['tf'].header.frame_id, observed['tf'].child_frame_id,
            observed['tf'].header.stamp.sec, observed['tf'].header.stamp.nanosec) == (
                'wbr_ground', 'camera_link', 1, 234)
    assert 'plane_valid' in observed['advisory']

    parents = {}
    for transform in static_transforms + [observed['tf']]:
        child, parent = transform.child_frame_id, transform.header.frame_id
        assert child not in parents or parents[child] == parent
        parents[child] = parent
    assert parents == {
        'camera_link': 'wbr_ground',
        'camera_depth_frame': 'camera_link',
        'camera_depth_optical_frame': 'camera_depth_frame',
        'camera_color_frame': 'camera_link',
        'camera_color_optical_frame': 'camera_color_frame',
    }
    for frame in (observed['markers'][2], observed['points'][2], observed['depth_points'][2],
                  'camera_depth_optical_frame', 'camera_color_optical_frame'):
        visited = set()
        while frame != 'wbr_ground':
            assert frame in parents and frame not in visited
            visited.add(frame)
            frame = parents[frame]

    static_by_child = {transform.child_frame_id: transform for transform in static_transforms}
    link_from_depth_optical = (_transform_matrix(static_by_child['camera_depth_frame'].transform)
                               @ _transform_matrix(static_by_child['camera_depth_optical_frame'].transform))
    ground_from_link = _transform_matrix(observed['tf'].transform)
    floor_in_depth_optical = np.array([0., .3, 0., 1.])
    assert abs((ground_from_link @ link_from_depth_optical @ floor_in_depth_optical)[2]) < 1e-6


class FoxgloveDemoTests(unittest.TestCase):
    def test_tiny_rosbag_gets_stamped_overlay(self):
        with tempfile.TemporaryDirectory() as d:
            _tiny_rosbag_gets_stamped_overlay(Path(d))


if __name__ == "__main__":
    unittest.main()
