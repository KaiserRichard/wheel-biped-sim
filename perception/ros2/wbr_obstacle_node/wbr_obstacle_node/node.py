"""Thin ROS 2 message adapter. All perception and policy live in core.py."""
import json
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Float64MultiArray, String, UInt8MultiArray
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs_py import point_cloud2

from .core import (AdvisoryCore, CoreParams, FrameRateLimiter,
                   depth_to_metres, pack_terrain_advisory)


class ObstacleNode(Node):
    def __init__(self):
        super().__init__('wbr_obstacle_node')
        defaults = CoreParams()
        self.declare_parameter('algo_path', '')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('advisory_topic', '/terrain_advisory')
        # Ignore obstacles whose nearest face is beyond this range (0 = off), e.g. the wall past a short table.
        self.declare_parameter('range_limit_m', 0.0)
        self.declare_parameter('range_hysteresis_m', 0.10)
        for name, value in vars(defaults).items():
            self.declare_parameter(name, value)
        params = CoreParams(**{name: self.get_parameter(name).value for name in vars(defaults)})
        algo_path = self.get_parameter('algo_path').value
        if not algo_path:
            raise RuntimeError('Set algo_path to repository root containing tools/obstacle_decision')
        self.core = AdvisoryCore(algo_path, params)
        self.limiter = FrameRateLimiter(params.max_rate_hz)
        self.K = None
        advisory_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.advisory_pub = self.create_publisher(String, '~/advisory', 10)
        self.terrain_advisory_pub = self.create_publisher(
            Float64MultiArray, self.get_parameter('advisory_topic').value, advisory_qos)
        self.bytes_pub = self.create_publisher(UInt8MultiArray, '~/advisory_bytes', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '~/markers', 10)
        self.points_pub = self.create_publisher(PointCloud2, '~/obstacle_points', 2)
        self.create_subscription(CameraInfo, self.get_parameter('info_topic').value, self.info_cb, 10)
        # Keep only the latest unread image; the callback gate drops excess rate.
        self.create_subscription(Image, self.get_parameter('depth_topic').value, self.depth_cb, 1)

    def info_cb(self, msg):
        self.K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)

    def depth_cb(self, msg):
        if not rclpy.ok():
            return
        receipt_monotonic = time.monotonic()
        if not self.limiter.accept(time.monotonic()) or self.K is None:
            return
        try:
            dtype = np.uint16 if msg.encoding.upper() == '16UC1' else np.float32
            image = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
            depth = depth_to_metres(image, msg.encoding)
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            frame_id = msg.header.frame_id
            limit = float(self.get_parameter('range_limit_m').value)
            out = self.core.process(depth, self.K, frame_stamp_ns=stamp_ns, frame_id=frame_id,
                                    range_limit_m=limit if limit > 0 else None,
                                    range_hysteresis=float(self.get_parameter('range_hysteresis_m').value))
            text = String(); text.data = json.dumps(out['advisory'], separators=(',', ':'), allow_nan=False)
            terrain_msg = Float64MultiArray()
            terrain_msg.data = pack_terrain_advisory(
                out['advisory'], out['sequence'], time.monotonic() - receipt_monotonic)
            if not rclpy.ok():
                return
            self.terrain_advisory_pub.publish(terrain_msg)
            if not rclpy.ok():
                return
            self.advisory_pub.publish(text)
            if not rclpy.ok():
                return
            byte_msg = UInt8MultiArray(); byte_msg.data = list(out['advisory_bytes']); self.bytes_pub.publish(byte_msg)
            if not rclpy.ok():
                return
            self.markers_pub.publish(self.to_markers(out['markers'], msg.header))
            if self.core.params.publish_obstacle_points:
                cloud = point_cloud2.create_cloud(msg.header,
                    [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                     PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                     PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)],
                    out['points'].tolist())
                if not rclpy.ok():
                    return
                self.points_pub.publish(cloud)
        except (ValueError, IndexError) as exc:
            self.get_logger().warning(f'dropping malformed depth frame: {exc}')
        except Exception:
            if rclpy.ok():
                raise  # only swallow publish failures during Ctrl-C shutdown

    @staticmethod
    def to_markers(specs, header):
        result = MarkerArray()
        types = {'TRIANGLE_LIST': Marker.TRIANGLE_LIST, 'LINE_LIST': Marker.LINE_LIST,
                 'CUBE': Marker.CUBE, 'TEXT_VIEW_FACING': Marker.TEXT_VIEW_FACING}
        for spec in specs:
            m = Marker(); m.header = header; m.ns = spec['ns']; m.id = spec['id']
            m.type = types[spec['type']]; m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            pos = spec.get('position', spec.get('center', [0,0,0]))
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in pos)
            scale = spec['scale']; m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
            m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in spec['color'])
            m.text = spec.get('text', '')
            for xyz in spec.get('points', []):
                from geometry_msgs.msg import Point
                # ROS fields need Python floats, not numpy scalars.
                pt = Point(); pt.x, pt.y, pt.z = (float(v) for v in xyz); m.points.append(pt)
            result.markers.append(m)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObstacleNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.try_shutdown()
