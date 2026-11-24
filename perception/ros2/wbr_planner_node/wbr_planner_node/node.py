"""Thin ROS 2 adapter around the ROS-independent NumPy planner core."""
import json
import math
import time
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster

from .core import PlannerCore
from .adapter_logic import (PendingFrame, core_parameters, current_arrow_geometry,
                            nearest_pose)


class PlannerNode(Node):
    def __init__(self):
        super().__init__('wbr_planner_node')
        defaults = {"max_rate_hz": 3.0, "goal_forward_m": 3.0,
                    "robot_half_width_m": 0.25, "grid_size_m": 6.0,
                    "resolution_m": 0.05, "stride": 4}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('pose_topic', '/d435_orb/pose')
        p = {k: self.get_parameter(k).value for k in defaults}
        self.core = PlannerCore(**core_parameters(p))
        self.max_rate_hz = float(p['max_rate_hz'])
        if self.max_rate_hz <= 0:
            raise ValueError('max_rate_hz must be positive')
        self.last_process_mono = -math.inf
        self.K = None
        self.poses = deque(maxlen=100)
        self.pending_frame = None
        self.suggestion_pub = self.create_publisher(String, '~/suggestion', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '~/markers', 2)
        self.grid_pub = self.create_publisher(OccupancyGrid, '~/grid', 2)
        self.tf_pub = TransformBroadcaster(self)
        self.create_subscription(CameraInfo, self.get_parameter('info_topic').value, self.info_cb, 5)
        # Keep only the newest image while the callback is busy.
        self.create_subscription(Image, self.get_parameter('depth_topic').value, self.depth_cb, 1)
        self.create_subscription(PoseStamped, self.get_parameter('pose_topic').value, self.pose_cb, 100)
        self.create_timer(0.005, self.poll_pending)

    def info_cb(self, msg):
        try:
            self.K = np.asarray(msg.k, dtype=float).reshape(3, 3)
        except Exception as exc:
            self.get_logger().warning(f'invalid camera_info: {exc}')

    def pose_cb(self, msg):
        try:
            stamp = int(msg.header.stamp.sec)*1_000_000_000 + int(msg.header.stamp.nanosec)
            p, q = msg.pose.position, msg.pose.orientation
            qx, qy, qz, qw = map(float, (q.x, q.y, q.z, q.w))
            norm = math.sqrt(qx*qx+qy*qy+qz*qz+qw*qw)
            if not math.isfinite(norm) or norm < 1e-12:
                return
            qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
            T = np.eye(4)
            T[:3, :3] = [[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                         [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                         [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]]
            T[:3, 3] = [float(p.x), float(p.y), float(p.z)]
            if np.isfinite(T).all():
                self.poses.append((stamp, T))
        except Exception as exc:
            self.get_logger().warning(f'invalid pose: {exc}')

    def _nearest_pose(self, stamp):
        return nearest_pose(self.poses, stamp)

    def depth_cb(self, msg):
        now = time.monotonic()
        if now - self.last_process_mono < 1.0/self.max_rate_hz or self.pending_frame is not None:
            return
        self.last_process_mono = now
        stamp = int(msg.header.stamp.sec)*1_000_000_000 + int(msg.header.stamp.nanosec)
        # One bounded pending frame lets pose_cb receive the ORB result that
        # follows the source image. Additional depth frames are dropped.
        self.pending_frame = PendingFrame(msg, stamp, now)

    def poll_pending(self):
        if not rclpy.ok():
            self.pending_frame = None
            return
        if self.pending_frame is None:
            return
        pending = self.pending_frame
        msg, stamp = pending.message, pending.stamp_ns
        pose = self._nearest_pose(stamp)
        if pending.should_wait(pose, time.monotonic()):
            return
        self.pending_frame = None
        try:
            if msg.encoding.upper() != '16UC1':
                raise ValueError(f'expected aligned 16UC1 depth, got {msg.encoding}')
            raw = np.frombuffer(msg.data, dtype=np.dtype('>u2' if msg.is_bigendian else '<u2'))
            depth_u16 = raw.reshape(int(msg.height), int(msg.step)//2)[:, :int(msg.width)]
            depth_m = depth_u16.astype(np.float32)*0.001
            if self.K is None:
                out = self.core.update(depth_m, np.eye(3), None, stamp)
            else:
                out = self.core.update(depth_m, self.K, pose, stamp)
            suggestion = String()
            suggestion.data = json.dumps(out, separators=(',', ':'), allow_nan=False)
            if not rclpy.ok():
                return
            self.suggestion_pub.publish(suggestion)
            if self.core.grid is not None:
                self.publish_map(msg, out)
        except Exception as exc:
            if not rclpy.ok():  # publish after Ctrl-C shutdown: exit quietly
                return
            self.get_logger().error(f'planner frame failed; continuing: {type(exc).__name__}: {exc}')

    def publish_map(self, image, out):
        if not rclpy.ok():
            return
        header = image.header
        header.frame_id = 'wbr_map'
        markers = MarkerArray()
        action = out['action']
        color = (0.1, 0.8, 0.2, 0.9) if action == 'GO_STRAIGHT' else (1., .65, 0., .95)
        markers.markers.extend(self._basic_markers(out, color))
        if not rclpy.ok():
            return
        self.markers_pub.publish(markers)
        occupancy = self.core.occupancy(out['stamp_ns'])
        if occupancy:
            grid = OccupancyGrid(); grid.header = header
            grid.info.resolution = float(occupancy['resolution'])
            grid.info.width = int(occupancy['width']); grid.info.height = int(occupancy['height'])
            grid.info.origin.position.x = float(occupancy['origin_x'])
            grid.info.origin.position.y = float(occupancy['origin_y'])
            grid.info.origin.orientation.w = 1.0
            grid.data = [int(v) for v in occupancy['data']]
            if not rclpy.ok():
                return
            self.grid_pub.publish(grid)
        if not rclpy.ok():
            return
        self.publish_tf(header)

    def _basic_markers(self, out, color):
        result = []
        def marker(ns, ident, typ, pos, scale, rgba, points=()):
            m = Marker(); m.header.frame_id = 'wbr_map'; m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type, m.action = ns, int(ident), typ, Marker.ADD
            m.pose.orientation.w = 1.0
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in pos)
            m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
            m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in rgba)
            for xyz in points:
                q = Point(); q.x, q.y, q.z = (float(v) for v in xyz); m.points.append(q)
            return m
        result.append(marker('goal', 0, Marker.SPHERE, [self.core.goal_forward_m, 0, .08], [.15,.15,.15], [0.,1.,0.,1.]))
        start = self.core.start_position
        result.append(marker('start', 1, Marker.SPHERE, [float(start[0]),float(start[1]),.08], [.12,.12,.12], [0.,0.,1.,1.]))
        if out.get('current_lateral_m') is not None:
            x = self.core.previous_pose
            cur = self.core._camera_map_position(x)
            marker_pos, arrow_points = current_arrow_geometry(cur[:2])
            result.append(marker('current', 2, Marker.ARROW, marker_pos, [.35,.06,.06], [1.,1.,0.,1.], arrow_points))
            target = float(out['target_offset_m'])
            half = self.core.robot_half_width_m
            result.append(marker('chosen_corridor', 3, Marker.LINE_LIST, [0,0,0], [.025,.0,.0], color,
                                 [[float(cur[0]),target-half,.03],[self.core.goal_forward_m,target-half,.03],
                                  [float(cur[0]),target+half,.03],[self.core.goal_forward_m,target+half,.03]]))
        pts = out.get('nearest_obstacle_cells', [])
        if pts:
            m = marker('obstacles', 4, Marker.CUBE_LIST, [0,0,0], [.05,.05,.05], [1.,0.,0.,.85])
            for c in pts:
                q = Point(); q.x, q.y, q.z = float(c['x']), float(c['y']), .05; m.points.append(q)
            result.append(m)
        return result

    def publish_tf(self, header):
        if not rclpy.ok():
            return
        # T_map_world maps child coordinates in slam_world into parent
        # coordinates in wbr_map, which is the transform TF expects here.
        T = self.core.T_map_world
        R = T[:3, :3]
        qw = math.sqrt(max(0., 1.+R[0,0]+R[1,1]+R[2,2]))*.5
        qx = math.copysign(math.sqrt(max(0.,1.+R[0,0]-R[1,1]-R[2,2]))*.5, R[2,1]-R[1,2])
        qy = math.copysign(math.sqrt(max(0.,1.-R[0,0]+R[1,1]-R[2,2]))*.5, R[0,2]-R[2,0])
        qz = math.copysign(math.sqrt(max(0.,1.-R[0,0]-R[1,1]+R[2,2]))*.5, R[1,0]-R[0,1])
        tf = TransformStamped(); tf.header.stamp = header.stamp
        tf.header.frame_id = 'wbr_map'; tf.child_frame_id = 'slam_world'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (float(v) for v in T[:3,3])
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float,(qx,qy,qz,qw))
        self.tf_pub.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PlannerNode()
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
