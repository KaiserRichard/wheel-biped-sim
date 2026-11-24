import math
from pathlib import Path
import tempfile
import unittest

import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

from tools.ground_plane.synth import Box, render
from tools.local_map.from_bag import build_map


class FromBagTest(unittest.TestCase):
    def _make_bag(self, path, *, insert_jump=False):
        store = get_typestore(Stores.ROS2_JAZZY)
        Time = store.types["builtin_interfaces/msg/Time"]
        Header = store.types["std_msgs/msg/Header"]
        Image = store.types["sensor_msgs/msg/Image"]
        CameraInfo = store.types["sensor_msgs/msg/CameraInfo"]
        PoseStamped = store.types["geometry_msgs/msg/PoseStamped"]
        Pose = store.types["geometry_msgs/msg/Pose"]
        Point = store.types["geometry_msgs/msg/Point"]
        Quaternion = store.types["geometry_msgs/msg/Quaternion"]
        Region = store.types["sensor_msgs/msg/RegionOfInterest"]
        fx, fy, cx, cy = 120.0, 120.0, 80.0, 60.0
        topics = ("/camera/camera/aligned_depth_to_color/image_raw",
                  "/camera/camera/color/camera_info", "/d435_orb/pose")
        with Writer(path, version=9) as writer:
            image_conn = writer.add_connection(topics[0], "sensor_msgs/msg/Image", typestore=store)
            info_conn = writer.add_connection(topics[1], "sensor_msgs/msg/CameraInfo", typestore=store)
            pose_conn = writer.add_connection(topics[2], "geometry_msgs/msg/PoseStamped", typestore=store)

            def header(ns, frame):
                return Header(Time(ns // 1_000_000_000, ns % 1_000_000_000), frame)

            stamp0 = 2_000_000_000
            info = CameraInfo(header(stamp0, "camera_color_optical_frame"), 120, 160,
                              "plumb_bob", np.zeros(5),
                              np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1], dtype=float),
                              np.eye(3).reshape(-1), np.array([fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0], dtype=float),
                              0, 0, Region(0, 0, 0, 0, 0, False))
            writer.write(info_conn, stamp0, store.serialize_cdr(info, info_conn.msgtype))
            # Quaternion for a -90 degree rotation about world x.
            q = Quaternion(-math.sqrt(0.5), 0., 0., math.sqrt(0.5))
            nframes = 11
            for i in range(nframes):
                ns = stamp0 + i * 200_000_000
                camera_y = i * 0.1
                z_mm = render(height=0.30, width=160, height_px=120, fx=fx, fy=fy,
                              cx=cx, cy=cy,
                              boxes=[Box((0., 1.5-camera_y, 0.025), (0.24, 0.24, 0.05))],
                              max_range=5.0)[0]
                depth_mm = np.where(z_mm > 0, np.rint(z_mm*1000), 0).astype("<u2")
                image = Image(header(ns, "camera_color_optical_frame"), 120, 160,
                              "16UC1", 0, 160*2,
                              np.frombuffer(depth_mm.tobytes(), dtype=np.uint8).copy())
                pose_y = camera_y + (1.0 if insert_jump and i == 5 else 0.0)
                pose = PoseStamped(header(ns, "slam_world"), Pose(
                    Point(0., pose_y, 0.30), q))
                writer.write(image_conn, ns, store.serialize_cdr(image, image_conn.msgtype))
                writer.write(pose_conn, ns + 1_000_000, store.serialize_cdr(pose, pose_conn.msgtype))

    def test_synthetic_box_height_and_jump_filter(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            bag = root / "synthetic_bag"
            self._make_bag(bag, insert_jump=True)
            result = build_map(bag, root / "maps", stride=2, size_m=4.0,
                               resolution_m=0.025, obstacle_threshold_m=0.025)
            self.assertEqual(result["depth_frames"], 11)
            self.assertGreaterEqual(result["frames_skipped_jump"], 1)
            self.assertGreaterEqual(result["frames_used"], 8)
            self.assertIsNotNone(result["obstacle_centroid_map_xyz_m"])
            # The fused obstacle cells describe the top surface of the 5 cm box.
            self.assertAlmostEqual(result["obstacle_centroid_map_xyz_m"][2], 0.05, delta=0.015)
            self.assertTrue((root / "maps" / "height_map.png").is_file())
            self.assertTrue((root / "maps" / "obstacle_classes.png").is_file())
            self.assertTrue((root / "maps" / "summary.json").is_file())


if __name__ == "__main__":
    unittest.main()
