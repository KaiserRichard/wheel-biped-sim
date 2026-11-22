from __future__ import annotations
import json
import tempfile
import unittest
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rosbags.rosbag2 import Writer, StoragePlugin
from rosbags.typesys import Stores, get_typestore

from orb_checks import (POSE_TOPIC, STATUS_TOPIC, l1_metrics, read_bag,
                        tracking_metrics, trajectory_metrics)


class BagFixture:
    def __init__(self, path: Path):
        self.path = path
        self.store = get_typestore(Stores.ROS2_JAZZY)

    def write(self, positions: list[tuple[float, float, float]], states: list[int]) -> None:
        types = self.store.types
        PoseStamped = types["geometry_msgs/msg/PoseStamped"]
        Header = types["std_msgs/msg/Header"]
        Time = types["builtin_interfaces/msg/Time"]
        Pose = types["geometry_msgs/msg/Pose"]
        Point = types["geometry_msgs/msg/Point"]
        Quaternion = types["geometry_msgs/msg/Quaternion"]
        String = types["std_msgs/msg/String"]
        with Writer(self.path, version=9, storage_plugin=StoragePlugin.SQLITE3) as writer:
            pose_conn = writer.add_connection(POSE_TOPIC, "geometry_msgs/msg/PoseStamped", typestore=self.store)
            status_conn = writer.add_connection(STATUS_TOPIC, "std_msgs/msg/String", typestore=self.store)
            times = sorted({i * 100_000_000 for i in range(len(positions))} |
                           {i * 100_000_000 for i in range(len(states))})
            for i, xyz in enumerate(positions):
                ns = 2_000_000_000 + i * 100_000_000
                sec, nanosec = divmod(ns, 1_000_000_000)
                msg = PoseStamped(Header(Time(sec, nanosec), "slam_world"),
                                  Pose(Point(*xyz), Quaternion(0.0, 0.0, 0.0, 1.0)))
                writer.write(pose_conn, ns + 1, self.store.serialize_cdr(msg, "geometry_msgs/msg/PoseStamped"))
            for i, state in enumerate(states):
                ns = 2_000_000_000 + i * 100_000_000
                msg = String(json.dumps({"tracking_state": state, "pose_published": state == 2}))
                writer.write(status_conn, ns + 2, self.store.serialize_cdr(msg, "std_msgs/msg/String"))


def straight_positions() -> list[tuple[float, float, float]]:
    # 1.0 s stationary, 3.0 m over 3.0 s, then 1.0 s stationary at 10 Hz.
    result = []
    for i in range(51):
        if i <= 10:
            x = 0.0
        elif i <= 40:
            x = 0.1 * (i - 10)
        else:
            x = 3.0
        result.append((x, 0.0, 0.0))
    return result


class OrbChecksTests(unittest.TestCase):
    def test_three_metre_line_with_stationary_ends(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "line-bag"
            BagFixture(bag).write(straight_positions(), [2] * 51)
            data = read_bag(bag)
            result = l1_metrics(data["poses"], 3.0)
            self.assertAlmostEqual(result["start_end_distance_m"], 3.0, delta=0.001)
            self.assertAlmostEqual(result["path_length_m"], 3.0, delta=0.001)

    def test_inserted_pose_jump_is_detected(self):
        poses = [{"t_ns": i * 100_000_000, "x": x, "y": 0.0, "z": 0.0}
                 for i, x in enumerate((0.0, 0.01, 0.02, 0.52, 0.53))]
        result = trajectory_metrics(poses)
        self.assertEqual(len(result["pose_jumps"]), 1)
        self.assertGreater(result["pose_jumps"][0]["speed_mps"], 1.0)

    def test_status_counting_from_bag(self):
        with tempfile.TemporaryDirectory() as tmp:
            bag = Path(tmp) / "status-bag"
            BagFixture(bag).write(straight_positions()[:3], [2, 3, 4, 2])
            counts = tracking_metrics(read_bag(bag)["statuses"])
            self.assertEqual(counts["state_counts"], {"OK": 2, "RECENTLY_LOST": 1, "LOST": 1})
            self.assertEqual(counts["tracking_ok_percent"], 50.0)
            self.assertEqual(counts["lost_transition_count"], 2)


if __name__ == "__main__":
    unittest.main()
