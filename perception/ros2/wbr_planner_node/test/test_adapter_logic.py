import unittest
from pathlib import Path

from ros2.wbr_planner_node.wbr_planner_node.adapter_logic import (
    PENDING_POSE_TIMEOUT_S, PendingFrame, core_parameters,
    current_arrow_geometry, nearest_pose,
)
from ros2.wbr_planner_node.wbr_planner_node.core import PlannerCore


class PlannerAdapterLogicTest(unittest.TestCase):
    def test_adapter_rate_parameter_is_not_passed_to_core(self):
        params = {"max_rate_hz": 3.0, "goal_forward_m": 3.0,
                  "robot_half_width_m": .25, "stride": 4}
        core = PlannerCore(**core_parameters(params))
        self.assertEqual(core.goal_forward_m, 3.0)

    def test_source_stamp_tolerance_and_processing_timeout_are_separate(self):
        pose = object()
        stamp = 1_000_000_000
        self.assertIs(nearest_pose([(stamp, pose)], stamp), pose)
        pending = PendingFrame(object(), stamp, 10.0)
        self.assertTrue(pending.should_wait(None, 10.080))
        self.assertFalse(pending.should_wait(pose, 10.080))
        self.assertFalse(pending.should_wait(None, 10.0 + PENDING_POSE_TIMEOUT_S))

    def test_current_arrow_uses_identity_marker_pose_and_absolute_endpoints(self):
        marker_pose, points = current_arrow_geometry((1.0, .2))
        self.assertEqual(marker_pose, (0., 0., 0.))
        self.assertEqual(points[0], (1.0, .2, .05))
        self.assertEqual(points[1], (1.25, .2, .05))

    def test_live_bag_topics_include_grid_and_transform_tree(self):
        root = Path(__file__).resolve().parents[3]
        script = (root / "scripts/camera_day/path_live.sh").read_text()
        for topic in ("/wbr_planner_node/grid", "/tf", "/tf_static"):
            self.assertIn(topic, script)


if __name__ == "__main__":
    unittest.main()
