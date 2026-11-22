import unittest
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from tools.ground_plane.synth import Box, render
from ros2.wbr_planner_node.wbr_planner_node.core import PlannerCore
from tools.local_map.grid import ElevationGrid


FX, FY, CX, CY = 386.953, 386.953, 321.034, 241.040
K = np.array([[FX, 0, CX], [0, FY, CY], [0, 0, 1]], dtype=float)


def pose_at(forward):
    # Synth renderer's world axes: right, forward, up. Camera at 0.4 m.
    T = np.eye(4)
    T[:3, :3] = np.array([[1., 0., 0.], [0., 0., 1.], [0., -1., 0.]])
    T[:3, 3] = [0., forward, .4]
    return T


class PlannerCoreTest(unittest.TestCase):
    def frames(self, box):
        core = PlannerCore(stride=3, resolution_m=.05)
        actions = []
        elapsed = []
        for i, forward in enumerate(np.linspace(0, .35, 8)):
            boxes = [Box((0., 1.5, .075), (.15, .15, .15))] if box else []
            depth, _, _ = render(boxes=boxes, max_range=4.5)
            t0 = time.perf_counter()
            out = core.update(depth, K, pose_at(float(forward)), int(i*66_666_667))
            elapsed.append((time.perf_counter()-t0)*1000)
            actions.append(out)
        return actions, elapsed

    def test_box_prompts_lateral_shift(self):
        outputs, elapsed = self.frames(True)
        final = outputs[-1]
        self.assertIn(final['action'], ('SHIFT_LEFT', 'SHIFT_RIGHT'))
        self.assertGreaterEqual(abs(final['target_offset_m']), .30)
        print(f"box sequence core runtime ms/frame: mean={np.mean(elapsed):.2f}, max={np.max(elapsed):.2f}")

    def test_clear_path_goes_straight(self):
        outputs, elapsed = self.frames(False)
        self.assertEqual(outputs[-1]['action'], 'GO_STRAIGHT')
        print(f"clear sequence core runtime ms/frame: mean={np.mean(elapsed):.2f}, max={np.max(elapsed):.2f}")

    def test_missing_pose_and_jump_are_skipped(self):
        depth, _, _ = render()
        core = PlannerCore(stride=3)
        self.assertEqual(core.update(depth, K, None, 0)['action'], 'NO_POSE')
        self.assertEqual(core.update(depth, K, pose_at(0.), 1_000_000_000)['action'], 'GO_STRAIGHT')
        jump = pose_at(2.)
        self.assertEqual(core.update(depth, K, jump, 1_066_666_667)['action'], 'NO_POSE')
        self.assertAlmostEqual(core.previous_pose[1, 3], 0.)

    def test_initial_bad_floor_reports_no_plane(self):
        core = PlannerCore(stride=3)
        blank = np.zeros((480, 640), dtype=np.float32)
        self.assertEqual(core.update(blank, K, pose_at(0.), 1)['action'], 'NO_PLANE')

    def test_discontinuity_requires_consistent_recovery_and_reinitializes_map(self):
        depth, _, _ = render()
        core = PlannerCore(stride=3)
        core.update(depth, K, pose_at(0.), 1_000_000_000)
        self.assertEqual(core.update(depth, K, pose_at(2.), 1_066_666_667)['action'], 'NO_POSE')
        # Even after the old timestamp gap would have made the jump appear slow,
        # one locally consistent sample is insufficient to resume fusion.
        self.assertEqual(core.update(depth, K, pose_at(2.), 3_100_000_000)['action'], 'NO_POSE')
        self.assertEqual(core.update(depth, K, pose_at(2.), 3_166_666_667)['action'], 'GO_STRAIGHT')
        self.assertFalse(core.discontinuity)
        self.assertAlmostEqual(core.previous_pose[1, 3], 2.0)

    def test_swept_shift_rejects_obstacle_between_current_and_lane(self):
        core = PlannerCore(grid_size_m=6., resolution_m=.05)
        core.grid = ElevationGrid(6., .05)
        obstacle = np.zeros((core.grid.n, core.grid.n), dtype=bool)
        known = np.ones_like(obstacle)
        row, col = core.grid._index(np.array([[.025, .30]]))
        obstacle[row[0], col[0]] = True
        advice = core._choose(np.array([0., .60, 0.]), {"obstacle": obstacle, "known": known})
        self.assertNotEqual(advice['action'], 'SHIFT_RIGHT')

    def test_goal_is_terminal_even_when_grid_is_occupied(self):
        core = PlannerCore()
        core.grid = ElevationGrid(6., .05)
        occupied = np.ones((core.grid.n, core.grid.n), dtype=bool)
        advice = core._choose(np.array([3.1, 0., 0.]), {"obstacle": occupied, "known": occupied})
        self.assertEqual(advice['action'], 'GOAL_REACHED')
        self.assertEqual(advice['distance_to_goal_m'], 0.)


if __name__ == '__main__':
    unittest.main()
