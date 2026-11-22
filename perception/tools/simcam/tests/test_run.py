import csv
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from sim.models import make_wbr_3d_simple as gen
from tools.ground_plane.synth import Box, render
from tools.jump_gate.gate import Phase
from tools.jump_gate.timeline import synthetic_pose
from tools.obstacle_decision.decide import Advisory
from tools.simcam.run import (
    CSV_FIELDS, PerceptionLoop, advisory_match, camera_measurement, expected_at,
    pose_at, record_frame, tick_times, timeline_row,
)
from tools.simcam.scenario import CAMERA_PITCH_DEG, intrinsics, load_scenario


def example():
    return load_scenario(Path(__file__).resolve().parents[3] / "docs/simcam/example_box132.json")


class TimelineTests(unittest.TestCase):
    def test_ticks_include_aligned_end_without_an_off_grid_extra_tick(self):
        self.assertEqual(tick_times(0.3), [0, 0.1, 0.2, 0.3])
        self.assertEqual(tick_times(0.35), [0, 0.1, 0.2, 0.3])
        self.assertEqual(tick_times(0.04), [0])
        self.assertEqual(len(tick_times(6)), 61)
        for duration, hz in ((0, 10), (1, 0), (float("nan"), 10), (1, float("inf"))):
            with self.subTest(duration=duration, hz=hz), self.assertRaises(ValueError):
                tick_times(duration, hz)

    def test_expected_intervals_boundaries_gaps_and_end(self):
        value = example()
        value["expected"] = [
            dict(t_from=0, t_to=1, advisory="NONE"),
            dict(t_from=1, t_to=2, advisory="UNCERTAIN"),
            dict(t_from=5, t_to=6, advisory="STOP_CANDIDATE"),
        ]
        self.assertEqual(expected_at(value, 0), "NONE")
        self.assertEqual(expected_at(value, 1), "UNCERTAIN")
        self.assertIsNone(expected_at(value, 2))
        self.assertIsNone(expected_at(value, 4.9))
        self.assertEqual(expected_at(value, 6), "STOP_CANDIDATE")
        self.assertIsNone(expected_at(value, 6.1))
        self.assertTrue(advisory_match("NONE", Advisory.NONE))
        self.assertFalse(advisory_match("STOP_CANDIDATE", Advisory.JUMP_CANDIDATE))
        self.assertEqual(advisory_match(None, Advisory.UNCERTAIN), "")

    def test_jump_phase_contact_mapping_is_the_existing_demo(self):
        robot = example()["robot"]
        robot["jump_at_s"] = 1.0
        for t in (0, 1, 1.2, 1.45, 1.6, 2.08, 2.2, 3):
            self.assertEqual(pose_at(t, robot), synthetic_pose(t, 1))
        self.assertEqual(pose_at(1, robot).phase, "CROUCH")
        self.assertEqual(pose_at(1.6, robot).phase, "FLIGHT")

    def test_nondefault_neutral_length_is_continuous_at_jump_endpoints(self):
        robot = example()["robot"]
        robot.update(l0=0.15, jump_at_s=1)
        self.assertEqual(pose_at(0, robot).leg_length_m, 0.15)
        self.assertAlmostEqual(pose_at(1, robot).leg_length_m, 0.15)
        for boundary in (1.4, 1.52, 2.07, 2.11, 2.51):
            a, b = pose_at(boundary - 1e-7, robot), pose_at(boundary + 1e-7, robot)
            self.assertLess(abs(a.leg_length_m - b.leg_length_m), 1e-5)
            self.assertLess(abs(a.vertical_offset_m - b.vertical_offset_m), 1e-5)
        robot["jump_at_s"] = None
        self.assertEqual(pose_at(10, robot).leg_length_m, 0.15)

    def test_camera_pitch_is_positive_down_from_actual_extrinsic(self):
        pitch = math.radians(12)
        right, up = (0, -1, 0), (math.sin(pitch), 0, math.cos(pitch))
        rotation = np.column_stack((right, up, np.cross(right, up)))
        data = SimpleNamespace(cam_xmat=np.array([rotation.ravel()]),
                               cam_xpos=np.array([[1, 0, 0.4]]))
        height, degrees = camera_measurement(data, 0)
        self.assertEqual(height, 0.4)
        self.assertAlmostEqual(degrees, 12)


class RealDetectorIntegrationTests(unittest.TestCase):
    """Analytic arrays exercise the REAL detector/decision/gate; no GL or MuJoCo."""

    @staticmethod
    def depth(pose, factor=4):
        K = intrinsics()
        K[:2] /= factor
        height = gen.R_WHEEL + pose.leg_length_m + gen.HIP_BELOW_TOP + pose.vertical_offset_m
        depth, _, _ = render(
            height=height, pitch=CAMERA_PITCH_DEG, width=640 // factor, height_px=480 // factor,
            fx=K[0, 0], fy=K[1, 1], cx=K[0, 2], cy=K[1, 2],
            boxes=(Box((0, 1.15, 0.066), (0.30, 0.20, 0.132)),))
        return depth, K

    def test_real_detector_recovers_132mm_box_and_gate_releases_after_dwell(self):
        scenario = example()
        pose = pose_at(0, scenario["robot"])
        depth, K = self.depth(pose, factor=1)
        loop = PerceptionLoop(K)
        outputs = [loop.update(t, depth, pose, 0.25) for t in tick_times(0.6)]
        self.assertEqual(outputs[0][2].phase, Phase.SETTLE)
        frame, decision, gated, reset = outputs[-1]
        self.assertTrue(frame.plane_valid)
        self.assertAlmostEqual(frame.nearest.height_m, 0.132, delta=0.005)
        self.assertAlmostEqual(frame.nearest.distance_m, 1.05, delta=0.015)
        self.assertEqual(decision.advisory, Advisory.STOP_CANDIDATE)
        self.assertEqual(gated.advisory, Advisory.STOP_CANDIDATE)
        self.assertFalse(reset)
        row = timeline_row(scenario, 0.6, 0.15, 0.2103, 3, pose,
                           frame, decision, gated, reset, 0)
        self.assertEqual(set(row), set(CSV_FIELDS))
        self.assertTrue(row["match"])
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "timeline.csv"
            with path.open("w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
                writer.writeheader()
                writer.writerow(row)
            with path.open() as stream:
                saved = next(csv.DictReader(stream))
            self.assertEqual(saved["gated_advisory"], "STOP_CANDIDATE")
            record_frame(np.zeros((480, 640, 3), dtype=np.uint8), depth, row,
                         Path(tmp) / "frame.png")
            self.assertTrue((Path(tmp) / "frame.png").is_file())

    def test_jump_inhibits_and_recovers_with_fresh_post_landing_history(self):
        robot = example()["robot"]
        robot["jump_at_s"] = 0.6
        loop = PerceptionLoop(self.depth(pose_at(0, robot))[1])
        phases, resets, last = set(), [], None
        for t in tick_times(2.8):
            pose = pose_at(t, robot)
            depth, _ = self.depth(pose)
            frame, decision, gated, reset = loop.update(t, depth, pose, 0.25)
            phases.add(gated.phase)
            if reset:
                resets.append(t)
            if gated.phase != Phase.STAND:
                self.assertEqual(gated.advisory, Advisory.UNCERTAIN)
                self.assertEqual(gated.confidence, 0)
            last = gated
        self.assertTrue({Phase.CROUCH, Phase.FLIGHT, Phase.LAND, Phase.SETTLE, Phase.STAND} <= phases)
        self.assertEqual(resets, [0.6, 1.7])  # LAND's 40ms interval is skipped at 10Hz.
        self.assertEqual(last.phase, Phase.STAND)
        self.assertEqual(last.advisory, Advisory.STOP_CANDIDATE)


class HistoryTests(unittest.TestCase):
    def test_histories_survive_inhibition_and_reset_only_on_crouch_entry_and_after_flight(self):
        # Supply constant valid geometry without rendering; use the real decision
        # and gate to check confirmation accumulates even while output is inhibited.
        robot = example()["robot"]
        robot["jump_at_s"] = .6
        loop = PerceptionLoop()
        nearest = SimpleNamespace(distance_m=1.0, height_m=.132, width_m=.3)
        frame = SimpleNamespace(plane_valid=True, nearest=nearest)
        detector_counts, resets, phases = [], [], set()

        def process_frame(depth, K, state):
            count = state.get("frames", 0) + 1
            detector_counts.append(count)
            return frame, {"frames": count}

        with patch("tools.simcam.run.detect.process_frame", side_effect=process_frame):
            since_reset = 0
            for t in tick_times(2.8):
                _, decision, gated, reset = loop.update(t, None, pose_at(t, robot), .25)
                if reset:
                    resets.append(t)
                    since_reset = 0
                since_reset += 1
                phases.add(gated.phase)
                self.assertEqual(detector_counts[-1], since_reset)
                self.assertEqual(loop.decision_state["hist"], [True] * min(since_reset, 5))
                self.assertEqual(loop.decision_state["ema"], (1.0, .132, .3))
                if since_reset >= 3:
                    self.assertEqual(decision.advisory, Advisory.STOP_CANDIDATE)
                else:
                    self.assertEqual(decision.advisory, Advisory.UNCERTAIN)
                if gated.phase != Phase.STAND:
                    self.assertEqual(gated.advisory, Advisory.UNCERTAIN)
        self.assertEqual(resets, [.6, 1.7])
        self.assertTrue({Phase.STAND, Phase.CROUCH, Phase.FLIGHT, Phase.LAND, Phase.SETTLE} <= phases)


if __name__ == "__main__":
    unittest.main()
