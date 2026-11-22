import csv
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import mujoco
import numpy as np

from sim.models import make_wbr_3d_simple as gen
from tools.jump_gate.gate import Phase
from tools.simcam import run as simcam
from tools.simcam.scenario import CAMERA_NAME, CAMERA_POS, build_xml, intrinsics, load_scenario
from tools.simcam_imu import run as replay
from tools.simcam_imu.tilt_source import Tilt, TiltSource
from tools.simcam_imu.tests.test_tilt_source import write_synthetic


def example():
    return load_scenario(replay.DEFAULT_SCENARIO)


class GeometryTests(unittest.TestCase):
    def test_axle_clearance_and_rigid_camera_for_both_roll_signs(self):
        scenario = example()
        model = mujoco.MjModel.from_xml_string(build_xml(scenario))
        data = mujoco.MjData(model)
        pose = simcam.pose_at(0, scenario["robot"])
        camera = model.camera(CAMERA_NAME).id
        body = model.body("body").id
        for pitch, roll in ((0, 0), (0.3, 0), (0, 0.25), (0.3, -0.25)):
            with self.subTest(pitch=pitch, roll=roll):
                replay.place_robot(model, data, scenario["robot"], 1, pose,
                                   Tilt(pitch, roll, 0, 0))
                az = math.cos(pitch) * math.sin(roll)
                rim = gen.R_WHEEL * math.sqrt(1 - az * az) + gen.WHEEL_HALF_WIDTH * abs(az)
                wheel_z = [data.xpos[model.body(f"wheel_{side}").id][2]
                           for side in ("left", "right")]
                self.assertAlmostEqual(min(wheel_z) - rim, 0, places=12)
                self.assertAlmostEqual(sum(wheel_z) / 2, replay.axle_height(pitch, roll))
                rotation = data.xmat[body].reshape(3, 3)
                np.testing.assert_allclose(data.cam_xpos[camera] - data.xpos[body],
                                           rotation @ CAMERA_POS, atol=1e-12)
                self.assertAlmostEqual(data.time, 1)
        self.assertAlmostEqual(replay.axle_height(0.8, 0), gen.R_WHEEL)

    def test_zero_tilt_placement_matches_original_simcam(self):
        scenario = example()
        model = mujoco.MjModel.from_xml_string(build_xml(scenario))
        a, b = mujoco.MjData(model), mujoco.MjData(model)
        scenario["robot"]["jump_at_s"] = 1
        for t in (0, 1.2, 1.6, 2.2):
            pose = simcam.pose_at(t, scenario["robot"])
            simcam.place_robot(model, a, scenario["robot"], t, pose)
            replay.place_robot(model, b, scenario["robot"], t, pose, Tilt(0, 0, 0, 0))
            np.testing.assert_allclose(a.mocap_pos, b.mocap_pos, atol=1e-12)
            np.testing.assert_allclose(a.cam_xpos, b.cam_xpos, atol=1e-12)
            np.testing.assert_allclose(a.cam_xmat, b.cam_xmat, atol=1e-12)


class GateIntegrationTests(unittest.TestCase):
    def test_real_detector_gate_uses_filtered_rates_and_recovers(self):
        # Closed-form floor depth, no image/graphics renderer. Pose is immaterial
        # to this rate-adapter check; detector fits an actual plane in the array.
        K = intrinsics()
        K[:2] /= 4
        v, _ = np.indices((120, 160))
        denominator = math.sin(math.radians(3)) + math.cos(math.radians(3)) * (v - K[1, 2]) / K[1, 1]
        depth = np.zeros_like(denominator)
        np.divide(0.21, denominator, out=depth, where=denominator > 0)
        depth[(depth > 6) | (depth <= 0)] = 0
        loop = replay.TiltPerceptionLoop(K)
        pose = simcam.pose_at(0, example()["robot"])
        phases = []
        for t in simcam.tick_times(0.6):
            rate = 8 if t < 0.3 else 0
            frame, _, gated, _ = loop.update(t, depth, pose, 0.25, Tilt(0, 0, rate, 0))
            self.assertTrue(frame.plane_valid)
            if rate:
                self.assertEqual(gated.reason, "unstable_attitude")
            self.assertEqual(len(loop.decision_state["hist"]), min(round(t * 10) + 1, 5))
            phases.append(gated.phase)
        self.assertEqual(phases[0], Phase.SETTLE)
        self.assertEqual(phases[-1], Phase.STAND)
        _, decision, gated, reset = loop.update(.7, depth, pose, .25, Tilt(0, 0, 15, -10))
        self.assertFalse(reset)
        self.assertEqual(gated.phase, Phase.STAND)
        self.assertEqual(gated.advisory, decision.advisory)
        self.assertEqual(gated.reason, "wobble_confidence_scaled")
        _, _, gated, _ = loop.update(.8, depth, pose, .25, Tilt(0, 0, 80, 0))
        self.assertEqual(gated.phase, Phase.SETTLE)
        self.assertEqual(gated.advisory, simcam.decide.Advisory.UNCERTAIN)


def comparison_row(t, height, advisory="NONE", valid=True):
    error = height - 0.132 if valid else math.nan
    return dict(t=t, detector_height_m=height, height_error_m=error,
                abs_height_error_m=abs(error), raw_advisory=advisory,
                gated_advisory=advisory, plane_valid=valid)


class ComparisonTests(unittest.TestCase):
    def test_join_by_time_not_input_order_and_signed_deltas(self):
        result = replay.comparison_join(
            [comparison_row(0.1, 0.134), comparison_row(0, 0.132)],
            [comparison_row(0, 0.131), comparison_row(0.1, 0.133, "UNCERTAIN")])
        self.assertEqual([r["t"] for r in result], [0, 0.1])
        self.assertAlmostEqual(result[1]["height_error_delta_m"], 0.001)
        self.assertAlmostEqual(result[1]["abs_height_error_delta_m"], 0.001)
        self.assertAlmostEqual(result[0]["abs_height_error_delta_m"], -0.001)
        self.assertFalse(result[1]["gated_advisory_agreement"])
        self.assertTrue(result[0]["raw_advisory_agreement"])

    def test_missing_detection_and_invalid_plane_stay_nan(self):
        result = replay.comparison_join([comparison_row(0, math.nan, valid=False)],
                                        [comparison_row(0, 0.132)])[0]
        self.assertTrue(math.isnan(result["height_error_delta_m"]))
        self.assertFalse(result["both_planes_valid"])

    def test_missing_and_duplicate_ticks_fail(self):
        row = comparison_row(0, 0.132)
        for left, right in (([row], []), ([row, row], [row]), ([], [row, row])):
            with self.subTest(left=left, right=right), self.assertRaises(ValueError):
                replay.comparison_join(left, right)

    def test_reference_height_rejects_multiple_targets_and_ramps(self):
        scenario = example()
        self.assertEqual(replay.reference_height(scenario), 0.132)
        scenario["obstacles"] *= 2
        self.assertTrue(math.isnan(replay.reference_height(scenario)))
        scenario["obstacles"] = [dict(type="ramp", rise=0.1)]
        self.assertTrue(math.isnan(replay.reference_height(scenario)))


class RunTests(unittest.TestCase):
    def test_two_run_orchestration_csv_schema_and_metadata_without_graphics(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            write_synthetic(path / "imu.csv")
            source = TiltSource(path / "imu.csv")
            out = path / "timeline.csv"
            scenario = example()
            pose = simcam.pose_at(0, scenario["robot"])
            nearest = SimpleNamespace(height_m=0.133, distance_m=1.0)
            frame = SimpleNamespace(nearest=nearest, plane_valid=True,
                                    plane_reused=False, plane_age=0)
            decision = SimpleNamespace(advisory=simcam.decide.Advisory.STOP_CANDIDATE,
                                       height_m=0.133, distance_m=1.0, confidence=0.9)
            gated = SimpleNamespace(advisory=decision.advisory, confidence=0.9,
                                    phase=Phase.STAND, reason="test")

            def fake_run(scenario, source, *, imu_start, zero_tilt=False, out, record):
                tilt = Tilt(0, 0, 0, 0) if zero_tilt else source.at(imu_start)
                row = replay.make_row(scenario, source, imu_start, 0, 0, 0.21, 3,
                                      pose, tilt, frame, decision, gated, False, zero_tilt)
                self.assertEqual(set(row), set(replay.CSV_FIELDS))
                replay.write_csv(out, [row], replay.CSV_FIELDS)
                return [row]

            with patch.object(replay, "render_run", side_effect=fake_run) as renderer:
                rows = replay.run(scenario, source, out=out, record=path / "frames")
            self.assertEqual(renderer.call_count, 2)
            self.assertTrue(renderer.call_args_list[1].kwargs["zero_tilt"])
            self.assertEqual(set(rows[0]), set(replay.COMPARISON_FIELDS))
            self.assertTrue(rows[0]["gated_advisory_agreement"])
            for file in replay.output_paths(out).values():
                self.assertTrue(file.is_file())
            with replay.output_paths(out)["comparison"].open() as stream:
                self.assertEqual(next(csv.DictReader(stream))["zero_run_kind"], "zero_tilt")

    def test_refuses_short_imu_before_rendering(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "imu.csv"
            write_synthetic(path)
            source = TiltSource(path)
            with patch.object(replay, "render_run") as renderer:
                with self.assertRaises(ValueError):
                    replay.run(example(), source, imu_start=2)
                renderer.assert_not_called()


if __name__ == "__main__":
    unittest.main()
