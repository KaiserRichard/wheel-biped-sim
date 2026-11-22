import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from tools.frames.frames import (Transform, Sample, convert, optical_to_body,
                                 quaternion_order, sdk_ir1_to_imu, base_imu_camera)
from tools.frames.convert_trajectory import parse_stamp, read_samples


def pose(parent, child, t=(0, 0, 0), q=(0, 0, 0, 1)):
    return Transform.from_pose(parent, child, t, q, order="xyzw")


class FramesTests(unittest.TestCase):
    def test_velocity_overflow_is_rejected_before_valid_output(self):
        mount = pose("base_link", "camera_optical")
        for positions in ((-1e308, 1e308), (0, 1e308)):
            samples = [Sample(i, "device", "m", 0,
                              pose("slam_world", "camera_optical", (x, 0, 0)))
                       for i, x in enumerate(positions)]
            with self.subTest(positions=positions), np.errstate(over="ignore"), self.assertRaisesRegex(ValueError, "velocity.*finite"):
                convert(samples, mount)

    def test_nonidentity_inverse_and_frame_names(self):
        a = pose("a", "b", (1, 2, 3), (0, 0, np.sqrt(.5), np.sqrt(.5)))
        inv = a.inverse()
        self.assertEqual((inv.parent, inv.child), ("b", "a"))
        np.testing.assert_allclose(inv.matrix[:3, 3], [-2, 1, -3], atol=1e-12)
        np.testing.assert_allclose((a @ inv).matrix, np.eye(4), atol=1e-12)
        np.testing.assert_allclose((inv @ a).matrix, np.eye(4), atol=1e-12)
        with self.assertRaisesRegex(ValueError, "frame mismatch"):
            a @ pose("c", "d")
        with self.assertRaisesRegex(ValueError, "frame mismatch"):
            inv @ inv

    def test_optical_basis_and_round_trip(self):
        t = optical_to_body()
        np.testing.assert_array_equal(t.matrix[:3, :3] @ np.eye(3),
                                      [[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
        np.testing.assert_allclose((t.inverse() @ t).matrix, np.eye(4))

    def test_orders(self):
        q = np.array([1, 2, 3, 4]) / np.sqrt(30)
        w = quaternion_order(q, "xyzw", "wxyz")
        np.testing.assert_allclose(w, q[[3, 0, 1, 2]])
        np.testing.assert_allclose(quaternion_order(w, "wxyz", "xyzw"), q)
        np.testing.assert_allclose(pose("a", "b", q=q).matrix,
                                  Transform.from_pose("a", "b", [0, 0, 0], w, order="wxyz").matrix)
        with self.assertRaises(ValueError):
            quaternion_order([0, 0, 0, 0], "xyzw", "wxyz")
        with self.assertRaises(ValueError):
            quaternion_order(q, "auto", "xyzw")

    def test_lever_arm_rotation_moves_base(self):
        # Camera 1m along base x with a nonidentity camera-to-base rotation.
        m = optical_to_body("base_link").matrix.copy()
        m[:3, 3] = [1, 0, 0]
        mount = Transform("base_link", "camera_optical", m)
        r = pose("slam_world", "camera_optical", q=(0, 0, np.sqrt(.5), np.sqrt(.5)))
        samples = [Sample(0, "device", "m", 0, pose("slam_world", "camera_optical")),
                   Sample(1_000_000_000, "device", "m", 0, r)]
        output = convert(samples, mount)
        # Camera forward (z) is base x, so base origin is -camera z.
        np.testing.assert_allclose(np.array(output[0]["T_world_base"])[:3, 3], [0, 0, -1])
        # Rotate around camera y instead, so the lever arm changes direction.
        r = pose("slam_world", "camera_optical", q=(0, np.sqrt(.5), 0, np.sqrt(.5)))
        output = convert([samples[0], Sample(1_000_000_000, "device", "m", 0, r)], mount)
        np.testing.assert_allclose(np.array(output[1]["T_world_base"])[:3, 3], [-1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(output[1]["diagnostic_linear_velocity_world_m_s"], [-1, 0, 1], atol=1e-12)
        for sample, row in zip([samples[0], Sample(1_000_000_000, "device", "m", 0, r)], output):
            recovered = Transform("slam_world", "base_link", row["T_world_base"]) @ mount
            np.testing.assert_allclose(recovered.matrix, sample.pose.matrix, atol=1e-12)

    def test_sdk_column_major_and_imu_route(self):
        sdk = sdk_ir1_to_imu([0, 1, 0, -1, 0, 0, 0, 0, 1], [.1, .2, .3])
        np.testing.assert_allclose(sdk.matrix[:3, :3] @ [1, 0, 0], [0, 1, 0])
        mount = optical_to_body("base_link")
        imu_mount = base_imu_camera(mount, sdk)
        self.assertEqual(imu_mount.child, "imu_camera")
        world_camera = pose("slam_world", "camera_optical", [2, 3, 4],
                            [np.sqrt(.5), 0, 0, np.sqrt(.5)])
        world_imu = world_camera @ sdk.inverse()
        a = convert([Sample(1, "d", "m", 0, world_camera)], mount)[0]
        b = convert([Sample(1, "d", "m", 0, world_imu)], imu_mount)[0]
        np.testing.assert_allclose(a["T_world_base"], b["T_world_base"], atol=1e-12)
        with self.assertRaises(ValueError):
            base_imu_camera(mount, sdk.inverse())

    def test_reset_map_split_and_no_boundary_velocity(self):
        specs = [(10, "a", 0, 0), (20, "a", 0, 1), (1, "a", 1, 100),
                 (2, "b", 1, 1000), (3, "a", 0, -100)]
        samples = [Sample(t, "d", m, r, pose("slam_world", "camera_optical", [x, 0, 0]))
                   for t, m, r, x in specs]
        result = convert(samples, pose("base_link", "camera_optical"))
        self.assertEqual([r["segment_id"] for r in result], [0, 0, 1, 2, 3])
        self.assertEqual([r["velocity_valid"] for r in result], [False, True, False, False, False])
        self.assertIsNone(result[2]["diagnostic_linear_velocity_world_m_s"])

    def test_domains_order_and_odom_rejected(self):
        first = Sample(1, "d", "m", 0, pose("slam_world", "camera_optical"))
        mount = pose("base_link", "camera_optical")
        for second in [Sample(2, "host", "m", 0, first.pose),
                       Sample(0, "d", "m", 0, first.pose), first]:
            with self.assertRaises(ValueError):
                convert([first, second], mount)
        with self.assertRaises(ValueError):
            convert([first], mount, world="odom")
        with self.assertRaises(ValueError):
            convert([first], pose("base_link", "imu_camera"))
        with self.assertRaises(ValueError):
            Sample(1.5, "d", "m", 0, first.pose)

    def test_bad_transforms(self):
        for matrix in [np.diag([-1, 1, 1, 1]), np.zeros((4, 4)), np.full((4, 4), np.nan)]:
            with self.assertRaises(ValueError):
                Transform("a", "b", matrix)

    def test_decimal_timestamp(self):
        self.assertEqual(parse_stamp("1700000000.000000001", "s"), 1700000000000000001)
        self.assertEqual(parse_stamp("1700000000000000001.000000", "ns"), 1700000000000000001)
        for text in ["NaN", "1.0000000001", "999999999999999999999"]:
            with self.assertRaises(ValueError):
                parse_stamp(text, "s")

    def test_cli_and_tcw_normalization(self):
        examples = ROOT / "tools/frames/examples"
        with tempfile.TemporaryDirectory() as d:
            d = Path(d)
            out = d / "out.jsonl"
            cmd = [sys.executable, str(ROOT / "tools/frames/convert_trajectory.py"),
                   str(examples / "camera.tum"), "--epochs", str(examples / "epochs.jsonl"),
                   "--mount", str(examples / "mount_proposed.json"), "--time-unit", "s",
                   "--sensor-frame", "camera_optical", "--pose-direction", "sensor-to-world",
                   "--quaternion-order", "xyzw", "--output", str(out)]
            done = subprocess.run(cmd, capture_output=True, text=True)
            self.assertEqual(done.returncode, 0, done.stderr)
            rows = [json.loads(line) for line in out.read_text().splitlines()]
            self.assertEqual([r["segment_id"] for r in rows], [0, 0, 1])
            self.assertEqual(rows[0]["provenance"]["mount"]["status"], "PROPOSED")
            self.assertIsNone(rows[2]["diagnostic_linear_velocity_world_m_s"])
            failed = subprocess.run(cmd + ["--world-frame", "odom"], capture_output=True, text=True)
            self.assertEqual(failed.returncode, 2)
            raw = d / "tcw.txt"
            epochs = d / "epoch.jsonl"
            raw.write_text("1 1 2 3 0 0 0.7071067811865476 0.7071067811865476\n")
            epochs.write_text(json.dumps({"stamp_ns": 1, "clock_domain": "d", "map_id": "m", "reset_counter": 0}))
            sample = read_samples(raw, epochs, unit="ns", sensor="camera_optical",
                                  direction="world-to-sensor", order="xyzw")[0]
            np.testing.assert_allclose(sample.pose.matrix[:3, 3], [-2, 1, -3], atol=1e-12)
            epochs.write_text('{"stamp_ns":2}\n')
            with self.assertRaisesRegex(ValueError, "exactly match"):
                read_samples(raw, epochs, unit="ns", sensor="camera_optical",
                             direction="world-to-sensor", order="xyzw")


if __name__ == "__main__":
    unittest.main()
