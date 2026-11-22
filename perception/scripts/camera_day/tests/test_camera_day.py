"""Small offline fixtures; no ROS, camera, network, training or dependency builds."""
import ast
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
CAMERA_DAY = HERE.parent
ROOT = CAMERA_DAY.parents[1]
FIXTURE = HERE / "fixtures/rs-enumerate-historical.txt"
SERIAL = "99003"
spec = importlib.util.spec_from_file_location("sdk_to_yaml", CAMERA_DAY / "sdk_to_yaml.py")
sdk = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sdk)


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.text = FIXTURE.read_text()

    def convert(self, mode="stereo", **kwargs):
        args = dict(serial=SERIAL, mode=mode, width=640, height=480, fps=30)
        args.update(kwargs)
        return sdk.convert(self.text, **args)

    def test_stereo_uses_ir_intrinsics_and_signed_sdk_baseline(self):
        output = self.convert()
        self.assertTrue(output.startswith("%YAML:1.0\n"))
        self.assertIn('Camera.type: "Rectified"', output)
        self.assertIn("Camera1.fx: 388.527435302734", output)
        self.assertIn("Stereo.b: 0.0500313304364681", output)
        self.assertNotIn("IMU.", output)
        self.assertNotIn("RGBD.", output)

    def test_rgbd_uses_color_calibration_and_normalized_depth(self):
        output = self.convert("rgbd")
        self.assertIn('Camera.type: "PinHole"', output)
        self.assertIn("Camera1.fx: 604.780822753906", output)
        self.assertIn("RGBD.DepthMapFactor: 1.0", output)
        self.assertIn("Camera1.k3: 0.0", output)

    def test_selects_requested_device_from_multiple_devices(self):
        self.text = self.text.replace(SERIAL, "99002") + self.text
        self.assertIn(f"SDK serial: {SERIAL}", self.convert())

    def test_wrong_serial_duplicate_device_and_missing_profile_rejected(self):
        with self.assertRaisesRegex(ValueError, "one device"):
            self.convert(serial="999")
        with self.assertRaisesRegex(ValueError, "profile"):
            self.convert(width=848)
        self.text += self.text
        with self.assertRaisesRegex(ValueError, "one device"):
            self.convert()

    def test_baseline_not_guessed_or_absoluted(self):
        for value in ("0", "0.05", "nan", "inf"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                sdk.convert(self.text.replace("-0.0500313304364681", value),
                            serial=SERIAL, mode="stereo", width=640, height=480, fps=30)
        self.text = self.text.split('Extrinsic from "Infrared 1"')[0]
        with self.assertRaisesRegex(ValueError, "extrinsic"):
            self.convert()

    def test_unrectified_extrinsic_rejected(self):
        self.text = self.text.replace("   1   0   0", "   0.999   0.01   0")
        with self.assertRaisesRegex(ValueError, "rectification"):
            self.convert()

    def test_mismatched_right_intrinsics_rejected(self):
        start = self.text.index('Intrinsic of "Infrared 2"')
        self.text = self.text[:start] + self.text[start:].replace("388.527435302734", "390.0")
        with self.assertRaisesRegex(ValueError, "matching"):
            self.convert()

    def test_nonzero_inverse_brown_is_not_silently_copied(self):
        start = self.text.index('Intrinsic of "Color"')
        end = self.text.index('Intrinsic of "Infrared 1"')
        color = self.text[start:end]
        color = color.replace("Coeffs:", "Coeffs: 0.1", 1)
        # Replace with exactly five coefficients, keeping the SDK field layout.
        import re
        color = re.sub(r"(?m)^  Coeffs:.*$", "  Coeffs: 0.1 0 0 0 0", color)
        self.text = self.text[:start] + color + self.text[end:]
        with self.assertRaisesRegex(ValueError, "model conversion"):
            self.convert("rgbd")

    def test_invalid_numeric_and_shape_fields_rejected(self):
        for before, after in (("388.527435302734", "nan"),
                              ("388.527435302734", "-1"),
                              ("Width:      \t640", "Width:      \t320")):
            with self.subTest(after=after), self.assertRaises(ValueError):
                sdk.convert(self.text.replace(before, after), serial=SERIAL,
                            mode="stereo", width=640, height=480, fps=30)
        with self.assertRaises(ValueError):
            self.convert(fps=0)

    def test_cli_never_overwrites_calibration(self):
        with tempfile.TemporaryDirectory(dir=HERE) as tmp:
            saved_readback = Path(tmp) / "rs-enumerate.txt"
            saved_readback.write_bytes(self.text.replace("\n", "\r\n").encode("utf-8"))
            output = Path(tmp) / "camera.yaml"
            command = [sys.executable, str(CAMERA_DAY / "sdk_to_yaml.py"),
                       str(saved_readback), str(output), "--serial", SERIAL, "--mode", "rgbd"]
            first = subprocess.run(command, capture_output=True, text=True, timeout=3)
            self.assertEqual(first.returncode, 0, first.stderr)
            original = output.read_bytes()
            digest = hashlib.sha256(saved_readback.read_bytes()).hexdigest()
            self.assertIn(f"input SHA256: {digest}".encode(), original)
            second = subprocess.run(command, capture_output=True, text=True, timeout=3)
            self.assertEqual(second.returncode, 2)
            self.assertEqual(output.read_bytes(), original)


class LaunchAndRecordTests(unittest.TestCase):
    @staticmethod
    def parameters(mode):
        # Execute only the pure parameter builder: never stub a "successful ROS launch".
        path = ROOT / "ros2/wbr_orb_node/launch/d435_camera.launch.py"
        tree = ast.parse(path.read_text(), filename=str(path))
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
                        and n.name == "camera_parameters")
        namespace = {}
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), "exec"), namespace)
        return namespace["camera_parameters"](mode, SERIAL, "640", "480", "30")

    def test_stereo_and_rgbd_camera_profiles(self):
        for mode in ("stereo", "rgbd"):
            with self.subTest(mode=mode):
                p = self.parameters(mode)
                self.assertTrue(p["enable_depth"])
                self.assertTrue(p["enable_sync"])
                self.assertFalse(p["enable_gyro"])
                self.assertFalse(p["enable_accel"])
                self.assertEqual(p["enable_infra1"], mode == "stereo")
                self.assertEqual(p["enable_infra2"], mode == "stereo")
                self.assertEqual(p["enable_color"], mode == "rgbd")
                self.assertEqual(p["align_depth.enable"], mode == "rgbd")
                self.assertEqual(p["depth_module.emitter_enabled"], int(mode == "rgbd"))
                self.assertEqual(p["depth_module.depth_profile"], "640,480,30")
                self.assertEqual(p["depth_module.infra1_format"], "Y8")
        with self.assertRaises(ValueError):
            self.parameters("imu-stereo")

    def test_record_commands_use_correct_image_and_calibration_topics(self):
        with tempfile.TemporaryDirectory(dir=HERE) as tmp:
            tmp = Path(tmp)
            fake_ros = tmp / "ros2"
            fake_ros.write_text(f"#!{sys.executable}\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n")
            fake_ros.chmod(0o755)
            env = dict(os.environ, PATH=str(tmp) + os.pathsep + os.environ["PATH"])
            for mode in ("stereo", "rgbd"):
                with self.subTest(mode=mode):
                    bag = str(tmp / f"new {mode} bag")
                    result = subprocess.run(
                        ["bash", str(CAMERA_DAY / "record.sh"), mode, bag],
                        env=env, capture_output=True, text=True, timeout=3)
                    self.assertEqual(result.returncode, 0, result.stderr)
                    argv = json.loads(result.stdout)
                    self.assertEqual(argv[:2], ["bag", "record"])
                    self.assertEqual(argv[argv.index("--output") + 1], bag)
                    self.assertIn("/tf_static", argv)
                    self.assertIn("/d435_orb/status", argv)
                    self.assertIn("/camera/camera/depth/camera_info", argv)
                    image = "/camera/camera/" + ("infra2/image_rect_raw" if mode == "stereo"
                                                else "aligned_depth_to_color/image_raw")
                    self.assertIn(image, argv)
                    self.assertNotIn("/camera/camera/imu", argv)
            existing = subprocess.run(
                ["bash", str(CAMERA_DAY / "record.sh"), "stereo", str(tmp)],
                env=env, capture_output=True, text=True, timeout=3)
            self.assertEqual(existing.returncode, 2)
            self.assertIn("Refusing existing bag path", existing.stderr)


if __name__ == "__main__":
    unittest.main()
