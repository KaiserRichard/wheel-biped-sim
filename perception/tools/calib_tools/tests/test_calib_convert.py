"""Synthetic convention, schema, provenance and rejection tests (no engine run)."""

import ast
from contextlib import redirect_stderr
import copy
import hashlib
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import yaml

TOOL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL))
import calib_convert as converter

FIXTURES = Path(__file__).parent / "fixtures"


def read_orb(path):
    """Minimal independent reader for this OpenCV scalar/matrix YAML subset."""
    lines = path.read_text().splitlines()
    if lines[0] != "%YAML:1.0":
        raise ValueError("missing OpenCV YAML directive")
    fields = {}
    index = 1
    while index < len(lines):
        line = lines[index]
        index += 1
        if not line or line.startswith("#"):
            continue
        key, value = line.split(":", 1)
        value = value.strip()
        if value == "!!opencv-matrix":
            attrs = {}
            for _ in range(4):
                name, text = lines[index].strip().split(":", 1)
                attrs[name] = text.strip()
                index += 1
            if attrs["dt"] != "f":
                raise ValueError("unsupported matrix dtype")
            # Match the declared float32 storage, rather than just comparing text.
            fields[key] = np.asarray(ast.literal_eval(attrs["data"]), dtype=np.float32).reshape(
                int(attrs["rows"]), int(attrs["cols"]))
        else:
            fields[key] = json.loads(value)
    return fields


class CalibrationTests(unittest.TestCase):
    def test_orb_rejects_values_that_overflow_or_underflow_native_float(self):
        for field, value in (("gyroscope_noise_density", 1e40),
                             ("gyroscope_noise_density", 1e-50)):
            with self.subTest(value=value):
                imu = {**self.imu, field: value}
                with self.assertRaisesRegex(ValueError, "float32"):
                    self.run_conversion(target="orb", imu=imu)
                self.assertFalse((self.work / "out").exists())
        cam = copy.deepcopy(self.cam)
        cam["cam0"]["intrinsics"][0] = 1e40
        with self.assertRaisesRegex(ValueError, "float32"):
            self.run_conversion(target="orb", cam=cam)
        self.assertFalse((self.work / "out").exists())

    def setUp(self):
        self.cam = yaml.safe_load((FIXTURES / "camchain-imucam.yaml").read_text())
        self.imu = yaml.safe_load((FIXTURES / "imu.yaml").read_text())
        self.temporary = tempfile.TemporaryDirectory(dir=Path(__file__).parent)
        self.addCleanup(self.temporary.cleanup)
        self.work = Path(self.temporary.name)

    def run_conversion(self, *, target="both", cam=None, imu=None, **kwargs):
        cam_path, imu_path = FIXTURES / "camchain-imucam.yaml", FIXTURES / "imu.yaml"
        if cam is not None:
            cam_path = self.work / "cam.yaml"
            cam_path.write_text(yaml.safe_dump(cam))
        if imu is not None:
            imu_path = self.work / "imu.yaml"
            imu_path.write_text(yaml.safe_dump(imu))
        options = {"target": target, "fps": 30, "units_si": True, **kwargs}
        stderr = io.StringIO()
        with redirect_stderr(stderr):
            converter.convert(cam_path, imu_path, self.work / "out", **options)
        return self.work / "out", stderr.getvalue()

    def test_nonidentity_inverse_and_transformed_point_roundtrip(self):
        out, _ = self.run_conversion()
        orb = read_orb(out / "orbslam3.yaml")
        t_c_i = np.asarray(self.cam["cam0"]["T_cam_imu"])
        np.testing.assert_allclose(orb["IMU.T_b_c1"] @ t_c_i, np.eye(4), atol=1e-7)
        np.testing.assert_allclose(t_c_i @ orb["IMU.T_b_c1"], np.eye(4), atol=1e-7)
        point_i = np.array([.14, -.37, 1.2, 1])
        point_c = t_c_i @ point_i
        np.testing.assert_allclose(orb["IMU.T_b_c1"] @ point_c, point_i, atol=1e-7)
        self.assertFalse(np.allclose(orb["IMU.T_b_c1"][:3, 3], -t_c_i[:3, 3]))

    def test_stereo_direction_baseline_sign_and_small_rotations(self):
        # Kalibr yaml-formats: cam1.T_cn_cnm1 maps cam0 -> cam1.
        # ORB Settings.cc L342-L348,L492-L505: T_c1_c2 is cam1 -> cam0.
        out, _ = self.run_conversion()
        orb = read_orb(out / "orbslam3.yaml")
        t_right_left = np.asarray(self.cam["cam1"]["T_cn_cnm1"])
        t_left_right = orb["Stereo.T_c1_c2"]
        np.testing.assert_allclose(t_left_right @ t_right_left, np.eye(4), atol=1e-7)
        np.testing.assert_allclose(t_left_right[:3, 3], [.05, .0008, -.0005], atol=1e-8)
        self.assertLess(t_right_left[0, 3], 0)
        self.assertGreater(t_left_right[0, 3], 0)
        self.assertFalse(np.allclose(t_left_right[:3, :3], np.eye(3)))
        point_left = np.array([.2, .1, 2., 1.])
        np.testing.assert_allclose(t_left_right @ (t_right_left @ point_left), point_left, atol=1e-7)
        provenance = json.loads((out / "provenance.json").read_text())
        self.assertAlmostEqual(provenance["baseline_m"], math.sqrt(.05**2 + .0008**2 + .0005**2))

    def test_schema_roundtrip_types_and_continuous_noise(self):
        out, _ = self.run_conversion()
        orb = read_orb(out / "orbslam3.yaml")
        self.assertEqual(orb["File.version"], "1.0")
        self.assertEqual(orb["Camera.type"], "PinHole")
        for i in (0, 1):
            cam = self.cam[f"cam{i}"]
            for key, value in zip(("fx", "fy", "cx", "cy"), cam["intrinsics"]):
                self.assertEqual(orb[f"Camera{i+1}.{key}"], value)
                self.assertIs(type(orb[f"Camera{i+1}.{key}"]), float)
            for key, value in zip(("k1", "k2", "p1", "p2"), cam["distortion_coeffs"]):
                self.assertEqual(orb[f"Camera{i+1}.{key}"], value)
        for key in ("Camera.width", "Camera.height", "Camera.fps", "Camera.RGB",
                    "ORBextractor.nFeatures", "ORBextractor.nLevels"):
            self.assertIs(type(orb[key]), int)
        for key, source in converter.NOISE_KEYS.items():
            self.assertEqual(orb[key], self.imu[source])
            self.assertIs(type(orb[key]), float)
        # Frequency changes must not discretize exported densities/walks.
        modified = copy.deepcopy(self.imu)
        modified["update_rate"] = 800.0
        calibration = converter.validate(self.cam, modified, "orb", 1.)
        for key in converter.NOISE_KEYS:
            if key != "IMU.Frequency":
                self.assertEqual(calibration["noise"][key], orb[key])
        self.assertNotIn("Stereo.b", orb)  # PinHole derives magnitude from transform.
        self.assertNotIn("Tbc", orb)
        self.assertNotIn("timeshift_cam_imu", orb)

    def test_provenance_hashes_timeshift_units_and_warning(self):
        out, warning = self.run_conversion()
        provenance = json.loads((out / "provenance.json").read_text())
        self.assertIn("t_imu = t_cam + timeshift_cam_imu", warning)
        self.assertIn("Shift camera stamps at association", warning)
        for name, fixture in (("camchain", "camchain-imucam.yaml"), ("imu", "imu.yaml")):
            self.assertEqual(provenance["inputs"][name]["sha256"],
                             hashlib.sha256((FIXTURES / fixture).read_bytes()).hexdigest())
        for name, record in provenance["outputs"].items():
            self.assertEqual(record["sha256"], hashlib.sha256((out / name).read_bytes()).hexdigest())
        for cam in ("cam0", "cam1"):
            shift = provenance["timeshift_cam_imu"][cam]
            self.assertEqual(shift["nanoseconds"], 12345678)
            self.assertEqual(shift["seconds"], "0.012345678")
            self.assertEqual(shift["rounding_error_ns"], "0E-9")
        self.assertEqual(provenance["units"]["translation"], "m")
        self.assertIn("UNKNOWN", provenance["conventions"]["clock_domain"])
        self.assertIn(converter.ORB_REV, provenance["sources"]["orb_noise_scaling"])

    def test_openvins_preserves_values_and_flat_imu_wrapper(self):
        out, _ = self.run_conversion(target="openvins", fps=None)
        cam_text = (out / "kalibr_imucam_chain.yaml").read_text()
        imu_text = (out / "kalibr_imu_chain.yaml").read_text()
        self.assertTrue(cam_text.startswith("%YAML:1.0\n"))
        self.assertTrue(imu_text.startswith("%YAML:1.0\nimu0:\n"))
        self.assertEqual(yaml.safe_load(cam_text.split("\n", 1)[1]), self.cam)
        self.assertEqual(yaml.safe_load(imu_text.split("\n", 1)[1]), {"imu0": self.imu})
        self.assertIn("HAND-MADE SYNTHETIC", cam_text)
        self.assertFalse((out / "orbslam3.yaml").exists())

    def test_nested_imu_and_opencv_inputs(self):
        cam_path, imu_path = self.work / "cam.yaml", self.work / "imu.yaml"
        cam_path.write_bytes(b"%YAML:1.0\n" + (FIXTURES / "camchain-imucam.yaml").read_bytes())
        imu_path.write_text("%YAML:1.0\n" + yaml.safe_dump({"imu0": self.imu}))
        with redirect_stderr(io.StringIO()):
            converter.convert(cam_path, imu_path, self.work / "out", target="openvins", fps=None, units_si=True)
        text = (self.work / "out/kalibr_imu_chain.yaml").read_text()
        self.assertEqual(yaml.safe_load(text.split("\n", 1)[1]), {"imu0": self.imu})
        self.assertEqual(text.count("%YAML:1.0"), 1)

    def test_openvins_equidistant_is_native_but_orb_rejects(self):
        for cam in self.cam.values():
            cam["distortion_model"] = "equidistant"
        converter.validate(self.cam, self.imu, "openvins", 1.)
        for target in ("orb", "both"):
            with self.assertRaisesRegex(ValueError, "equidistant.*KannalaBrandt8"):
                converter.validate(self.cam, self.imu, target, 1.)

    def test_unequal_shifts_openvins_rejected_orb_preserved(self):
        self.cam["cam1"]["timeshift_cam_imu"] = -.001
        for target in ("openvins", "both"):
            with self.assertRaisesRegex(ValueError, "only cam0 timeshift"):
                converter.validate(self.cam, self.imu, target, 1.)
        out, _ = self.run_conversion(target="orb", cam=self.cam)
        provenance = json.loads((out / "provenance.json").read_text())
        self.assertEqual(provenance["timeshift_cam_imu"]["cam1"]["nanoseconds"], -1000000)

    def test_subnanosecond_negative_shift_rounding_is_reported(self):
        for cam in self.cam.values():
            cam["timeshift_cam_imu"] = -1.5e-9
        out, warning = self.run_conversion(cam=self.cam)
        shift = json.loads((out / "provenance.json").read_text())["timeshift_cam_imu"]["cam0"]
        self.assertEqual(shift["nanoseconds"], -2)
        self.assertEqual(shift["rounding_error_ns"], "0.5000000000")
        self.assertIn("Sub-nanosecond", warning)

    def test_rejection_cases(self):
        cases = [
            ("cam0", "camera_model", "omni", "camera_model"),
            ("cam0", "distortion_model", "plumb_bob", "distortion_model"),
            ("cam0", "intrinsics", [0, 401, 320, 240], "focal"),
            ("cam0", "intrinsics", [400, 401, 640, 240], "principal point"),
            ("cam0", "intrinsics", [400, 401, 320], "shape"),
            ("cam0", "distortion_coeffs", [0, 0, 0, 0, 0], "shape"),
            ("cam0", "distortion_coeffs", [float("nan"), 0, 0, 0], "finite"),
            ("cam0", "resolution", [640., 480], "positive integers"),
            ("cam0", "resolution", [True, 480], "positive integers"),
            ("cam1", "resolution", [800, 600], "identical"),
            ("cam0", "timeshift_cam_imu", float("inf"), "finite"),
            ("cam0", "timeshift_cam_imu", "12 ms", "unit suffix"),
            ("cam0", "timeshift_cam_imu", 1e20, "int64"),
            ("cam0", "units", {"translation": "mm"}, "unsupported unit"),
            ("cam0", "units", {"gyroscope": "deg/s"}, "unsupported unit"),
        ]
        for camera, key, value, pattern in cases:
            with self.subTest(key=key, value=value):
                bad = copy.deepcopy(self.cam)
                bad[camera][key] = value
                with self.assertRaisesRegex(ValueError, pattern):
                    converter.validate(bad, self.imu, "both", 1.)

    def test_transform_rejections(self):
        for kind in ("scale", "reflection", "bottom_row", "millimetres", "inconsistent"):
            with self.subTest(kind=kind):
                bad = copy.deepcopy(self.cam)
                matrix = bad["cam1"]["T_cn_cnm1"]
                if kind == "scale":
                    matrix[0][0] *= 1.01
                elif kind == "reflection":
                    for row in range(3):
                        matrix[row][0] *= -1
                elif kind == "bottom_row":
                    matrix[3][0] = .001
                elif kind == "millimetres":
                    for row in range(3):
                        matrix[row][3] *= 1000
                else:
                    matrix[0][3] += .001
                with self.assertRaises(ValueError):
                    converter.validate(bad, self.imu, "both", 1.)
        self.cam["cam1"]["T_cam_imu"] = self.cam["cam0"]["T_cam_imu"]
        self.cam["cam1"]["T_cn_cnm1"] = np.eye(4).tolist()
        with self.assertRaisesRegex(ValueError, "baseline"):
            converter.validate(self.cam, self.imu, "both", 1.)

    def test_missing_fields_noise_and_units(self):
        for key in self.cam["cam0"]:
            if key == "rostopic":
                continue  # Neither engine needs ROS topics for native calibration loading.
            bad = copy.deepcopy(self.cam)
            del bad["cam0"][key]
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "missing"):
                converter.validate(bad, self.imu, "both", 1.)
        for key in converter.NOISE_KEYS.values():
            for value in (None, 0, -.1, True, float("inf"), "1 mg"):
                bad = copy.deepcopy(self.imu)
                bad[key] = value
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    converter.validate(self.cam, bad, "both", 1.)
            bad = copy.deepcopy(self.imu)
            del bad[key]
            with self.assertRaisesRegex(ValueError, "missing"):
                converter.validate(self.cam, bad, "both", 1.)
        for units in ({"update_rate": "ms"}, {"timeshift": "ms"}, {"accelerometer": "g"},
                      {"gyroscope_noise_density": "rad/s"}):
            bad = {**self.imu, "units": units}
            with self.assertRaisesRegex(ValueError, "unsupported unit"):
                converter.validate(self.cam, bad, "both", 1.)

    def test_ambiguous_inputs(self):
        with self.assertRaisesRegex(ValueError, "duplicate"):
            converter.read_yaml(b"cam0: {}\ncam0: {}\n", "cam")
        with self.assertRaisesRegex(ValueError, "mapping"):
            converter.read_yaml(b"[]", "cam")
        quoted = converter.read_yaml(b'gyroscope_noise_density: "1e-3"\n', "imu")
        with self.assertRaisesRegex(ValueError, "unquoted"):
            converter.number(quoted["gyroscope_noise_density"], "gyro")
        plain = converter.read_yaml(b'gyroscope_noise_density: 1e-3\n', "imu")
        self.assertEqual(converter.number(plain["gyroscope_noise_density"], "gyro"), .001)
        bad = {**self.cam, "cam2": self.cam["cam1"]}
        with self.assertRaisesRegex(ValueError, "exactly"):
            converter.validate(bad, self.imu, "both", 1.)
        with self.assertRaisesRegex(ValueError, "ambiguous"):
            converter.validate(self.cam, {**self.imu, "imu0": self.imu}, "both", 1.)
        self.cam["cam0"]["T_imu_cam"] = self.cam["cam0"]["T_cam_imu"]
        with self.assertRaisesRegex(ValueError, "conflicting"):
            converter.validate(self.cam, self.imu, "both", 1.)

    def test_explicit_units_fps_and_no_overwrite(self):
        for kwargs, pattern in (({"units_si": False}, "units-si"), ({"fps": None}, "fps"),
                                ({"fps": 0}, "fps"), ({"max_translation_m": 0}, "positive")):
            with self.subTest(kwargs=kwargs), self.assertRaisesRegex(ValueError, pattern):
                self.run_conversion(**kwargs)
            self.assertFalse((self.work / "out").exists())
        out, _ = self.run_conversion()
        before = {p.name: p.read_bytes() for p in out.iterdir()}
        with self.assertRaisesRegex(ValueError, "overwrite"):
            self.run_conversion()
        self.assertEqual(before, {p.name: p.read_bytes() for p in out.iterdir()})

    def test_cli_success_and_validation_failure_before_outputs(self):
        cmd = [sys.executable, str(TOOL / "calib_convert.py"),
               "--camchain", str(FIXTURES / "camchain-imucam.yaml"),
               "--imu", str(FIXTURES / "imu.yaml"), "--output-dir", str(self.work / "cli")]
        invalid = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(invalid.returncode, 2)
        self.assertIn("units-si", invalid.stderr)
        self.assertFalse((self.work / "cli").exists())
        valid = subprocess.run(cmd + ["--units-si", "--fps", "30"], capture_output=True, text=True)
        self.assertEqual(valid.returncode, 0, valid.stderr)
        self.assertIn("timeshift_cam_imu", valid.stderr)
        self.assertEqual(len(valid.stdout.splitlines()), 4)


if __name__ == "__main__":
    unittest.main()
