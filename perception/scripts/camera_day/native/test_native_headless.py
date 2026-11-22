"""Offline tests only: no SDK import, camera access, ORB executable or Pi load.

Optional WBR_ORIGINAL_EXAMPLES points at read-only saved example source copies.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ASSETS = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("native_headless", ASSETS.parent / "native_headless.py")
native = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(native)
native.SERIAL = "99005"


def sdk_fixture():
    # Nonsymmetric rotation catches accidental SDK-array/printed-row transpose.
    transform = '''
Rotation Matrix:
  0 -1 0
  1  0 0
  0  0 1
Translation Vector: -0.00552 0.0051 0.01174
'''
    return (f"Device info\x3a\n Serial Number : {native.SERIAL}\n" +
            "Extrinsic from \"Infrared 1\" To \"Gyro\" :\n" + transform +
            "Extrinsic from \"Infrared 1\" To \"Accel\" :\n" + transform)


class NativeTests(unittest.TestCase):
    def test_extrinsic_printed_rows_and_translation_direction(self):
        result = native.extrinsic(sdk_fixture(), "Infrared 1", "Gyro")
        self.assertEqual(result, [0., -1., 0., -.00552, 1., 0., 0., .0051,
                                  0., 0., 1., .01174, 0., 0., 0., 1.])

    def test_missing_ambiguous_and_invalid_extrinsics(self):
        for text in ("", sdk_fixture() * 2, sdk_fixture().replace("0 -1 0", "0 -2 0")):
            with self.assertRaises(ValueError):
                native.extrinsic(text, "Infrared 1", "Gyro")

    def test_settings_provenance_noise_and_output_exclusivity(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            evidence, upstream, output = root / "evidence", root / "orb", root / "settings"
            evidence.mkdir(); output.mkdir()
            (upstream / "Examples/Stereo-Inertial").mkdir(parents=True)
            text = sdk_fixture()
            (evidence / "rs-enumerate.txt").write_text(text)
            base = (f"# SDK serial: {native.SERIAL}; input SHA256: {hashlib.sha256(text.encode()).hexdigest()}\n"
                    "Camera.fps: 30\nORBextractor.nFeatures: 1250\nRGBD.DepthMapFactor: 1.0\n")
            for name in ("stereo.yaml", "rgbd-ros.yaml"):
                (evidence / name).write_text(base)
            (upstream / "Examples/Stereo-Inertial/RealSense_D435i.yaml").write_text(
                "IMU.NoiseGyro: 1e-3\nIMU.NoiseAcc: 1e-2\nIMU.GyroWalk: 1e-6\nIMU.AccWalk: 1e-4\n")
            native.settings(evidence, upstream, output, 15, 800)
            inertial = (output / "stereo_inertial-15fps-800features.yaml").read_text()
            self.assertIn("Camera.fps: 15", inertial)
            self.assertIn("ORBextractor.nFeatures: 800", inertial)
            self.assertIn("# ASSUMED noise:", inertial)
            self.assertIn("IMU.NoiseGyro: 1e-3", inertial)
            self.assertIn("IMU.Frequency: 200.0", inertial)
            self.assertNotIn("IMU.", (output / "rgbd-15fps-800features.yaml").read_text())
            with self.assertRaises(FileExistsError):
                native.settings(evidence, upstream, output, 15, 800)
            (evidence / "rs-enumerate.txt").write_text(text.replace(native.SERIAL, "0000"))
            with self.assertRaises(ValueError):
                native.settings(evidence, upstream, output, 30, 1250)

    def test_compile_and_link_only_private_outputs(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root / "metadata").mkdir()
            (root / "manifest.json").write_text(json.dumps({"orb_root": "/existing/orb"}))
            for mode, folder in native.MODES.items():
                name = f"{mode}_realsense_D435i"
                (root / "metadata" / f"{mode}-flags.make").write_text(
                    'CXX_DEFINES = -DPANGO_DEFAULT_WIN_URI=\\"wayland\\"\n'
                    'CXX_INCLUDES = -I/existing/orb/include\nCXX_FLAGS = -std=c++17 -O3\n')
                (root / "metadata" / f"{mode}-link.txt").write_text(
                    f'/usr/bin/c++ "CMakeFiles/{name}.dir/Examples/{folder}/{name}.cc.o" '
                    f'-o /existing/orb/Examples/{folder}/{name} -Wl,-rpath,/existing/orb/lib /existing/orb/lib/libORB_SLAM3.so')
                compile_command, link = native.compile_commands(root, mode)
                self.assertIn('-DPANGO_DEFAULT_WIN_URI="wayland"', compile_command)
                self.assertEqual(compile_command[compile_command.index("-o") + 1], str(root / "obj" / f"{name}.o"))
                self.assertEqual(link[link.index("-o") + 1], str(root / "bin" / name))
                self.assertIn("/existing/orb/lib/libORB_SLAM3.so", link)
                self.assertFalse(any("System.cc" in part for part in compile_command))

    def test_destination_exclusive_before_source_access(self):
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaises(FileExistsError):
                native.prepare(Path("/nonexistent"), Path(temporary), Path("/nonexistent"))

    def test_counts_distinguish_startup_recovery_and_losses(self):
        # Tiny standard-library C++ fixture, never links ORB or RealSense.
        compiler = shutil.which("c++")
        self.assertIsNotNone(compiler)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "metrics_test.cc"
            source.write_text('''#include "metrics.hpp"
#include <iostream>
int main() {
    wbr::Counts counts;
    for (int state : {0, 1, 2, 2, 3, 4, 1, 2, 4}) counts.add(state);
    std::cout << "{"; counts.json(std::cout, 3.0); std::cout << "}";
}
''')
            subprocess.run([compiler, "-std=c++17", "-I", str(ASSETS), str(source), "-o", str(root / "metrics_test")], check=True)
            result = json.loads(subprocess.check_output([str(root / "metrics_test")], text=True))
            self.assertEqual(result["processed_frames"], 9)
            self.assertEqual(result["tracked_frames"], 3)
            self.assertEqual(result["startup_untracked_frames"], 2)
            self.assertEqual(result["untracked_after_established_frames"], 4)
            self.assertEqual(result["recently_lost_or_lost_state_frames"], 3)
            self.assertEqual(result["loss_transitions_after_established"], 2)
            self.assertEqual(result["processed_fps"], 3)
            self.assertEqual(result["successfully_tracked_fps"], 1)

    @unittest.skipUnless(os.environ.get("WBR_ORIGINAL_EXAMPLES"), "set WBR_ORIGINAL_EXAMPLES for real-source patch tests")
    def test_patches_on_real_saved_sources(self):
        originals = Path(os.environ["WBR_ORIGINAL_EXAMPLES"])
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            for mode in native.MODES:
                name = f"{mode}_realsense_D435i.cc"
                original = (originals / name).read_text()
                copied = root / name
                copied.write_text(original.rstrip("\n") + "\n")
                subprocess.run(["patch", "--batch", "--fuzz=0", str(copied), str(ASSETS / f"{mode}.patch")], check=True, capture_output=True)
                if mode == "stereo_inertial":
                    subprocess.run(["patch", "--batch", "--fuzz=0", str(copied),
                                    str(ASSETS / "stereo_inertial_unified.patch")],
                                   check=True, capture_output=True)
                patched = copied.read_text()
                self.assertTrue(patched.startswith(original.split("#include", 1)[0]))
                self.assertIn(", false, 0, file_name)", patched)
                self.assertNotIn(".set_option(", patched)
                self.assertNotIn("cond_image_rec.wait(lk)", patched)
                self.assertIn("runtime.finish(SLAM", patched)
                self.assertIn("runtime.begin(timestamp)", patched)
                self.assertNotIn("dropped frs", patched)
                if mode == "rgbd":
                    self.assertNotIn("cfg.enable_stream(RS2_STREAM_GYRO", patched)
                    self.assertNotIn("cfg.enable_stream(RS2_STREAM_ACCEL", patched)
                    self.assertIn("depth_frame.get_units()", patched)
                else:
                    self.assertIn("ir_frameL.get_stride_in_bytes()).clone()", patched)
                    self.assertIn("ir_frameR.get_stride_in_bytes()).clone()", patched)


if __name__ == "__main__":
    unittest.main()
