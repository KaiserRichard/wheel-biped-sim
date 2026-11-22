import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("stereo_quality", ROOT / "stereo_bag_quality.py")
quality = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(quality)


class QualityTests(unittest.TestCase):
    def stamps(self):
        # Epoch-sized integers retain source-clock precision.
        return {key: [1792550000000000000 + n * 66666667 for n in range(300)]
                for key in quality.TOPICS}

    def test_complete_stereo(self):
        result = quality.analyze(self.stamps(), 15)
        self.assertTrue(result["observed_target_met"])
        self.assertEqual(result["nominal_common_span_expected_estimate"], 300)

    def test_single_stream_loss_detected_even_below_one_percent(self):
        stamps = self.stamps()
        del stamps["left_image"][100]
        result = quality.analyze(stamps, 15)
        self.assertFalse(result["observed_target_met"])
        self.assertEqual(result["right_only_headers"], 1)
        self.assertEqual(result["streams"]["left"]["missing_images_with_recorded_camera_info"], 1)

    def test_shared_missing_frames_cannot_hide_behind_equal_counts(self):
        stamps = self.stamps()
        for values in stamps.values():
            del values[100:110]
        result = quality.analyze(stamps, 15)
        self.assertTrue(result["counts_equal"])
        self.assertFalse(result["observed_target_met"])
        self.assertEqual(result["streams"]["left"]["nominal_internal_gap_frames_estimate"], 10)

    def test_equal_counts_with_different_headers_fail(self):
        stamps = self.stamps()
        stamps["right_image"] = [v + 1 for v in stamps["right_image"]]
        self.assertFalse(quality.analyze(stamps, 15)["observed_target_met"])

    def test_duplicate_backward_empty_and_invalid_rate_rejected(self):
        for values in ([], [1, 1], [2, 1]):
            stamps = self.stamps()
            stamps["left_image"] = values
            with self.assertRaises(ValueError):
                quality.analyze(stamps, 15)
        for fps in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                quality.analyze(self.stamps(), fps)

    def test_record_only_four_stereo_topics_and_never_replace_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            fake = root / "ros2"
            fake.write_text(f"#!{sys.executable}\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n")
            fake.chmod(0o755)
            env = dict(os.environ, PATH=str(root) + os.pathsep + os.environ["PATH"])
            command = ["bash", str(ROOT / "record_stereo.sh"), str(root / "new bag")]
            result = subprocess.run(command, capture_output=True, text=True, env=env, check=True)
            args = json.loads(result.stdout)
            self.assertEqual(args[args.index("--storage") + 1], "mcap")
            self.assertEqual(set(args[args.index("--topics") + 1:]), set(quality.TOPICS.values()))
            (root / "new bag").mkdir()
            blocked = subprocess.run(command, capture_output=True, text=True, env=env)
            self.assertEqual(blocked.returncode, 2)


if __name__ == "__main__":
    unittest.main()
