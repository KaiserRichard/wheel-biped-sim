import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mcap_stereo import LEFT_IMAGE, RIGHT_IMAGE, calibration_from_camera_info, pair_by_stamp
from sgbm_depth import compute_depth


class StereoDepthTests(unittest.TestCase):
    def test_camera_info_baseline_from_projection(self):
        left = SimpleNamespace(p=[400.0, 0, 320.0, 0, 0, 400.0, 240.0, 0, 0, 0, 1, 0],
                               width=640, height=480)
        right = SimpleNamespace(p=[400.0, 0, 320.0, -20.0, 0, 400.0, 240.0, 0, 0, 0, 1, 0],
                                width=640, height=480)
        calibration = calibration_from_camera_info(left, right)
        self.assertAlmostEqual(calibration.baseline_m, 0.05)
        self.assertAlmostEqual(calibration.fb, 20.0)

    def test_pairing_exact_stamps_and_counts_unpaired(self):
        image = np.zeros((2, 2), np.uint8)
        messages = iter([
            (LEFT_IMAGE, 10, image),
            (RIGHT_IMAGE, 10, image + 1),
            (LEFT_IMAGE, 11, image + 2),
            (RIGHT_IMAGE, 12, image + 3),
        ])
        pairs, counts = pair_by_stamp(messages)
        self.assertEqual([p.stamp_ns for p in pairs], [10])
        self.assertEqual(counts["unpaired_left"], 1)
        self.assertEqual(counts["unpaired_right"], 1)

    def test_sgbm_recovers_synthetic_shift_depth(self):
        rng = np.random.default_rng(5)
        h, w = 192, 320
        left = rng.integers(0, 256, (h, w), dtype=np.uint8)
        # The right camera sees the same scene shifted left by disparity d.
        d = 8
        right = np.zeros_like(left)
        right[:, :-d] = left[:, d:]
        calibration = SimpleNamespace(fb=80.0, fx=400.0, cx=160.0, cy=96.0,
                                      baseline_m=0.2, width=w, height=h)
        depth, valid = compute_depth(left, right, calibration, num_disparities=32)
        region = depth[35:-20, 55:-35][valid[35:-20, 55:-35]]
        self.assertGreater(region.size, 100)
        self.assertAlmostEqual(float(np.median(region)), 10.0, delta=0.5)


if __name__ == "__main__":
    unittest.main()
