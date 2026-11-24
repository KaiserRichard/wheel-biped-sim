import unittest

import numpy as np

from tools.simcam.depth import clipping_distances, optical_depth, pinhole_frustum, stereo_noise
from tools.simcam.scenario import HEIGHT, WIDTH, intrinsics


class DepthTests(unittest.TestCase):
    def test_metric_planar_depth_stays_constant_at_image_edges(self):
        # A frontoparallel wall has identical Z everywhere, unlike radial range.
        depth = np.full((5, 7), 2.0, dtype=np.float32)
        np.testing.assert_array_equal(optical_depth(depth, 0.01, 6.0), depth)

    def test_clipping_invalid_far_background_and_no_input_mutation(self):
        depth = np.array([[0, -1, np.nan, np.inf, 0.005], [0.01, 0.1, 1.5, 6.0, 9.0]])
        copy = depth.copy()
        result = optical_depth(depth, 0.01, 6.0)
        np.testing.assert_allclose(result, [[0, 0, 0, 0, 0], [0.01, 0.1, 1.5, 0, 0]])
        np.testing.assert_array_equal(depth, copy)
        self.assertEqual(result.dtype, np.float32)
        self.assertEqual(clipping_distances(0.01, 3, 2), (0.02, 6.0))

    def test_invalid_helpers(self):
        for near, far in ((0, 6), (1, 1), (6, 1), (0.1, np.inf)):
            with self.subTest(near=near, far=far), self.assertRaises(ValueError):
                optical_depth(np.ones((2, 2)), near, far)
        with self.assertRaisesRegex(ValueError, "2D"):
            optical_depth(np.ones((2, 2, 3)), 0.1, 6)
        with self.assertRaisesRegex(ValueError, "square-pixel"):
            pinhole_frustum(np.eye(2), 640, 480, 0.01)

    def test_frustum_reconstructs_exact_source_pixel_rays(self):
        K, near = intrinsics(), 0.01
        f = pinhole_frustum(K, WIDTH, HEIGHT, near)
        half_width = (f["frustum_top"] - f["frustum_bottom"]) * WIDTH / HEIGHT / 2
        left = f["frustum_center"] - half_width
        for u, v in ((0, 0), (639, 479), (320, 240), (125, 367)):
            x = left + (u + 0.5) / WIDTH * 2 * half_width
            y_up = f["frustum_top"] - (v + 0.5) / HEIGHT * (f["frustum_top"] - f["frustum_bottom"])
            self.assertAlmostEqual(x / near, (u - K[0, 2]) / K[0, 0])
            self.assertAlmostEqual(-y_up / near, (v - K[1, 2]) / K[1, 1])

    def test_seeded_noise_preserves_holes_and_has_quadratic_range_scaling(self):
        depth = np.array([[0, 1.0, 2.0]], dtype=np.float32)
        class UnitNormal:
            def normal(self, mean, sigma):
                return sigma
        result = stereo_noise(depth, sigma_d=0.1, fx=400, rng=UnitNormal())
        self.assertEqual(result[0, 0], 0)
        self.assertAlmostEqual(float(result[0, 2] - 2), 4 * float(result[0, 1] - 1))
        a = stereo_noise(depth, sigma_d=0.1, fx=400, rng=np.random.default_rng(8))
        b = stereo_noise(depth, sigma_d=0.1, fx=400, rng=np.random.default_rng(8))
        np.testing.assert_array_equal(a, b)
        np.testing.assert_array_equal(stereo_noise(depth, sigma_d=0, fx=400), depth)


if __name__ == "__main__":
    unittest.main()
