import math
import unittest

from tools.wbr_pose.fivebar import forward_kinematics, inverse_kinematics
from sim.models.make_wbr_3d_simple import D_HIP, L_LOW, L_UP


class FivebarTests(unittest.TestCase):
    def test_fk_ik_grid_round_trip(self):
        for l0 in (0.07, 0.09, 0.12, 0.16, 0.20):
            for theta in (-0.12, -0.06, 0.0, 0.06, 0.12):
                with self.subTest(l0=l0, theta=theta):
                    pose = forward_kinematics(l0, theta)
                    got_l0, got_theta = inverse_kinematics(pose.phi1, pose.phi4)
                    self.assertAlmostEqual(got_l0, l0, places=9)
                    self.assertAlmostEqual(got_theta, theta, places=9)

    def test_all_bar_lengths_preserved(self):
        for l0 in (0.07, 0.12, 0.20):
            for theta in (-0.1, 0.0, 0.1):
                p = forward_kinematics(l0, theta)
                self.assertAlmostEqual(math.dist(p.hip_left, p.hip_right), D_HIP)
                self.assertAlmostEqual(math.dist(p.hip_left, p.knee_left), L_UP)
                self.assertAlmostEqual(math.dist(p.hip_right, p.knee_right), L_UP)
                self.assertAlmostEqual(math.dist(p.knee_left, p.axle), L_LOW)
                self.assertAlmostEqual(math.dist(p.knee_right, p.axle), L_LOW)
                self.assertAlmostEqual(math.dist((0, 0), p.axle), l0)

    def test_unreachable_rejected(self):
        with self.assertRaises(ValueError):
            forward_kinematics(0.30, 0.0)
        with self.assertRaises(ValueError):
            inverse_kinematics(0.0, 0.0)


if __name__ == "__main__":
    unittest.main()
