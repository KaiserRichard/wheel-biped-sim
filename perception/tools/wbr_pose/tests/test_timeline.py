import unittest

from tools.wbr_pose.jump_demo import state_at, total_time


class TimelineTests(unittest.TestCase):
    def test_phase_order_and_time(self):
        phases = [state_at(t)[0] for t in (0, 1.1, 1.5, 1.8, 2.08, 2.3, 2.8)]
        self.assertEqual(phases, ["STAND", "CROUCH", "PUSH", "FLIGHT", "LAND", "SETTLE", "STAND"])
        self.assertGreater(total_time(), 0)

    def test_continuous_boundaries(self):
        boundaries = (1.0, 1.4, 1.52, 2.07, 2.11, 2.51)
        for boundary in boundaries:
            left = state_at(boundary - 1e-8)
            right = state_at(boundary)
            with self.subTest(boundary=boundary):
                self.assertLess(abs(left[1] - right[1]), 1e-6)
                self.assertLess(abs(left[2] - right[2]), 1e-6)

    def test_monotonic_sample_time_and_expected_excursion(self):
        times = [i / 100 for i in range(int(total_time() * 100) + 1)]
        self.assertEqual(times, sorted(times))
        states = [state_at(t) for t in times]
        self.assertAlmostEqual(min(s[1] for s in states), 0.07, places=3)
        self.assertGreater(max(s[2] for s in states), 0.3)


if __name__ == "__main__":
    unittest.main()
