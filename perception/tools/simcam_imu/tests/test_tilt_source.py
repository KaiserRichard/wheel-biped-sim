import csv
import math
from pathlib import Path
import tempfile
import unittest

import numpy as np

from tools.imu_view.stream import CSV_HEADER
from tools.simcam_imu.tilt_source import TiltSource, find_windows


def write_synthetic(path, *, static=False, gap=False):
    """Known gravity/rates: still 2s, Euler ramp 2s, then hold 20/-10 deg."""
    bias = np.array([1.25, -0.30, 0.15])
    with Path(path).open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(CSV_HEADER)
        for i in range(701):
            t = i / 100
            if gap and 300 < i < 325:
                continue
            progress = 1.0 if static else min(max((t - 2) / 2, 0), 1)
            p, r = math.radians(20 * progress), math.radians(-10 * progress)
            pd = math.radians(10) if not static and 2 < t <= 4 else 0.0
            rd = math.radians(-5) if not static and 2 < t <= 4 else 0.0
            acc = (-math.sin(p), math.cos(p) * math.sin(r), math.cos(p) * math.cos(r))
            gyro = (rd, math.cos(r) * pd, -math.sin(r) * pd)
            writer.writerow((100 + t, *acc, *(np.degrees(gyro) + bias)))
    return bias


class TiltSourceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name) / "imu.csv"
        self.bias = write_synthetic(self.path)

    def test_known_dynamic_tilt_and_bias(self):
        source = TiltSource(self.path)
        np.testing.assert_allclose(np.degrees(source.bias), self.bias, atol=1e-12)
        self.assertEqual(source.timestamp_origin_s, 100)
        self.assertEqual(source.duration_s, 7)
        for t in (0, 1.5, 2, 2.5, 3, 4, 5, 7):
            progress = min(max((t - 2) / 2, 0), 1)
            tilt = source.at(t)
            self.assertAlmostEqual(math.degrees(tilt.pitch_rad), 20 * progress, delta=0.03)
            self.assertAlmostEqual(math.degrees(tilt.roll_rad), -10 * progress, delta=0.03)

    def test_static_nonzero_tilt_is_preserved_not_zeroed(self):
        write_synthetic(self.path, static=True)
        source = TiltSource(self.path)
        for t in (0, 3, 7):
            tilt = source.at(t)
            self.assertAlmostEqual(math.degrees(tilt.pitch_rad), 20, places=9)
            self.assertAlmostEqual(math.degrees(tilt.roll_rad), -10, places=9)

    def test_interpolation_matches_adjacent_samples_and_rates(self):
        source = TiltSource(self.path)
        a, b, middle = source.at(3.0), source.at(3.01), source.at(3.005)
        self.assertAlmostEqual(middle.pitch_rad, (a.pitch_rad + b.pitch_rad) / 2, places=12)
        self.assertAlmostEqual(middle.roll_rad, (a.roll_rad + b.roll_rad) / 2, places=12)
        self.assertAlmostEqual(middle.pitch_rate_deg_s,
                               math.degrees(b.pitch_rad - a.pitch_rad) / 0.01, places=8)
        self.assertAlmostEqual(middle.roll_rate_deg_s,
                               math.degrees(b.roll_rad - a.roll_rad) / 0.01, places=8)

    def test_scale_applies_to_angles_and_rates_after_filter(self):
        full, scaled = TiltSource(self.path), TiltSource(self.path, scale=0.25)
        np.testing.assert_allclose(full.bias, scaled.bias)
        a, b = full.at(3.005), scaled.at(3.005)
        for key in ("pitch_rad", "roll_rad", "pitch_rate_deg_s", "roll_rate_deg_s"):
            self.assertAlmostEqual(getattr(b, key), 0.25 * getattr(a, key))

    def test_explicit_bias_window(self):
        source = TiltSource(self.path, bias_window=(4.1, 5.0))
        np.testing.assert_allclose(np.degrees(source.bias), self.bias, atol=1e-12)
        self.assertEqual(source.bias_window, (4.1, 5.0))

    def test_reject_outside_interval_and_invalid_configuration(self):
        source = TiltSource(self.path)
        for t in (-0.1, 7.1, float("nan")):
            with self.subTest(t=t), self.assertRaises(ValueError):
                source.at(t)
        for interval in ((-1, 1), (5, 3), (0, float("inf"))):
            with self.subTest(interval=interval), self.assertRaises(ValueError):
                source.require_interval(*interval)
        for kw in (dict(scale=-1), dict(scale=float("nan")),
                   dict(bias_window=(0, 8)), dict(bias_window=(2, 1))):
            with self.subTest(kw=kw), self.assertRaises(ValueError):
                TiltSource(self.path, **kw)

    def test_gaps_are_not_replayed_as_measured_motion(self):
        write_synthetic(self.path, gap=True)
        source = TiltSource(self.path)
        with self.assertRaisesRegex(ValueError, "gap"):
            source.at(3.1)
        with self.assertRaisesRegex(ValueError, "gap"):
            source.require_interval(2.5, 1)
        source.require_interval(4, 2)

    def test_malformed_csv_is_rejected(self):
        for text in (",".join(CSV_HEADER) + "\n",
                     ",".join(CSV_HEADER) + "\n0,0,0,1,0,0,0\n0,0,0,1,0,0,0\n",
                     ",".join(CSV_HEADER) + "\n0,nan,0,1,0,0,0\n1,0,0,1,0,0,0\n"):
            with self.subTest(text=text):
                self.path.write_text(text)
                with self.assertRaises(ValueError):
                    TiltSource(self.path)


class WindowTests(unittest.TestCase):
    def test_pitch_or_roll_qualifies_and_short_span_is_excluded(self):
        times = np.arange(0, 10.01, 0.01)
        pitch = np.where((times >= 1) & (times <= 4), math.radians(4), 0)
        roll = np.where((times >= 5) & (times <= 7.5), math.radians(-5), 0)
        pitch[times >= 9] = math.radians(20)
        windows = find_windows(times, pitch, roll)
        self.assertEqual(len(windows), 2)
        self.assertEqual((windows[0].start_s, windows[0].end_s), (1, 4))
        self.assertEqual(windows[1].duration_s, 2.5)
        self.assertAlmostEqual(windows[1].peak_roll_deg, 5)

    def test_threshold_and_duration_are_strict(self):
        t = np.arange(0, 2.01, 0.01)
        self.assertEqual(find_windows(t, np.full_like(t, math.radians(4)), t * 0), [])
        self.assertEqual(find_windows(t, np.full_like(t, math.radians(3)), t * 0,
                                      min_duration_s=1), [])

    def test_sample_gap_splits_window(self):
        times = [0, 0.1, 0.2, 4, 4.1, 4.2]
        self.assertEqual(find_windows(times, [0.2] * 6, [0] * 6), [])

    def test_empty_and_invalid_vectors(self):
        self.assertEqual(find_windows([], [], []), [])
        for values in (([0, 0], [1, 1], [0, 0]), ([0, 1], [1], [0, 0]),
                       ([0, float("nan")], [1, 1], [0, 0])):
            with self.subTest(values=values), self.assertRaises(ValueError):
                find_windows(*values)


if __name__ == "__main__":
    unittest.main()
