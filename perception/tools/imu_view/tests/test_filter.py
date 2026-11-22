"""Filter, parser and sign-convention tests for tools/imu_view (no hardware, no MuJoCo)."""

import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from tools.imu_view import stream  # noqa: E402
from tools.imu_view.filter import (Madgwick, TiltFilters, accel_tilt, alpha_for,  # noqa: E402
                                   quat_from_pitch_roll)

D = math.radians


def still(pitch, roll):
    """Accelerometer reading (g) of a board at rest at this pitch/roll."""
    return (-math.sin(pitch), math.cos(pitch) * math.sin(roll), math.cos(pitch) * math.cos(roll))


class AccelTilt(unittest.TestCase):
    def test_flat(self):
        self.assertEqual(accel_tilt(0, 0, 1), (0.0, 0.0))

    def test_nose_down_is_positive_pitch(self):
        # Nose down: gravity's reaction leans toward -x, so ax < 0.
        pitch, roll = accel_tilt(*still(D(30), 0))
        self.assertAlmostEqual(pitch, D(30), places=9)
        self.assertLess(still(D(30), 0)[0], 0)
        self.assertAlmostEqual(roll, 0, places=9)

    def test_left_up_is_positive_roll(self):
        pitch, roll = accel_tilt(*still(0, D(-15)))
        self.assertAlmostEqual(roll, D(-15), places=9)

    def test_combined(self):
        p, r = accel_tilt(*still(D(-25), D(40)))
        self.assertAlmostEqual(p, D(-25), places=9)
        self.assertAlmostEqual(r, D(40), places=9)


class Complementary(unittest.TestCase):
    def test_alpha(self):
        self.assertAlmostEqual(alpha_for(0.5, 0.01), 0.98039, places=5)

    def test_static_tilt_with_gyro_bias(self):
        """Board held still at 20 deg nose down, gyro bias 3.25 deg/s on y (pitch axis)."""
        f = TiltFilters(tau_s=0.5)
        bias = (0, D(3.25), 0)
        for _ in range(1000):  # 10 s at 100 Hz
            f.update(still(D(20), D(5)), bias, 0.01)
        # Accel-only is exact for a still board.
        self.assertAlmostEqual(math.degrees(f.acc.pitch), 20, places=6)
        # Gyro-only drifts by bias * time = 32.5 deg in 10 s.
        self.assertAlmostEqual(math.degrees(f.gyro.pitch) - 20, 32.5, delta=0.6)
        # Complementary stays put, offset by bias * tau ~ 1.6 deg (see LESSON worked example).
        err = math.degrees(f.comp.pitch) - 20
        self.assertAlmostEqual(err, 3.25 * 0.5 * 1.0, delta=0.1)
        self.assertAlmostEqual(math.degrees(f.comp.roll), 5, delta=0.1)

    def test_static_tilt_no_bias_converges_exactly(self):
        f = TiltFilters(tau_s=0.5)
        f.update(still(0, 0), (0, 0, 0), 0.01)      # starts flat
        for _ in range(500):
            f.update(still(D(-10), D(8)), (0, 0, 0), 0.01)
        self.assertAlmostEqual(math.degrees(f.comp.pitch), -10, delta=0.01)
        self.assertAlmostEqual(math.degrees(f.comp.roll), 8, delta=0.01)

    def test_tracks_fake_motion_better_than_either_alone(self):
        f = TiltFilters(tau_s=0.5)
        errs = {"acc": 0.0, "gyro": 0.0, "comp": 0.0}
        n = 0
        for s in stream.fake(seconds=20, noise_g=0.05, seed=1):
            f.update(s.accel, s.gyro, 0.01)
            for k in errs:
                errs[k] += (getattr(f, k).pitch - s.truth[0]) ** 2
            n += 1
        rms = {k: math.degrees(math.sqrt(v / n)) for k, v in errs.items()}
        self.assertLess(rms["comp"], rms["acc"])
        self.assertLess(rms["comp"], rms["gyro"])
        self.assertLess(rms["comp"], 2.0)


class MadgwickSigns(unittest.TestCase):
    def test_static_tilt_matches_convention(self):
        m = Madgwick(beta=0.1)
        m.update(still(0, 0), (0, 0, 0), 0.01)
        for _ in range(3000):
            e = m.update(still(D(20), D(-10)), (0, 0, 0), 0.01)
        self.assertAlmostEqual(math.degrees(e.pitch), 20, delta=0.3)
        self.assertAlmostEqual(math.degrees(e.roll), -10, delta=0.3)

    def test_gyro_sign(self):
        """Positive gyro y = nose going down, so pitch increases."""
        m = Madgwick(beta=0.0)
        m.update(still(0, 0), (0, 0, 0), 0.01)
        for _ in range(100):
            e = m.update(still(0, 0), (0, D(10), 0), 0.01)
        self.assertAlmostEqual(math.degrees(e.pitch), 10, delta=0.2)


class Quaternion(unittest.TestCase):
    def test_nose_down_quaternion_moves_nose_down(self):
        w, x, y, z = quat_from_pitch_roll(D(30), 0)
        # Rotate the nose vector (1,0,0) by q: z component must become negative.
        nz = 2 * (x * z - w * y)
        self.assertAlmostEqual(nz, -math.sin(D(30)), places=9)

    def test_left_up_quaternion(self):
        w, x, y, z = quat_from_pitch_roll(0, D(20))
        lz = 2 * (y * z + w * x)  # z of rotated (0,1,0)
        self.assertAlmostEqual(lz, math.sin(D(20)), places=9)


class Stream(unittest.TestCase):
    def test_parse_bench_line(self):
        s = stream.parse_line("I,80010,1,-434,-90,7872,-1800,-213,1,-12")
        self.assertAlmostEqual(s.t, 80.010)
        self.assertAlmostEqual(s.accel[2], 7872 / 8192)
        self.assertAlmostEqual(math.degrees(s.gyro[0]), -213 / 65.5)

    def test_ignores_other_lines(self):
        lines = ["BOOT fw=imu_can_bench ver=1", "STAT ms=1 imu_ok=1", "I,12,1,1,2", "I,10,1,0,0,8192,0,0,0,0"]
        out = list(stream.from_lines(lines))
        self.assertEqual(len(out), 1)

    def test_garbage_bytes_skipped_and_counted(self):
        """8E1-garbled UART bytes must not crash the live reader (no UnicodeDecodeError)."""
        stats = stream.LineStats()
        lines = [b"\xb3\xf9I\x8c,\xff\xfe\n", b"I,10,1,0,0,8192,0,0,0,0\n", b"\x80\x81\x82\n",
                 b"I,20,1,0,0,7950,1,2,3\n"]
        out = list(stream.from_lines(lines, stats))
        self.assertEqual(len(out), 2)
        self.assertEqual((stats.ok, stats.bad), (2, 2))

    def test_line_without_temperature(self):
        s = stream.parse_line("I,20,1,0,0,7950,131,0,0")
        self.assertAlmostEqual(s.accel[2], 7950 / 8192)
        self.assertAlmostEqual(math.degrees(s.gyro[0]), 2.0)

    def test_remap_upside_down(self):
        s = stream.Sample(0, (0.1, 0.2, -1.0), (1, 2, 3))
        r = stream.remap(s, ("x", "-y", "-z"))
        self.assertEqual(r.accel, (0.1, -0.2, 1.0))
        self.assertEqual(r.gyro, (1, -2, -3))

    def test_csv_round_trip(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "run.csv"
            w = stream.Writer(path)
            src = list(stream.fake(seconds=0.5))
            for s in src:
                w.write(s)
            w.close()
            back = list(stream.from_csv(path))
        self.assertEqual(len(back), len(src))
        self.assertAlmostEqual(back[10].gyro[1], src[10].gyro[1], places=4)


if __name__ == "__main__":
    unittest.main()
