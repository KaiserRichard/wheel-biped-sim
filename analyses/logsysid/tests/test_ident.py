"""Identification: each estimator recovers a known synthetic truth."""
import math
import os
import random
import subprocess
import sys
import unittest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

from ident import allan, delay, motor  # noqa: E402
from ident.linalg import lstsq, nelder_mead, solve  # noqa: E402
from wbrlog import importer, spec, synth  # noqa: E402


class TestLinalg(unittest.TestCase):
    def test_solve_and_lstsq(self):
        self.assertEqual([round(v, 12) for v in solve([[2, 1], [1, 3]], [3, 5])], [0.8, 1.4])
        X = [[1.0, float(i)] for i in range(10)]
        y = [2.0 + 0.5 * i for i in range(10)]
        beta, cov, rss, dof = lstsq(X, y)
        self.assertAlmostEqual(beta[0], 2.0, places=10)
        self.assertAlmostEqual(beta[1], 0.5, places=10)

    def test_nelder_mead(self):
        p, f = nelder_mead(lambda x: (x[0] - 1) ** 2 + 10 * (x[1] + 2) ** 2, [0, 0], [0.5, 0.5])
        self.assertAlmostEqual(p[0], 1, places=4)
        self.assertAlmostEqual(p[1], -2, places=4)


class TestAllan(unittest.TestCase):
    def test_white_noise_level(self):
        rng = random.Random(11)
        dt, sigma = 1e-3, 0.0115
        y = [rng.gauss(0, sigma) for _ in range(200_000)]
        ad = allan.allan(y, dt)
        tau, a, c = ad[0]
        self.assertAlmostEqual(a / sigma, 1.0, delta=0.01)       # sigma_A(dt) = per-sample std
        f = allan.fit(ad, terms=("N", "K"))
        self.assertAlmostEqual(f["N"] / (sigma * math.sqrt(dt)), 1.0, delta=0.03)
        self.assertLess(f["K"], 1e-5)

    def test_studio_model_recovered(self):
        rng = random.Random(5)
        p = allan.STUDIO_GYRO
        y, b0 = allan.simulate(int(600 / 0.01), 0.01, rng=rng, **p)
        f = allan.fit(allan.allan(y, 0.01))
        self.assertAlmostEqual(f["N"] / (p["sigma_w"] * math.sqrt(1e-3)), 1.0, delta=0.05)
        self.assertAlmostEqual(f["K"] / p["drift"], 1.0, delta=0.3)

    def test_bias_window(self):
        N, K = 1.66e-5, 5.24e-5
        T = allan.best_bias_window(N, K)
        self.assertAlmostEqual(T, math.sqrt(3) * N / K)
        self.assertLess(allan.bias_error_std(N, K, T), allan.bias_error_std(N, K, 3 * T))
        self.assertLess(allan.bias_error_std(N, K, T), allan.bias_error_std(N, K, T / 3))


class TestMotor(unittest.TestCase):
    def test_lever(self):
        rng = random.Random(2)
        rows, r = motor.lever_experiment([0.1, 0.2, 0.3, 0.4, 0.5], motor.TRUTH, rng, r_err_rel=0.0)
        f = motor.fit_lever(rows, r)
        self.assertAlmostEqual(f["k"], motor.TRUTH["k"], delta=0.02)
        self.assertAlmostEqual(f["hyst"], -0.5 * motor.TRUTH["Tc"], delta=0.004)

    def test_friction(self):
        rng = random.Random(3)
        rows = motor.friction_experiment([-16, -4, -1, 1, 4, 16], 2.0, motor.TRUTH, motor.SENSOR, rng)
        f = motor.fit_friction(rows, motor.TRUTH["k"])
        self.assertAlmostEqual(f["Tc"] / motor.TRUTH["Tc"], 1.0, delta=0.05)
        self.assertAlmostEqual(f["b"] / motor.TRUTH["b"], 1.0, delta=0.05)

    def test_lag_delay_and_spin(self):
        rng = random.Random(4)
        log = motor.step_experiment(2.0, motor.TRUTH, motor.SENSOR, rng, period=0.001)
        ld = motor.fit_lag_delay(log)
        self.assertLess(abs(ld["d"] - motor.TRUTH["d"]), 2e-4)
        self.assertLess(abs(ld["lag"] / motor.TRUTH["lag"] - 1), 0.1)
        sp = motor.fit_spin(log, ld["d"], ld["lag"], friction=(motor.TRUTH["Tc"], motor.TRUTH["b"]))
        self.assertLess(abs(sp["a"] / (motor.TRUTH["k"] / motor.TRUTH["J"]) - 1), 0.02)

    def test_lag_response_step(self):
        y = motor.lag_response([0.0], [1.0], 0.001, 0.005, [0.0, 0.001, 0.006])
        self.assertEqual(y[0], 0.0)
        self.assertAlmostEqual(y[1], 0.0, places=12)
        self.assertAlmostEqual(y[2], 1 - math.exp(-1), places=12)  # Studio lag_expected 0.632


class TestDelay(unittest.TestCase):
    def test_regression(self):
        rng = random.Random(8)
        d = delay.simulate_rocking(5.0, 0.0036, rng)
        r = delay.fit_delay_regression(*d)
        self.assertLess(abs(r["D"] - 0.0036), 3e-4)
        self.assertAlmostEqual(r["K2"], -0.8, delta=0.05)

    def test_timestamps(self):
        meta = spec.default_meta()
        sig = {"gyro_S": lambda t: (0, 0, 0), "accel_S": lambda t: (0, 0, 9.8), "q": lambda a, t: 0.0,
               "dq": lambda a, t: 0.0, "tau_fb": lambda a, t: 0.0, "applied": lambda a, t: 0.0}
        w, info = synth.synth_log(0.3, meta, sig, imu_phase_us=500,
                                  ctrl=lambda t: {k: 0.0 for k in spec.CTRL_FIELDS})
        ds = importer.import_log(w.to_bytes())
        got = delay.delay_from_timestamps(ds, ["R_hip3", "R_hip4", "R_wheel"])
        # IMU 500 us old at every cycle; sends at +50, +1050, +2050 us
        for ax, expect in (("R_hip3", 550e-6), ("R_hip4", 1550e-6), ("R_wheel", 2550e-6)):
            self.assertTrue(all(abs(v - expect) < 1e-9 for v in got[ax]), ax)


class TestRehearsalQuick(unittest.TestCase):
    def test_quick_run(self):
        out = subprocess.run([sys.executable, os.path.join(HERE, "rehearsal.py"), "--quick"],
                             capture_output=True, text=True, cwd=HERE, timeout=600)
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertTrue(os.path.exists(os.path.join(HERE, "out_quick", "REHEARSAL.md")))


if __name__ == "__main__":
    unittest.main()
