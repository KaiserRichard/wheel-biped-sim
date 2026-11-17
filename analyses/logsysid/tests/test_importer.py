"""Importer, unit conversion, mapping, Studio loaders, resampling and alignment."""
import csv
import math
import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

from wbrlog import dm, importer, spec, studio, synth  # noqa: E402

AX = spec.STUDIO_AXES


def signals():
    return {
        "gyro_S": lambda t: (0.5 * math.sin(3 * t), -0.2, 0.1 * t),
        "accel_S": lambda t: (0.3, -0.1 * math.cos(t), 9.80665),
        "q": lambda a, t: (2.9 + t) if a == "R_wheel" else 0.4 * math.sin(2 * t + AX.index(a)),
        "dq": lambda a, t: 0.8 * math.cos(2 * t + AX.index(a)),
        "tau_fb": lambda a, t: 0.3 * math.sin(t + AX.index(a)),
        "applied": lambda a, t: 0.5 * math.cos(t + AX.index(a)),
    }


def ctrl_fn(t):
    f = {k: 0.0 for k in spec.CTRL_FIELDS}
    f.update({"alpha_R": 0.1 + t, "alpha_L": -(0.1 + t), "d_alpha_R": 1.0, "d_alpha_L": -1.0,
              "pitch": 0.02, "pitch_gyro": 0.3, "x_filter": 0.5 * t, "v_filter": 0.5,
              "L0_R": 0.15, "L0_L": 0.16, "roll": 0.01, "yaw_total": 7.0,
              "tau_req_R0": 1.0, "tau_req_R1": 2.0, "tau_req_L0": 3.0, "tau_req_L1": 4.0,
              "wheel_req_R": 0.5, "wheel_req_L": -0.5})
    return f


def est_fn(t):
    return {"ready": 1.0, "pitch": 0.01 * t, "roll": 0.0, "yaw": 0.0, "body_speed": 0.2,
            "L_L": 0.12, "L_R": 0.13, "gbias_x": 1e-3, "gbias_y": 0.0, "gbias_z": 0.0,
            "accel_bias": 0.0, "accel_norm": 9.8}


def make(meta=None, T=0.6, **kw):
    meta = meta or spec.default_meta()
    w, info = synth.synth_log(T, meta, signals(), ctrl=ctrl_fn, est=est_fn, **kw)
    return meta, w, info


class TestImporter(unittest.TestCase):
    def test_units_and_values_within_quantisation(self):
        meta, w, info = make()
        ds = importer.import_log(w.to_bytes(), t0="none")
        t0 = info["t0_us"] * 1e-6
        sig = signals()
        g_lsb = 2000 / 32768 * math.pi / 180
        a_lsb = 6 / 32768 * 9.80665
        s = ds.ch["imu.gyro_S.x"]
        for t, y in zip(s.t[::50], s.y[::50]):
            self.assertLessEqual(abs(y - sig["gyro_S"](t - t0)[0]), g_lsb / 2 + 1e-12)
        s = ds.ch["imu.accel_S.z"]
        for t, y in zip(s.t[::50], s.y[::50]):
            self.assertLessEqual(abs(y - 9.80665), a_lsb / 2 + 1e-12)
        for m in meta["motors"]:
            ax = m["axis"]
            q, dq, tau = ds.ch[f"enc.q.{ax}"], ds.ch[f"enc.dq.{ax}"], ds.ch[f"motor.tau_fb.{ax}"]
            for i in range(0, len(q.t), 20):
                t = q.t[i] - t0
                self.assertLessEqual(abs(importer.wrap_pi(q.y[i] - sig["q"](ax, t))), dm.lsb(-12.5, 12.5, 16))
                self.assertLessEqual(abs(dq.y[i] - sig["dq"](ax, t)), dm.lsb(-m["vmax"], m["vmax"], 12))
                self.assertLessEqual(abs(tau.y[i] - sig["tau_fb"](ax, t)), dm.lsb(-10, 10, 12))
            ap = ds.ch[f"ctrl.applied.{ax}"]
            for i in range(0, len(ap.t), 20):
                self.assertLessEqual(abs(ap.y[i] - sig["applied"](ax, ap.t[i] - t0)), dm.lsb(-10, 10, 12))
        self.assertEqual(ds.stats["crc_errors"], 0)
        self.assertEqual(ds.stats["seq_gaps"], 0)

    def test_encoder_wraps_like_studio(self):
        meta, w, info = make()
        ds = importer.import_log(w.to_bytes())
        q = ds.ch["enc.q.R_wheel"].y
        self.assertTrue(all(-math.pi <= v < math.pi for v in q))
        self.assertTrue(any(v < 0 for v in q))  # 2.9 + t rad passes pi at t = 0.24 s and wraps

    def test_sign_and_offset_mapping(self):
        meta = spec.default_meta()
        meta["motors"][0]["sign"] = -1
        meta["motors"][0]["offset_rad"] = 0.25
        meta, w, info = make(meta)
        ds = importer.import_log(w.to_bytes(), t0="none")
        t0 = info["t0_us"] * 1e-6
        ax = meta["motors"][0]["axis"]
        q = ds.ch[f"enc.q.{ax}"]
        for i in range(0, len(q.t), 25):
            self.assertLessEqual(abs(q.y[i] - signals()["q"](ax, q.t[i] - t0)), 4e-4)
        # request uses the motor sign too
        self.assertAlmostEqual(ds.ch[f"ctrl.request.{ax}"].y[0], -3.0, places=6)

    def test_ctrl_to_studio_state(self):
        meta, w, info = make()
        ds = importer.import_log(w.to_bytes(), t0="none")
        t0 = info["t0_us"] * 1e-6
        x3 = ds.ch["ctrl.x.leg_minus_pitch"]
        for t, y in zip(x3.t[:20], x3.y[:20]):
            tt = t - t0
            # mirror_left: alpha_c = (alpha_R - alpha_L)/2 = 0.1 + t ; minus pitch 0.02
            self.assertAlmostEqual(y, 0.1 + tt - 0.02, places=5)
        self.assertAlmostEqual(ds.ch["ctrl.x.leg_minus_pitch_rate"].y[0], 1.0 - 0.3, places=6)
        self.assertAlmostEqual(ds.ch["ctrl.yaw"].y[0], importer.wrap_pi(7.0), places=6)
        self.assertAlmostEqual(ds.ch["ctrl.L.L"].y[0], 0.16, places=6)
        self.assertAlmostEqual(ds.ch["ctrl.request.R_wheel"].y[0], 0.5, places=6)
        self.assertAlmostEqual(ds.ch["ctrl.request.L_hip4"].y[0], 4.0, places=6)

    def test_text_equals_binary(self):
        meta, w, info = make(T=0.2)
        a = importer.import_log(w.to_bytes())
        b = importer.import_log(w.to_text().encode())
        self.assertEqual(a.names(), b.names())
        for n in a.names():
            self.assertEqual(len(a.ch[n].y), len(b.ch[n].y), n)
            for x, y in zip(a.ch[n].y, b.ch[n].y):
                self.assertAlmostEqual(x, y, places=6, msg=n)

    def test_timestamp_wrap(self):
        meta, w, info = make(T=0.5, t0_us=2**32 - 200_000)
        ds = importer.import_log(w.to_bytes())
        t = ds.ch["imu.gyro_S.x"].t
        d = [b - a for a, b in zip(t, t[1:])]
        self.assertAlmostEqual(min(d), 1e-3, places=9)
        self.assertAlmostEqual(max(d), 1e-3, places=9)
        self.assertGreater(t[-1], 0.45)

    def test_imu_age_and_unmapped(self):
        meta = spec.default_meta()
        meta2 = spec.default_meta()
        meta2["motors"] = meta2["motors"][:5]  # importer will not know R_wheel
        meta, w, info = make(meta, imu_phase_us=300)
        ds = importer.import_log(w.to_bytes(), meta_override=meta2)
        ages = ds.ch["ctrl.imu_age"].y
        self.assertTrue(all(abs(a - 0.0007) < 1e-9 for a in ages))  # 3 ms cycle, IMU at +300 us
        self.assertGreater(ds.stats["unmapped_tx"], 0)
        self.assertNotIn("enc.q.R_wheel", ds.ch)

    def test_wide_csv(self):
        meta, w, info = make(T=0.1)
        ds = importer.import_log(w.to_bytes())
        grid = studio.uniform_grid(0.01, 0.09, 0.001)
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "x.csv")
            importer.write_wide_csv(ds, p, ["imu.gyro_S.y", "enc.q.L_hip3"], grid, header_meta={"source": "test"})
            with open(p, newline="") as fh:
                rows = [r for r in csv.reader(fh) if r and not r[0].startswith("#")]
        self.assertEqual(rows[0], ["t_s", "imu.gyro_S.y", "enc.q.L_hip3"])
        self.assertEqual(len(rows), len(grid) + 1)
        self.assertAlmostEqual(float(rows[5][1]), -0.2, delta=2000 / 32768 * math.pi / 180)


class TestStudio(unittest.TestCase):
    def test_load_estimation_csv(self):
        p = os.path.join(HERE, "m13_loop3_estimation.csv")
        if not os.path.exists(p):
            self.skipTest("Studio CSV not present")
        ds = studio.load_estimation_csv(p)
        self.assertEqual(len(ds.ch["est.pitch"]), 501)
        # first row: estimator not ready, so est L = 0 and truth L = 0.12 m
        self.assertEqual(ds.ch["est.L.L"].y[0], 0.0)
        self.assertAlmostEqual(ds.ch["truth.L.L"].y[0], 0.12)
        self.assertEqual(ds.ch["est.ready"].y[0], 0.0)
        self.assertEqual(ds.ch["est.ready"].y[101], 1.0)  # ready at t = 0.101 s

    def test_wire_and_controller_loaders(self):
        with tempfile.TemporaryDirectory() as d:
            pw = os.path.join(d, "w.csv")
            with open(pw, "w") as f:
                for k in range(5):
                    row = [k * 1e-3, k] + [0.1 * k] * 6 + [k if k != 2 else -1] + [0.2] * 6 + [0.3] * 6
                    f.write(",".join(map(str, row)) + "\n")
            ds = studio.load_wire_csv(pw)
            self.assertEqual(len(ds.ch["imu.gyro_S.x"]), 5)
            self.assertEqual(len(ds.ch["enc.q.L_hip3"]), 4)  # one dropped encoder packet
            pc = os.path.join(d, "c.csv")
            with open(pc, "w") as f:
                f.write(",".join(map(str, [0.002] + list(range(23)))) + "\n")
            dc = studio.load_controller_csv(pc)
            self.assertEqual(dc.ch["ctrl.applied.L_hip3"].y[0], 0.0)
            self.assertEqual(dc.ch["ctrl.request.R_wheel"].y[0], 11.0)
            self.assertEqual(dc.ch["ctrl.x.pitch_rate"].y[0], 17.0)
            self.assertEqual(dc.ch["ctrl.yaw"].y[0], 22.0)

    def test_wire_export_for_studio_replay(self):
        meta, w, info = make(T=0.3)
        ds = importer.import_log(w.to_bytes())
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "wire.csv")
            studio.write_wire_csv(ds, p, 0.05, 0.25)
            back = studio.load_wire_csv(p)
            with open(p) as fh:
                ncol = {len(line.split(",")) for line in fh}
        self.assertEqual(ncol, {21})
        g = back.ch["imu.gyro_S.x"]
        self.assertEqual(len(g), 201)  # one IMU packet per 1 ms tick
        src = ds.ch["imu.gyro_S.x"]
        # each tick carries the newest IMU sample acquired in (t - 1 ms, t]
        for t, y in zip(g.t[::40], g.y[::40]):
            self.assertAlmostEqual(y, importer.interp(src, [t + 0.05], "zoh")[0], places=12)
        q = back.ch["enc.q.R_hip3"]
        self.assertAlmostEqual(q.y[100], importer.interp(ds.ch["enc.q.R_hip3"], [0.05 + 0.1], "zoh")[0], places=12)

    def test_resample(self):
        s = importer.Series([0.0, 1.0, 2.0], [0.0, 10.0, 20.0])
        self.assertEqual(importer.interp(s, [0.5, 1.5], "linear"), [5.0, 15.0])
        self.assertEqual(importer.interp(s, [0.5, 1.5, 2.0], "zoh"), [0.0, 10.0, 20.0])
        self.assertTrue(math.isnan(importer.interp(s, [-0.1], "zoh")[0]))

    def test_event_sync_and_compare_zero(self):
        meta = spec.default_meta()
        off = 0.1234
        f = lambda t: 0.05 * math.sin(2 * math.pi * (0.5 + 2 * t) * t)  # noqa: E731
        st = importer.Dataset()
        for k in range(1001):
            st.add("est.pitch", k * 1e-3, f(k * 1e-3))
        w, info = synth.synth_log(1.2, meta, signals(), events=[(off, 1, "studio_t0")],
                                  est=lambda t: dict(est_fn(t), pitch=f(t - off)))
        ds = importer.import_log(w.to_bytes(), t0="first")
        lag = studio.event_time(ds, "studio_t0")
        self.assertAlmostEqual(lag, off + 10e-6, places=9)
        lag2 = studio.align_offset(st.ch["est.pitch"], ds.ch["est.pitch"], max_lag=0.3)
        self.assertLess(abs(lag2 - lag), 20e-6)
        c = studio.compare(ds, st, ["est.pitch"], offset=lag, t_range=(0.01, 0.99))["est.pitch"]
        # 3 ms samples, linear interpolation of a chirp up to 4.5 Hz: |e| <= dt^2/8 * max|f''|
        bound = (3e-3) ** 2 / 8 * 0.05 * (2 * math.pi * 4.5) ** 2 * 1.5
        self.assertLess(c["max_abs"], bound)


if __name__ == "__main__":
    unittest.main()
