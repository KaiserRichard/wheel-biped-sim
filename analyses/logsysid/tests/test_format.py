"""Format, codec and DM frame tests. Run: python -m unittest discover -s tests -v"""
import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from wbrlog import codec, dm, importer, spec  # noqa: E402


def sample_records():
    meta = spec.default_meta()
    ctrl = {k: float(i) * 0.25 for i, k in enumerate(spec.CTRL_FIELDS)}
    ctrl.update({"ctrl_tick": 7, "imu_t_us_used": 123456, "imu_seq_used": 9, "flags": 1})
    est = {k: float(i) - 3.5 for i, k in enumerate(spec.EST_FIELDS)}
    return [
        codec.Record("META", 1, {"json": meta}, 0),
        codec.Record("SYNC", 2, {"sync_count": 3, "source": 0, "host_time_ns": 2**40 + 5}, 1),
        codec.Record("EVENT", 3, {"code": 1, "arg": 2, "label": b"studio_t0\0\0\0"}, 2),
        codec.Record("IMU", 4, {"gx": -32768, "gy": 32767, "gz": 0, "ax": 1, "ay": -1, "az": 4000, "imu_seq": 65535}, 3),
        codec.Record("IMU_TEMP", 5, {"temp_raw": -12}, 4),
        codec.Record("CAN_TX", 6, {"bus": 1, "flags": 0, "can_id": 0x01, "dlc": 8, "data": bytes(range(8))}, 5),
        codec.Record("CAN_RX", 7, {"bus": 3, "flags": 0, "can_id": 0x11, "dlc": 8, "data": bytes(range(8, 16))}, 6),
        codec.Record("CTRL", 8, ctrl, 7),
        codec.Record("EST", 9, est, 8),
        codec.Record("TIMING", 0xFFFFFFFF, {"loop_period_us": 3000, "exec_us": 200, "log_drop_count": 1, "can_tx_err": 0}, 9),
    ]


def same(a, b):
    if a.name != b.name or a.t_us != b.t_us or a.seq != b.seq:
        return False
    for k, v in a.fields.items():
        w = b.fields[k]
        if isinstance(v, float):
            if abs(v - w) > 1e-6 * max(1.0, abs(v)):
                return False
        elif v != w:
            return False
    return True


class TestSpec(unittest.TestCase):
    def test_sizes(self):
        self.assertEqual(spec.FRAME_OVERHEAD, 13)
        self.assertEqual(spec.record_size("IMU"), 27)
        self.assertEqual(spec.record_size("CAN_TX"), 26)
        self.assertEqual(spec.record_size("CTRL"), 153)
        self.assertEqual(spec.record_size("EST"), 61)
        self.assertEqual(spec.record_size("TIMING"), 21)

    def test_budget_matches_log_spec(self):
        # LOG_SPEC.md section 6 quotes these totals (bytes/s, rounded).
        self.assertEqual(round(spec.budget("FULL")[1]), 209374)
        self.assertEqual(round(spec.budget("IMU_ONLY")[1]), 27041)
        self.assertEqual(round(spec.budget("ONE_MOTOR")[1]), 73084)
        full = spec.budget("FULL")[1]
        self.assertLess(full / spec.LINKS_BYTES_PER_S["USB_FS_CDC"], 0.3)
        self.assertGreater(full / spec.LINKS_BYTES_PER_S["UART_2M"], 1.0)  # UART cannot carry FULL

    def test_registry_covers_every_studio_signal(self):
        reg = importer.CHANNELS
        studio_side = [k for k, v in reg.items() if v[1] != "none" and not v[1].startswith("none")]
        # M13 wire: 6 IMU + 12 encoder values (ticks are timestamps, not signals)
        self.assertEqual(sum(1 for k in studio_side if k.startswith(("imu.gyro_S", "imu.accel_S", "enc."))), 18)
        # controller output y: 23 values
        self.assertEqual(sum(1 for k, v in reg.items() if v[1].startswith("y[")), 23)
        # estimator diagnostic: 11 values
        self.assertEqual(sum(1 for k, v in reg.items() if v[1].startswith("diag[")), 11)
        # each Studio source index used exactly once
        srcs = [v[1].split()[0] for v in reg.values() if v[1].startswith(("y[", "diag[", "wire["))]
        self.assertEqual(len(srcs), len(set(srcs)))


class TestCodec(unittest.TestCase):
    def test_crc_check_value(self):
        self.assertEqual(codec.crc16_ccitt(b"123456789"), 0x29B1)

    def test_binary_roundtrip_all_types(self):
        recs = sample_records()
        blob = b"".join(codec.encode(r) for r in recs)
        out, st = codec.decode(blob)
        self.assertEqual(len(out), len(recs))
        self.assertEqual(st.crc_errors, 0)
        self.assertEqual(st.seq_gaps, 0)
        for a, b in zip(recs, out):
            self.assertTrue(same(a, b), a.name)

    def test_resync_after_garbage_and_corruption(self):
        recs = sample_records()
        frames = [codec.encode(r) for r in recs]
        bad = bytearray(frames[3])
        bad[15] ^= 0x40  # flip a payload bit in the IMU frame
        blob = b"\x00\xA5\x13garbage" + frames[0] + frames[1] + frames[2] + bytes(bad) + b"".join(frames[4:])
        out, st = codec.decode(blob)
        self.assertEqual([r.name for r in out], [r.name for r in recs if r.name != "IMU"])
        self.assertGreaterEqual(st.crc_errors, 1)
        self.assertEqual(st.seq_gaps, 1)  # the lost IMU frame shows up in the sequence
        self.assertGreater(st.skipped_bytes, 0)

    def test_truncated_tail(self):
        blob = b"".join(codec.encode(r) for r in sample_records())
        out, st = codec.decode(blob[:-5])
        self.assertEqual(len(out), len(sample_records()) - 1)

    def test_text_roundtrip_and_checksum(self):
        recs = sample_records()
        lines = [codec.to_text_line(r) for r in recs]
        out, st = codec.from_text_lines(lines)
        self.assertEqual(len(out), len(recs))
        for a, b in zip(recs, out):
            self.assertTrue(same(a, b), a.name)
        broken = lines[3].replace(",1,", ",2,", 1)
        out, st = codec.from_text_lines([broken])
        self.assertEqual(len(out), 0)
        self.assertEqual(st.crc_errors, 1)


class TestDM(unittest.TestCase):
    def test_zero_command_frame(self):
        # p=v=kp=kd=t=0 with PMAX 12.5, VMAX 30, TMAX 10
        self.assertEqual(dm.pack_mit_cmd(0, 0, 0, 0, 0, 12.5, 30, 10).hex(), "7fff7ff0000007ff")

    def test_roundtrip_within_half_lsb(self):
        rng = random.Random(3)
        P, V, T = 12.5, 45.0, 10.0
        for _ in range(500):
            p, v, t = rng.uniform(-P, P), rng.uniform(-V, V), rng.uniform(-T, T)
            kp, kd = rng.uniform(0, 500), rng.uniform(0, 5)
            c = dm.unpack_mit_cmd(dm.pack_mit_cmd(p, v, kp, kd, t, P, V, T), P, V, T)
            self.assertLessEqual(abs(c["p"] - p), dm.lsb(-P, P, 16) + 1e-12)
            self.assertLessEqual(abs(c["v"] - v), dm.lsb(-V, V, 12) + 1e-12)
            self.assertLessEqual(abs(c["t"] - t), dm.lsb(-T, T, 12) + 1e-12)
            self.assertLessEqual(abs(c["kp"] - kp), dm.lsb(0, 500, 12) + 1e-12)
            f = dm.unpack_feedback(dm.pack_feedback(5, 1, p, v, t, 41, 52, P, V, T), P, V, T)
            self.assertEqual((f["id"], f["err"], f["t_mos"], f["t_rotor"]), (5, 1, 41.0, 52.0))
            self.assertLessEqual(abs(f["pos"] - p), dm.lsb(-P, P, 16) + 1e-12)
            self.assertLessEqual(abs(f["vel"] - v), dm.lsb(-V, V, 12) + 1e-12)
            self.assertLessEqual(abs(f["tau"] - t), dm.lsb(-T, T, 12) + 1e-12)

    def test_saturation(self):
        c = dm.unpack_mit_cmd(dm.pack_mit_cmd(0, 0, 0, 0, 99.0, 12.5, 30, 10), 12.5, 30, 10)
        self.assertAlmostEqual(c["t"], 10.0, places=9)


if __name__ == "__main__":
    unittest.main()
