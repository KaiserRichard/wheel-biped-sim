"""Regression checks for damaged logs, timing boundaries and offline CLI."""
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

from wbrlog import codec, importer, spec, studio
from ident import delay, motor
from test_format import sample_records
from test_importer import make


class TestRobustness(unittest.TestCase):
    def test_corrupt_length_recovers_later_frame(self):
        a, b = sample_records()[3:5]
        bad = bytearray(codec.encode(a))
        struct.pack_into("<H", bad, 3, 4000)
        records, stats = codec.decode(bytes(bad) + codec.encode(b))
        self.assertEqual([r.name for r in records], [b.name])
        self.assertGreater(stats.skipped_bytes, 0)

    def test_malformed_text_and_sequence_wrap(self):
        def line(body):
            return f"{body}*{codec._xor(body):02X}"
        malformed = ["IMU", "IMU,xx,1,0", "IMU,1,1,999999,0,0,0,0,0,0",
                     "CAN_TX,1,1,1,0,1,8,00", "META,1,1,7b"]
        rec = sample_records()[4]
        rec.seq = 65535
        good1 = codec.to_text_line(rec)
        rec.seq = 1
        out, stats = codec.from_text_lines([good1] + [line(s) for s in malformed] + [codec.to_text_line(rec)])
        self.assertEqual(len(out), 2)
        self.assertEqual(stats.malformed_records, len(malformed))
        self.assertEqual(stats.seq_gaps, 1)

    def test_long_garbage_prefix(self):
        _, writer, _ = make(T=.02)
        ds = importer.import_log(b"garbage" * 100 + writer.to_bytes())
        self.assertIn("imu.gyro_S.x", ds.ch)

    def test_can_mode_commands_and_short_frames(self):
        meta = spec.default_meta()
        m = meta["motors"][0]
        records = [codec.Record("META", 0, {"json": meta})]
        for i, (dlc, data) in enumerate([(8, b"\xff" * 7 + b"\xfc"), (2, bytes(8))], 1):
            records.append(codec.Record("CAN_TX", i, dict(bus=m["bus"], flags=0,
                           can_id=m["can_id"], dlc=dlc, data=data), i))
        ds = importer.import_log(b"".join(map(codec.encode, records)))
        self.assertNotIn("ctrl.applied." + m["axis"], ds.ch)
        self.assertEqual(ds.stats["motor_mode_commands"], 1)
        self.assertEqual(ds.stats["unsupported_can"], 1)

    def test_empty_and_angular_resampling(self):
        self.assertTrue(math.isnan(importer.interp(importer.Series(), [0])[0]))
        s = importer.Series([0, 1], [3.1, -3.1])
        self.assertAlmostEqual(abs(importer.interp(s, [.5], "angle")[0]), math.pi)
        s = importer.Series([0, 1], [1, float("nan")])
        self.assertEqual(importer.interp(s, [0])[0], 1)
        with self.assertRaises(ValueError):
            importer.interp(s, [0], "typo")

    def test_multiwrap_imu_age(self):
        records = []
        for stamp in (200, 200):
            fields = {k: 0 for k in spec.CTRL_FIELDS + spec.CTRL_HEAD_FIELDS}
            fields["imu_t_us_used"] = 100
            records.append(codec.Record("CTRL", stamp, fields))
        ds = importer.Dataset()
        importer._imu_age(ds, records, [.0002, (2**32 + 200) * 1e-6], 0)
        for age in ds.ch["ctrl.imu_age"].y:
            self.assertAlmostEqual(age, .0001, places=12)

    def test_missing_tx_not_matched_to_next_cycle(self):
        ds = importer.Dataset()
        for t in (0, .003, .006):
            ds.add("ctrl.imu_age", t, .0005)
        ds.add("ctrl.applied.R_wheel", .005, 0)
        self.assertEqual(len(delay.delay_from_timestamps(ds, ["R_wheel"])["R_wheel"]), 1)

    def test_speed_sensitivity_respects_time_grid(self):
        corr = motor.speed_channel_corr(motor.TRUTH, motor.SENSOR, T=.5)
        self.assertTrue(math.isfinite(corr))
        self.assertLessEqual(abs(corr), 1)

    def test_cli_export_and_receipt(self):
        _, writer, _ = make(T=.02, events=[(0, 1, "studio_t0")])
        with tempfile.TemporaryDirectory() as temp:
            src, dst = Path(temp) / "input.bin", Path(temp) / "output.csv"
            src.write_bytes(writer.to_bytes())
            out = subprocess.run([sys.executable, "-m", "wbrlog", str(src), str(dst),
                                  "--format", "wide", "--event", "studio_t0",
                                  "--channels", "est.pitch"], capture_output=True, text=True)
            self.assertEqual(out.returncode, 0, out.stderr)
            receipt = json.loads(Path(str(dst) + ".json").read_text())
            self.assertEqual(receipt["source_sha256"], importer.file_sha256(src))
            self.assertEqual(receipt["meta"]["measurement_domain"], "synthetic")
            self.assertIn("t_s,est.pitch", dst.read_text())


if __name__ == "__main__":
    unittest.main()
