import contextlib
import csv
import io
import json
from pathlib import Path
import sqlite3
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import take_tools as t
from fixture import make, image, imu, string


def read_png(path):
    blob = path.read_bytes()
    assert blob[:8] == b'\x89PNG\r\n\x1a\n'
    pos, compressed, dimensions = 8, b'', None
    while pos < len(blob):
        n = struct.unpack('>I', blob[pos:pos+4])[0]
        kind, data = blob[pos+4:pos+8], blob[pos+8:pos+8+n]
        crc = struct.unpack('>I', blob[pos+8+n:pos+12+n])[0]
        assert crc == zlib.crc32(kind+data) & 0xffffffff
        if kind == b'IHDR':
            dimensions = struct.unpack('>IIBBBBB', data)
        if kind == b'IDAT':
            compressed += data
        pos += n+12
    width, height, depth, color, *_ = dimensions
    assert (width, height, depth, color) == (8, 6, 8, 0)
    raw = zlib.decompress(compressed)
    assert len(raw) == height*(width+1)
    assert all(raw[y*(width+1)] == 0 for y in range(height))
    return b''.join(raw[y*(width+1)+1:(y+1)*(width+1)] for y in range(height))


class TakeTests(unittest.TestCase):
    def test_finite_extreme_accel_interpolation_preserves_endpoints(self):
        streams = {"gyro": [{"sdk_ns": ns, "gyro": [0., 0., 0.]}
                            for ns in (0, 5_000_000, 10_000_000)],
                   "accel": [{"sdk_ns": 0, "accel": [-1e308, 0., 0.]},
                             {"sdk_ns": 10_000_000, "accel": [1e308, 0., 0.]}]}
        report = {"single_known_domain": True, "streams": {
            name: dict(backward=0, duplicates=0, domain_changes=0, missing_metadata=0)
            for name in ("gyro", "accel")}}
        rows, counts = t.combined_imu(streams, report)
        self.assertEqual([r[4] for r in rows], [-1e308, 0., 1e308])
        self.assertEqual(counts["emitted"], 3)

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.file = self.root / 'take.db3'

    def fixture(self, variant=None, endian='<'):
        return make(self.file, variant, endian)

    def cli(self, *args):
        with contextlib.redirect_stdout(io.StringIO()) as out, contextlib.redirect_stderr(io.StringIO()) as err:
            code = t.main(list(map(str, args)))
        return code, out.getvalue(), err.getvalue()

    def test_round_trip_pixels_stamps_hashes_and_derivative(self):
        expected = self.fixture()
        out = self.root/'euroc'
        m = t.export_euroc(self.file, out)
        self.assertEqual(m['status'], 'EXPORTED')
        self.assertEqual(m['raw_counts'], {'gyro': 201, 'accel': 99})
        self.assertEqual(m['imu_derivative']['counts'], dict(input_gyro=201, input_accel=99,
                         emitted=197, dropped_prefix=2, dropped_tail=2,
                         dropped_invalid_bracket=0, dropped_clock_or_order=0))
        self.assertEqual((out/'times_ns.txt').read_text().splitlines(),
                         [str(s['sdk_ns']) for s in expected['ir1'][1:-1]])
        self.assertEqual(m['times_file']['counts'], dict(input_ir1=31, emitted=29,
                         dropped_head=1, dropped_tail=1, dropped_unpaired_or_mismatched=0,
                         dropped_clock_or_order=0, dropped_no_combined_imu=0))
        self.assertEqual(m['times_file']['stamp_domain'], 'hardware_clock')
        self.assertEqual(json.loads((out/'manifest.json').read_text())['times_file'], m['times_file'])
        for i, name in enumerate(('ir1', 'ir2')):
            rows = list(csv.reader(io.StringIO((out/f'mav0/cam{i}/data.csv').read_text())))[1:]
            self.assertEqual([int(r[0]) for r in rows], [s['sdk_ns'] for s in expected[name]])
            self.assertEqual(m['camera_counts'][name]['exported'], 31)
            for row, sample in zip(rows, expected[name]):
                self.assertEqual(read_png(out/f'mav0/cam{i}/data'/row[1]), sample['pixels'])
        rows = list(csv.reader(io.StringIO((out/'mav0/imu0/data.csv').read_text())))[1:]
        self.assertEqual([int(r[0]) for r in rows], [s['sdk_ns'] for s in expected['gyro'][2:-2]])
        for row, g in zip(rows, expected['gyro'][2:-2]):
            j = (g['sdk_ns']-expected['accel'][0]['sdk_ns'])/10_000_000
            self.assertEqual(list(map(float, row[1:4])), g['vector'])
            self.assertEqual(list(map(float, row[4:])), [j, 2*j, -j])
        for name in ('gyro', 'accel'):
            raw = list(csv.reader(io.StringIO((out/f'mav0/imu0/raw_{name}.csv').read_text())))[1:]
            self.assertEqual(len(raw), len(expected[name]))
            self.assertEqual([int(r[0]) for r in raw], [s['sdk_ns'] for s in expected[name]])
            self.assertEqual([int(r[2]) for r in raw], [s['bag_ns'] for s in expected[name]])
            self.assertEqual([int(r[3]) for r in raw], [s['bag_ns'] for s in expected[name]])
            self.assertEqual([list(map(float, r[6:9])) for r in raw], [s['vector'] for s in expected[name]])
        self.assertEqual(m['input']['sha256'], t.sha256(self.file))
        self.assertEqual(set(m['output_hashes']), {str(p.relative_to(out)) for p in out.rglob('*') if p.is_file() and p.name != 'manifest.json'})
        for name, digest in m['output_hashes'].items():
            self.assertEqual(t.sha256(out/name), digest)

    def test_times_strict_combined_imu_endpoints_and_png_names(self):
        first, last = 1_244_567_891, 2_224_567_891
        stamps = [first-1, first, first+1, (first+last)//2, last-1, last, last+1]
        make(self.file, camera_stamps=stamps)
        out = self.root/'endpoints'
        code, raw, err = self.cli('export-euroc', self.file, out)
        self.assertEqual(code, 0, err)
        info = json.loads(raw)['times_file']
        self.assertEqual((info['first_imu_ns'], info['last_imu_ns']), (first, last))
        self.assertEqual(info['counts']['dropped_head'], 2)
        self.assertEqual(info['counts']['dropped_tail'], 2)
        self.assertIn('preceding IMU', info['exclusion_reasons']['dropped_head'])
        self.assertIn('following IMU', info['exclusion_reasons']['dropped_tail'])
        lines = (out/'times_ns.txt').read_text().splitlines()
        self.assertEqual(lines, list(map(str, stamps[2:5])))
        for cam in (0, 1):
            entries = list(csv.reader(io.StringIO((out/f'mav0/cam{cam}/data.csv').read_text())))[1:]
            self.assertEqual([int(r[0]) for r in entries], stamps)
            for line in lines:
                self.assertTrue((out/f'mav0/cam{cam}/data/{line}.png').is_file())
        # Mirror the upstream IMU cursor, including its unguarded look-ahead.
        imu_stamps = [int(row[0]) for row in list(csv.reader(
            io.StringIO((out/'mav0/imu0/data.csv').read_text())))[1:]]
        cursor = 0
        while imu_stamps[cursor] <= int(lines[0]):
            cursor += 1
        cursor -= 1
        self.assertGreaterEqual(cursor, 0)
        for line in lines[1:]:
            while imu_stamps[cursor] <= int(line):
                cursor += 1
            self.assertLess(cursor, len(imu_stamps))

    def test_times_empty_when_all_cameras_outside_combined_interval(self):
        first, last = 1_244_567_891, 2_224_567_891
        for stamps, head, tail in (([first-1, first], 2, 0),
                                   ([last, last+1], 0, 2),
                                   ([first, last], 1, 1)):
            with self.subTest(stamps=stamps):
                self.file.unlink(missing_ok=True)
                make(self.file, camera_stamps=stamps)
                out = self.root/f'outside-{head}-{tail}'
                m = t.export_euroc(self.file, out)
                self.assertEqual((out/'times_ns.txt').read_bytes(), b'')
                self.assertEqual(m['status'], 'EXPORTED')
                self.assertEqual(m['times_file']['counts']['emitted'], 0)
                self.assertEqual(m['times_file']['counts']['dropped_head'], head)
                self.assertEqual(m['times_file']['counts']['dropped_tail'], tail)

    def test_times_no_combined_imu_or_single_sample(self):
        for keep_one in (False, True):
            with self.subTest(keep_one=keep_one):
                self.file.unlink(missing_ok=True)
                self.fixture()
                with sqlite3.connect(self.file) as con:
                    if keep_one:
                        con.execute('DELETE FROM messages WHERE topic_id IN (5,6) AND timestamp != ?',
                                    (100_015_000_000,))
                    else:
                        con.execute('DELETE FROM messages WHERE topic_id IN (5,6)')
                out = self.root/str(keep_one)
                m = t.export_euroc(self.file, out, allow_diagnostic=True)
                info = m['times_file']
                self.assertEqual((out/'times_ns.txt').read_bytes(), b'')
                self.assertEqual(info['counts']['emitted'], 0)
                if keep_one:
                    self.assertEqual(info['first_imu_ns'], info['last_imu_ns'])
                    self.assertEqual(info['counts']['dropped_head'], 1)
                    self.assertEqual(info['counts']['dropped_tail'], 30)
                else:
                    self.assertIsNone(info['first_imu_ns'])
                    self.assertIsNone(info['last_imu_ns'])
                    self.assertEqual(info['counts']['dropped_no_combined_imu'], 31)

    def test_times_require_true_pairs_and_identical_original_stamps(self):
        for variant in ('small_skew', 'skew', 'missing_frame', 'frame_mismatch'):
            with self.subTest(variant=variant):
                self.file.unlink(missing_ok=True)
                expected = self.fixture(variant)
                out = self.root/variant
                m = t.export_euroc(self.file, out, allow_diagnostic=True)
                excluded = 10 if variant == 'missing_frame' else 15
                lines = (out/'times_ns.txt').read_text().splitlines()
                self.assertEqual(lines, [str(s['sdk_ns']) for i, s in enumerate(expected['ir1'])
                                         if 0 < i < 30 and i != excluded])
                self.assertEqual(m['times_file']['counts']['dropped_unpaired_or_mismatched'], 1)
                self.assertEqual(m['times_file']['counts']['emitted'], 28)
                self.assertEqual(m['status'], 'EXPORTED' if variant == 'small_skew' else 'DIAGNOSTIC')
                for line in lines:
                    for cam in (0, 1):
                        self.assertTrue((out/f'mav0/cam{cam}/data/{line}.png').is_file())

    def test_times_suppressed_for_bad_clock_or_order(self):
        for variant in ('backward', 'domain_change', 'cross_domain', 'unknown_domain',
                        'missing_metadata', 'duplicate', 'camera_backward'):
            with self.subTest(variant=variant):
                self.file.unlink(missing_ok=True)
                self.fixture(variant)
                out = self.root/variant
                m = t.export_euroc(self.file, out, allow_diagnostic=True)
                self.assertEqual((out/'times_ns.txt').read_bytes(), b'')
                self.assertEqual(m['times_file']['counts']['emitted'], 0)
                key = 'dropped_clock_or_order' if variant in ('duplicate', 'camera_backward') else 'dropped_no_combined_imu'
                self.assertEqual(m['times_file']['counts'][key], 31)

    def test_inspection_and_cli_outputs(self):
        self.fixture()
        code, raw, _ = self.cli('inspect', self.file)
        self.assertEqual(code, 0)
        info = json.loads(raw)
        self.assertEqual(info['format'], 'sqlite')
        self.assertEqual(len(info['topics']), 9)
        self.assertEqual(info['message_count'], 725)
        self.assertEqual(set(info['topics'][0]) & {'name', 'type', 'serialization_format', 'offered_qos_profiles', 'message_count', 'first_ns', 'last_ns'},
                         {'name', 'type', 'serialization_format', 'offered_qos_profiles', 'message_count', 'first_ns', 'last_ns'})
        code, raw, md = self.cli('audit', self.file)
        self.assertEqual(code, 0)
        report = json.loads(raw)
        self.assertIn('Take audit: PASS', md)
        self.assertEqual(report['streams']['gyro']['rate_hz'], 200)
        self.assertEqual(report['stereo']['valid_pairs'], 31)
        self.assertEqual(report['stereo']['max_abs_skew_ns'], 0)
        self.assertEqual(report['streams']['ir1']['max_interval_ns'], 33_333_333)
        code, _, _ = self.cli('audit', self.file, '--json', self.root/'a.json', '--markdown', self.root/'a.md')
        self.assertEqual(code, 0)
        self.assertEqual(json.loads((self.root/'a.json').read_text()), report)

    def test_corruptions_refused_or_marked(self):
        for variant in ('missing_frame', 'backward', 'duplicate', 'domain_change', 'skew', 'cross_domain', 'unknown_domain', 'missing_metadata'):
            with self.subTest(variant=variant):
                self.file.unlink(missing_ok=True)
                self.fixture(variant)
                report = t.audit(self.file)
                self.assertEqual(report['status'], 'FAIL')
                out = self.root/variant
                self.assertEqual(self.cli('audit', self.file)[0], 1)
                self.assertEqual(self.cli('export-euroc', self.file, out)[0], 2)
                self.assertFalse(out.exists())
                self.assertEqual(self.cli('export-euroc', self.file, out, '--allow-diagnostic')[0], 0)
                m = json.loads((out/'manifest.json').read_text())
                self.assertEqual(m['status'], 'DIAGNOSTIC')
                self.assertTrue(m['rejections'])
                if variant in ('backward', 'domain_change', 'cross_domain', 'unknown_domain', 'missing_metadata'):
                    self.assertEqual(m['imu_derivative']['counts']['emitted'], 0)
                if variant == 'missing_frame':
                    self.assertEqual(report['streams']['ir2']['gaps_gt_1_5_periods'], 1)
                    self.assertEqual(report['stereo']['valid_pairs'], 30)
                if variant == 'duplicate':
                    self.assertEqual(report['streams']['ir1']['duplicates'], 1)
                    self.assertEqual(m['camera_counts']['ir1']['exported'], 30)
                if variant == 'domain_change':
                    self.assertEqual(report['streams']['gyro']['domain_changes'], 1)
                    self.assertIsNone(report['streams']['gyro']['rate_hz'])
                if variant == 'backward':
                    self.assertEqual(report['streams']['gyro']['backward'], 1)
                if variant == 'skew':
                    self.assertEqual(report['stereo']['max_abs_skew_ns'], 2_000_000)

    def test_unknown_encoding_truncation_and_encapsulation_are_fatal(self):
        for variant in ('encoding', 'truncated', 'encapsulation'):
            with self.subTest(variant=variant):
                self.file.unlink(missing_ok=True)
                self.fixture(variant)
                out = self.root/variant
                code, _, err = self.cli('export-euroc', self.file, out, '--allow-diagnostic')
                self.assertEqual(code, 2)
                self.assertFalse(out.exists())
                self.assertIn('message', err)

    def test_gap_is_not_interpolated(self):
        self.fixture('accel_gap')
        m = t.export_euroc(self.file, self.root/'gap')
        self.assertEqual(t.audit(self.file)['streams']['accel']['gaps_gt_1_5_periods'], 1)
        self.assertEqual(m['imu_derivative']['counts']['dropped_invalid_bracket'], 4)
        self.assertEqual(m['imu_derivative']['counts']['emitted'], 193)

    def test_big_endian_and_mono8(self):
        for variant, endian in ((None, '>'), ('mono8', '<')):
            self.file.unlink(missing_ok=True)
            expected = self.fixture(variant, endian)
            samples, _, _ = t.load(self.file)
            self.assertEqual(samples['ir1'][0]['pixels'], expected['ir1'][0]['pixels'])
            self.assertEqual(samples['gyro'][40]['gyro'], expected['gyro'][40]['vector'])
            self.assertEqual(t.audit(self.file)['status'], 'PASS')

    def test_magic_not_extension(self):
        for blob, fmt in ((b'#ROSBAG V2.0\nextra', 'ros1'), (b'not a bag', 'unknown')):
            self.file.write_bytes(blob)
            code, raw, _ = self.cli('inspect', self.file)
            self.assertEqual(code, 3)
            self.assertEqual(json.loads(raw)['format'], fmt)
            self.assertEqual(self.cli('audit', self.file)[0], 3)
            self.assertEqual(self.cli('export-euroc', self.file, self.root/'no')[0], 3)

    def test_exact_decimal_conversion_rejects_sub_ns(self):
        self.assertEqual(t.ms_to_ns('1234567890123.000001'), 1234567890123000001)
        for value in ('NaN', 'Infinity', '1.0000001', '-1', 'garbage'):
            with self.assertRaises(ValueError):
                t.ms_to_ns(value)

    def test_decoder_strictness(self):
        blob = string('hello')
        self.assertEqual(t.decode(blob, 'std_msgs/msg/String'), 'hello')
        for corrupt in (blob+b'\0', blob[:-1], b'\0\1\0\1'+blob[4:]):
            with self.assertRaises(ValueError):
                t.decode(corrupt, 'std_msgs/msg/String')
        with self.assertRaises(ValueError):
            t.decode(image(1, 'ir', b'\0'*48, 'Y8'), 'sensor_msgs/msg/Image')
        with self.assertRaises(ValueError):
            t.decode(imu(1, 'gyro', [float('nan'), 0, 0], True), 'sensor_msgs/msg/Imu')

    def test_missing_schema_and_ambiguous_topics(self):
        with sqlite3.connect(self.file) as con:
            con.execute('CREATE TABLE other(x)')
        self.assertEqual(self.cli('inspect', self.file)[0], 3)
        self.assertEqual(self.cli('audit', self.file)[0], 2)
        self.file.unlink()
        self.fixture()
        with sqlite3.connect(self.file) as con:
            con.execute("INSERT INTO topics VALUES(10,'/device_1/sensor_0/Infrared_1/image/data','sensor_msgs/msg/Image','cdr','')")
        with self.assertRaisesRegex(ValueError, 'ambiguous'):
            t.load(self.file)

    def test_existing_output_not_overwritten(self):
        self.fixture()
        out = self.root/'existing'
        out.mkdir()
        (out/'sentinel').write_text('retained')
        self.assertEqual(self.cli('export-euroc', self.file, out)[0], 2)
        self.assertEqual((out/'sentinel').read_text(), 'retained')

    def test_audit_outputs_cannot_destroy_input(self):
        self.fixture()
        before = t.sha256(self.file)
        self.assertEqual(self.cli('audit', self.file, '--json', self.file)[0], 2)
        self.assertEqual(t.sha256(self.file), before)
        destination = self.root/'report'
        self.assertEqual(self.cli('audit', self.file, '--json', destination, '--markdown', destination)[0], 2)
        self.assertFalse(destination.exists())

    def test_multiple_hardware_clocks_and_wal_refused(self):
        self.fixture()
        with sqlite3.connect(self.file) as con:
            con.execute("UPDATE topics SET name=replace(name,'device_0','device_1') WHERE id IN (7,8)")
        with self.assertRaisesRegex(ValueError, 'multiple devices'):
            t.load(self.file)
        wal = Path(str(self.file)+'-wal')
        wal.write_bytes(b'uncheckpointed')
        with self.assertRaisesRegex(ValueError, 'WAL'):
            t.load(self.file)

    def test_real_cli_entry_point(self):
        self.fixture()
        proc = subprocess.run([sys.executable, str(Path(t.__file__)), 'audit', str(self.file)],
                              text=True, capture_output=True, check=False)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(json.loads(proc.stdout)['status'], 'PASS')


if __name__ == '__main__':
    unittest.main()
