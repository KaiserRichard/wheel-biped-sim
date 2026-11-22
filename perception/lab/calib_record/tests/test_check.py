"""Offline fixtures only: no hardware, network, packages or external commands."""
import copy
import csv
from decimal import Decimal
import hashlib
import importlib.util
import json
from pathlib import Path
import sqlite3
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


c = load('check_calib', HERE/'check_calib_take.py')
fixture = load('take_fixture', ROOT/'tools/take_tools/tests/fixture.py')
tt = load('take_tools_test', ROOT/'tools/take_tools/take_tools.py')


def write(path, value):
    path.write_text(json.dumps(value))


def checksum(base, names=None):
    paths = [base/n for n in names] if names else [p for p in sorted(base.iterdir()) if p.is_file() and p.name != 'SHA256SUMS']
    (base/'SHA256SUMS').write_text(''.join(f'{tt.sha256(p)}  {p.name}\n' for p in paths))


class TakeCheck(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.take = self.root/'take'
        self.take.mkdir()
        self.config = json.loads((HERE/'criteria.json').read_text())
        self.config['camera_duration_s']['value'] = [0.9, 1.1]
        self.config['allan_duration_hours']['value'] = 0.0002
        self.config['ir_dimensions']['value'] = [8, 6]
        for item in self.config.values():
            item['source'] = 'ASSUMED SYNTHETIC TEST POLICY; ' + item['source']
        self.policy = self.root/'policy.json'
        write(self.policy, self.config)
        self.counter = 0

    def run_check(self, kind='allan', default=False):
        self.counter += 1
        out = self.root/f'out{self.counter}'
        cmd = [sys.executable, str(HERE/'check_calib_take.py'), '--analyze-only', str(self.take),
               '--kind', kind, '--outdir', str(out)]
        if not default:
            cmd += ['--config', str(self.policy)]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertIn(result.returncode, (0, 1, 2), result.stderr)
        report = json.loads((out/'result.json').read_text())
        self.assertEqual(result.returncode, c.EXIT[report['status']])
        self.assertTrue(report['started_utc'] <= report['ended_utc'])
        for row in report['criteria']:
            self.assertTrue((out/row['evidence']).is_file())
            self.assertTrue(row['threshold_source'])
        for raw in report['raw_files']:
            self.assertEqual(tt.sha256(out/raw['path']), raw['sha256'])
        return report, out

    def statuses(self, report):
        return {r['id']: r['status'] for r in report['criteria']}

    def allan(self, variant=None):
        for name, period, count in [('gyro', 5, 201), ('accel', 10, 101)]:
            with (self.take/(name+'.csv')).open('w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(c.ALLAN_HEADER)
                for i in range(count):
                    sdk = Decimal('1791950284544.070068') + i*period
                    domain = 'Global Time'
                    frame, host = i+1, 9007199254740993+i*period*1000000
                    if name == 'gyro' and i == 10:
                        if variant == 'reverse': sdk -= 2*period
                        if variant == 'duplicate': sdk -= period
                        if variant == 'domain': domain = 'Hardware Clock'
                        if variant == 'system': domain = 'System Time'
                        if variant == 'host': host -= 2*period*1000000
                        if variant == 'frame': frame += 1
                        if variant == 'nan': sdk = 'NaN'
                        if variant == 'gap': continue
                    writer.writerow([sdk, domain, frame, host, 0.001, -0.002, 9.81 if name == 'accel' else 0.003])
        write(self.take/'meta.json', {'sdk_device_serial': '99005', 'firmware': 'LOCAL_FIRMWARE_VERSION',
                                     'frame': 'imu_camera', 'serial_check_bypassed': False,
                                     'profiles_resolved': [{'stream': s, 'format': 'MOTION_XYZ32F', 'fps': hz}
                                                           for s, hz in [('Gyro', 200), ('Accel', 100)]],
                                     'stop_reason': 'duration', 'error': '', 'queue_overflow': 0,
                                     'callback_samples': [200 if variant == 'gap' else 201, 101],
                                     'gyro': {'sample_count': 200 if variant == 'gap' else 201},
                                     'accel': {'sample_count': 101}})
        (self.take/'sdk.log').write_bytes(b'SYNTHETIC SDK log\r\n')
        write(self.take/'emitter.json', {'enabled_readback': 0, 'sdk_device_serial': '99005',
                                        'clock': 'linux_clock_realtime', 'observed_utc': '2026-10-18T00:00:00Z',
                                        'source': 'SYNTHETIC actual option readback receipt'})
        write(self.take/'storage.json', {'free_bytes_after': 10**10, 'clock': 'linux_clock_realtime',
                                        'observed_utc': '2026-10-18T00:00:00Z', 'source': 'SYNTHETIC df receipt'})
        checksum(self.take)

    def camera(self, variant=None):
        fixture.make(self.take/'stereo_imu.db3', variant=variant)
        # Keep the independent serializer's schema; replace flat images with a
        # known sharp checkerboard and its deliberately constant system-time stub.
        db = sqlite3.connect(self.take/'stereo_imu.db3')
        for ident, topic, stamp, payload in db.execute('SELECT id,topic_id,timestamp,data FROM messages').fetchall():
            if topic in (1, 3):
                pixels = bytes(255*((x+y) % 2) for y in range(6) for x in range(8))
                db.execute('UPDATE messages SET data=? WHERE id=?', (fixture.image(stamp, 'IR', pixels), ident))
            if topic in (2, 4, 6, 8):
                text = tt.decode(payload, 'std_msgs/msg/String')
                text = text.replace('system_time=1700000000000.000000', f'system_time={Decimal(stamp)/1000000}')
                db.execute('UPDATE messages SET data=? WHERE id=?', (fixture.string(text), ident))
        db.commit()
        db.close()
        streams, _, _ = tt.load(self.take/'stereo_imu.db3')
        with (self.take/'timestamps.csv').open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(c.CAM_HEADER)
            for name, records in streams.items():
                for sample in records:
                    name_label = {'ir1': 'Infrared', 'ir2': 'Infrared', 'gyro': 'Gyro', 'accel': 'Accel'}[name]
                    vector = sample[name] if name in ('gyro', 'accel') else ['', '', '']
                    writer.writerow([name_label, {'ir1': 1, 'ir2': 2, 'gyro': 0, 'accel': 0}[name],
                                     sample['frame_number'], Decimal(sample['sdk_ns'])/1000000,
                                     sample['domain'], sample['bag_ns'], sample['bag_ns']+1000000000000, *vector, ''])
        (self.take/'sdk.log').write_text('SYNTHETIC SDK log\n')
        (self.take/'record.log').write_text('SDK serial=99005 firmware=LOCAL_FIRMWARE_VERSION\n'
            'Infrared index=1 format=Y8 fps=30\nInfrared index=2 format=Y8 fps=30\n'
            'Gyro index=0 format=MOTION_XYZ32F fps=200\nAccel index=0 format=MOTION_XYZ32F fps=100\n'
            'emitter=0 (read back before streaming)\ntake-end=2026-10-18T00:00:00Z rc=0 interrupted=0\n'
            'storage_free=10000000000 storage_required=1\n')
        write(self.take/'storage.json', {'free_bytes_after': 10**10, 'clock': 'linux_clock_realtime',
                                        'observed_utc': '2026-10-18T00:00:00Z', 'source': 'SYNTHETIC df receipt'})
        checksum(self.take)

    def test_allan_pass_and_reanalysis_contract(self):
        self.allan()
        report, out = self.run_check()
        self.assertEqual(report['status'], 'PASS')
        self.assertEqual((out/'raw/sdk.log').read_bytes(), (self.take/'sdk.log').read_bytes())
        second, _ = self.run_check()
        self.assertEqual(report['criteria'], second['criteria'])
        metrics = json.loads((out/'metrics.json').read_text())
        self.assertEqual(metrics['streams']['gyro']['rate_hz'], 200)
        self.assertEqual(metrics['streams']['gyro']['span_s'], 1)

    def test_default_duration_fails_short_fixture(self):
        self.allan()
        report, _ = self.run_check(default=True)
        self.assertEqual(self.statuses(report)['duration'], 'FAIL')

    def test_allan_timing_failures(self):
        for variant, criterion in [('reverse', 'timestamp.gyro'), ('duplicate', 'timestamp.gyro'),
                                   ('host', 'timestamp.gyro'), ('frame', 'rate_drops.gyro'),
                                   ('gap', 'rate_drops.gyro'), ('domain', 'sdk_domain'), ('system', 'sdk_domain'),
                                   ('nan', 'samples')]:
            with self.subTest(variant=variant):
                self.allan(variant)
                report, _ = self.run_check()
                self.assertEqual(report['status'], 'FAIL')
                self.assertEqual(self.statuses(report)[criterion], 'FAIL')

    def test_missing_receipts_blocked(self):
        self.allan()
        (self.take/'storage.json').unlink()
        (self.take/'SHA256SUMS').unlink()
        report, _ = self.run_check()
        self.assertEqual(report['status'], 'BLOCKED')
        self.assertEqual(self.statuses(report)['storage.free_space'], 'BLOCKED')
        self.assertEqual(self.statuses(report)['capture_checksums'], 'BLOCKED')

    def test_missing_stream_fail_overrides_blocked(self):
        self.allan()
        (self.take/'accel.csv').unlink()
        report, _ = self.run_check()
        self.assertEqual(report['status'], 'FAIL')
        self.assertEqual(self.statuses(report)['completeness'], 'FAIL')
        self.assertEqual(self.statuses(report)['samples'], 'BLOCKED')

    def test_manifest_tampering_and_traversal(self):
        self.allan()
        for bad in ['0'*64+'  gyro.csv\n', '0'*64+'  ../outside\n', 'malformed\n']:
            (self.take/'SHA256SUMS').write_text(bad)
            report, _ = self.run_check()
            self.assertEqual(self.statuses(report)['capture_checksums'], 'FAIL')

    def test_identity_profile_and_recorder_failure(self):
        for key, val in [('sdk_device_serial', 'old-unit'), ('serial_check_bypassed', True),
                         ('firmware', 'UNKNOWN'), ('frame', 'imu_body'), ('profiles_resolved', [])]:
            self.allan()
            meta = json.loads((self.take/'meta.json').read_text())
            meta[key] = val
            write(self.take/'meta.json', meta)
            checksum(self.take)
            report, _ = self.run_check()
            self.assertEqual(self.statuses(report)['metadata'], 'FAIL')
        meta['error'] = 'queue overflow'
        write(self.take/'meta.json', meta)
        report, _ = self.run_check()
        self.assertEqual(self.statuses(report)['recorder_stop'], 'FAIL')

    def test_storage_no_space_or_bandwidth(self):
        self.allan()
        receipt = json.loads((self.take/'storage.json').read_text())
        receipt['free_bytes_after'] = 1
        write(self.take/'storage.json', receipt)
        checksum(self.take)
        self.config['card_limit_bytes_s']['value'] = 1
        write(self.policy, self.config)
        report, _ = self.run_check()
        self.assertEqual(self.statuses(report)['storage.free_space'], 'FAIL')
        self.assertEqual(self.statuses(report)['storage.write_headroom'], 'FAIL')
        self.assertTrue(any('WARN:' in n for n in report['notes']))

    def test_camera_valid_checks_pass_target_blocked(self):
        self.camera()
        report, _ = self.run_check('camera-imu')
        self.assertEqual(report['status'], 'BLOCKED')
        self.assertEqual({k: v for k, v in self.statuses(report).items() if v != 'PASS'}, {'target_detection': 'BLOCKED'})

    def test_camera_stereo_missing_and_skew(self):
        for variant in ('missing_frame', 'skew', 'frame_mismatch'):
            (self.take/'stereo_imu.db3').unlink(missing_ok=True)
            self.camera(variant)
            report, _ = self.run_check('camera-imu')
            self.assertEqual(self.statuses(report)['stereo_pairing'], 'FAIL')

    def test_camera_callback_bag_disagreement(self):
        self.camera()
        text = (self.take/'timestamps.csv').read_text()
        (self.take/'timestamps.csv').write_text(text.replace('1234.567891', '1234.569891'))
        checksum(self.take)
        report, _ = self.run_check('camera-imu')
        self.assertEqual(self.statuses(report)['callback_bag_agreement'], 'FAIL')

    def test_camera_excitation_and_blur_fail(self):
        self.camera()
        with (self.take/'timestamps.csv').open() as source:
            rows = list(csv.reader(source))
        for row in rows[1:]:
            if row[0] == 'Gyro': row[7:10] = ['0', '0', '0']
        with (self.take/'timestamps.csv').open('w', newline='') as f:
            csv.writer(f).writerows(rows)
        checksum(self.take)
        report, _ = self.run_check('camera-imu')
        self.assertEqual(self.statuses(report)['excitation.gyro'], 'FAIL')
        # Independently check flat versus sharp images through the full bag path.
        db = sqlite3.connect(self.take/'stereo_imu.db3')
        for ident, stamp in db.execute('SELECT id,timestamp FROM messages WHERE topic_id IN (1,3)').fetchall():
            db.execute('UPDATE messages SET data=? WHERE id=?', (fixture.image(stamp, 'IR', bytes(48)), ident))
        db.commit()
        db.close()
        checksum(self.take)
        report, _ = self.run_check('camera-imu')
        self.assertEqual(self.statuses(report)['motion_blur.ir1'], 'FAIL')
        self.assertEqual(self.statuses(report)['motion_blur.ir2'], 'FAIL')

    def test_no_motion_is_blur_blocked(self):
        self.camera()
        self.config['motion_gyro_norm_rad_s']['value'] = 1e9
        write(self.policy, self.config)
        report, _ = self.run_check('camera-imu')
        self.assertEqual(self.statuses(report)['motion_blur.ir1'], 'BLOCKED')

    def test_unsupported_bag_blocked(self):
        self.camera()
        (self.take/'stereo_imu.db3').write_bytes(b'#ROSBAG V2.0\n')
        checksum(self.take)
        report, _ = self.run_check('camera-imu')
        self.assertEqual(self.statuses(report)['bag_audit'], 'BLOCKED')

    def test_camera_wrapper_layout_original_manifest(self):
        self.camera()
        inner = self.take/'take'
        inner.mkdir()
        for name in ('stereo_imu.db3', 'timestamps.csv', 'sdk.log'):
            (self.take/name).rename(inner/name)
        checksum(inner)
        (inner/'SHA256SUMS').rename(self.take/'ram-sha256.txt')
        (self.take/'SHA256SUMS').unlink()
        report, _ = self.run_check('camera-imu')
        self.assertEqual(report['status'], 'BLOCKED')
        self.assertEqual(self.statuses(report)['capture_checksums'], 'PASS')

    def test_bad_policy_and_output_reuse_usage_3(self):
        self.allan()
        report, out = self.run_check()
        self.assertEqual(c.main(['--analyze-only', str(self.take), '--kind', 'allan', '--outdir', str(out)]), 3)
        self.assertEqual(c.main(['--analyze-only', str(self.take), '--kind', 'allan', '--outdir', str(self.take/'out')]), 3)
        for bad in (float('nan'), -1):
            self.config['sharpness_min']['value'] = bad
            write(self.policy, self.config)
            with self.assertRaises(ValueError): c.load_config(self.policy)
        self.assertEqual(c.main([]), 3)

    def test_known_sharpness(self):
        self.assertEqual(c.sharpness(8, 6, bytes(48), 1), 0)
        pixels = bytes(255*((x+y) % 2) for y in range(6) for x in range(8))
        self.assertGreater(c.sharpness(8, 6, pixels, 1), 1e6)
        self.assertIsNone(c.sharpness(2, 2, bytes(4), 1))

    def test_missing_dependency_blocked_contract(self):
        with patch.object(c, 'module', side_effect=FileNotFoundError('missing take_tools')):
            out = self.root/'missing-tools'
            self.assertEqual(c.run(self.take, out, 'allan', self.policy), 2)
        result = json.loads((out/'result.json').read_text())
        self.assertEqual(result['status'], 'BLOCKED')
        self.assertEqual(result['criteria'][0]['id'], 'dependencies')

    def test_allan_stop_counts_and_missing_metadata(self):
        self.allan()
        meta = json.loads((self.take/'meta.json').read_text())
        meta['stop_reason'] = 'SIGINT'
        write(self.take/'meta.json', meta)
        checksum(self.take)
        report, _ = self.run_check()
        self.assertEqual(report['status'], 'PASS')
        meta['callback_samples'][0] += 1
        write(self.take/'meta.json', meta)
        checksum(self.take)
        report, _ = self.run_check()
        self.assertEqual(self.statuses(report)['recorder_counts'], 'FAIL')
        (self.take/'meta.json').write_text('{malformed')
        report, _ = self.run_check()
        self.assertEqual(self.statuses(report)['metadata'], 'FAIL')

    def test_pre_capture_free_space_is_not_post_capture_measurement(self):
        self.camera()
        (self.take/'storage.json').unlink()
        checksum(self.take)
        report, out = self.run_check('camera-imu')
        self.assertEqual(self.statuses(report)['storage.free_space'], 'BLOCKED')
        metrics = json.loads((out/'metrics.json').read_text())
        self.assertIn('INFERRED', metrics['storage']['free_estimate_note'])

    def test_allan_emitter_readback_unknown_or_on(self):
        self.allan()
        (self.take/'emitter.json').unlink()
        checksum(self.take)
        report, _ = self.run_check()
        self.assertEqual(report['status'], 'BLOCKED')
        self.assertEqual(self.statuses(report)['emitter_state'], 'BLOCKED')
        write(self.take/'emitter.json', {'enabled_readback': 1, 'sdk_device_serial': '99005',
                                       'clock': 'linux_clock_realtime', 'observed_utc': '2026-10-18T00:00:00Z',
                                       'source': 'SYNTHETIC enabled projector'})
        checksum(self.take)
        report, _ = self.run_check()
        self.assertEqual(self.statuses(report)['emitter_state'], 'FAIL')

    def test_camera_missing_emitter_and_failed_stop(self):
        self.camera()
        log = (self.take/'record.log').read_text()
        (self.take/'record.log').write_text(log.replace('emitter=0 (read back before streaming)', 'requested emitter OFF'))
        checksum(self.take)
        report, _ = self.run_check('camera-imu')
        self.assertEqual(self.statuses(report)['metadata'], 'BLOCKED')
        (self.take/'record.log').write_text(log.replace('rc=0 interrupted=0', 'rc=1 interrupted=1'))
        checksum(self.take)
        report, _ = self.run_check('camera-imu')
        self.assertEqual(self.statuses(report)['recorder_stop'], 'FAIL')

    def test_each_excitation_axis_is_required(self):
        self.camera()
        with (self.take/'timestamps.csv').open() as f:
            rows = list(csv.reader(f))
        for row in rows[1:]:
            if row[0] == 'Accel': row[8] = '9.81'
        with (self.take/'timestamps.csv').open('w', newline='') as f:
            csv.writer(f).writerows(rows)
        checksum(self.take)
        report, _ = self.run_check('camera-imu')
        item = next(r for r in report['criteria'] if r['id'] == 'excitation.accel')
        self.assertEqual(item['status'], 'FAIL')
        self.assertEqual(item['measured']['y'], 0)
        self.assertGreater(item['measured']['x'], 0.5)

    def test_corrupt_supported_bag_fails(self):
        self.camera()
        db = sqlite3.connect(self.take/'stereo_imu.db3')
        db.execute('UPDATE messages SET data=? WHERE id=(SELECT MIN(id) FROM messages WHERE topic_id=1)', (b'bad',))
        db.commit()
        db.close()
        checksum(self.take)
        report, _ = self.run_check('camera-imu')
        self.assertEqual(self.statuses(report)['bag_audit'], 'FAIL')
        self.assertEqual(self.statuses(report)['motion_blur.ir1'], 'BLOCKED')

    def test_symlink_evidence_refused(self):
        (self.take/'alias').symlink_to(self.policy)
        self.assertEqual(c.main(['--analyze-only', str(self.take), '--kind', 'allan', '--outdir', str(self.root/'linked')]), 3)

    def test_real_saved_analysis_rates(self):
        # Genuine archived old-unit transport observations. No calibration or
        # new-unit acceptance is inferred from the domain-unknown SDK CSV.
        report = json.loads((HERE/'tests/fixtures/real-sustained-analysis.json').read_text())
        for name, hz in [('Gyro', 200), ('Accel', 100)]:
            measured = report['all_received_samples'][name]
            rate = (measured['count']-1)/(measured['last_stamp_seconds']-measured['first_stamp_seconds'])
            stats = {'count': measured['count'], 'rate_hz': rate,
                     'max_interval_ns': measured['max_stamp_gap_seconds']*1e9,
                     'frame_missing': measured['missing_frame_numbers'], 'frame_nonincreasing': measured['nonpositive_frame_steps']}
            self.assertTrue(c.rate_ok(stats, hz, self.config))
        # Real old-unit 15 Hz IR is correctly rejected by the 30 Hz calibration policy.
        measured = report['all_received_samples']['Infrared']
        stats = {'count': measured['count'],
                 'rate_hz': (measured['count']-1)/(measured['last_stamp_seconds']-measured['first_stamp_seconds']),
                 'max_interval_ns': measured['max_stamp_gap_seconds']*1e9}
        self.assertFalse(c.rate_ok(stats, 30, self.config))


if __name__ == '__main__':
    unittest.main()
