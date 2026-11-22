"""Regression cases from the independent phase-4 review; all ROS commands are mocked."""
import copy
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import sqlite3
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import analyze_bag
import record_check
import replay_check
from bag_core import CDR, analyze_database, evaluate_report, _digest_stamps
from qual_common import load_config, overall_status
from test_qual import make_bag, make_config, _header, _string, _align


class ReviewTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = make_config()
        defaults = load_config()
        for key in ('camera_node', 'qos_overrides_file'):
            self.config[key] = defaults[key]
        for section in ('record', 'replay', 'validation'):
            if section in defaults:
                self.config.setdefault(section, {}).update({k: v for k, v in defaults[section].items()
                                                           if k not in self.config.get(section, {})})
        self.config['storage']['storage_id'] = 'sqlite3'
        self.config['record']['duration_s'] = 1
        self.config['qos_overrides_file'] = str(HERE / 'qos_overrides.yaml')
        self.config_path = self.root / 'custom.json'
        self.config_path.write_text(json.dumps(self.config))
        self.bag = self.root / 'test.db3'
        make_bag(self.bag, [1_000_000_000, 1_500_000_000, 2_000_000_000])
        self.stored, _ = analyze_database(self.bag, self.config)
        self.observations = {'player_returncode': 0}
        for name, item in self.stored['topics'].items():
            stamps = ([1_000_000_000, 1_500_000_000, 2_000_000_000] if name in ('/left', '/right')
                      else [1_000_000_000 + i * 250_000_000 for i in range(5)])
            self.observations[name] = {'count': item['count'], 'ros_header_stamps_ns': stamps,
                                       'received_monotonic_ns': [stamp + 90_000_000_000 for stamp in stamps], 'errors': []}

    def quiet(self, function, argv):
        with redirect_stdout(io.StringIO()):
            return function(argv)

    def save_replay(self, out):
        out.mkdir()
        for name, value in [('config.json', self.config), ('stored_analysis.json', self.stored),
                            ('replay_observations.json', self.observations)]:
            (out / name).write_text(json.dumps(value))
        (out / 'node_list.json').write_text(json.dumps({'returncode': 0, 'stdout': '', 'stderr': ''}))
        (out / 'replay_node_list.json').write_text(json.dumps({'returncode': 0, 'stdout': '', 'stderr': ''}))
        for name in ('node_list.stdout', 'node_list.stderr', 'replay_node_list.stdout', 'replay_node_list.stderr', 'player.log'):
            (out / name).write_text('')
        (out / 'qos_overrides.yaml').write_text((HERE / 'qos_overrides.yaml').read_text())
        (out / 'player_command.json').write_text(json.dumps({'argv': ['ros2', 'bag', 'play', 'fixture']}))

    def test_replay_analyze_only_reduces_raw_observations_and_loads_saved_config(self):
        out = self.root / 'replay'; self.save_replay(out)
        self.assertEqual(self.quiet(replay_check.main, ['--analyze-only', str(out)]), 0)

    def test_missing_saved_record_is_blocked_without_crashing(self):
        out = self.root / 'record'; out.mkdir()
        self.assertEqual(self.quiet(record_check.main, ['--analyze-only', str(out)]), 2)

    def test_empty_saved_record_does_not_pass(self):
        out = self.root / 'record'; out.mkdir()
        (out / 'record_raw.json').write_text('{}')
        self.assertEqual(self.quiet(record_check.main, ['--analyze-only', str(out)]), 2)

    def test_corrupt_saved_replay_is_blocked(self):
        out = self.root / 'replay'; self.save_replay(out)
        (out / 'replay_observations.json').write_text('{')
        self.assertEqual(self.quiet(replay_check.main, ['--analyze-only', str(out)]), 2)

    def test_missing_saved_replay_graph_guard_is_blocked(self):
        out = self.root / 'replay'; self.save_replay(out)
        (out / 'node_list.json').unlink()
        self.assertEqual(self.quiet(replay_check.main, ['--analyze-only', str(out)]), 2)

    def test_missing_replay_player_evidence_is_blocked(self):
        report = replay_check._replay_report(self.observations, self.config)
        criteria = replay_check._criteria(self.stored, report, self.config, 'raw.json')
        self.assertEqual(next(c['status'] for c in criteria if c['id'] == 'replay:player'), 'BLOCKED')

    def test_replay_uses_recorded_receive_rate(self):
        stored = copy.deepcopy(self.stored)
        for item in stored['topics'].values():
            item['bag_receive_rate_hz'] /= 2
        observations = copy.deepcopy(self.observations)
        for name in stored['topics']:
            observations[name]['received_monotonic_ns'] = [v * 2 for v in observations[name]['received_monotonic_ns']]
        report = replay_check._replay_report(observations, self.config) | {'player_returncode': 0}
        self.assertEqual(overall_status(replay_check._criteria(stored, report, self.config, 'raw.json')), 'PASS')

    def test_replay_missing_recorded_topic_is_blocked(self):
        stored = copy.deepcopy(self.stored); stored['topics'].pop('/left')
        actual = replay_check._replay_report(self.observations, self.config) | {'player_returncode': 0}
        self.assertEqual(overall_status(replay_check._criteria(stored, actual, self.config, 'raw.json')), 'BLOCKED')
        stored = copy.deepcopy(self.stored); stored['topics']['/left'].pop('header_count')
        self.assertEqual(overall_status(replay_check._criteria(stored, actual, self.config, 'raw.json')), 'BLOCKED')

    def test_replay_subscription_error_cannot_pass(self):
        self.observations['/left']['errors'] = ['subscription failed']
        actual = replay_check._replay_report(self.observations, self.config) | {'player_returncode': 0}
        self.assertEqual(overall_status(replay_check._criteria(self.stored, actual, self.config, 'raw.json')), 'BLOCKED')

    def test_replay_stamp_change_is_fail(self):
        self.observations['/left']['ros_header_stamps_ns'][1] += 1
        actual = replay_check._replay_report(self.observations, self.config) | {'player_returncode': 0}
        self.assertEqual(overall_status(replay_check._criteria(self.stored, actual, self.config, 'raw.json')), 'FAIL')

    def test_saved_offline_missing_topic_is_blocked(self):
        report = copy.deepcopy(self.stored); report['topics'].pop('/left')
        self.assertEqual(overall_status(evaluate_report(report, self.config)), 'BLOCKED')

    def test_saved_offline_missing_order_measurement_is_blocked(self):
        report = copy.deepcopy(self.stored); report['topics']['/left'].pop('header_reversals')
        self.assertEqual(overall_status(evaluate_report(report, self.config)), 'BLOCKED')

    def test_receive_rate_outside_band_fails(self):
        conn = sqlite3.connect(self.bag)
        conn.execute('UPDATE messages SET timestamp=timestamp*10'); conn.commit(); conn.close()
        _, criteria = analyze_database(self.bag, self.config)
        self.assertEqual(overall_status(criteria), 'FAIL')

    def test_repeated_stamp_fails(self):
        report = copy.deepcopy(self.stored)
        report['topics']['/left']['header_repeated_stamps'] = 1
        self.assertEqual(overall_status(evaluate_report(report, self.config)), 'FAIL')

    def test_invalid_ros_nanoseconds_rejected(self):
        from bag_core import extract_header
        payload = b'\x00\x01\x00\x00' + struct.pack('<iII', 1, 1_000_000_000, 2) + b'f\0'
        with self.assertRaises(ValueError):
            extract_header(payload, 'sensor_msgs/msg/Image', 'image_left')

    def test_cdr_alignment_is_relative_to_encapsulation(self):
        cdr = CDR(b'\x00\x01\x00\x00' + struct.pack('<d', 3.25))
        self.assertEqual(cdr.unpack('d'), 3.25)

    def test_explicit_missing_config_is_not_replaced_by_default(self):
        out = self.root / 'analysis'
        self.assertEqual(self.quiet(analyze_bag.main, ['--bag', str(self.bag), '--output-dir', str(out),
                          '--config', str(self.root / 'absent.json')]), 3)

    def test_usage_exit_code_is_three(self):
        for module in (analyze_bag, record_check, replay_check):
            with self.subTest(module=module.__name__), redirect_stdout(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    module.main([])
                self.assertEqual(error.exception.code, 3)

    def test_tf_stamps_are_observed(self):
        from types import SimpleNamespace as NS
        msg = NS(transforms=[NS(header=NS(stamp=NS(sec=1, nanosec=2))),
                             NS(header=NS(stamp=NS(sec=3, nanosec=4)))])
        self.assertEqual(replay_check._message_stamps(msg), [1_000_000_002, 3_000_000_004])

    def test_live_camera_in_replay_domain_blocks_player(self):
        with patch.object(replay_check, 'run_command', side_effect=[
            {'returncode': 0, 'stdout': '', 'stderr': ''},
            {'returncode': 0, 'stdout': '/camera/camera\n', 'stderr': ''}]), \
             patch.object(replay_check, '_observe_replay') as observe:
            code = self.quiet(replay_check.main, ['--config', str(self.config_path), '--bag', str(self.bag),
                                                  '--output-dir', str(self.root / 'live')])
            self.assertEqual(code, 2); observe.assert_not_called()

    def test_record_waits_only_requested_duration_and_saves_recomputable_raw(self):
        output = self.root / 'record'
        class Process:
            returncode = 0
            def wait(inner, timeout):
                inner.timeout = timeout
                raise subprocess.TimeoutExpired('ros2', timeout)
            def poll(inner):
                return 0
        process = Process()
        def launch(*args, **kwargs):
            make_bag(output / 'bag' / 'test.db3', [1_000_000_000, 1_500_000_000, 2_000_000_000])
            return process
        with patch.object(record_check, 'run_command', return_value={'returncode': 0, 'stdout': '/camera/camera\n', 'stderr': ''}), \
             patch.object(record_check.subprocess, 'Popen', side_effect=launch) as popen, \
             patch.object(record_check, '_stop', return_value='SIGINT'):
            code = self.quiet(record_check.main, ['--config', str(self.config_path), '--output-dir', str(output)])
        self.assertEqual(process.timeout, self.config['record']['duration_s'])
        self.assertNotIn('--max-bag-duration', popen.call_args.args[0])
        self.assertEqual(code, 0)
        self.assertEqual(self.quiet(record_check.main, ['--analyze-only', str(output)]), 0)
        raw = json.loads((output / 'record_raw.json').read_text())
        raw['commands']['bag_info']['returncode'] = 1
        (output / 'record_raw.json').write_text(json.dumps(raw))
        self.assertEqual(self.quiet(record_check.main, ['--analyze-only', str(output)]), 1)

    def test_static_tf_offline_and_subscriber_hash_match_all_stamps(self):
        from types import SimpleNamespace as NS
        spec = {'name': '/tf_static', 'type': 'tf2_msgs/msg/TFMessage', 'role': 'tf_static',
                'required': True, 'threshold_source': 'PROPOSED: test fixture'}
        config = copy.deepcopy(self.config); config['topics'].append(spec)
        data = bytearray(b'\x00\x01\x00\x00')
        data.extend(struct.pack('<I', 2))
        for stamp, frame in [(0, 'base'), (5_000_000_001, 'gyro')]:
            _align(data, 4)
            data.extend(_header(stamp, frame)[4:])
            _string(data, frame + '_child')
            _align(data, 8)
            data.extend(struct.pack('<7d', 0, 0, 0, 0, 0, 0, 1))
        conn = sqlite3.connect(self.bag)
        conn.execute("INSERT INTO topics VALUES (5, '/tf_static', 'tf2_msgs/msg/TFMessage', 'cdr', '')")
        conn.execute('INSERT INTO messages VALUES (999, 5, 1000000000, ?)', (bytes(data),))
        conn.commit(); conn.close()
        stored, criteria = analyze_database(self.bag, config)
        self.assertEqual(overall_status(criteria), 'PASS')
        observed = copy.deepcopy(self.observations)
        observed['/tf_static'] = {'count': 1, 'ros_header_stamps_ns': [0, 5_000_000_001], 'received_monotonic_ns': [90_000_000_000], 'errors': []}
        report = replay_check._replay_report(observed, config) | {'player_returncode': 0}
        self.assertEqual(overall_status(replay_check._criteria(stored, report, config, 'raw.json')), 'PASS')
        observed['/tf_static']['ros_header_stamps_ns'][1] += 1
        report = replay_check._replay_report(observed, config) | {'player_returncode': 0}
        self.assertEqual(overall_status(replay_check._criteria(stored, report, config, 'raw.json')), 'FAIL')

    def test_partial_stream_cannot_hide_inside_full_bag(self):
        report = copy.deepcopy(self.stored)
        report['last_bag_receive_stamp_ns'] += 10_000_000_000
        report['duration_s'] += 10
        self.assertEqual(overall_status(evaluate_report(report, self.config)), 'FAIL')

    def test_short_record_window_fails_and_missing_window_blocks(self):
        raw = {'commands': {'bag_info': {'returncode': 0}, 'parameter_dump': {'returncode': 0},
                            'topic_info': {spec['name']: {'returncode': 0} for spec in self.config['topics']}},
               'stop_reason': 'SIGINT', 'record_returncode': 0, 'elapsed_monotonic_s': 1.0,
               'duration_s': 1.0, 'bag_receive_duration_s': 0.1}
        self.assertEqual(overall_status(record_check._record_criteria(raw, self.config)), 'FAIL')
        raw.pop('bag_receive_duration_s')
        self.assertEqual(overall_status(record_check._record_criteria(raw, self.config)), 'BLOCKED')
        raw.update(bag_receive_duration_s=1.0); raw.pop('elapsed_monotonic_s')
        self.assertEqual(overall_status(record_check._record_criteria(raw, self.config)), 'BLOCKED')
        raw.update(bag_receive_duration_s=1.0, stop_reason='self_terminated', elapsed_monotonic_s=0.1)
        self.assertEqual(overall_status(record_check._record_criteria(raw, self.config)), 'FAIL')

    def test_ros_absent_checks_are_blocked(self):
        missing = {'returncode': None, 'stdout': '', 'stderr': 'missing ros2', 'missing_command': True}
        for module in (record_check, replay_check):
            with self.subTest(module=module.__name__), patch.object(module, 'run_command', return_value=missing):
                argv = ['--output-dir', str(self.root / module.__name__), '--config', str(self.config_path)]
                if module is replay_check:
                    argv += ['--bag', str(self.bag)]
                self.assertEqual(self.quiet(module.main, argv), 2)

    def test_replay_live_pass_roundtrip_and_changed_stamp_fail(self):
        output = self.root / 'replay_live'
        def observe(*args):
            log = output / 'player.log'; log.write_text('fixture player output\n')
            (output / 'player_command.json').write_text(json.dumps({'argv': ['ros2', 'bag', 'play', str(self.bag)]}))
            return copy.deepcopy(self.observations), 0, str(log)
        with patch.object(replay_check, 'run_command', return_value={'returncode': 0, 'stdout': '', 'stderr': ''}), \
             patch.object(replay_check, '_observe_replay', side_effect=observe):
            self.assertEqual(self.quiet(replay_check.main, ['--bag', str(self.bag), '--config', str(self.config_path),
                                                          '--output-dir', str(output)]), 0)
        self.assertEqual(self.quiet(replay_check.main, ['--analyze-only', str(output)]), 0)
        data = json.loads((output / 'replay_observations.json').read_text()); data['/left']['ros_header_stamps_ns'][0] += 1
        (output / 'replay_observations.json').write_text(json.dumps(data))
        self.assertEqual(self.quiet(replay_check.main, ['--analyze-only', str(output)]), 1)

    def test_replay_failure_precedence_and_stored_fail_prevents_player(self):
        report = copy.deepcopy(self.stored)
        report['topics']['/left']['header_reversals'] = 1
        with patch.object(replay_check, 'run_command', return_value={'returncode': 0, 'stdout': '', 'stderr': ''}), \
             patch.object(replay_check, 'analyze_database', return_value=(report, [])), \
             patch.object(replay_check, '_observe_replay') as observe:
            self.assertEqual(self.quiet(replay_check.main, ['--bag', str(self.bag), '--config', str(self.config_path),
                                                          '--output-dir', str(self.root / 'failed')]), 1)
            observe.assert_not_called()

    def test_unsafe_durations_and_config_numbers_are_usage_errors(self):
        for seconds in ('0', '-1', 'nan', 'inf'):
            self.assertEqual(self.quiet(record_check.main, ['--seconds', seconds, '--config', str(self.config_path),
                                                           '--output-dir', str(self.root / 'invalid')]), 3)
        for value in (0, -1, float('nan')):
            bad = copy.deepcopy(self.config); bad['topics'][0]['nominal_hz'] = value
            self.config_path.write_text(json.dumps(bad))
            with self.assertRaises(ValueError):
                load_config(self.config_path)

    def test_sqlite_uri_handles_hash_and_question_mark(self):
        path = self.root / 'bag#question?.db3'
        make_bag(path, [1_000_000_000, 1_500_000_000, 2_000_000_000])
        _, criteria = analyze_database(path, self.config)
        self.assertEqual(overall_status(criteria), 'PASS')

    def test_missing_observation_is_blocked_but_measured_zero_delivery_fails(self):
        observations = copy.deepcopy(self.observations); observations.pop('/left')
        report = replay_check._replay_report(observations, self.config) | {'player_returncode': 0}
        self.assertEqual(overall_status(replay_check._criteria(self.stored, report, self.config, 'raw.json')), 'BLOCKED')
        observations['/left'] = {'count': 0, 'ros_header_stamps_ns': [], 'received_monotonic_ns': [], 'errors': []}
        report = replay_check._replay_report(observations, self.config) | {'player_returncode': 0}
        self.assertEqual(overall_status(replay_check._criteria(self.stored, report, self.config, 'raw.json')), 'FAIL')

    def test_bag_size_excludes_unrelated_files(self):
        from bag_core import bag_bytes
        directory = self.root / 'bagdir'; directory.mkdir()
        bag = directory / 'test.db3'
        make_bag(bag, [1_000_000_000, 1_500_000_000, 2_000_000_000])
        (directory / 'unrelated.log').write_bytes(b'x' * 1000)
        self.assertEqual(bag_bytes(directory), bag.stat().st_size)

    def test_portable_headers_from_real_evidence(self):
        from bag_core import extract_header
        fixture = json.loads((HERE / 'tests/real_header_fixture.json').read_text())
        self.assertEqual(fixture['evidence_label'], 'MEASURED')
        for spec in fixture['topics']:
            for row in spec['rows']:
                stamp, frame = extract_header(bytes.fromhex(row['cdr_header_hex']), spec['type'], spec['role'])
                self.assertEqual(stamp, row['ros_header_stamp_ns'])
                self.assertEqual(frame, row['frame_id'])
                self.assertIsInstance(row['bag_receive_stamp_ns'], int)

    def test_missing_non_sqlite_support_is_blocked(self):
        path = self.root / 'empty.mcap'; path.write_bytes(b'MCAP fixture')
        with patch.dict(sys.modules, {'rosbag2_py': None}):
            self.assertEqual(self.quiet(analyze_bag.main, ['--bag', str(path), '--config', str(self.config_path),
                                                          '--output-dir', str(self.root / 'mcapout')]), 2)

    def test_count_fail_overrides_blocked_rate_and_missing_guard(self):
        output = self.root / 'bad_replay'; self.save_replay(output)
        data = json.loads((output / 'replay_observations.json').read_text())
        data['/left'] = {'count': 0, 'ros_header_stamps_ns': [], 'received_monotonic_ns': [], 'errors': []}
        (output / 'replay_observations.json').write_text(json.dumps(data))
        (output / 'node_list.json').unlink()
        self.assertEqual(self.quiet(replay_check.main, ['--analyze-only', str(output)]), 1)

    def test_replay_missing_verbatim_player_log_blocks(self):
        output = self.root / 'replay_no_log'; self.save_replay(output)
        (output / 'player.log').unlink()
        self.assertEqual(self.quiet(replay_check.main, ['--analyze-only', str(output)]), 2)

    def test_saved_analysis_requires_its_config(self):
        output = self.root / 'no_config'; output.mkdir()
        (output / 'analysis.json').write_text(json.dumps(self.stored))
        self.assertEqual(self.quiet(analyze_bag.main, ['--analyze-only', str(output)]), 2)


if __name__ == '__main__':
    unittest.main()
