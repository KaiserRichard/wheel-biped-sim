import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from tools.run_harness import harness

EVALUATOR = ROOT / 'tools/trajectory_eval/traj_eval.py'
if not EVALUATOR.exists():
    EVALUATOR = ROOT.parent / 'wbr-perception-phase1-runtime/tools/trajectory_eval/traj_eval.py'


class HarnessTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.base = Path(self.temp.name)
        self.dataset = self.base / 'dataset'
        self.dataset.mkdir()
        for name in ('vocabulary', 'settings', 'times', 'manifest'):
            (self.dataset / name).write_text('0\n1\n2\n3\n' if name == 'times' else '{}')
        self.fake = self.base / 'fake_engine.py'
        self.fake.write_text((Path(__file__).parent / 'fake_engine.py').read_text())
        self.fake.chmod(0o755)
        self.spec = {'binary': str(self.fake), 'args': ['success'],
                     'vocabulary': str(self.dataset / 'vocabulary'),
                     'settings': str(self.dataset / 'settings'), 'dataset': str(self.dataset),
                     'times': str(self.dataset / 'times'), 'dataset_manifest': str(self.dataset / 'manifest'),
                     'repeats': 1, 'headless': True, 'clock_domain': 'dataset_clock',
                     'timeout_s': 5, 'jump_threshold_m': 1.5,
                     'trajectory': {'path': 'trajectory.txt', 'time_unit': 's',
                                    'sensor_frame': 'camera_optical',
                                    'pose_direction': 'sensor-to-world', 'quaternion_order': 'xyzw',
                                    'epochs': 'epochs.jsonl'}}
        self.spec_path = self.base / 'spec.yaml'
        self.output = self.base / 'out'

    def run_harness(self):
        self.spec_path.write_text(yaml.safe_dump(self.spec))
        summary = harness.run(self.spec_path, self.output)
        return summary, json.loads((self.output / 'repeat_001/receipt.json').read_text())

    def test_success_provenance_and_isolation(self):
        self.spec['repeats'] = 2
        summary, receipt = self.run_harness()
        self.assertEqual(summary['denominator'], 2)
        self.assertEqual(summary['status_counts']['OK'], 2)
        self.assertAlmostEqual(receipt['continuity']['path_length_m'], 3)
        self.assertEqual(receipt['continuity']['reset_count'], 0)
        self.assertEqual(receipt['inputs']['binary']['sha256'], harness.sha256(self.fake))
        self.assertEqual(receipt['engine']['exit_code'], 0)
        self.assertGreater(receipt['engine']['wall_time_s'], 0)
        self.assertEqual(receipt['engine']['start']['monotonic_clock_domain'], 'host_monotonic')
        self.assertEqual(receipt['tracking_log_counts']['imu_initialized'], 1)
        self.assertIn('fake stderr', (self.output / 'repeat_001/stderr.log').read_text())
        self.assertFalse((self.dataset / 'trajectory.txt').exists())
        self.assertIn('revision', receipt['git'])
        with self.assertRaisesRegex(ValueError, 'must be new'):
            harness.run(self.spec_path, self.output)

    def test_missing_or_escaping_extra_artifact_fails_batch(self):
        for escape in (False, True):
            with self.subTest(escape=escape):
                self.output = self.base / ('escape-extra' if escape else 'missing-extra')
                self.spec['extra_trajectories'] = ['extra.txt']
                if escape:
                    with self.fake.open('a') as stream:
                        stream.write("\nfrom pathlib import Path\nPath('extra.txt').symlink_to(" +
                                     repr(str(self.dataset / 'manifest')) + ")\n")
                self.spec_path.write_text(yaml.safe_dump(self.spec))
                self.assertEqual(harness.main([str(self.spec_path), '--out', str(self.output)]), 1)
                receipt = json.loads((self.output / 'repeat_001/receipt.json').read_text())
                summary = json.loads((self.output / 'summary.json').read_text())
                self.assertEqual(receipt['engine']['exit_code'], 0)
                self.assertEqual(receipt['status'], 'OK')
                self.assertTrue(receipt['trajectory_valid'])
                self.assertEqual(summary['analysis_failures'], 1)
                expected = 'artifact escapes' if escape else 'missing artifact'
                self.assertTrue(any(expected in error for error in receipt['errors']))

    def test_leader_exit_cleans_descendants_before_return(self):
        import time
        marker = self.base / 'child_finished'
        child = "import time; from pathlib import Path; time.sleep(.4); Path(" + repr(str(marker)) + ").write_text('late')"
        leader = "import subprocess, sys; subprocess.Popen([sys.executable, '-c', " + repr(child) + "])"
        result = harness.launch([sys.executable, '-c', leader], self.base, 5)
        # Cleanup on the failing pre-fix run too: the finite child exits on its own.
        time.sleep(.6)
        self.assertFalse(marker.exists(), 'descendant mutated artifacts after launch returned')
        self.assertEqual(result['exit_code'], 0)
        self.assertFalse(result['timed_out'])

    def test_mixed_failures_stay_in_denominator_and_partial_retained(self):
        self.spec.update(repeats=6, args=['mixed', '{repeat}'], timeout_s=3)
        summary, _ = self.run_harness()
        self.assertEqual(summary['denominator'], 6)
        self.assertEqual(summary['status_counts'], {'OK': 3, 'CRASH': 1, 'NO_TRAJECTORY': 1, 'TIMEOUT': 1})
        self.assertEqual(summary['ok_fraction'], .5)
        crash = json.loads((self.output / 'repeat_002/receipt.json').read_text())
        self.assertEqual(crash['engine']['exit_code'], 7)
        self.assertEqual(crash['continuity']['pose_count'], 2)
        self.assertTrue(crash['trajectory_valid'])
        timeout = json.loads((self.output / 'repeat_006/receipt.json').read_text())
        self.assertTrue(timeout['engine']['timed_out'])
        reset = json.loads((self.output / 'repeat_005/receipt.json').read_text())
        self.assertEqual(reset['continuity']['path_length_m'], 3)
        self.assertEqual(reset['continuity']['jump_count'], 1)
        self.assertEqual(reset['continuity']['segment_count'], 2)
        self.assertEqual(reset['continuity']['reset_count'], 1)
        self.assertEqual(reset['tracking_log_counts']['reset_messages'], 1)
        self.assertEqual(reset['tracking_log_counts']['map_count_reported'], 2)
        self.assertEqual(reset['tracking_log_counts']['observed_map_ids'], ['2'])

    def test_without_epochs_does_not_claim_reset_identity(self):
        del self.spec['trajectory']['epochs']
        self.spec['args'] = ['backward']
        _, receipt = self.run_harness()
        self.assertIsNone(receipt['continuity']['reset_count'])
        self.assertIsNone(receipt['continuity']['segment_count'])
        self.assertEqual(receipt['continuity']['timestamp_order_breaks'], 1)

    def test_invalid_trajectory_and_symlink(self):
        for mode in ('malformed', 'escape', 'backward'):
            with self.subTest(mode=mode):
                self.output = self.base / mode
                self.spec['args'] = [mode, str(self.dataset / 'manifest')]
                _, receipt = self.run_harness()
                self.assertEqual(receipt['status'], 'NO_TRAJECTORY')
                self.assertFalse(receipt['trajectory_valid'])

    def test_mount_nonidentity_and_epoch_boundaries(self):
        self.spec['args'] = ['reset']
        mount = {'status': 'PROPOSED', 'provenance': 'synthetic test only',
                 'T_base_camera': [[0, -1, 0, 1], [1, 0, 0, 2], [0, 0, 1, 3], [0, 0, 0, 1]]}
        mount_path = self.base / 'mount.json'
        mount_path.write_text(json.dumps(mount))
        self.spec['mount'] = str(mount_path)
        _, receipt = self.run_harness()
        rows = [json.loads(line) for line in (self.output / 'repeat_001/base_link.jsonl').read_text().splitlines()]
        self.assertEqual(rows[0]['child_frame'], 'base_link')
        self.assertEqual(rows[0]['T_world_base'][0][3], -2)
        self.assertEqual(rows[0]['T_world_base'][1][3], 1)
        self.assertEqual(rows[0]['T_world_base'][2][3], -3)
        self.assertFalse(rows[2]['velocity_valid'])
        self.assertEqual(rows[2]['segment_id'], 1)
        self.assertEqual(receipt['continuity_base_link']['path_length_m'], 3)
        self.assertIn('mount', receipt['inputs'])

    def ground_truth(self):
        path = self.base / 'gt.tum'
        path.write_text('0 0 0 0 0 0 0 1\n1 1 0 0 0 0 0 1\n2 1 1 0 0 0 0 1\n3 0 1 0 0 0 0 1\n')
        self.spec['ground_truth'] = {'path': str(path), 'evaluator': str(EVALUATOR),
                                     'clock_domain': 'dataset_clock', 'frame': 'camera_optical',
                                     'clock_assumption': 'Synthetic timestamps share dataset_clock.',
                                     'format': 'tum', 'time_unit': 's', 'align': 'se3', 'rpe_delta_s': 1}

    @unittest.skipUnless(EVALUATOR.is_file(), 'existing trajectory_eval source unavailable')
    def test_real_evaluator_zero_ate_and_rpe(self):
        self.ground_truth()
        _, receipt = self.run_harness()
        self.assertEqual(receipt['evaluation']['status'], 'OK')
        result = json.loads((self.output / 'repeat_001/evaluation/result.json').read_text())
        metrics = result['metrics']['se3']
        self.assertAlmostEqual(metrics['ate_translation_m']['rmse'], 0, places=12)
        self.assertAlmostEqual(metrics['rpe_translation_m']['rmse'], 0, places=12)
        self.assertEqual(metrics['rpe_translation_m']['n'], 3)

    @unittest.skipUnless(EVALUATOR.is_file(), 'existing trajectory_eval source unavailable')
    def test_reset_evaluation_refused_but_engine_status_preserved(self):
        self.ground_truth()
        self.spec['args'] = ['reset']
        summary, receipt = self.run_harness()
        self.assertEqual(receipt['status'], 'OK')
        self.assertEqual(receipt['evaluation']['status'], 'FAILED')
        self.assertEqual(summary['analysis_failures'], 1)

    @unittest.skipUnless(EVALUATOR.is_file(), 'existing trajectory_eval source unavailable')
    def test_drift_and_partial_evaluation_failure(self):
        self.ground_truth()
        self.spec['ground_truth']['align'] = 'none'
        reference = Path(self.spec['ground_truth']['path'])
        reference.write_text('0 0 0 0 0 0 0 1\n1 2 0 0 0 0 0 1\n2 2 2 0 0 0 0 1\n3 0 2 0 0 0 0 1\n')
        _, receipt = self.run_harness()
        metrics = json.loads((self.output / 'repeat_001/evaluation/result.json').read_text())['metrics']['none']
        self.assertAlmostEqual(metrics['ate_translation_m']['rmse'], 1)
        self.assertAlmostEqual(metrics['rpe_translation_m']['rmse'], 1)
        self.assertAlmostEqual(metrics['rpe_rotation_deg']['rmse'], 0)
        self.output = self.base / 'partial-eval'
        self.spec['args'] = ['partial']
        self.spec['ground_truth']['align'] = 'se3'
        summary, receipt = self.run_harness()
        self.assertEqual(receipt['status'], 'OK')
        self.assertEqual(receipt['evaluation']['status'], 'FAILED')
        self.assertEqual(summary['analysis_failures'], 1)
        self.assertEqual(receipt['coverage_proxy']['pose_count_per_input_stamp'], .5)

    @unittest.skipUnless(EVALUATOR.is_file(), 'existing trajectory_eval source unavailable')
    def test_normalized_inverse_ns_wxyz_rotation(self):
        self.ground_truth()
        self.spec['args'] = ['inverse']
        self.spec['trajectory'].update(time_unit='ns', pose_direction='world-to-sensor', quaternion_order='wxyz')
        gt = Path(self.spec['ground_truth']['path'])
        gt.write_text(gt.read_text().replace('0 0 0 1', '0 0 0.7071067811865476 0.7071067811865476'))
        _, receipt = self.run_harness()
        self.assertEqual(receipt['evaluation']['status'], 'OK')
        metrics = json.loads((self.output / 'repeat_001/evaluation/result.json').read_text())['metrics']['se3']
        self.assertAlmostEqual(metrics['ate_translation_m']['rmse'], 0, places=12)
        self.assertAlmostEqual(metrics['rpe_rotation_deg']['rmse'], 0, places=5)

    def test_log_map_points_are_not_map_ids(self):
        counts = harness.log_counts('New Map created with 200 points\nThere are 3 maps in the atlas\n')
        self.assertEqual(counts['observed_map_ids'], [])
        self.assertEqual(counts['map_count_reported'], 3)

    def test_launch_error_retains_all_trials(self):
        self.fake.write_text('invalid executable format\n')
        self.spec['repeats'] = 2
        summary, receipt = self.run_harness()
        self.assertEqual(summary['status_counts']['CRASH'], 2)
        self.assertIsNone(receipt['engine']['exit_code'])
        self.assertTrue(receipt['engine']['launch_error'])

    def test_ground_truth_clock_and_frame_rejected(self):
        self.ground_truth()
        self.spec['ground_truth']['evaluator'] = str(self.fake)
        self.spec['ground_truth']['clock_domain'] = 'other_clock'
        with self.assertRaisesRegex(ValueError, 'clock domain'):
            self.run_harness()
        self.spec['ground_truth']['clock_domain'] = 'dataset_clock'
        self.spec['ground_truth']['frame'] = 'base_link'
        with self.assertRaisesRegex(ValueError, 'frame'):
            self.run_harness()
        self.assertFalse(self.output.exists())

    def test_spec_rejections_before_outputs(self):
        original = copy.deepcopy(self.spec)
        variants = [({'repeats': 0}, 'repeats'), ({'timeout_s': float('nan')}, 'timeout_s'),
                    ({'headless': False}, 'headless'), ({'args': ['{settings.__class__}']}, 'template'),
                    ({'trajectory': {**original['trajectory'], 'path': '../escape'}}, 'artifact'),
                    ({'trajectory': {**original['trajectory'], 'path': 'receipt.json'}}, 'conflicts')]
        for update, error in variants:
            self.spec = {**copy.deepcopy(original), **update}
            with self.subTest(update=update), self.assertRaisesRegex(ValueError, error):
                self.run_harness()
            self.assertFalse(self.output.exists())
        self.spec = original
        self.output = self.dataset / 'out'
        with self.assertRaisesRegex(ValueError, 'overlap'):
            self.run_harness()

    def test_changed_inputs_are_reported(self):
        self.spec['args'] = ['mutate', '{settings}']
        summary, receipt = self.run_harness()
        self.assertEqual(receipt['input_changes'], ['settings'])
        self.assertEqual(summary['input_change_runs'], 1)
        self.assertEqual(summary['denominator'], 1)

    def test_comment_only_times_refused(self):
        Path(self.spec['times']).write_text('# empty input\n')
        with self.assertRaisesRegex(ValueError, 'times file is empty'):
            self.run_harness()
        self.assertFalse(self.output.exists())

    def test_duplicate_yaml_refused(self):
        self.spec_path.write_text('repeats: 1\nrepeats: 2\n')
        with self.assertRaisesRegex(ValueError, 'duplicate'):
            harness.run(self.spec_path, self.output)

    def test_cli_success_failure_and_template_paths_with_spaces(self):
        self.spec['args'] = ['partial']
        self.spec_path = self.base / 'spec with spaces.yaml'
        self.output = self.base / 'out with spaces'
        self.spec_path.write_text(yaml.safe_dump(self.spec))
        cli = subprocess.run([sys.executable, str(ROOT / 'tools/run_harness/harness.py'),
                              str(self.spec_path), '--out', str(self.output)], capture_output=True, text=True)
        self.assertEqual(cli.returncode, 0, cli.stderr)
        self.assertEqual(json.loads(cli.stdout)['denominator'], 1)
        self.spec['args'] = ['crash']
        self.spec_path.write_text(yaml.safe_dump(self.spec))
        cli = subprocess.run([sys.executable, str(ROOT / 'tools/run_harness/harness.py'),
                              str(self.spec_path), '--out', str(self.base / 'crash-cli')], capture_output=True, text=True)
        self.assertEqual(cli.returncode, 1, cli.stderr)
        self.assertEqual(json.loads(cli.stdout)['status_counts']['CRASH'], 1)


if __name__ == '__main__':
    unittest.main()
