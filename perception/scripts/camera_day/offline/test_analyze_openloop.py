"""No native replay: small loss/map/jump fixtures exercise endpoint boundaries."""
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

import analyze_openloop as analysis
from test_align_replays import BASE, write_replay


class SegmentTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.replay = self.root / 'replay'
        self.settings, self.log, self.pairs = [self.root / name for name in ('stereo.yaml', 'replay.log', 'pairs.csv')]
        self.settings.write_text('%YAML:1.0\nloopClosing: 0\n')
        self.log.write_text('Creation of new map with last KF id: 0\n')

    def fixture(self, points, **kwargs):
        self.stamps = [BASE + i * 66666667 for i in range(len(points))]
        write_replay(self.replay, self.stamps, points, **kwargs)
        with self.pairs.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['source_ns', 'left', 'right'])
            writer.writerows((s, 'left.png', 'right.png') for s in self.stamps)

    def analyze(self, **kwargs):
        return analysis.analyze(self.replay, self.pairs, self.settings, self.log,
                                expected_pairs=len(self.stamps), **kwargs)

    def test_complete_return_is_conditional(self):
        self.fixture([[0, 0, 0], [.1, 0, 0], [.02, 0, 0]])
        r = self.analyze(physical_return='owner-reported')
        self.assertTrue(r['closed_path_drift_candidate']['eligible'])
        self.assertAlmostEqual(r['closed_path_drift_candidate']['endpoint_m'], .02)
        self.assertAlmostEqual(r['tracked_segments'][0]['within_segment_path_m'], .18)
        self.assertFalse(self.analyze()['closed_path_drift_candidate']['eligible'])

    def test_loss_same_map_never_bridged(self):
        self.fixture([[0, 0, 0], [.1, 0, 0], None, None, [10, 0, 0], [10.1, 0, 0]],
                     states=[2, 2, 3, 4, 2, 2])
        r = self.analyze(physical_return='owner-reported')
        self.assertEqual(r['loss_transitions'], 1)
        self.assertEqual(len(r['state_spans']), 4)
        self.assertEqual([s['tracked_count'] for s in r['tracked_segments']], [2, 2])
        self.assertEqual(r['start_segment_before_first_break']['end_source_ns'], self.stamps[1])
        self.assertAlmostEqual(r['per_map'][0]['sum_within_segment_path_m'], .2)
        self.assertIsNone(r['closed_path_drift_candidate']['endpoint_m'])

    def test_map_changes_even_if_all_tracked(self):
        self.fixture([[0, 0, 0], [.1, 0, 0], [9, 0, 0], [9.1, 0, 0], [.02, 0, 0]],
                     maps=[0, 0, 1, 1, 0])
        r = self.analyze(physical_return='owner-reported')
        self.assertEqual(r['map_changes'], 2)
        self.assertEqual([s['map_id'] for s in r['tracked_segments']], [0, 1, 0])
        self.assertEqual(r['per_map'][0]['segment_ids'], [1, 3])
        self.assertFalse(r['closed_path_drift_candidate']['eligible'])
        self.assertAlmostEqual(r['per_map'][0]['sum_within_segment_path_m'], .1)

    def test_translation_jump_never_bridged(self):
        self.fixture([[0, 0, 0], [.1, 0, 0], [1.2, 0, 0], [1.3, 0, 0]])
        r = self.analyze(physical_return='owner-reported')
        self.assertEqual(len(r['tracked_segments']), 2)
        self.assertEqual(r['boundaries'][0]['reasons'], ['pose_jump'])
        self.assertAlmostEqual(r['boundaries'][0]['translation_step_m'], 1.1)
        self.assertAlmostEqual(r['per_map'][0]['sum_within_segment_path_m'], .2)
        self.assertFalse(r['closed_path_drift_candidate']['eligible'])

    def test_rotation_jump_and_quaternion_sign(self):
        self.fixture([[0, 0, 0], [.01, 0, 0], [.02, 0, 0]],
                     quaternions=[[0, 0, 0, 1], [0, 0, 0, -1], [0, 0, np.sqrt(.5), np.sqrt(.5)]])
        r = self.analyze()
        self.assertEqual([s['tracked_count'] for s in r['tracked_segments']], [2, 1])
        self.assertAlmostEqual(r['boundaries'][0]['rotation_step_deg'], 90)

    def test_late_initialization_does_not_cover_physical_start(self):
        self.fixture([None, [0, 0, 0], [.1, 0, 0], [0, 0, 0]], states=[1, 2, 2, 2])
        r = self.analyze(physical_return='owner-reported')
        self.assertFalse(r['closed_path_drift_candidate']['covers_full_record'])
        self.assertFalse(r['closed_path_drift_candidate']['eligible'])

    def test_final_loss_and_no_tracked_poses(self):
        self.fixture([[0, 0, 0], [.1, 0, 0], None])
        r = self.analyze(physical_return='owner-reported')
        self.assertEqual(r['tracked_segments'][0]['end_source_ns'], self.stamps[1])
        self.assertFalse(r['closed_path_drift_candidate']['eligible'])
        (self.replay / 'CameraTrajectory.txt').write_text('')
        frames = self.replay / 'frames.csv'
        frames.write_text(frames.read_text().replace(',2,true,', ',3,false,'))
        r = self.analyze()
        self.assertEqual(r['tracked_segments'], [])
        self.assertIsNone(r['start_segment_before_first_break'])

    def test_publication_loss_even_with_state_ok(self):
        self.fixture([[0, 0, 0], None, [.1, 0, 0]], states=[2, 2, 2])
        r = self.analyze()
        self.assertEqual(r['loss_transitions'], 1)
        self.assertEqual(len(r['tracked_segments']), 2)

    def test_native_markers_invalidate_candidate(self):
        self.fixture([[0, 0, 0], [.1, 0, 0], [0, 0, 0]])
        for marker in ('*Loop detected', '*Merge detected', 'Merge finished!',
                       'LM: Active map reset, Done!!!', 'LM: Map reset, Done!!!',
                       'Creation of new map with last KF id: 0\nCreation of new map with last KF id: 10'):
            self.log.write_text(marker + '\n')
            with self.subTest(marker=marker):
                self.assertFalse(self.analyze(physical_return='owner-reported')['closed_path_drift_candidate']['eligible'])

    def test_settings_default_duplicate_and_malformed(self):
        self.fixture([[0, 0, 0], [.1, 0, 0]])
        self.settings.write_text('%YAML:1.0\nCamera.fps: 15\n')
        self.assertTrue(self.analyze()['loop_closing_enabled_from_settings'])
        for text in ('loopClosing: 0\nloopClosing: 1\n', 'loopClosing: false\n', 'loopClosing: 0.0\n'):
            self.settings.write_text(text)
            with self.subTest(text=text), self.assertRaisesRegex(ValueError, 'one integer'):
                self.analyze()

    def test_coverage_mismatch_and_missing_rows_rejected(self):
        self.fixture([[0, 0, 0], [.1, 0, 0], [.01, 0, 0]])
        self.pairs.write_text(self.pairs.read_text().replace(str(self.stamps[1]), str(self.stamps[1] + 1)))
        with self.assertRaisesRegex(ValueError, 'every expected pair'):
            self.analyze()

    def test_bad_frame_metadata(self):
        self.fixture([[0, 0, 0], [.1, 0, 0]])
        path = self.replay / 'frames.csv'
        original = path.read_text()
        for old, new in [(',2,true,', ',3,true,'), (',true,', ',yes,'), (',1,0,0', ',2,0,0'),
                         (',1,0,0', ',1,1,0'), (str(BASE), str(BASE) + '.0')]:
            path.write_text(original.replace(old, new, 1))
            with self.subTest(new=new), self.assertRaises(ValueError):
                self.analyze()

    def test_invalid_thresholds(self):
        self.fixture([[0, 0, 0], [.1, 0, 0]])
        for kwargs in ({'jump_threshold_m': 0}, {'jump_threshold_m': float('nan')},
                       {'jump_threshold_deg': 181}, {'jump_threshold_deg': 0}):
            with self.subTest(kwargs=kwargs), self.assertRaisesRegex(ValueError, 'thresholds'):
                self.analyze(**kwargs)

    def test_cli_with_loss_fixture(self):
        self.fixture([[0, 0, 0], [.1, 0, 0], None, [3, 0, 0]], maps=[0, 0, 0, 1])
        output = self.root / 'output'
        cmd = [sys.executable, str(Path(analysis.__file__)), str(self.replay), str(self.pairs), str(output),
               '--settings', str(self.settings), '--log', str(self.log), '--expected-pairs', '4',
               '--physical-return', 'owner-reported']
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads((output / 'report.json').read_text())
        self.assertEqual(len(report['tracked_segments']), 2)
        self.assertIsNone(report['closed_path_drift_candidate']['endpoint_m'])
        original = (output / 'report.json').read_bytes()
        self.assertNotEqual(subprocess.run(cmd, capture_output=True).returncode, 0)
        self.assertEqual((output / 'report.json').read_bytes(), original)


if __name__ == '__main__':
    unittest.main()
