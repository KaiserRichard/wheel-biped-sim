"""Small, CPU-only tests for exact matching and fixed-scale rigid alignment."""
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

import align_replays as alignment


BASE = 1792551185752719482
POINTS = np.array([[0, 0, 0], [1, 0, 0], [0, 2, 0], [0, 0, 3], [1, 2, 3]], dtype=float)


def write_replay(directory, stamps, points, maps=None, states=None, quaternions=None):
    """None position means no published pose; preserve the complete frame schedule."""
    directory.mkdir()
    maps = [0] * len(stamps) if maps is None else maps
    states = [2 if p is not None else 3 for p in points] if states is None else states
    quaternions = [[0, 0, 0, 1]] * len(stamps) if quaternions is None else quaternions
    with (directory / 'frames.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['source_ns', 'tracking_state', 'pose_published', 'processed', 'application_dropped', 'map_id'])
        for i, (stamp, point, map_id, state) in enumerate(zip(stamps, points, maps, states), 1):
            writer.writerow([stamp, state, 'true' if point is not None else 'false', i, 0, map_id])
    with (directory / 'CameraTrajectory.txt').open('w') as stream:
        for stamp, point, quaternion in zip(stamps, points, quaternions):
            if point is not None:
                stream.write(f'{stamp // 10**9}.{stamp % 10**9:09d} ' +
                             ' '.join(str(v) for v in [*point, *quaternion]) + '\n')


class RigidFitTests(unittest.TestCase):
    def test_known_rigid_transform(self):
        rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
        translation = np.array([5, -7, 11])
        target = POINTS @ rotation.T + translation
        fitted_r, fitted_t, _ = alignment.umeyama_se3(POINTS, target)
        np.testing.assert_allclose(fitted_r, rotation, atol=1e-14)
        np.testing.assert_allclose(fitted_t, translation, atol=1e-14)
        self.assertAlmostEqual(np.linalg.det(fitted_r), 1)

    def test_reflection_cannot_fit_as_rotation(self):
        target = POINTS * [-1, 1, 1]
        rotation, translation, diagnostic = alignment.umeyama_se3(POINTS, target)
        self.assertAlmostEqual(np.linalg.det(rotation), 1)
        self.assertTrue(diagnostic['reflection_corrected'])
        self.assertGreater(np.linalg.norm(POINTS @ rotation.T + translation - target), 0.5)

    def test_scale_never_fitted(self):
        rotation, translation, _ = alignment.umeyama_se3(POINTS, 2 * POINTS)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-14)
        residual = POINTS @ rotation.T + translation - 2 * POINTS
        self.assertGreater(np.linalg.norm(residual), 1)

    def test_planar_noncollinear_valid(self):
        planar = POINTS.copy()
        planar[:, 2] = 0
        rotation, translation, diag = alignment.umeyama_se3(planar, planar + 1)
        self.assertEqual(diag['source_rank'], 2)
        np.testing.assert_allclose(planar @ rotation.T + translation, planar + 1, atol=1e-14)

    def test_degenerate_trajectories(self):
        for points in (np.zeros((5, 3)), np.array([[i, 2*i, 3*i] for i in range(5)])):
            with self.subTest(points=points), self.assertRaisesRegex(ValueError, 'degenerate'):
                alignment.umeyama_se3(points, points)

    def test_invalid_shapes_and_nonfinite(self):
        for a, b in ((POINTS, POINTS[:-1]), (POINTS[:2], POINTS[:2]),
                     (POINTS[:, :2], POINTS[:, :2]), (POINTS * np.nan, POINTS)):
            with self.subTest(a=a), self.assertRaises(ValueError):
                alignment.umeyama_se3(a, b)

    def test_stats_are_euclidean_rms(self):
        stats = alignment.error_stats(np.array([0, 3, 4]))
        self.assertAlmostEqual(stats['rms_m'], np.sqrt(25/3))
        self.assertEqual(stats['median_m'], 3)
        self.assertAlmostEqual(stats['p95_m'], 3.9)
        self.assertEqual(stats['max_m'], 4)
        self.assertEqual(stats['count'], 3)


class ReplayInputTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.stamps = [BASE + i for i in range(5)]  # one ns apart at a large epoch
        self.a, self.b = self.root / 'a', self.root / 'b'

    def test_stamp_conversion_is_exact(self):
        self.assertEqual(alignment.source_ns('1792551185.752719482'), BASE)
        self.assertEqual(alignment.source_ns('1792551185.752719483'), BASE + 1)
        self.assertEqual(alignment.source_ns('1.1'), 1100000000)

    def test_malformed_stamps(self):
        for stamp in ('1e9', '1.1234567890', '-1.0', 'nan', '1', '01.0', '1.', ' 1.0', '9223372037.0'):
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                alignment.source_ns(stamp)
        for stamp in ('1.0', '1e9', '-0', '01', ' 1'):
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                alignment.integer(stamp, 'source_ns')

    def test_exact_intersection_unequal_schedules(self):
        write_replay(self.a, self.stamps, POINTS)
        write_replay(self.b, self.stamps[1:], POINTS[1:] + 3)
        result, residuals = alignment.align(self.a, self.b)
        self.assertFalse(result['same_source_stamp_schedule'])
        self.assertEqual(result['only_a_tracked_count'], 1)
        self.assertEqual(result['shared_count'], 4)
        self.assertEqual([r[0] for r in residuals], self.stamps[1:])
        self.assertLess(result['after_alignment']['rms_m'], 1e-14)
        self.assertEqual(len(result['input_sha256']), 4)

    def test_same_schedule_with_missing_tracked_pose(self):
        write_replay(self.a, self.stamps, POINTS)
        write_replay(self.b, self.stamps, [None, *POINTS[1:]])
        result, _ = alignment.align(self.a, self.b)
        self.assertTrue(result['same_source_stamp_schedule'])
        self.assertEqual(result['shared_count'], 4)
        self.assertEqual(result['only_a_tracked_count'], 1)

    def test_order_and_duplicate_rejected(self):
        for name, stamps in [('reversed', self.stamps[::-1]), ('duplicate', [BASE] * 5)]:
            path = self.root / name
            write_replay(path, stamps, POINTS)
            with self.assertRaisesRegex(ValueError, 'strictly increasing'):
                alignment.read_replay(path)

    def test_pose_order_rejected_even_when_frames_ordered(self):
        write_replay(self.a, self.stamps, POINTS)
        path = self.a / 'CameraTrajectory.txt'
        path.write_text('\n'.join(path.read_text().splitlines()[::-1]) + '\n')
        with self.assertRaisesRegex(ValueError, 'strictly increasing'):
            alignment.read_replay(self.a)

    def test_unmatched_or_nonfinite_pose_rejected(self):
        write_replay(self.a, self.stamps, POINTS)
        path = self.a / 'CameraTrajectory.txt'
        original = path.read_text()
        path.write_text('\n'.join(original.splitlines()[1:]) + '\n')
        with self.assertRaisesRegex(ValueError, 'published frames'):
            alignment.read_replay(self.a)
        path.write_text(original.replace('0.0', 'nan', 1))
        with self.assertRaisesRegex(ValueError, 'nonfinite'):
            alignment.read_replay(self.a)

    def test_cli_outputs_and_exclusive_creation(self):
        write_replay(self.a, self.stamps, POINTS)
        write_replay(self.b, self.stamps, POINTS + 3)
        output = self.root / 'report'
        cmd = [sys.executable, str(Path(alignment.__file__)), str(self.a), str(self.b), str(output)]
        result = subprocess.run(cmd, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads((output / 'report.json').read_text())
        self.assertEqual(report['fit']['scale'], 1)
        self.assertEqual(report['shared_count'], 5)
        original = (output / 'report.json').read_bytes()
        self.assertNotEqual(subprocess.run(cmd, capture_output=True).returncode, 0)
        self.assertEqual((output / 'report.json').read_bytes(), original)


if __name__ == '__main__':
    unittest.main()
