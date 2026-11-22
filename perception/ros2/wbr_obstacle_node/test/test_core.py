import pathlib
import sys
import unittest

import numpy as np

HERE = pathlib.Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1]))
sys.path.insert(0, str(HERE.parents[3]))
from wbr_obstacle_node.core import (AdvisoryCore, CoreParams, FrameRateLimiter,
                                    depth_to_metres, marker_specs,
                                    pack_terrain_advisory)

ROOT = HERE.parents[3]


class CoreTests(unittest.TestCase):
    def test_16uc1_millimetres_and_zeros(self):
        got = depth_to_metres(np.array([[0, 750, 2000]], dtype=np.uint16), '16UC1')
        np.testing.assert_allclose(got, [[0, .75, 2.]])

    def test_rate_limiter_drops_without_queue(self):
        limiter = FrameRateLimiter(10)
        self.assertTrue(limiter.accept(1.0))
        self.assertFalse(limiter.accept(1.02))
        self.assertTrue(limiter.accept(1.1))

    def test_marker_spec_contents(self):
        advisory = {'plane_valid': True, 'plane_height_m': .4, 'plane_pitch_rad': 0.,
                    'distance_m': 1.2, 'height_m': .05, 'width_m': .25,
                    'advisory': 'JUMP_CANDIDATE'}
        specs = marker_specs(advisory, .3, 2., 'depth_optical')
        self.assertEqual([s['type'] for s in specs],
                         ['TRIANGLE_LIST', 'CUBE', 'TEXT_VIEW_FACING', 'LINE_LIST'])
        self.assertTrue(all(s['frame_id'] == 'depth_optical' for s in specs))
        self.assertEqual(specs[2]['text'], '0.050 m')
        self.assertEqual(len(specs[0]['points']), 6)
        self.assertEqual(len(specs[3]['points']), 4)
        self.assertAlmostEqual(specs[1]['scale'][1], .05)

    def test_terrain_advisory_array_packing(self):
        advisory = {'advisory_code': 1, 'distance_m': 1.25, 'height_m': .05,
                    'width_m': .30, 'confidence': .8, 'plane_valid': True}
        self.assertEqual(pack_terrain_advisory(advisory, 42, .012),
                         [1.0, 1.25, .05, .30, .8, 1.0, 42.0, .012])

    def test_terrain_advisory_missing_values_are_finite_zeros(self):
        advisory = {'advisory_code': 0, 'distance_m': None, 'height_m': None,
                    'width_m': None, 'confidence': float('nan'), 'plane_valid': False}
        packed = pack_terrain_advisory(advisory, 0, float('inf'))
        self.assertEqual(packed, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.assertTrue(all(np.isfinite(packed)))

    def test_terrain_advisory_rejects_unknown_code(self):
        with self.assertRaises(ValueError):
            pack_terrain_advisory({'advisory_code': 4}, 1, 0.0)

    def test_synthetic_five_cm_box(self):
        from tools.ground_plane import synth
        depth, labels, _ = synth.render(
            height=.4, width=160, height_px=120, fx=100., fy=100., cx=79.5, cy=59.5,
            boxes=[synth.Box(position=(0., 1.0, .025), size=(.30, .25, .05))],
            max_range=4.)
        core = AdvisoryCore(ROOT, CoreParams(stride=1, point_stride=4, confirm_n=1,
                                             confirm_m=1, min_cells=2))
        out = core.process(depth, np.array([[100., 0., 79.5], [0., 100., 59.5], [0., 0., 1.]]),
                           frame_stamp_ns=123, frame_id='depth_optical', now_monotonic=10.)
        self.assertTrue(out['advisory']['plane_valid'])
        self.assertGreater(int(np.count_nonzero(labels == 2)), 0)
        self.assertAlmostEqual(out['advisory']['height_m'], .05, delta=.025)
        self.assertGreater(out['advisory']['distance_m'], .7)
        self.assertEqual(out['advisory']['frame_stamp_ns'], 123)
        self.assertGreater(len(out['advisory_bytes']), 0)
        self.assertGreater(out['points'].shape[0], 0)


if __name__ == '__main__':
    unittest.main()
