from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
from tools.scene_twin.replay import (Packet, Replay, read_packets, bag_start,
                                     CAMERA_OFFSET, COLORS, NAMES)
from tools.scene_twin.scene import build_xml


class ReplayTests(unittest.TestCase):
    def packets(self, distances):
        return [Packet(i, i % 4, .1+i*.01, d, 1792900867686080566+i*100000000)
                for i, d in enumerate(distances)]

    def test_parse_exact_nanoseconds_and_invalid_distance(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'packets.csv'
            path.write_text('seq,advisory,height_m,distance_m,frame_stamp_ns\n1,2,.13,nan,1792900867686080566\n')
            rows = read_packets(path)
            self.assertEqual(rows[0].stamp, 1792900867686080566)
            self.assertEqual(rows[0].advisory, 2)

    def test_parse_rejects_empty_bad_advisory_and_reordered_stamps(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/'packets.csv'
            header = 'seq,advisory,height_m,distance_m,frame_stamp_ns\n'
            for body in ('', '1,4,.1,1,100\n', '1,1,.1,1,100\n2,1,.1,1,99\n'):
                path.write_text(header+body)
                with self.assertRaises(ValueError):
                    read_packets(path)

    def test_anchor_camera_gap_and_floor_box(self):
        r = Replay(self.packets([0, 1.2, .9]), 0)
        self.assertEqual(r.anchor, 1)
        self.assertAlmostEqual(r.face-r.x[1]-CAMERA_OFFSET, 1.2)
        root = ET.fromstring(build_xml(r))
        box = root.find("worldbody/geom[@name='obstacle']")
        center = list(map(float, box.get('pos').split()))
        half = list(map(float, box.get('size').split()))
        self.assertAlmostEqual(center[0]-half[0], r.face)
        self.assertAlmostEqual(center[2]-half[2], 0)
        self.assertAlmostEqual(half[2]*2, .115)  # median advisory 1/2 heights
        self.assertEqual(len(root.findall('worldbody/body')), 1)

    def test_median_outlier_hold_and_backfill(self):
        r = Replay(self.packets([0, 1, 1, 99, float('nan'), 0, .7, .6]), 0)
        self.assertEqual(r.distances[:6], [1]*6)
        self.assertEqual(r.distances[6:], [1, .7])
        self.assertAlmostEqual(r.x[-1], .3)

    def test_static_runs_stay_still_despite_detector_jitter(self):
        r = Replay(self.packets([1, .9, .8, 0]), 0, motion='static')
        self.assertEqual(r.x, [0]*4)

    def test_no_observation_placeholder_is_labelled(self):
        rows = [Packet(1, 0, 0, 0, 100)]
        r = Replay(rows, 200)
        self.assertIsNone(r.anchor)
        self.assertIn('ASSUMED placeholder', r.text(0))
        self.assertAlmostEqual(r.x[0], 0)

    def test_time_lookup_preserves_epoch_precision(self):
        rows = self.packets([1, 1, 1])
        r = Replay(rows, rows[0].stamp+100000000)
        self.assertEqual([r.index_at(t) for t in (-1, 0, .099, .1, 99)], [0, 0, 0, 1, 2])
        self.assertIn('bag-relative=-0.100s', r.text(0))
        self.assertIn(str(rows[0].stamp), r.text(0))

    def test_bag_origin(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/'camera-bag'
            p.mkdir()
            (p/'metadata.yaml').write_text('  starting_time\x3a\n    nanoseconds_since_epoch: 1792900867686080566\n')
            self.assertEqual(bag_start(d), 1792900867686080566)

    def test_missing_bag_origin_is_explicit(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertIsNone(bag_start(d))
        r = Replay(self.packets([1]), None)
        self.assertFalse(r.bag_origin_known)
        self.assertIn('ASSUMED origin (bag UNKNOWN)', r.text(0))
        self.assertNotIn('bag-relative=', r.text(0))

    def test_advisory_mapping(self):
        self.assertEqual(NAMES, ('NONE', 'JUMP_CANDIDATE', 'STOP_CANDIDATE', 'UNCERTAIN'))
        green, blue, red, grey = COLORS
        self.assertGreater(green[1], green[0])
        self.assertGreater(blue[2], blue[0])
        self.assertGreater(red[0], red[1])
        self.assertEqual(grey[0], grey[1])
        self.assertEqual(grey[1], grey[2])


if __name__ == '__main__':
    unittest.main()
