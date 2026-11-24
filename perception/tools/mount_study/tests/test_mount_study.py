"""No graphics context or MuJoCo import required."""
import math
import unittest
import xml.etree.ElementTree as ET

from tools.mount_study.geometry import vertical_fov, visibility
from tools.mount_study.sweep import Config, aggregate, build_xml, configurations, make_scenario
from tools.ground_plane.synth import FY


class GeometryTests(unittest.TestCase):
    def test_level_90_degree_camera(self):
        result = visibility(.30, .10, 0, 90)
        self.assertAlmostEqual(result['floor_near_m'], .30)
        self.assertAlmostEqual(result['top_near_m'], .20)
        self.assertTrue(math.isinf(result['top_far_m']))

    def test_downward_camera_has_finite_far_edge(self):
        result = visibility(.30, .10, 45, 60)
        self.assertAlmostEqual(result['top_near_m'], .20/math.tan(math.radians(75)))
        self.assertAlmostEqual(result['top_far_m'], .20/math.tan(math.radians(15)))

    def test_monotonic_pitch_height_and_floor(self):
        a = visibility(.21, .046, 0, vertical_fov(FY))
        b = visibility(.21, .046, 25, vertical_fov(FY))
        taller = visibility(.21, .132, 0, vertical_fov(FY))
        self.assertLess(b['top_near_m'], a['top_near_m'])
        self.assertLess(taller['top_near_m'], a['top_near_m'])
        self.assertGreater(a['floor_near_m'], a['top_near_m'])
        self.assertAlmostEqual(vertical_fov(FY), 63.616892, places=3)

    def test_floor_is_zero_height_box(self):
        r = visibility(.15, 0, 10, 60)
        self.assertEqual(r['floor_near_m'], r['top_near_m'])

    def test_reject_invalid_geometry(self):
        for args in ((.1,.2,0,60), (.1,-1,0,60), (.1,0,70,60),
                     (.1,0,-40,60), (.1,0,0,0), (math.nan,0,0,60)):
            with self.assertRaises(ValueError):
                visibility(*args)
        for fy in (0, -1, math.inf, math.nan):
            with self.assertRaises(ValueError):
                vertical_fov(fy)


class SweepTests(unittest.TestCase):
    def test_grid_and_pose_mapping(self):
        configs = configurations()
        self.assertEqual(len(configs), 54)
        self.assertEqual(len({c.key for c in configs}), 54)
        for c in configs:
            self.assertTrue(.06 <= c.l0 <= .20)
            world_body = .0603+c.l0+.03-.06
            self.assertAlmostEqual(world_body+c.camera_body_z, c.height_m)
            scenario = make_scenario(c)
            self.assertAlmostEqual(-scenario['robot']['start_x']-.15, 2.)
            self.assertAlmostEqual(-scenario['robot']['start_x']-.15-.25*7.9, .025)
            root = ET.fromstring(build_xml(c))
            camera = root.find(".//camera[@name='simcam_d435']")
            self.assertAlmostEqual(float(camera.get('pos').split()[2]), c.camera_body_z)
            axes = list(map(float, camera.get('xyaxes').split()))
            self.assertAlmostEqual(math.degrees(math.atan2(axes[3], axes[5])), c.pitch_deg)

    def test_fake_timeline_excludes_held_advisory_and_contact(self):
        c = Config(.21, 10, .132)
        rows = []
        for i in range(6):
            rows.append(dict(t=i/10, gap_m=1-i*.1, precontact=i<5,
                             raw_advisory='STOP_CANDIDATE' if i>=1 else 'UNCERTAIN',
                             fresh_correct=i in (1,2,4,5), plane_valid=i!=3,
                             plane_truth=i!=3, plane_reused=i==3))
        r = aggregate(rows, c)
        self.assertAlmostEqual(r['advisory_far_gap_m'], .9)
        self.assertAlmostEqual(r['supported_blind_gap_m'], .6)
        self.assertEqual(r['supported_last_tick'], .4)
        self.assertEqual(r['supported_first_tick'], .1)
        self.assertEqual(r['supported_runs'], 2)
        self.assertEqual(r['plane_reused_ticks'], 1)
        self.assertAlmostEqual(r['plane_truth_fraction'], .8)
        self.assertTrue(r['end_censored'])
        self.assertFalse(r['start_censored'])
        held = aggregate([dict(rows[3], precontact=True)], c)
        self.assertIsNone(held['supported_blind_gap_m'])
        self.assertEqual(held['advisory_blind_gap_m'], .7)

    def test_no_observations_is_unknown(self):
        r = aggregate([], Config(.30, 0, .046))
        self.assertIsNone(r['supported_blind_gap_m'])
        self.assertIsNone(r['plane_truth_fraction'])
        self.assertEqual(r['ticks'], 0)


if __name__ == '__main__':
    unittest.main()
