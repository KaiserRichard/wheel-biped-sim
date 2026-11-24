import unittest
import numpy as np
from tools.tilt_sweep.sweep import run_case

class TiltSweepTests(unittest.TestCase):
    def test_pose_compensation_and_fixed_mount_error(self):
        zero=run_case(0,0,1.,.132,0)
        tilted=run_case(5,0,1.,.132,0)
        p0=next(x for x in zero if x['method']=='plane_fit')
        p5=next(x for x in tilted if x['method']=='plane_fit')
        f5=next(x for x in tilted if x['method']=='fixed_mount')
        self.assertIsNotNone(p0['height_error_m'])
        self.assertLess(abs(p0['height_error_m']),.01)
        self.assertIsNotNone(p5['height_error_m'])
        self.assertLess(abs(p5['height_error_m']),.01)
        expected=np.sin(np.deg2rad(5))
        self.assertAlmostEqual(f5['height_error_m'],expected,delta=.025)

if __name__=='__main__': unittest.main()
