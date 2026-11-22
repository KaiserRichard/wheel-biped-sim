import sys
import unittest
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT.parent))
from ground_plane.synth import render, Box
from ground_plane.plane import estimate_plane
from ground_plane.obstacles import angular_height_error


class GroundPlaneTests(unittest.TestCase):
    def test_noiseless_plane_recovery(self):
        d,_,_=render(height=.4,pitch=5.0,width=160,height_px=120,
                     fx=386.953,fy=386.953,cx=80,cy=60)
        out=estimate_plane(d,fx=386.953,fy=386.953,cx=80,cy=60,
                           stride=1,iterations=100,threshold=1e-5,min_inliers=100)
        expected=np.array([0,-np.cos(np.deg2rad(5)),-np.sin(np.deg2rad(5))])
        self.assertTrue(out.valid)
        self.assertLess(np.linalg.norm(out.normal-expected),1e-6)
        self.assertAlmostEqual(out.camera_height,.4,places=6)

    def test_ransac_with_half_depth_outliers(self):
        d,_,_=render(height=.4,pitch=4,width=160,height_px=120,cx=80,cy=60)
        rng=np.random.default_rng(44)
        bad=rng.random(d.shape)<.5
        d[bad]=rng.uniform(.5,5,size=bad.sum())
        out=estimate_plane(d,fx=386.953,fy=386.953,cx=80,cy=60,
                           stride=1,iterations=300,threshold=.002,seed=11,min_inliers=500)
        expected=np.array([0,-np.cos(np.deg2rad(4)),-np.sin(np.deg2rad(4))])
        self.assertTrue(out.valid)
        self.assertLess(np.rad2deg(np.arccos(np.clip(out.normal@expected,-1,1))),.15)

    def test_fixed_mount_angular_height_error(self):
        d=2.3; delta=np.deg2rad(6)
        self.assertAlmostEqual(float(angular_height_error(d,delta)),d*np.tan(delta),places=12)

    def test_floor_depth_at_bottom_row(self):
        pitch=np.deg2rad(4)
        depth,labels,_=render(height=.4,pitch=4,width=640,height_px=480,seed=1)
        col=int(round(321.034)); row=479
        slope=(row-241.040)/386.953
        # Axial Z depth; equivalently ray range is h/sin(ray angle).
        expected=.4/(np.sin(pitch)+np.cos(pitch)*slope)
        self.assertEqual(int(labels[row,col]),1)
        self.assertAlmostEqual(float(depth[row,col]),expected,delta=.002)


if __name__=="__main__": unittest.main()
