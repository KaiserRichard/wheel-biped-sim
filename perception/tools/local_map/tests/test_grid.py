import unittest
import numpy as np
from tools.local_map.grid import ElevationGrid


class ElevationGridTests(unittest.TestCase):
    def test_flat_floor_is_zero_height(self):
        g=ElevationGrid(size_m=2.,resolution_m=.1)
        pts=np.array([[0.,0.,0.],[.1,0.,0.],[0.,.1,0.]])
        g.update(pts,time_s=0.)
        out=g.outputs(time_s=0.)
        self.assertTrue(np.allclose(out["height"][out["known"]],0.))

    def test_box_cell_height_is_within_one_centimetre(self):
        g=ElevationGrid(size_m=2.,resolution_m=.1)
        g.update(np.array([[0.01,0.01,.05],[0.02,0.02,.05]]),time_s=0.)
        out=g.outputs(time_s=0.)
        self.assertAlmostEqual(out["height"][out["known"]][0],.05,delta=.01)

    def test_scrolling_preserves_world_cell(self):
        g=ElevationGrid(size_m=2.,resolution_m=.1)
        g.update(np.array([[0.01,0.01,.07]]),time_s=0.)
        before=g._index(np.array([[0.01,0.01]]))
        g.scroll((.2,0.))
        after=g._index(np.array([[0.01,0.01]]))
        self.assertNotEqual(before,after)
        r,c=after
        self.assertAlmostEqual(g.mean[r[0],c[0]],.07)

    def test_variance_decreases_on_repeated_observations(self):
        g=ElevationGrid(size_m=2.,resolution_m=.1)
        p=np.array([[.01,.01,.05]])
        g.update(p,.01,time_s=0.)
        r,c=g._index(p[:,:2]); first=g.variance[r[0],c[0]]
        g.update(p,.01,time_s=.1); second=g.variance[r[0],c[0]]
        self.assertLess(second,first)


if __name__=="__main__": unittest.main()
