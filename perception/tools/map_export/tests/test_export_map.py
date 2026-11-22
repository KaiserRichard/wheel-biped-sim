import struct
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

from tools.map_export.export_map import decode_pointcloud2, read_ply, voxel_downsample, write_ply, classify_status


class ExportTests(unittest.TestCase):
    def test_pointcloud2_xyz_decode(self):
        fields = [NS(name=n, offset=i * 4, datatype=7, count=1) for i, n in enumerate(("x", "y", "z"))]
        data = struct.pack("<ffffff", 1.25, -2.0, 3.5, 4.0, 5.0, 6.0)
        msg = NS(fields=fields, width=2, height=1, point_step=12, row_step=24, data=data, is_bigendian=False)
        np.testing.assert_allclose(decode_pointcloud2(msg), [[1.25, -2, 3.5], [4, 5, 6]])

    def test_voxel_downsample(self):
        pts = np.array([[.001, .001, .001], [.019, .019, .019], [.021, .001, .001], [.05, 0, 0]])
        out = voxel_downsample(pts, .02)
        self.assertEqual(len(out), 3)
        np.testing.assert_array_equal(out, pts[[0, 2, 3]])

    def test_ply_round_trip(self):
        pts = np.array([[1.25, -2, 3.5], [0, .125, -9]], dtype=float)
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "map.ply"
            write_ply(path, pts)
            np.testing.assert_allclose(read_ply(path), pts, rtol=1e-7)

    def test_numeric_tracking_status(self):
        # ORB_SLAM3 Tracking.h: NOT_INITIALIZED=1, OK=2, RECENTLY_LOST=3.
        self.assertEqual(classify_status("", {"tracking_state": 1}), "NOT_INITIALIZED")
        self.assertEqual(classify_status("", {"tracking_state": 2}), "OK")
        self.assertEqual(classify_status("", {"tracking_state": 3}), "RECENTLY_LOST")
        self.assertEqual(classify_status("", {"tracking_state": "TRACKING"}), "TRACKING")


if __name__ == "__main__":
    unittest.main()
