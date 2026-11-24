"""The generated five-bar closes with the SOURCE-SUPPORTED link lengths (no MuJoCo needed)."""

import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))

from sim.models import make_wbr_3d_simple as gen  # noqa: E402


class FiveBar(unittest.TestCase):
    def test_link_lengths_close(self):
        for l0 in (0.08, 0.12, 0.18):
            o, d, a, c, b = gen.leg_points(l0)
            self.assertAlmostEqual(math.dist(o, d), 0.08, places=9)
            self.assertAlmostEqual(math.dist(o, a), 0.075, places=9)
            self.assertAlmostEqual(math.dist(d, c), 0.075, places=9)
            self.assertAlmostEqual(math.dist(a, b), 0.14, places=9)
            self.assertAlmostEqual(math.dist(c, b), 0.14, places=9)
            self.assertAlmostEqual(math.dist(((o[0] + d[0]) / 2, l0), b), l0, places=9)
            self.assertLess(a[0], o[0])   # knees splay outwards, as in report Fig. 2.1
            self.assertGreater(c[0], d[0])

    def test_unreachable_length_rejected(self):
        with self.assertRaises(ValueError):
            gen.leg_points(0.25)

    def test_committed_xml_is_generated(self):
        xml = (ROOT / "sim/models/wbr_3d_simple.xml").read_text()
        self.assertEqual(xml, gen.build(0.12), "re-run: python -m sim.models.make_wbr_3d_simple")


if __name__ == "__main__":
    unittest.main()
