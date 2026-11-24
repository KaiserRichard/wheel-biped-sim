"""Check rotation geometry and reproducibility of the research table."""

import math
from pathlib import Path
import unittest

from tools.scene_twin.tilt_error import markdown_table, pitch_errors


class TiltErrorTests(unittest.TestCase):
    def test_zero_and_quarter_turn(self):
        self.assertEqual(pitch_errors(1, .12, .30, 0), (0, 0))
        dh, dr = pitch_errors(1, .12, .30, 90)
        self.assertAlmostEqual(dh, 1.18)
        self.assertAlmostEqual(dr, -.82)

    def test_rotation_preserves_camera_range(self):
        for angle in (-15, -1, 1, 15):
            dh, dr = pitch_errors(1.3, .12, .30, angle)
            self.assertAlmostEqual(math.hypot(1.3 + dr, -.18 + dh), math.hypot(1.3, -.18))

    def test_first_order_sensitivity(self):
        angle = 1e-5
        dh, dr = pitch_errors(1.3, .12, .30, angle)
        self.assertAlmostEqual(dh / math.radians(angle), 1.3, places=6)
        self.assertAlmostEqual(dr / math.radians(angle), .18, places=6)

    def test_invalid_inputs(self):
        for args in ((-1, .12, .30, 1), (1, float('nan'), .30, 1),
                     (1, .12, float('inf'), 1), (1, .12, .30, float('nan'))):
            with self.assertRaises(ValueError):
                pitch_errors(*args)

    def test_document_table_matches_script(self):
        root = Path(__file__).resolve().parents[3]
        doc = (root / 'docs/scene_twin/TILT_JUMP_RESEARCH.md').read_text()
        self.assertIn(markdown_table(), doc)


if __name__ == '__main__':
    unittest.main()
