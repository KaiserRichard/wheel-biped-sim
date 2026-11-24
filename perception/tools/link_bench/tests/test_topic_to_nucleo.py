import math
import unittest

from tools.link_bench.pi.topic_to_nucleo import message_to_packet
from tools.obstacle_decision.advisory_codec import decode


class TopicToNucleoTests(unittest.TestCase):
    def test_valid_message_encodes_values_and_sequence(self):
        seq, packet = message_to_packet([2, 1.2, .08, .3, .9, 1, 42, .02],
                                        session=123, monotonic_ns=456)
        result = decode(packet)
        self.assertEqual(seq, 42)
        self.assertEqual(result['sequence'], 42)
        self.assertEqual(result['session'], 123)
        self.assertEqual(result['pi_monotonic_ns'], 456)
        self.assertEqual(result['frame_stamp_ns'], 0)
        self.assertEqual(result['advisory'], 2)
        self.assertEqual(result['plane_valid'], True)
        self.assertEqual(result['plane_pitch_rad'], 0)

    def test_invalid_payloads_are_rejected(self):
        base = [1, 1, .1, .2, .8, 0, 4, 0]
        cases = [base[:-1], base + [0], base[:1] + [math.nan] + base[2:],
                 base[:1] + [math.inf] + base[2:], [4] + base[1:],
                 [-1] + base[1:], base[:6] + [1.5, 0],
                 base[:5] + [2] + base[6:]]
        for values in cases:
            with self.subTest(values=values), self.assertRaises(ValueError):
                message_to_packet(values)


if __name__ == '__main__':
    unittest.main()
