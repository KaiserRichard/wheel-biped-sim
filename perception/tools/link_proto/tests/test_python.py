"""CANDIDATE synthetic cross-language fixtures, not hardware qualification."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('link_proto', ROOT / 'py/link_proto.py')
lp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lp)


def examples():
    return {
        'health': lp.perception_health(
            0x0102030405060708, 8, estimate_seq=7, map_epoch=3,
            transform_revision=4, mapping_revision=5, reset_counter=6,
            tracking=2, estimator_id=1, source_domain=1, flags=0x7f,
            frame_id=2, child_frame_id=1, quality=0, reasons=0,
            source_ns=10001000000, mapped_mcu_ns=1000000,
            pi_send_ns=10002000000, age_at_send_ns=1000000,
            uncertainty_ns=1000000),
        'ref': lp.body_velocity_ref(
            0x0102030405060708, 9, estimate_seq=7, map_epoch=3,
            mapping_revision=5, frame_id=1, reset_counter=6,
            pi_creation_ns=10002000000, mapped_mcu_creation_ns=2000000,
            lease_us=100000, forward_m_s=0.25, yaw_rad_s=-0.5),
        'status': lp.mcu_status(
            42, 10, sample_mcu_ns=3000000, last_health_seq=8,
            last_estimate_seq=7, last_ref_seq=9, faults=0,
            action_mcu_ns=2000000, battery_mv=24000, fsm=2,
            armed=1, operator_enable=1, state=2, reason=0, battery_valid=1),
        'advisory': lp.terrain_advisory(0x0102030405060708, 11,
            pi_monotonic_ns=10002000000, frame_stamp_ns=2000000000,
            advisory=1, confidence=0.75, distance_m=1.25, height_m=0.08,
            width_m=0.3, plane_valid=1, plane_pitch_rad=-0.1, plane_height_m=0.42),
        'echo': lp.advisory_echo(42, 12, sample_mcu_ns=3000000,
            advisory_seq=11, advisory=1, result=0, accepted_count=9,
            rejected_count=2, stale=0, ms_since_accepted=15),
    }


class EncoderTests(unittest.TestCase):
    def test_frozen_golden_vectors(self):
        vectors = {}
        for line in (ROOT / 'tests/vectors.txt').read_text().splitlines():
            if line and not line.startswith('#'):
                name, hex_wire = line.split()
                vectors[name] = bytes.fromhex(hex_wire)
        self.assertEqual(examples(), vectors)

    def test_crc(self):
        self.assertEqual(lp.crc32c(b'123456789'), 0xE3069283)
        self.assertEqual(lp.crc32c(b''), 0)

    def test_cobs_known(self):
        self.assertEqual(lp.cobs_encode(b''), b'\x01')
        self.assertEqual(lp.cobs_encode(b'\0'), b'\x01\x01')
        self.assertEqual(lp.cobs_encode(b'\x11\x22\0\x33'), b'\x03\x11\x22\x02\x33')
        self.assertEqual(lp.cobs_encode(b'\x01' * 254), b'\xff' + b'\x01' * 255)

    def test_nonfinite_rejected(self):
        for value in [float('nan'), float('inf'), -float('inf')]:
            with self.assertRaises(ValueError):
                lp.body_velocity_ref(1, 1, estimate_seq=1, map_epoch=1,
                    mapping_revision=1, frame_id=1, reset_counter=0,
                    pi_creation_ns=1, mapped_mcu_creation_ns=1, lease_us=100000,
                    forward_m_s=value, yaw_rad_s=0)

    def test_advisory_validation_and_echo_decode(self):
        for value in [float('nan'), float('inf'), -float('inf')]:
            with self.assertRaises(ValueError):
                lp.terrain_advisory(1, 1, pi_monotonic_ns=1, frame_stamp_ns=1,
                    advisory=0, confidence=value, distance_m=0, height_m=0,
                    width_m=0, plane_valid=0, plane_pitch_rad=0, plane_height_m=0)
        args=dict(pi_monotonic_ns=1,frame_stamp_ns=1,confidence=0.,distance_m=0.,
            height_m=0.,width_m=0.,plane_pitch_rad=0.,plane_height_m=0.)
        for advisory,valid in ((4,0),(0,2)):
            with self.assertRaises(ValueError): lp.terrain_advisory(1,1,advisory=advisory,plane_valid=valid,**args)
        malformed=lp._wire(lp.TERRAIN_ADVISORY,1,1,b'\0'*41)
        with self.assertRaises(ValueError): lp.unpack(malformed)
        header, body = lp.unpack(examples()['echo'])
        self.assertEqual((header['type'], body['result'], body['advisory_seq']), (5, 0, 11))


if __name__ == '__main__':
    unittest.main()
