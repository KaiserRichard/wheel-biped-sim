"""TerrainAdvisory API backed by the shared link_proto implementation."""
from tools.link_proto.py import link_proto as lp

TYPE = lp.TERRAIN_ADVISORY
PAYLOAD_SIZE = 42


def encode(session, sequence, *, pi_monotonic_ns, frame_stamp_ns, advisory,
           confidence, distance_m, height_m, width_m, plane_valid,
           plane_pitch_rad, plane_height_m):
    return lp.terrain_advisory(
        session, sequence, pi_monotonic_ns=pi_monotonic_ns,
        frame_stamp_ns=frame_stamp_ns, advisory=advisory,
        confidence=confidence, distance_m=distance_m, height_m=height_m,
        width_m=width_m, plane_valid=plane_valid,
        plane_pitch_rad=plane_pitch_rad, plane_height_m=plane_height_m)


def decode(packet):
    header, body = lp.unpack(packet)
    if header['type'] != TYPE:
        raise ValueError('not a TerrainAdvisory')
    decoded = dict(session=header['session'], sequence=header['sequence'], **body)
    decoded['plane_valid'] = bool(body['plane_valid'])
    return decoded
