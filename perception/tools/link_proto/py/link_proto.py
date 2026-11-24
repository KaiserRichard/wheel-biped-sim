"""CANDIDATE / PROPOSED Pi encoder; firmware owner review required.

No transport, clock mapping, enable action, or live estimate generation.
Inputs must come from a reviewed snapshot/mapping service; see ../SPEC.md.
"""
import math
import struct

VERSION = 1
HEALTH, BODY_REF, MCU_STATUS, TERRAIN_ADVISORY, ADVISORY_ECHO = 1, 2, 3, 4, 5
MAX_RAW = 192


def crc32c(data):
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x82F63B78 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


def cobs_encode(data):
    out = bytearray([0])
    code_index, code = 0, 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
        else:
            out.append(byte)
            code += 1
            if code == 255:
                out[code_index] = code
                code_index = len(out)
                out.append(0)
                code = 1
    out[code_index] = code
    return bytes(out)


def _wire(kind, session, seq, payload):
    raw = struct.pack('<BBHQI', VERSION, kind, len(payload), session, seq) + payload
    raw += struct.pack('<I', crc32c(raw))
    if len(raw) > MAX_RAW:
        raise ValueError('record exceeds PROPOSED cap')
    return cobs_encode(raw) + b'\x00'


def perception_health(session, seq, *, estimate_seq, map_epoch,
                      transform_revision, mapping_revision, reset_counter,
                      tracking, estimator_id, source_domain, flags, frame_id,
                      child_frame_id, quality, reasons, source_ns,
                      mapped_mcu_ns, pi_send_ns, age_at_send_ns, uncertainty_ns):
    if not -1 <= quality <= 100:
        raise ValueError('quality must be -1, 0, or 1..100')
    payload = struct.pack('<IIIIBBBBHHHb x IQQQQI12x',
                          estimate_seq, map_epoch, transform_revision,
                          mapping_revision, reset_counter, tracking,
                          estimator_id, source_domain, flags, frame_id,
                          child_frame_id, quality, reasons, source_ns,
                          mapped_mcu_ns, pi_send_ns, age_at_send_ns, uncertainty_ns)
    return _wire(HEALTH, session, seq, payload)


def body_velocity_ref(session, seq, *, estimate_seq, map_epoch,
                      mapping_revision, frame_id, reset_counter,
                      pi_creation_ns, mapped_mcu_creation_ns, lease_us,
                      forward_m_s, yaw_rad_s):
    if not all(math.isfinite(v) for v in (forward_m_s, yaw_rad_s)):
        raise ValueError('nonfinite reference')
    payload = struct.pack('<IIIHBxQQIff', estimate_seq, map_epoch,
                          mapping_revision, frame_id, reset_counter,
                          pi_creation_ns, mapped_mcu_creation_ns, lease_us,
                          forward_m_s, yaw_rad_s)
    return _wire(BODY_REF, session, seq, payload)


def mcu_status(session, seq, *, sample_mcu_ns, last_health_seq,
               last_estimate_seq, last_ref_seq, faults, action_mcu_ns,
               battery_mv, fsm, armed, operator_enable, state, reason,
               battery_valid):
    payload = struct.pack('<QIIIIQHBBBBBB4x', sample_mcu_ns,
                          last_health_seq, last_estimate_seq, last_ref_seq,
                          faults, action_mcu_ns, battery_mv, fsm, armed,
                          operator_enable, state, reason, battery_valid)
    return _wire(MCU_STATUS, session, seq, payload)


def terrain_advisory(session, seq, *, pi_monotonic_ns, frame_stamp_ns, advisory,
                     confidence, distance_m, height_m, width_m, plane_valid,
                     plane_pitch_rad, plane_height_m):
    vals = (confidence, distance_m, height_m, width_m, plane_pitch_rad, plane_height_m)
    if int(advisory) not in range(4) or int(plane_valid) not in (0, 1):
        raise ValueError('invalid advisory or plane_valid')
    if not all(math.isfinite(v) for v in vals):
        raise ValueError('nonfinite advisory field')
    payload = struct.pack('<QQBBffffff', pi_monotonic_ns, frame_stamp_ns,
                          int(advisory), int(plane_valid), *vals)
    return _wire(TERRAIN_ADVISORY, session, seq, payload)


def advisory_echo(session, seq, *, sample_mcu_ns, advisory_seq, advisory,
                  result, accepted_count, rejected_count, stale, ms_since_accepted):
    if int(advisory) not in range(4) or int(result) not in range(11) or int(stale) not in (0, 1):
        raise ValueError('invalid advisory echo field')
    payload = struct.pack('<QIBB IIBI', sample_mcu_ns, advisory_seq, advisory,
                          result, accepted_count, rejected_count, stale,
                          ms_since_accepted)
    return _wire(ADVISORY_ECHO, session, seq, payload)


def unpack(packet):
    """Decode one complete wire packet into (header, payload dict)."""
    if not packet.endswith(b'\0'):
        raise ValueError('missing delimiter')
    raw = cobs_decode(packet[:-1])
    if len(raw) < 20:
        raise ValueError('short record')
    version, kind, size, session, seq = struct.unpack_from('<BBHQI', raw)
    if version != VERSION or len(raw) != 20 + size:
        raise ValueError('header/length')
    if struct.unpack_from('<I', raw, len(raw) - 4)[0] != crc32c(raw[:-4]):
        raise ValueError('CRC')
    p = raw[16:-4]
    if kind == TERRAIN_ADVISORY:
        if size != 42: raise ValueError('advisory length')
        stamp, frame, adv, valid, conf, dist, height, width, pitch, plane_h = struct.unpack('<QQBBffffff', p)
        if adv > 3 or valid > 1 or not all(math.isfinite(v) for v in (conf, dist, height, width, pitch, plane_h)):
            raise ValueError('invalid advisory fields')
        body = dict(pi_monotonic_ns=stamp, frame_stamp_ns=frame, advisory=adv,
                    plane_valid=valid, confidence=conf, distance_m=dist, height_m=height,
                    width_m=width, plane_pitch_rad=pitch, plane_height_m=plane_h)
    elif kind == ADVISORY_ECHO:
        if size != 27: raise ValueError('echo length')
        stamp, aseq, adv, result, accepted, rejected, stale, age = struct.unpack('<QIBBIIBI', p)
        if adv > 3 or result > 10 or stale > 1: raise ValueError('invalid echo fields')
        body = dict(sample_mcu_ns=stamp, advisory_seq=aseq, advisory=adv, result=result,
                    accepted_count=accepted, rejected_count=rejected, stale=stale,
                    ms_since_accepted=age)
    else:
        body = dict(payload=p)
    return dict(type=kind, session=session, sequence=seq), body


def cobs_decode(data):
    out, i = bytearray(), 0
    while i < len(data):
        code = data[i]; i += 1
        if code == 0 or i + code - 1 > len(data): raise ValueError('invalid COBS')
        out.extend(data[i:i + code - 1]); i += code - 1
        if code < 255 and i < len(data): out.append(0)
    return bytes(out)
