"""Offline DAMIAO codec. SDK ranges are codec limits, never motor ratings.

Bit layout: https://files.seeedstudio.com/products/Damiao/DM-J4310-en.pdf
pp. 9-10. Arithmetic order follows DM-jump dm4310_drv.c float_to_uint.
"""
import math
import struct

FIELDS = ('Position', 'Velocity', 'Kp', 'Kd', 'Torque')
BITS = (16, 12, 12, 12, 12)
SPECIAL = {251: 'clear_error', 252: 'enable', 253: 'disable', 254: 'zero'}


def limits(profile):
    p, v, t = {'hip': (12.5, 30, 10), 'wheel': (12.5, 45, 10),
               'firmware_wheel': (12, 45, 18)}[profile]
    return ((-p, p), (-v, v), (0, 500), (0, 5), (-t, t))


def f32(x):
    return struct.unpack('f', struct.pack('f', x))[0]


def quantize(values, profile):
    """Clamp BEFORE original float32/truncation arithmetic. Reject nonfinite."""
    result = []
    if len(values) != 5 or not all(math.isfinite(x) for x in values):
        raise ValueError('five finite physical values required')
    for x, (lo, hi), bits in zip(values, limits(profile), BITS):
        x = f32(min(hi, max(lo, x)))
        result.append(int(f32(f32(f32(x-lo) * ((1 << bits)-1)) / (hi-lo))))
    return result


def pack_codes(q):
    p, v, kp, kd, t = q
    return bytes((p >> 8, p & 255, v >> 4, ((v & 15) << 4) | (kp >> 8),
                  kp & 255, kd >> 4, ((kd & 15) << 4) | (t >> 8), t & 255))


def command(values, profile):
    data = pack_codes(quantize(values, profile))
    if data[:7] == b'\xff'*7 and data[7] in SPECIAL:
        raise ValueError('normal MIT values collide with reserved lifecycle payload')
    return data


def feedback(q, motor_id, state=1, mos=30, rotor=31):
    p, v, _, _, t = q
    return bytes(((state << 4) | motor_id, p >> 8, p & 255, v >> 4,
                  ((v & 15) << 4) | (t >> 8), t & 255, mos, rotor))


def special(opcode):
    if opcode not in SPECIAL:
        raise ValueError('unsupported lifecycle opcode')
    return b'\xff'*7 + bytes((opcode,))


def frame_bits(identifier, payload):
    """Classical 11-bit data frame, CRC15 + stuffing + intermission.

    Bosch CAN 2.0 part B; same recurrence as S12 can_timing.bits.
    """
    if not 0 <= identifier <= 2047 or len(payload) > 8:
        raise ValueError('classical standard data frames only')
    serial = f'0{identifier:011b}000{len(payload):04b}'
    serial += ''.join(f'{b:08b}' for b in payload)
    crc = 0
    for bit in serial:
        carry = (crc >> 14) ^ int(bit)
        crc = ((crc << 1) & 0x7fff) ^ (0x4599 if carry else 0)
    last, count, stuffed = '', 0, 0
    for bit in serial + f'{crc:015b}':
        count = count+1 if bit == last else 1
        last = bit
        if count == 5:
            stuffed += 1
            last, count = str(1-int(bit)), 1
    return 47 + 8*len(payload) + stuffed


def worst_bits(dlc=8):
    if dlc not in range(9):
        raise ValueError('DLC 0..8 required')
    return 47 + 8*dlc + (34 + 8*dlc - 1)//4
