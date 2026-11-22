"""Bench-only byte helpers; no serial device access. All rates are ASSUMED."""
import math
import pathlib
import sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'link_proto/py'))
import link_proto as lp

IDS = (6, 8, 0, 1)  # SOURCE-SUPPORTED: README source table
SESSION = 0x5742524C494E4B31


def command(seq, motor_id, p=0, v=0, kp=0, kd=0, t=0, *, action='C'):
    if not 0 <= seq <= 0xFFFFFFFF or motor_id not in IDS or action not in ('C', 'E', 'D'):
        raise ValueError('invalid command header')
    head = f'{action},{seq},{motor_id}'
    if action == 'C':
        values = (p, v, kp, kd, t)
        if not all(math.isfinite(x) and abs(x) < 999999 for x in values):
            raise ValueError('decimal command magnitude')
        head += ''.join(f',{x:.6f}' for x in values)
    wire = (head + '\n').encode('ascii')
    if len(wire) >= 128:
        raise ValueError('command too long')
    return wire


def advisory(seq, value=0, **fields):
    args = dict(pi_monotonic_ns=0, frame_stamp_ns=0, advisory=value,
                plane_valid=0, confidence=0, distance_m=0, height_m=0,
                width_m=0, plane_pitch_rad=0, plane_height_m=0)
    args.update(fields)
    return b'\0' + lp.terrain_advisory(SESSION, seq, **args)


class Decoder:
    """Incremental MCU output -> ('line', str) / ('echo', (header, payload)).

    Binary bytes, including LF, are consumed only inside the zero envelope.
    A UART reader should have a single owner and distribute decoded records.
    """
    def __init__(self):
        self.binary = False
        self.buf = bytearray()
        self.errors = 0

    def feed(self, data):
        result = []
        for b in data:
            if self.binary:
                if b:
                    self.buf.append(b)
                else:
                    try:
                        result.append(('echo', lp.unpack(bytes(self.buf) + b'\0')))
                    except ValueError:
                        self.errors += 1
                    self.buf.clear()
                    self.binary = False
            elif b == 0:
                self.buf.clear()
                self.binary = True
            elif b == 10:
                if self.buf:
                    result.append(('line', self.buf.decode('ascii', errors='replace')))
                self.buf.clear()
            else:
                self.buf.append(b)
            if len(self.buf) > 1024:  # ASSUMED host recovery cap
                self.buf.clear()
                self.binary = False
                self.errors += 1
        return result


def bandwidth(command_hz=100, m_hz=20):
    """Conservative 460800 8N1 wire budget; M rate is per motor.

    STAT bound 1536 B; up to 20 echoes/s; 100 fresh IMU samples/s.
    """
    tx = 100 * 64 + min(command_hz, 4 * m_hz) * 48 + 20 * 51 + 1536
    rx = command_hz * 128 + 10 * 65
    return dict(tx_bytes_s=tx, rx_bytes_s=rx,
                tx_fraction=tx / 46080, rx_fraction=rx / 46080)
