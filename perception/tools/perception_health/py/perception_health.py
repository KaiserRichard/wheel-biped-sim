"""Bit-exact optional UART v3 PERCEPTION_HEALTH framing and stream parsing."""
from __future__ import annotations
from dataclasses import dataclass
import struct

SOF = b"\xA5\x5A"
VERSION = 3
TYPE_HEARTBEAT = 0x01
TYPE_PERCEPTION_HEALTH = 0x05
MAX_PAYLOAD = 64
PAYLOAD = struct.Struct("<QIBBHHB")
HEARTBEAT = struct.Struct("<QQ")

TRACKING_STATES = {
    # ORB-SLAM3 include/Tracking.h @ 4452a3c; upstream -1 is u8 value 255.
    "SYSTEM_NOT_READY": 255, "NO_IMAGES_YET": 0, "NOT_INITIALIZED": 1,
    "OK": 2, "RECENTLY_LOST": 3, "LOST": 4, "OK_KLT": 5,
}
TRACKING_NAMES = {v: k for k, v in TRACKING_STATES.items()}
FLAG_AGE_VALID = 1 << 0
FLAG_MAPPING_VALID = 1 << 1
FLAG_DEGRADED = 1 << 2
U32_MAX = 0xFFFFFFFF

def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc

def encode_frame(msg_type: int, seq: int, payload: bytes) -> bytes:
    if not 0 <= msg_type <= 255 or not 0 <= seq <= 0xFFFF or len(payload) > MAX_PAYLOAD:
        raise ValueError("invalid v3 frame fields")
    body = bytes((VERSION, msg_type)) + struct.pack("<H", seq) + bytes((len(payload),)) + payload
    return SOF + body + struct.pack("<H", crc16_ccitt_false(body))

@dataclass(frozen=True)
class PerceptionHealth:
    linux_send_ns: int
    pose_age_us: int
    tracking_state: int
    map_reset_counter: int
    imu_initialized: int
    tracked_features: int
    flags: int

def encode_health(seq: int, value: PerceptionHealth) -> bytes:
    fields = (value.linux_send_ns, value.pose_age_us, value.tracking_state,
              value.imu_initialized, value.map_reset_counter, value.tracked_features, value.flags)
    limits = (0xFFFFFFFFFFFFFFFF, U32_MAX, 255, 1, 0xFFFF, 0xFFFF, 255)
    if any(not 0 <= v <= lim for v, lim in zip(fields, limits)):
        raise ValueError("health field out of range")
    return encode_frame(TYPE_PERCEPTION_HEALTH, seq, PAYLOAD.pack(
        value.linux_send_ns, value.pose_age_us, value.tracking_state, value.imu_initialized,
        value.map_reset_counter, value.tracked_features, value.flags))

def decode_health_payload(payload: bytes) -> PerceptionHealth:
    if len(payload) != PAYLOAD.size:
        raise ValueError(f"PERCEPTION_HEALTH payload must be {PAYLOAD.size} bytes")
    send, age, state, imu, reset, features, flags = PAYLOAD.unpack(payload)
    if imu not in (0, 1):
        raise ValueError("imu_initialized must be encoded as 0 or 1")
    return PerceptionHealth(send, age, state, reset, imu, features, flags)

def encode_heartbeat(seq: int, send_ns: int, session_id: int) -> bytes:
    if not (0 <= send_ns <= 0xFFFFFFFFFFFFFFFF and 0 <= session_id <= 0xFFFFFFFFFFFFFFFF):
        raise ValueError("heartbeat field out of range")
    return encode_frame(TYPE_HEARTBEAT, seq, HEARTBEAT.pack(send_ns, session_id))

class StreamParser:
    """Byte-at-a-time v3 frame parser. As in upstream, bad candidates aren't rescanned."""
    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = self.len_errors = self.bytes_dropped = 0

    def feed(self, data: bytes) -> list[tuple[int, int, bytes]]:
        out = []
        for byte in data:
            if not self.buf:
                if byte == SOF[0]: self.buf.append(byte)
                else: self.bytes_dropped += 1
                continue
            if len(self.buf) == 1:
                if byte == SOF[1]: self.buf.append(byte)
                elif byte == SOF[0]: self.bytes_dropped += 1
                else: self.bytes_dropped += 2; self.buf.clear()
                continue
            self.buf.append(byte)
            if len(self.buf) == 7 and self.buf[6] > MAX_PAYLOAD:
                self.len_errors += 1; self.bytes_dropped += len(self.buf); self.buf.clear(); continue
            if len(self.buf) >= 7:
                expected = 9 + self.buf[6]
                if len(self.buf) == expected:
                    body = self.buf[2:-2]
                    got = int.from_bytes(self.buf[-2:], "little")
                    if crc16_ccitt_false(body) == got:
                        version, typ = self.buf[2], self.buf[3]
                        seq = int.from_bytes(self.buf[4:6], "little")
                        if version in (1, 2, VERSION): out.append((typ, seq, bytes(self.buf[7:-2])))
                        else: self.bytes_dropped += len(self.buf)
                    else:
                        self.crc_errors += 1; self.bytes_dropped += len(self.buf)
                    self.buf.clear()
        return out

def saturating_age_us(send_ns: int, source_ns: int) -> int:
    if send_ns < source_ns: raise ValueError("Pi monotonic clock regression")
    return min(U32_MAX, (send_ns - source_ns + 999) // 1000)
