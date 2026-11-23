"""TX rate gate and host command gate matching S12_comms_in_loop/host_gate.py limits."""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
import struct
import zlib

from .messages import CmdTwist
from .protocol import (
    UsbLinkStatus,
    UsbMsgType,
    encode_frame,
    usb_link_decode,
)


class Command(IntEnum):
    TWIST = 1
    ENABLE = 2
    JUMP = 3


PAYLOAD = struct.Struct("<2sBBQIQQIfffff")
CRC = struct.Struct("<I")
U32_MAX: int = (1 << 32) - 1
TTL_US: int = 100_000
FIRMWARE_TTL_US: int = 150_000
MIN_INTERVAL_US: int = 20_000
NOMINAL_INTERVAL_US: int = 40_000
MAX_RATE_HZ: float = 50.0
NOMINAL_RATE_HZ: float = 25.0
MAX_CLOCK_ERROR_US: int = 5_000
MAX_V: float = 0.20
MAX_V_WIRE: float = struct.unpack("<f", struct.pack("<f", MAX_V))[0]
MAX_YAW_RATE: float = 0.50
MAX_ACCEL: float = 0.40
MAX_YAW_ACCEL: float = 1.0
RELOC_DIST_BASE_M: float = 0.10
RELOC_DIST_SLOPE_M_S: float = 0.50
RELOC_TURN_BASE_RAD: float = 0.20
RELOC_TURN_SLOPE_RAD_S: float = 1.50


@dataclass(frozen=True)
class Packet:
    command: int
    session: int
    seq: int
    sent_us: int
    pose_us: int
    pose_seq: int
    v: float
    yaw_rate: float
    x: float
    y: float
    yaw: float

    def encode(self) -> bytes:
        payload = PAYLOAD.pack(
            b"HF",
            1,
            self.command,
            self.session,
            self.seq,
            self.sent_us,
            self.pose_us,
            self.pose_seq,
            self.v,
            self.yaw_rate,
            self.x,
            self.y,
            self.yaw,
        )
        return payload + CRC.pack(zlib.crc32(payload))

    @classmethod
    def from_cmd_twist(cls, cmd: CmdTwist, seq: int) -> Packet:
        return cls(
            command=int(Command.TWIST),
            session=cmd.session_id,
            seq=seq,
            sent_us=cmd.sent_us,
            pose_us=cmd.pose_us,
            pose_seq=cmd.pose_seq,
            v=cmd.v_cmd,
            yaw_rate=cmd.yaw_rate_cmd,
            x=cmd.pose_x,
            y=cmd.pose_y,
            yaw=cmd.pose_yaw,
        )


@dataclass(frozen=True)
class Decision:
    reason: str
    accepted: bool = False
    command_age_upper_us: int | None = None
    pose_age_upper_us: int | None = None


class TxRateGate:
    """Velocity channel TX/RX rate & safety gate matching S12_comms_in_loop/host_gate.py limits.

    Supports both:
      1. HostGate-compatible receive(frame, now_us, clock_error_us=...) for legacy HF frames
         and COBS+CRC-32C USB CMD_TWIST frames.
      2. Pi-side TX gating via check_twist(...) and encode_twist(...) enforcing the exact
         rate ceiling (MIN_INTERVAL_US = 20_000 -> <=50 Hz), TTL, clock uncertainty (<=5 ms),
         velocity envelope (0 <= v <= 0.20 m/s, |yaw_rate| <= 0.50 rad/s), monotonic sequence
         and acquisition order, pose relocalization guard, and slew-rate limiter (tick).
    """

    def __init__(self, *, local_only: bool = True, ttl_us: int = TTL_US) -> None:
        self._local_only: bool = local_only
        self.ttl_us: int = ttl_us
        self.session: int | None = None
        self._last_session: int = 0
        self.fault: str | None = None
        self._now_us: int = 0
        self._last_tick_us: int = 0
        self._last_rx_us: int | None = None
        self._packet: Packet | None = None
        self._error_us: int = 0
        self.target: tuple[float, float] = (0.0, 0.0)
        self.output: tuple[float, float] = (0.0, 0.0)

    def local_grant(
        self,
        session: int,
        now_us: int,
        *,
        balance_ready: bool,
        clock_error_us: int,
    ) -> bool:
        """Grant session authority after entering BALANCE."""
        self._advance(now_us)
        if (
            self._local_only
            or self.session is not None
            or self.fault
            or not balance_ready
            or not self._valid_clock(clock_error_us)
            or type(session) is not int
            or not (self._last_session < session < (1 << 64))
        ):
            return False
        self.session = self._last_session = session
        self._last_rx_us = self._last_tick_us = now_us
        return True

    def local_reset(self, *, supervisor_disarmed: bool) -> bool:
        """After supervisor manual reset, clear fault and require a fresh local grant."""
        if not supervisor_disarmed:
            return False
        self._clear()
        self.fault = None
        return True

    def local_revoke(self) -> None:
        self._clear()

    def _clear(self) -> None:
        self.session = self._last_rx_us = self._packet = None
        self.target = self.output = (0.0, 0.0)

    def _trip(self, reason: str) -> None:
        self._clear()
        self.fault = reason

    @staticmethod
    def _valid_clock(error_us: int) -> bool:
        return type(error_us) is int and 0 <= error_us <= MAX_CLOCK_ERROR_US

    def _advance(self, now_us: int) -> None:
        if type(now_us) is not int or now_us < self._now_us:
            if self.session is not None:
                self._trip("receiver_clock")
            raise ValueError("receiver clock must be monotonic integer microseconds")
        self._now_us = now_us

    def _watchdog(self, now_us: int, error_us: int) -> None:
        if self.session is None:
            return
        assert self._last_rx_us is not None
        if not self._valid_clock(error_us):
            self._trip("clock_unverified")
        elif now_us - self._last_rx_us > self.ttl_us:
            self._trip("host_timeout")
        elif self._packet is not None:
            conservative_error = max(self._error_us, error_us)
            if now_us - self._packet.sent_us + conservative_error > self.ttl_us:
                self._trip("command_timeout")
            elif now_us - self._packet.pose_us + conservative_error > self.ttl_us:
                self._trip("pose_timeout")

    def _validate_packet(
        self,
        p: Packet,
        now_us: int,
        clock_error_us: int,
        extra_floats: tuple[float, ...] = (),
    ) -> Decision:
        if p.command != Command.TWIST:
            return Decision("command_forbidden")
        if p.session != self.session:
            return Decision("session")
        old = self._packet
        if p.seq == 0 or (old is not None and p.seq <= old.seq):
            return Decision("sequence")
        command_age = now_us - p.sent_us
        pose_age = now_us - p.pose_us
        upper = (command_age + clock_error_us, pose_age + clock_error_us)

        def reject(reason: str) -> Decision:
            return Decision(reason, False, *upper)

        if min(command_age, pose_age) < -clock_error_us or p.pose_us > p.sent_us:
            return reject("future_stamp")
        if upper[0] > self.ttl_us:
            return reject("stale_command")
        assert self._last_rx_us is not None
        if old is not None and now_us - self._last_rx_us < MIN_INTERVAL_US:
            return reject("packet_rate")
        if not all(map(math.isfinite, (p.v, p.yaw_rate, p.x, p.y, p.yaw, *extra_floats))):
            return reject("nonfinite")
        if not (0 <= p.v <= MAX_V_WIRE and abs(p.yaw_rate) <= MAX_YAW_RATE):
            return reject("velocity_envelope")
        if p.pose_seq == 0 or (
            old is not None and (p.sent_us <= old.sent_us or p.pose_seq < old.pose_seq)
        ):
            return reject("acquisition_order")
        if upper[1] > self.ttl_us:
            self._trip("stale_pose")
            return reject("stale_pose")
        if old is not None:
            same_sample = p.pose_seq == old.pose_seq
            if same_sample and (p.pose_us, p.x, p.y, p.yaw) != (
                old.pose_us,
                old.x,
                old.y,
                old.yaw,
            ):
                return reject("pose_sample_changed")
            if not same_sample:
                dt = (p.pose_us - old.pose_us) / 1e6
                if dt <= 0:
                    return reject("acquisition_order")
                distance = math.hypot(p.x - old.x, p.y - old.y)
                turn = abs(math.remainder(p.yaw - old.yaw, math.tau))
                if (
                    distance > RELOC_DIST_BASE_M + RELOC_DIST_SLOPE_M_S * dt
                    or turn > RELOC_TURN_BASE_RAD + RELOC_TURN_SLOPE_RAD_S * dt
                ):
                    self._trip("relocalization")
                    return reject("relocalization")
        self._packet, self._last_rx_us, self._error_us = p, now_us, clock_error_us
        self.target = (min(p.v, MAX_V), p.yaw_rate)
        return Decision("accepted", True, *upper)

    def receive(
        self,
        frame: bytes | bytearray | memoryview | Packet | CmdTwist,
        now_us: int,
        *,
        clock_error_us: int,
        seq: int | None = None,
    ) -> Decision:
        """Validate a legacy HF frame, a COBS+CRC-32C USB frame, a Packet, or a CmdTwist."""
        self._advance(now_us)
        self._watchdog(now_us, clock_error_us)
        if self._local_only:
            return Decision("local_only")
        if self.session is None:
            return Decision(self.fault or "no_local_grant")

        if isinstance(frame, Packet):
            return self._validate_packet(frame, now_us, clock_error_us)

        if isinstance(frame, CmdTwist):
            pkt_seq = 1 if seq is None else seq
            pkt = Packet.from_cmd_twist(frame, pkt_seq)
            return self._validate_packet(
                pkt,
                now_us,
                clock_error_us,
                extra_floats=(frame.height_cmd, frame.roll_cmd),
            )

        raw_bytes = bytes(frame)

        # Case 1: COBS+CRC-32C USB protocol frame (ends with 0x00 delimiter or decodes cleanly)
        if len(raw_bytes) != PAYLOAD.size + CRC.size or (
            raw_bytes and raw_bytes[-1] == 0x00 and raw_bytes[:2] != b"HF"
        ):
            st, hdr, payload = usb_link_decode(raw_bytes)
            if st == UsbLinkStatus.OK and hdr is not None and payload is not None:
                if hdr.msg_type != int(UsbMsgType.CMD_TWIST):
                    return Decision("command_forbidden")
                try:
                    cmd = CmdTwist.unpack(payload, header_timestamp_us=hdr.timestamp_us)
                except ValueError:
                    return Decision("length")
                pkt = Packet.from_cmd_twist(cmd, hdr.seq)
                return self._validate_packet(
                    pkt,
                    now_us,
                    clock_error_us,
                    extra_floats=(cmd.height_cmd, cmd.roll_cmd),
                )

        # Case 2: Exact host_gate.py 60-byte HF frame
        if len(raw_bytes) != PAYLOAD.size + CRC.size:
            return Decision("length")
        payload = raw_bytes[: -CRC.size]
        if zlib.crc32(payload) != CRC.unpack(raw_bytes[-CRC.size :])[0]:
            return Decision("crc")
        magic, version, *values = PAYLOAD.unpack(payload)
        if (magic, version) != (b"HF", 1):
            return Decision("protocol")
        p = Packet(*values)
        return self._validate_packet(p, now_us, clock_error_us)

    def check_twist(
        self,
        cmd: CmdTwist,
        seq: int,
        now_us: int,
        *,
        clock_error_us: int = 0,
    ) -> Decision:
        """Check whether an outgoing Pi CMD_TWIST satisfies all host_gate limits."""
        return self.receive(cmd, now_us, clock_error_us=clock_error_us, seq=seq)

    def encode_twist(
        self,
        cmd: CmdTwist,
        seq: int,
        now_us: int,
        *,
        clock_error_us: int = 0,
        flags: int = 0,
    ) -> tuple[Decision, bytes | None]:
        """Gate an outgoing CMD_TWIST and return (Decision, encoded_usb_wire_frame_or_None)."""
        decision = self.check_twist(cmd, seq, now_us, clock_error_us=clock_error_us)
        if not decision.accepted:
            return decision, None
        wire = encode_frame(
            UsbMsgType.CMD_TWIST,
            cmd.pack(),
            seq=seq,
            timestamp_us=cmd.sent_us,
            flags=flags,
        )
        return decision, wire

    def tick(self, now_us: int, *, clock_error_us: int) -> tuple[float, float]:
        self._advance(now_us)
        self._watchdog(now_us, clock_error_us)
        # A missed loop cannot accumulate permission for a large reference step.
        dt = min(now_us - self._last_tick_us, 1_000) / 1e6
        self._last_tick_us = now_us
        self.output = tuple(
            current + max(-slope * dt, min(slope * dt, target - current))
            for current, target, slope in zip(
                self.output, self.target, (MAX_ACCEL, MAX_YAW_ACCEL)
            )
        )
        return self.output


HostGate = TxRateGate
