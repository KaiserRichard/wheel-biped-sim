"""18-byte USB link wire header, versioning, frame codec, and streaming decoder.

Matches c_ref/usb_link.h, c_ref/usb_link.c, and FIRMWARE_DESIGN_v2.md Section 7.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import struct

from .cobs import CobsStatus, cobs_decode_status, cobs_encode_status
from .crc32c import crc32c_compute

USB_LINK_VER_MAJOR: int = 1
USB_LINK_VER_MINOR: int = 0
USB_LINK_HEADER_SIZE: int = 18
USB_LINK_CRC_SIZE: int = 4
USB_LINK_MAX_PAYLOAD: int = 256
USB_LINK_MAX_RAW_FRAME: int = USB_LINK_HEADER_SIZE + USB_LINK_MAX_PAYLOAD + USB_LINK_CRC_SIZE
USB_LINK_MAX_WIRE_FRAME: int = 1 + USB_LINK_MAX_RAW_FRAME + (USB_LINK_MAX_RAW_FRAME // 254) + 1
USB_LINK_DELIMITER: int = 0x00

USB_LINK_FLAG_ACK_REQ: int = 1 << 0
USB_LINK_FLAG_BENCH_MODE: int = 1 << 1

HEADER_STRUCT = struct.Struct("<BBBBIQH")
CRC_STRUCT = struct.Struct("<I")


class UsbMsgType(IntEnum):
    CMD_TWIST = 0x01
    CMD_MODE = 0x02
    TIME_SYNC_REQ = 0x03
    PARAM_OP = 0x04
    TELEM_FAST = 0x81
    TELEM_HEALTH = 0x82
    TIME_SYNC_RESP = 0x83
    FAULT_SNAPSHOT = 0x84
    BENCH_RAW_LOG = 0x85


class UsbLinkStatus(IntEnum):
    OK = 0
    NEED_MORE = 1
    ERR_NULL_PTR = 2
    ERR_PAYLOAD_TOO_LARGE = 3
    ERR_BUFFER_TOO_SMALL = 4
    ERR_COBS = 5
    ERR_LENGTH = 6
    ERR_CRC = 7
    ERR_VERSION = 8
    ERR_OVERFLOW = 9


class UsbLinkError(ValueError):
    """Exception raised when frame encoding or decoding fails."""

    def __init__(self, status: UsbLinkStatus, message: str = "") -> None:
        self.status = status
        super().__init__(message or f"UsbLink error: {status.name} ({int(status)})")


@dataclass(frozen=True)
class UsbLinkHeader:
    """18-byte little-endian wire header from FIRMWARE_DESIGN_v2.md Section 7.2."""

    msg_type: int
    seq: int = 0
    timestamp_us: int = 0
    payload_len: int = 0
    flags: int = 0
    ver_major: int = USB_LINK_VER_MAJOR
    ver_minor: int = USB_LINK_VER_MINOR

    def pack(self) -> bytes:
        if not (0 <= self.ver_major <= 0xFF and 0 <= self.ver_minor <= 0xFF):
            raise ValueError("ver_major and ver_minor must fit in uint8")
        if not (0 <= int(self.msg_type) <= 0xFF and 0 <= self.flags <= 0xFF):
            raise ValueError("msg_type and flags must fit in uint8")
        if not (0 <= self.seq <= 0xFFFFFFFF):
            raise ValueError("seq must fit in uint32")
        if not (0 <= self.timestamp_us <= 0xFFFFFFFFFFFFFFFF):
            raise ValueError("timestamp_us must fit in uint64")
        if not (0 <= self.payload_len <= 0xFFFF):
            raise ValueError("payload_len must fit in uint16")
        return HEADER_STRUCT.pack(
            self.ver_major,
            self.ver_minor,
            int(self.msg_type),
            self.flags,
            self.seq,
            self.timestamp_us,
            self.payload_len,
        )

    @classmethod
    def unpack(cls, raw: bytes | bytearray | memoryview) -> UsbLinkHeader:
        if len(raw) < USB_LINK_HEADER_SIZE:
            raise UsbLinkError(UsbLinkStatus.ERR_LENGTH, "buffer shorter than 18-byte header")
        ver_major, ver_minor, msg_type, flags, seq, timestamp_us, payload_len = (
            HEADER_STRUCT.unpack_from(raw, 0)
        )
        return cls(
            ver_major=ver_major,
            ver_minor=ver_minor,
            msg_type=msg_type,
            flags=flags,
            seq=seq,
            timestamp_us=timestamp_us,
            payload_len=payload_len,
        )


def usb_link_encode(
    hdr: UsbLinkHeader,
    payload: bytes | bytearray | memoryview = b"",
    out_cap: int = USB_LINK_MAX_WIRE_FRAME,
) -> tuple[UsbLinkStatus, bytes]:
    """Encode 18-byte header + payload + CRC-32C -> COBS + trailing 0x00 delimiter."""
    if hdr is None:
        return UsbLinkStatus.ERR_NULL_PTR, b""
    if hdr.payload_len > USB_LINK_MAX_PAYLOAD:
        return UsbLinkStatus.ERR_PAYLOAD_TOO_LARGE, b""
    if payload is None:
        if hdr.payload_len > 0:
            return UsbLinkStatus.ERR_NULL_PTR, b""
        payload_bytes = b""
    else:
        payload_bytes = bytes(payload)
        if len(payload_bytes) < hdr.payload_len:
            return UsbLinkStatus.ERR_LENGTH, b""
        payload_bytes = payload_bytes[: hdr.payload_len]

    if out_cap < 2:
        return UsbLinkStatus.ERR_BUFFER_TOO_SMALL, b""

    raw_body = hdr.pack() + payload_bytes
    crc = crc32c_compute(raw_body)
    raw_frame = raw_body + CRC_STRUCT.pack(crc)

    cs, cobs_bytes = cobs_encode_status(raw_frame, dst_cap=out_cap - 1)
    if cs == CobsStatus.ERR_BUFFER_TOO_SMALL:
        return UsbLinkStatus.ERR_BUFFER_TOO_SMALL, b""
    if cs != CobsStatus.OK:
        return UsbLinkStatus.ERR_COBS, b""

    return UsbLinkStatus.OK, cobs_bytes + b"\x00"


def usb_link_decode(
    wire_buf: bytes | bytearray | memoryview,
    out_payload_cap: int = USB_LINK_MAX_PAYLOAD,
) -> tuple[UsbLinkStatus, UsbLinkHeader | None, bytes | None]:
    """Decode one COBS-encoded frame (with or without trailing 0x00 delimiter)."""
    if wire_buf is None:
        return UsbLinkStatus.ERR_NULL_PTR, None, None

    mv = memoryview(wire_buf)
    cobs_len = len(mv)
    if cobs_len > 0 and mv[cobs_len - 1] == USB_LINK_DELIMITER:
        cobs_len -= 1
    if cobs_len == 0:
        return UsbLinkStatus.ERR_COBS, None, None

    cs, raw = cobs_decode_status(mv[:cobs_len], dst_cap=USB_LINK_MAX_RAW_FRAME)
    if cs == CobsStatus.ERR_BUFFER_TOO_SMALL:
        return UsbLinkStatus.ERR_OVERFLOW, None, None
    if cs != CobsStatus.OK:
        return UsbLinkStatus.ERR_COBS, None, None

    raw_len = len(raw)
    if raw_len < (USB_LINK_HEADER_SIZE + USB_LINK_CRC_SIZE):
        return UsbLinkStatus.ERR_LENGTH, None, None

    body_len = raw_len - USB_LINK_CRC_SIZE
    expected_crc = CRC_STRUCT.unpack_from(raw, body_len)[0]
    actual_crc = crc32c_compute(memoryview(raw)[:body_len])
    if actual_crc != expected_crc:
        return UsbLinkStatus.ERR_CRC, None, None

    hdr = UsbLinkHeader.unpack(raw)
    if hdr.ver_major != USB_LINK_VER_MAJOR:
        return UsbLinkStatus.ERR_VERSION, None, None

    avail_payload = body_len - USB_LINK_HEADER_SIZE
    if hdr.ver_minor == USB_LINK_VER_MINOR:
        if hdr.payload_len != avail_payload:
            return UsbLinkStatus.ERR_LENGTH, None, None
    else:
        # Backward-compatible minor extension: ignore extra trailing bytes
        if hdr.payload_len > avail_payload:
            return UsbLinkStatus.ERR_LENGTH, None, None

    if hdr.payload_len > out_payload_cap:
        return UsbLinkStatus.ERR_BUFFER_TOO_SMALL, None, None

    payload = raw[USB_LINK_HEADER_SIZE : USB_LINK_HEADER_SIZE + hdr.payload_len]
    return UsbLinkStatus.OK, hdr, payload


def encode_frame(
    msg_type: int,
    payload: bytes | bytearray | memoryview = b"",
    *,
    seq: int = 0,
    timestamp_us: int = 0,
    flags: int = 0,
    ver_major: int = USB_LINK_VER_MAJOR,
    ver_minor: int = USB_LINK_VER_MINOR,
) -> bytes:
    """Convenience function that builds UsbLinkHeader and encodes a wire frame."""
    payload_bytes = bytes(payload)
    hdr = UsbLinkHeader(
        ver_major=ver_major,
        ver_minor=ver_minor,
        msg_type=int(msg_type),
        flags=flags,
        seq=seq,
        timestamp_us=timestamp_us,
        payload_len=len(payload_bytes),
    )
    status, wire = usb_link_encode(hdr, payload_bytes)
    if status != UsbLinkStatus.OK:
        raise UsbLinkError(status)
    return wire


def decode_frame(
    wire_buf: bytes | bytearray | memoryview,
) -> tuple[UsbLinkHeader, bytes]:
    """Convenience function that decodes a wire frame or raises UsbLinkError."""
    status, hdr, payload = usb_link_decode(wire_buf)
    if status != UsbLinkStatus.OK or hdr is None or payload is None:
        raise UsbLinkError(status)
    return hdr, payload


class StreamingDecoder:
    """Streaming byte-by-byte COBS + CRC-32C receiver matching UsbLinkRxParser in c_ref/usb_link.c."""

    def __init__(self, payload_cap: int = USB_LINK_MAX_PAYLOAD) -> None:
        self._buf = bytearray(USB_LINK_MAX_WIRE_FRAME)
        self._payload_cap = payload_cap
        self.len: int = 0
        self.overflowed: bool = False
        self.frames_ok: int = 0
        self.crc_errors: int = 0
        self.proto_errors: int = 0
        self.overflow_errors: int = 0

    def reset(self) -> None:
        """Reset parser buffer and all counters (equivalent to usb_link_rx_init)."""
        self.len = 0
        self.overflowed = False
        self.frames_ok = 0
        self.crc_errors = 0
        self.proto_errors = 0
        self.overflow_errors = 0

    def feed_byte(
        self, byte: int
    ) -> tuple[UsbLinkStatus, UsbLinkHeader | None, bytes | None]:
        """Feed one byte into the streaming parser (equivalent to usb_link_rx_feed)."""
        b = byte & 0xFF
        if b == USB_LINK_DELIMITER:
            if self.overflowed:
                self.overflowed = False
                self.len = 0
                self.overflow_errors += 1
                return UsbLinkStatus.ERR_OVERFLOW, None, None
            if self.len == 0:
                # Leading or back-to-back delimiter: flush line noise
                return UsbLinkStatus.NEED_MORE, None, None
            st, hdr, payload = usb_link_decode(
                memoryview(self._buf)[: self.len],
                out_payload_cap=self._payload_cap,
            )
            self.len = 0
            if st == UsbLinkStatus.OK:
                self.frames_ok += 1
            elif st == UsbLinkStatus.ERR_CRC:
                self.crc_errors += 1
            else:
                self.proto_errors += 1
            return st, hdr, payload

        if self.overflowed:
            return UsbLinkStatus.NEED_MORE, None, None

        if self.len >= (USB_LINK_MAX_WIRE_FRAME - 1):
            self.overflowed = True
            return UsbLinkStatus.NEED_MORE, None, None

        self._buf[self.len] = b
        self.len += 1
        return UsbLinkStatus.NEED_MORE, None, None

    def feed_bytes(
        self, data: bytes | bytearray | memoryview
    ) -> list[tuple[UsbLinkStatus, UsbLinkHeader | None, bytes | None]]:
        """Feed a chunk of bytes and return all terminal frame events (OK or ERR_*)."""
        events: list[tuple[UsbLinkStatus, UsbLinkHeader | None, bytes | None]] = []
        for b in memoryview(data):
            st, hdr, payload = self.feed_byte(int(b))
            if st != UsbLinkStatus.NEED_MORE:
                events.append((st, hdr, payload))
        return events


UsbLinkRxParser = StreamingDecoder
