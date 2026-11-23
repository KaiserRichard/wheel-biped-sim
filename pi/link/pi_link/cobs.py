"""Consistent Overhead Byte Stuffing (COBS) codec matching c_ref/cobs.c."""
from __future__ import annotations

from enum import IntEnum


class CobsStatus(IntEnum):
    OK = 0
    ERR_NULL_PTR = 1
    ERR_BUFFER_TOO_SMALL = 2
    ERR_EMPTY_INPUT = 3
    ERR_ZERO_IN_INPUT = 4
    ERR_TRUNCATED = 5


class CobsError(ValueError):
    """Raised when COBS encoding or decoding fails."""

    def __init__(self, status: CobsStatus, message: str = "") -> None:
        self.status = status
        super().__init__(message or f"COBS error: {status.name} ({int(status)})")


def cobs_max_encoded_len(raw_len: int) -> int:
    """Tight upper bound on COBS-encoded length (excluding trailing 0x00 delimiter)."""
    if raw_len < 0:
        raise ValueError("raw_len must be non-negative")
    return 1 + raw_len + (raw_len // 254)


def cobs_encode_status(
    src: bytes | bytearray | memoryview,
    dst_cap: int | None = None,
) -> tuple[CobsStatus, bytes]:
    """Encode raw bytes into COBS form (no 0x00 bytes), returning (CobsStatus, encoded)."""
    mv = memoryview(src)
    src_len = len(mv)
    max_len = cobs_max_encoded_len(src_len)
    cap = max_len if dst_cap is None else dst_cap
    if cap <= 0:
        return CobsStatus.ERR_BUFFER_TOO_SMALL, b""

    dst = bytearray(cap)
    read_idx = 0
    write_idx = 1
    code_idx = 0
    code = 1

    while read_idx < src_len:
        b = mv[read_idx]
        if b == 0x00:
            dst[code_idx] = code
            code = 1
            if write_idx >= cap:
                return CobsStatus.ERR_BUFFER_TOO_SMALL, b""
            code_idx = write_idx
            write_idx += 1
            read_idx += 1
        else:
            if write_idx >= cap:
                return CobsStatus.ERR_BUFFER_TOO_SMALL, b""
            dst[write_idx] = b
            write_idx += 1
            read_idx += 1
            code += 1
            if code == 0xFF:
                dst[code_idx] = code
                code = 1
                if read_idx < src_len:
                    if write_idx >= cap:
                        return CobsStatus.ERR_BUFFER_TOO_SMALL, b""
                    code_idx = write_idx
                    write_idx += 1
                else:
                    code_idx = write_idx

    if code_idx < write_idx:
        dst[code_idx] = code

    return CobsStatus.OK, bytes(dst[:write_idx])


def cobs_decode_status(
    src: bytes | bytearray | memoryview,
    dst_cap: int | None = None,
) -> tuple[CobsStatus, bytes]:
    """Decode COBS-encoded bytes (must not contain 0x00) into raw bytes."""
    mv = memoryview(src)
    src_len = len(mv)
    if src_len == 0:
        return CobsStatus.ERR_EMPTY_INPUT, b""

    cap = src_len if dst_cap is None else dst_cap
    out = bytearray()
    read_idx = 0

    while read_idx < src_len:
        code = mv[read_idx]
        if code == 0x00:
            return CobsStatus.ERR_ZERO_IN_INPUT, b""
        read_idx += 1

        data_count = code - 1
        if data_count > (src_len - read_idx):
            return CobsStatus.ERR_TRUNCATED, b""
        if data_count > (cap - len(out)):
            return CobsStatus.ERR_BUFFER_TOO_SMALL, b""

        for i in range(data_count):
            b = mv[read_idx + i]
            if b == 0x00:
                return CobsStatus.ERR_ZERO_IN_INPUT, b""
            out.append(b)

        read_idx += data_count

        if code < 0xFF and read_idx < src_len:
            if len(out) >= cap:
                return CobsStatus.ERR_BUFFER_TOO_SMALL, b""
            out.append(0x00)

    return CobsStatus.OK, bytes(out)


def cobs_encode(src: bytes | bytearray | memoryview) -> bytes:
    """Encode raw bytes into COBS form (raises CobsError on failure)."""
    status, encoded = cobs_encode_status(src)
    if status != CobsStatus.OK:
        raise CobsError(status)
    return encoded


def cobs_decode(src: bytes | bytearray | memoryview) -> bytes:
    """Decode COBS-encoded bytes into raw bytes (raises CobsError on failure)."""
    status, decoded = cobs_decode_status(src)
    if status != CobsStatus.OK:
        raise CobsError(status)
    return decoded
