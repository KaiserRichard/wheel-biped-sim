"""Castagnoli CRC-32C implementation matching c_ref/crc32c.c.

Parameters:
  - Normal polynomial:    0x1EDC6F41
  - Reflected polynomial: 0x82F63B78
  - Initial value:        0xFFFFFFFF
  - Final XOR:            0xFFFFFFFF
  - Check("123456789"):   0xE3069283
"""
from __future__ import annotations

CRC32C_POLY_NORMAL: int = 0x1EDC6F41
CRC32C_POLY_REFLECTED: int = 0x82F63B78
CRC32C_INIT: int = 0xFFFFFFFF
CRC32C_XOROUT: int = 0xFFFFFFFF
CRC32C_KNOWN_123456789: int = 0xE3069283


def _build_crc32c_table() -> tuple[int, ...]:
    table: list[int] = []
    for byte in range(256):
        crc = byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ CRC32C_POLY_REFLECTED
            else:
                crc >>= 1
        table.append(crc & 0xFFFFFFFF)
    return tuple(table)


_CRC32C_LUT: tuple[int, ...] = _build_crc32c_table()


def crc32c_init() -> int:
    """Return initial CRC-32C accumulator state (0xFFFFFFFF)."""
    return CRC32C_INIT


def crc32c_update(crc: int, data: bytes | bytearray | memoryview) -> int:
    """Update running CRC-32C accumulator over a byte sequence."""
    acc = crc & 0xFFFFFFFF
    lut = _CRC32C_LUT
    for b in memoryview(data):
        acc = (acc >> 8) ^ lut[(acc ^ b) & 0xFF]
    return acc & 0xFFFFFFFF


def crc32c_final(crc: int) -> int:
    """Finalize CRC-32C accumulator (XOR with 0xFFFFFFFF)."""
    return (crc ^ CRC32C_XOROUT) & 0xFFFFFFFF


def crc32c_compute(data: bytes | bytearray | memoryview) -> int:
    """Compute one-shot Castagnoli CRC-32C over data."""
    return crc32c_final(crc32c_update(crc32c_init(), data))


crc32c = crc32c_compute
