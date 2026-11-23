"""Minimal transport interface and in-memory loopback implementation for Pi-STM32 USB link."""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from .messages import (
    CmdTwist,
    decode_payload,
    encode_message_frame,
)
from .protocol import (
    StreamingDecoder,
    UsbLinkHeader,
    UsbLinkStatus,
    usb_link_encode,
)
from .tx_gate import Decision, TxRateGate


class Transport(ABC):
    """Abstract byte-stream transport interface (e.g., USB CDC ACM serial port or loopback)."""

    @abstractmethod
    def write(self, data: bytes | bytearray | memoryview) -> int:
        """Write raw wire bytes to the transport and return the number of bytes written."""

    @abstractmethod
    def read(self, max_bytes: int = 4096) -> bytes:
        """Read up to max_bytes of available raw wire bytes (non-blocking)."""

    @abstractmethod
    def close(self) -> None:
        """Close the transport."""

    @property
    @abstractmethod
    def is_open(self) -> bool:
        """Return True if transport is open."""


class LoopbackTransport(Transport):
    """In-memory loopback byte-stream transport.

    Can be used standalone (writes loop back into its own read buffer) or as a
    connected bidirectional pair via LoopbackTransport.create_pair() (Pi <-> STM32).
    """

    def __init__(
        self,
        rx_buf: bytearray | None = None,
        tx_buf: bytearray | None = None,
    ) -> None:
        if rx_buf is None and tx_buf is None:
            shared = bytearray()
            self._rx_buf = shared
            self._tx_buf = shared
        else:
            self._rx_buf = rx_buf if rx_buf is not None else bytearray()
            self._tx_buf = tx_buf if tx_buf is not None else self._rx_buf
        self._closed: bool = False

    @classmethod
    def create_pair(cls) -> tuple[LoopbackTransport, LoopbackTransport]:
        """Create a full-duplex pair (pi_transport, stm32_transport)."""
        pi_to_stm32 = bytearray()
        stm32_to_pi = bytearray()
        pi_side = cls(rx_buf=stm32_to_pi, tx_buf=pi_to_stm32)
        stm32_side = cls(rx_buf=pi_to_stm32, tx_buf=stm32_to_pi)
        return pi_side, stm32_side

    @property
    def is_open(self) -> bool:
        return not self._closed

    @property
    def available(self) -> int:
        return len(self._rx_buf)

    def write(self, data: bytes | bytearray | memoryview) -> int:
        if self._closed:
            raise RuntimeError("Cannot write to closed LoopbackTransport")
        mv = memoryview(data)
        self._tx_buf.extend(mv)
        return len(mv)

    def read(self, max_bytes: int = 4096) -> bytes:
        if self._closed:
            raise RuntimeError("Cannot read from closed LoopbackTransport")
        if max_bytes <= 0 or not self._rx_buf:
            return b""
        n = min(max_bytes, len(self._rx_buf))
        chunk = bytes(self._rx_buf[:n])
        del self._rx_buf[:n]
        return chunk

    def close(self) -> None:
        self._closed = True


class PiLinkEndpoint:
    """High-level endpoint combining a Transport, StreamingDecoder, and optional TxRateGate."""

    def __init__(
        self,
        transport: Transport,
        *,
        tx_gate: TxRateGate | None = None,
    ) -> None:
        self.transport: Transport = transport
        self.decoder: StreamingDecoder = StreamingDecoder()
        self.tx_gate: TxRateGate | None = tx_gate
        self.tx_seq: int = 0

    def next_seq(self) -> int:
        self.tx_seq = (self.tx_seq + 1) & 0xFFFFFFFF
        if self.tx_seq == 0:
            self.tx_seq = 1
        return self.tx_seq

    def send_frame(
        self,
        hdr: UsbLinkHeader,
        payload: bytes | bytearray | memoryview = b"",
    ) -> int:
        st, wire = usb_link_encode(hdr, payload)
        if st != UsbLinkStatus.OK:
            raise ValueError(f"Failed to encode frame: {st.name}")
        return self.transport.write(wire)

    def send_message(
        self,
        msg: Any,
        *,
        timestamp_us: int = 0,
        seq: int | None = None,
        flags: int = 0,
    ) -> int:
        s = self.next_seq() if seq is None else seq
        wire = encode_message_frame(msg, seq=s, timestamp_us=timestamp_us, flags=flags)
        return self.transport.write(wire)

    def send_twist_gated(
        self,
        cmd: CmdTwist,
        now_us: int,
        *,
        clock_error_us: int = 0,
        seq: int | None = None,
        flags: int = 0,
    ) -> Decision:
        if self.tx_gate is None:
            raise RuntimeError("No TxRateGate configured on PiLinkEndpoint")
        s = self.next_seq() if seq is None else seq
        decision, wire = self.tx_gate.encode_twist(
            cmd,
            s,
            now_us,
            clock_error_us=clock_error_us,
            flags=flags,
        )
        if decision.accepted and wire is not None:
            self.transport.write(wire)
        return decision

    def poll_raw_frames(
        self, max_bytes: int = 4096
    ) -> list[tuple[UsbLinkStatus, UsbLinkHeader | None, bytes | None]]:
        chunk = self.transport.read(max_bytes)
        if not chunk:
            return []
        return self.decoder.feed_bytes(chunk)

    def poll_messages(self, max_bytes: int = 4096) -> list[tuple[UsbLinkHeader, Any]]:
        decoded: list[tuple[UsbLinkHeader, Any]] = []
        for st, hdr, payload in self.poll_raw_frames(max_bytes):
            if st == UsbLinkStatus.OK and hdr is not None and payload is not None:
                msg = decode_payload(
                    hdr.msg_type,
                    payload,
                    header_timestamp_us=hdr.timestamp_us,
                )
                decoded.append((hdr, msg))
        return decoded
