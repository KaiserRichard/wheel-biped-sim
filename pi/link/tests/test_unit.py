"""Pure-Python unit tests for pi_link using pytest-free plain asserts."""
from __future__ import annotations

import math
import struct

from pi_link import (
    CRC32C_KNOWN_123456789,
    CobsError,
    CobsStatus,
    Command,
    CmdMode,
    CmdModeAction,
    CmdTwist,
    Direction,
    FaultSnapshot,
    FIRMWARE_TTL_US,
    FlightRecord,
    FlightRingBuffer,
    HostGate,
    LOG_BENCH_RAW_SIZE,
    LOG_FLIGHT_RECORD_SIZE,
    LOG_TELEM_FAST_SIZE,
    LOG_TELEM_HEALTH_SIZE,
    LoopbackTransport,
    MAX_ACCEL,
    MAX_CLOCK_ERROR_US,
    MAX_V,
    MAX_V_WIRE,
    MAX_YAW_ACCEL,
    MAX_YAW_RATE,
    MESSAGE_CATALOG,
    MIN_INTERVAL_US,
    MSG_CMD_MODE_SIZE,
    MSG_CMD_TWIST_SHORT_SIZE,
    MSG_CMD_TWIST_SIZE,
    MSG_FAULT_SNAPSHOT_SIZE,
    MSG_PARAM_OP_SIZE,
    MSG_TIME_SYNC_REQ_SIZE,
    MSG_TIME_SYNC_RESP_SIZE,
    Packet,
    ParamOp,
    ParamOpCode,
    PiLinkEndpoint,
    StreamingDecoder,
    TTL_US,
    TelemFast,
    TelemHealth,
    TimeSyncReq,
    TimeSyncResp,
    TxRateGate,
    USB_LINK_CRC_SIZE,
    USB_LINK_FLAG_ACK_REQ,
    USB_LINK_FLAG_BENCH_MODE,
    USB_LINK_HEADER_SIZE,
    USB_LINK_MAX_PAYLOAD,
    USB_LINK_MAX_RAW_FRAME,
    USB_LINK_MAX_WIRE_FRAME,
    USB_LINK_VER_MAJOR,
    USB_LINK_VER_MINOR,
    UsbLinkError,
    UsbLinkHeader,
    UsbLinkStatus,
    UsbMsgType,
    BenchRawLog,
    cobs_decode,
    cobs_decode_status,
    cobs_encode,
    cobs_encode_status,
    cobs_max_encoded_len,
    crc32c_compute,
    crc32c_final,
    crc32c_init,
    crc32c_update,
    decode_bench_raw,
    decode_flight_dump,
    decode_flight_record,
    decode_frame,
    decode_message_frame,
    decode_payload,
    decode_telem_fast,
    decode_telem_health,
    encode_frame,
    encode_message_frame,
    frame_bench_raw,
    frame_telem_fast,
    frame_telem_health,
    pack_bench_raw,
    pack_flight_record,
    pack_telem_fast,
    pack_telem_health,
    usb_link_decode,
    usb_link_encode,
)


def make_sample_telem_fast() -> TelemFast:
    return TelemFast(
        fsm_state=3,
        s11_mode=1,
        s11_max_misses=2,
        flags=0x05,
        fault_mask=0x00000000,
        euler_rad=(0.015, -0.008, 1.234),
        gyro_rad_s=(0.02, -0.11, 0.35),
        v_est_m_s=0.185,
        x_est_m=1.42,
        leg_length_m=(0.142, 0.141),
        leg_theta_rad=(0.031, 0.029),
        motor_pos_rad=(0.51, -0.48, 12.3, -0.52, 0.49, 12.1),
        motor_vel_rad_s=(0.12, -0.15, 3.4, -0.11, 0.14, 3.35),
        motor_tor_cmd_nm=(1.8, -1.7, 0.45, -1.85, 1.75, 0.44),
        motor_tor_fb_nm=(1.79, -1.68, 0.43, -1.82, 1.74, 0.42),
        enc_age_us=(310, 340, 290, 315, 345, 295),
        s9_trim_rad=0.0125,
        gyro_bias_y_rad_s=-0.0018,
        v_bus_mv=24150,
        i_bus_ma=-3420,
    )


def make_sample_telem_health() -> TelemHealth:
    return TelemHealth(
        can_tec=(0, 1, 0),
        can_rec=(0, 0, 2),
        can_busoff_cnt=(0, 0, 0),
        can_rx_drop=(1, 0, 3),
        motor_state=(1, 1, 1, 1, 1, 1),
        t_mos_c=(34, 35, 38, 34, 36, 39),
        t_coil_c=(41, 42, 47, 40, 43, 48),
        rc_rssi_dbm=-62,
        rc_lq_pct=98,
        rc_age_ms=4,
        rc_switches=0x0003,
        power_io_bits=0x003F,
        reset_cause=0x01,
        cpu_load_pct=17,
        min_stack_words=(310, 640, 220, 480, 250),
    )


def make_sample_bench_raw() -> BenchRawLog:
    return BenchRawLog(
        imu_ts_us=9876543210,
        gyro_rad_s=(0.01, -0.02, 0.03),
        accel_m_s2=(-0.15, 0.08, 9.79),
        imu_temp_c=36.5,
        motor_pos_rad=(0.1, 0.2, 0.3, -0.1, -0.2, -0.3),
        motor_vel_rad_s=(1.1, 1.2, 1.3, -1.1, -1.2, -1.3),
        motor_rx_us=(1000, 1010, 1020, 1005, 1015, 1025),
        fresh_mask=0x3F,
        reserved=(0, 0, 0),
        pitch_base_rad=0.014,
        v_base_m_s=0.18,
        pitch_s10_rad=0.015,
        v_s10_m_s=0.182,
    )


def assert_float_tuple_close(a: tuple[float, ...], b: tuple[float, ...], tol: float = 1e-6) -> None:
    assert len(a) == len(b), f"Length mismatch: {len(a)} vs {len(b)}"
    for idx, (x, y) in enumerate(zip(a, b)):
        assert math.isclose(x, y, rel_tol=tol, abs_tol=tol), f"Index {idx}: {x} != {y}"


def test_crc32c_known_vectors() -> None:
    # Standard check value for ASCII "123456789"
    vec = b"123456789"
    crc = crc32c_compute(vec)
    assert crc == 0xE3069283, f"Expected 0xE3069283, got 0x{crc:08X}"
    assert crc == CRC32C_KNOWN_123456789

    # Empty vector -> 0x00000000
    assert crc32c_compute(b"") == 0x00000000

    # 32 zero bytes and 32 0xFF bytes (RFC 3720 / iSCSI known Castagnoli vectors)
    assert crc32c_compute(b"\x00" * 32) == 0x8A9136AA
    assert crc32c_compute(b"\xFF" * 32) == 0x62A8AB43
    assert crc32c_compute(bytes(range(32))) == 0x46DD794E

    # Streaming / chunked update equivalence
    acc = crc32c_init()
    acc = crc32c_update(acc, b"1234")
    acc = crc32c_update(acc, b"")
    acc = crc32c_update(acc, b"56789")
    assert crc32c_final(acc) == 0xE3069283


def test_cobs_vectors_and_edge_cases() -> None:
    # Known COBS vectors
    assert cobs_encode(b"") == b"\x01"
    assert cobs_decode(b"\x01") == b""

    assert cobs_encode(b"\x00") == b"\x01\x01"
    assert cobs_decode(b"\x01\x01") == b"\x00"

    assert cobs_encode(b"\x00\x00") == b"\x01\x01\x01"
    assert cobs_decode(b"\x01\x01\x01") == b"\x00\x00"

    assert cobs_encode(b"\x11\x22\x00\x33") == b"\x03\x11\x22\x02\x33"
    assert cobs_decode(b"\x03\x11\x22\x02\x33") == b"\x11\x22\x00\x33"

    assert cobs_encode(b"\x11\x22\x33\x44") == b"\x05\x11\x22\x33\x44"
    assert cobs_decode(b"\x05\x11\x22\x33\x44") == b"\x11\x22\x33\x44"

    # 254 non-zero bytes -> single 0xFF code block of total length 255
    b254 = bytes(range(1, 255))
    e254 = cobs_encode(b254)
    assert len(e254) == 255
    assert e254[0] == 0xFF
    assert 0x00 not in e254
    assert cobs_decode(e254) == b254

    # 255 non-zero bytes -> 0xFF + 254 bytes + 0x02 + 1 byte (length 257)
    b255 = bytes(range(1, 255)) + b"\xAA"
    e255 = cobs_encode(b255)
    assert len(e255) == 257
    assert e255[0] == 0xFF
    assert e255[255] == 0x02
    assert e255[256] == 0xAA
    assert 0x00 not in e255
    assert cobs_decode(e255) == b255

    # 254 non-zero bytes followed by 0x00 -> 0xFF + 254 bytes + 0x01 + 0x01
    b254_z = bytes(range(1, 255)) + b"\x00"
    e254_z = cobs_encode(b254_z)
    assert cobs_decode(e254_z) == b254_z

    # Error checks
    st, _ = cobs_decode_status(b"")
    assert st == CobsStatus.ERR_EMPTY_INPUT

    st, _ = cobs_decode_status(b"\x00\x01")
    assert st == CobsStatus.ERR_ZERO_IN_INPUT

    st, _ = cobs_decode_status(b"\x03\x11\x00")
    assert st == CobsStatus.ERR_ZERO_IN_INPUT

    st, _ = cobs_decode_status(b"\x05\x11\x22")
    assert st == CobsStatus.ERR_TRUNCATED

    st, _ = cobs_encode_status(b"hello", dst_cap=3)
    assert st == CobsStatus.ERR_BUFFER_TOO_SMALL

    st, _ = cobs_decode_status(b"\x05\x11\x22\x33\x44", dst_cap=2)
    assert st == CobsStatus.ERR_BUFFER_TOO_SMALL

    assert cobs_max_encoded_len(0) == 1
    assert cobs_max_encoded_len(254) == 256
    assert cobs_max_encoded_len(278) == 280


def test_header_and_versioning() -> None:
    assert USB_LINK_HEADER_SIZE == 18
    assert USB_LINK_CRC_SIZE == 4
    assert USB_LINK_MAX_PAYLOAD == 256
    assert USB_LINK_MAX_RAW_FRAME == 278
    assert USB_LINK_MAX_WIRE_FRAME == 281

    hdr = UsbLinkHeader(
        ver_major=USB_LINK_VER_MAJOR,
        ver_minor=USB_LINK_VER_MINOR,
        msg_type=UsbMsgType.CMD_MODE,
        flags=USB_LINK_FLAG_ACK_REQ | USB_LINK_FLAG_BENCH_MODE,
        seq=0x12345678,
        timestamp_us=0x0102030405060708,
        payload_len=13,
    )
    packed = hdr.pack()
    assert len(packed) == 18
    unpacked = UsbLinkHeader.unpack(packed)
    assert unpacked == hdr

    # Incompatible ver_major != 1 must be rejected with ERR_VERSION
    bad_major_wire = encode_frame(
        UsbMsgType.TIME_SYNC_REQ,
        b"\x01" * 8,
        seq=1,
        timestamp_us=100,
        ver_major=2,
        ver_minor=0,
    )
    st, _, _ = usb_link_decode(bad_major_wire)
    assert st == UsbLinkStatus.ERR_VERSION

    # Backward-compatible minor version extension (ver_minor > 0):
    # Extra trailing bytes beyond hdr.payload_len must be ignored cleanly.
    ext_hdr = UsbLinkHeader(
        ver_major=1,
        ver_minor=2,
        msg_type=UsbMsgType.TIME_SYNC_REQ,
        flags=0,
        seq=7,
        timestamp_us=5555,
        payload_len=8,
    )
    core_payload = struct.pack("<Q", 123456789)
    extra_extension = b"\xAA\xBB\xCC\xDD"
    raw_body = ext_hdr.pack() + core_payload + extra_extension
    raw_frame = raw_body + struct.pack("<I", crc32c_compute(raw_body))
    wire_ext = cobs_encode(raw_frame) + b"\x00"

    st, dec_hdr, dec_payload = usb_link_decode(wire_ext)
    assert st == UsbLinkStatus.OK
    assert dec_hdr == ext_hdr
    assert dec_payload == core_payload

    # But if ver_minor == 0, extra trailing bytes must be rejected with ERR_LENGTH
    strict_hdr = UsbLinkHeader(
        ver_major=1,
        ver_minor=0,
        msg_type=UsbMsgType.TIME_SYNC_REQ,
        flags=0,
        seq=7,
        timestamp_us=5555,
        payload_len=8,
    )
    raw_body_strict = strict_hdr.pack() + core_payload + extra_extension
    raw_frame_strict = raw_body_strict + struct.pack("<I", crc32c_compute(raw_body_strict))
    wire_strict = cobs_encode(raw_frame_strict) + b"\x00"
    st_strict, _, _ = usb_link_decode(wire_strict)
    assert st_strict == UsbLinkStatus.ERR_LENGTH


def test_catalog_and_all_message_roundtrips() -> None:
    # Verify all 9 message types from FIRMWARE_DESIGN_v2.md Section 7.3 are in catalog
    expected_types = {
        UsbMsgType.CMD_TWIST,
        UsbMsgType.CMD_MODE,
        UsbMsgType.TIME_SYNC_REQ,
        UsbMsgType.PARAM_OP,
        UsbMsgType.TELEM_FAST,
        UsbMsgType.TELEM_HEALTH,
        UsbMsgType.TIME_SYNC_RESP,
        UsbMsgType.FAULT_SNAPSHOT,
        UsbMsgType.BENCH_RAW_LOG,
    }
    assert set(MESSAGE_CATALOG.keys()) == expected_types

    assert MESSAGE_CATALOG[UsbMsgType.CMD_TWIST].nominal_rate_hz == 25.0
    assert MESSAGE_CATALOG[UsbMsgType.CMD_TWIST].max_rate_hz == 50.0
    assert MESSAGE_CATALOG[UsbMsgType.CMD_TWIST].min_interval_us == 20_000
    assert MESSAGE_CATALOG[UsbMsgType.CMD_TWIST].direction == Direction.PI_TO_STM32

    assert MESSAGE_CATALOG[UsbMsgType.TELEM_FAST].nominal_rate_hz == 200.0
    assert MESSAGE_CATALOG[UsbMsgType.TELEM_FAST].payload_size == 176
    assert MESSAGE_CATALOG[UsbMsgType.TELEM_FAST].direction == Direction.STM32_TO_PI

    assert MESSAGE_CATALOG[UsbMsgType.TELEM_HEALTH].nominal_rate_hz == 20.0
    assert MESSAGE_CATALOG[UsbMsgType.TELEM_HEALTH].payload_size == 56

    assert MESSAGE_CATALOG[UsbMsgType.BENCH_RAW_LOG].nominal_rate_hz == 500.0
    assert MESSAGE_CATALOG[UsbMsgType.BENCH_RAW_LOG].payload_size == 128

    # Round-trip every message type through full COBS+CRC-32C framing
    fast = make_sample_telem_fast()
    health = make_sample_telem_health()
    bench = make_sample_bench_raw()
    frec = FlightRecord(timestamp_us=12345678, seq=42, fast=fast)

    samples = [
        CmdTwist(
            session_id=0xCAFEBABE12345678,
            sent_us=1_000_000,
            pose_us=998_000,
            pose_seq=15,
            v_cmd=0.15,
            yaw_rate_cmd=-0.25,
            height_cmd=0.14,
            roll_cmd=0.01,
            pose_x=1.25,
            pose_y=-0.45,
            pose_yaw=0.75,
        ),
        CmdMode(session_id=99, action=CmdModeAction.REQ_BALANCE, arg=0x42),
        TimeSyncReq(pi_t1_us=555_666_777),
        ParamOp(op=ParamOpCode.SET, param_id=0x0012, value=4.85),
        fast,
        health,
        TimeSyncResp(
            pi_t1_us=1_000_000,
            stm32_rx_us=1_000_400,
            stm32_tx_us=1_000_450,
            clock_err_bound_us=350,
        ),
        FaultSnapshot(
            trigger_reason=0x00000010,
            fault_timestamp_us=12345678,
            record_index=3,
            total_records=250,
            record=frec,
        ),
        bench,
    ]

    for idx, msg in enumerate(samples, start=1):
        wire = encode_message_frame(msg, seq=idx, timestamp_us=idx * 1000, flags=1)
        assert wire[-1] == 0x00
        assert 0x00 not in wire[:-1]
        hdr, decoded = decode_message_frame(wire)
        assert hdr.msg_type == int(msg.MSG_TYPE)
        assert hdr.seq == idx
        assert hdr.timestamp_us == idx * 1000
        assert hdr.flags == 1
        assert decoded.pack() == msg.pack()

    # Check 52-byte CMD_TWIST unpack compatibility
    short_twist_bytes = struct.pack(
        "<QQIfffffff",
        77,
        990_000,
        5,
        0.10,
        0.05,
        0.14,
        0.0,
        0.5,
        0.2,
        0.1,
    )
    assert len(short_twist_bytes) == MSG_CMD_TWIST_SHORT_SIZE
    short_twist = CmdTwist.unpack(short_twist_bytes, header_timestamp_us=991_000)
    assert short_twist.session_id == 77
    assert short_twist.sent_us == 991_000
    assert short_twist.pose_us == 990_000

    # Check TimeSyncResp NTP offset and RTT calculation
    ts_resp = TimeSyncResp(
        pi_t1_us=1_000_000,
        stm32_rx_us=1_002_500,
        stm32_tx_us=1_002_600,
        clock_err_bound_us=200,
    )
    offset_us, rtt_us, err_bound_us = ts_resp.estimate_offset_and_rtt_us(pi_t4_us=1_001_100)
    assert rtt_us == 1000
    assert offset_us == 2000
    assert err_bound_us == 500


def test_telem_and_flight_record_decoders() -> None:
    fast = make_sample_telem_fast()
    raw_fast = pack_telem_fast(fast)
    assert len(raw_fast) == LOG_TELEM_FAST_SIZE == 176
    dec_fast = decode_telem_fast(raw_fast)
    assert dec_fast.fsm_state == fast.fsm_state
    assert dec_fast.v_bus_mv == 24150
    assert dec_fast.i_bus_ma == -3420
    assert math.isclose(dec_fast.v_bus_v, 24.15, rel_tol=1e-6)
    assert math.isclose(dec_fast.i_bus_a, -3.42, rel_tol=1e-6)
    assert_float_tuple_close(dec_fast.euler_rad, fast.euler_rad)
    assert_float_tuple_close(dec_fast.motor_tor_cmd_nm, fast.motor_tor_cmd_nm)

    health = make_sample_telem_health()
    raw_health = pack_telem_health(health)
    assert len(raw_health) == LOG_TELEM_HEALTH_SIZE == 56
    dec_health = decode_telem_health(raw_health)
    assert dec_health == health

    bench = make_sample_bench_raw()
    raw_bench = pack_bench_raw(bench)
    assert len(raw_bench) == LOG_BENCH_RAW_SIZE == 128
    dec_bench = decode_bench_raw(raw_bench)
    assert dec_bench.imu_ts_us == bench.imu_ts_us
    assert dec_bench.fresh_mask == 0x3F
    assert_float_tuple_close(dec_bench.gyro_rad_s, bench.gyro_rad_s)

    # FlightRecord pack/decode + circular FlightRingBuffer pre/post-fault freeze
    frec = FlightRecord(timestamp_us=2_500_000, seq=1250, fast=fast)
    raw_frec = pack_flight_record(frec)
    assert len(raw_frec) == LOG_FLIGHT_RECORD_SIZE == 188
    dec_frec = decode_flight_record(raw_frec)
    assert dec_frec.timestamp_us == frec.timestamp_us
    assert dec_frec.seq == frec.seq
    assert dec_frec.fast.pack() == fast.pack()

    # Bare 188-byte FlightRecord inside FAULT_SNAPSHOT payload
    snap_from_bare = FaultSnapshot.unpack(raw_frec)
    assert snap_from_bare.record.seq == 1250
    assert snap_from_bare.fault_timestamp_us == 2_500_000

    # Ring buffer wrapping and post-fault freeze
    ring = FlightRingBuffer(capacity=5, post_fault_ticks=2)
    for i in range(1, 8):
        ring.push(FlightRecord(timestamp_us=i * 2000, seq=i, fast=fast))
        if i == 4:
            ring.trigger_fault()
    # Triggered at i=4 -> records i=5 (rem=1), i=6 (rem=0 -> frozen), ignores i=7
    assert ring.is_frozen
    assert ring.count == 5
    seqs = [r.seq for r in ring.records()]
    assert seqs == [2, 3, 4, 5, 6]

    dump_bytes = ring.pack_dump()
    assert len(dump_bytes) == 5 * LOG_FLIGHT_RECORD_SIZE
    unpacked_dump = decode_flight_dump(dump_bytes)
    assert [r.seq for r in unpacked_dump] == [2, 3, 4, 5, 6]


def test_corrupted_frames_and_streaming_decoder() -> None:
    fast = make_sample_telem_fast()
    valid_wire = frame_telem_fast(seq=10, timestamp_us=20_000, rec=fast)

    # 1. Bit flip in payload/CRC -> ERR_CRC
    corrupted_crc = bytearray(valid_wire)
    corrupted_crc[10] ^= 0x01
    if corrupted_crc[10] == 0x00:
        corrupted_crc[10] = 0x02
    st, _, _ = usb_link_decode(corrupted_crc)
    assert st == UsbLinkStatus.ERR_CRC

    # 2. Unexpected zero byte inside COBS body -> ERR_COBS
    corrupted_cobs = bytearray(valid_wire)
    corrupted_cobs[5] = 0x00
    st, _, _ = usb_link_decode(corrupted_cobs[:-1])
    assert st == UsbLinkStatus.ERR_COBS

    # 3. Truncated frame -> ERR_COBS or ERR_LENGTH
    st, _, _ = usb_link_decode(valid_wire[:8])
    assert st in (UsbLinkStatus.ERR_COBS, UsbLinkStatus.ERR_LENGTH)

    # 4. Valid COBS of too-short raw frame (<22 bytes) -> ERR_LENGTH
    short_raw = cobs_encode(b"\x01" * 10) + b"\x00"
    st, _, _ = usb_link_decode(short_raw)
    assert st == UsbLinkStatus.ERR_LENGTH

    # 5. Header payload_len mismatch with actual raw body -> ERR_LENGTH
    bad_hdr = UsbLinkHeader(
        msg_type=UsbMsgType.TIME_SYNC_REQ,
        seq=1,
        timestamp_us=100,
        payload_len=12,  # Claims 12 bytes, actual is 8 bytes
    )
    raw_body = bad_hdr.pack() + (b"\x00" * 8)
    raw_frame = raw_body + struct.pack("<I", crc32c_compute(raw_body))
    bad_len_wire = cobs_encode(raw_frame) + b"\x00"
    st, _, _ = usb_link_decode(bad_len_wire)
    assert st == UsbLinkStatus.ERR_LENGTH

    # 6. Payload larger than USB_LINK_MAX_PAYLOAD (256) -> ERR_PAYLOAD_TOO_LARGE
    huge_hdr = UsbLinkHeader(
        msg_type=UsbMsgType.TELEM_FAST,
        seq=1,
        timestamp_us=100,
        payload_len=257,
    )
    st, _ = usb_link_encode(huge_hdr, b"\x00" * 257)
    assert st == UsbLinkStatus.ERR_PAYLOAD_TOO_LARGE

    # 7. StreamingDecoder with leading 0x00 noise, valid frame, corrupted frame, overflow frame, and recovery
    rx = StreamingDecoder()
    stream = bytearray()
    stream.extend(b"\x00\x00\x00")  # Leading delimiter noise
    stream.extend(valid_wire)       # Valid frame #1
    stream.extend(corrupted_crc)    # CRC-corrupted frame
    stream.extend(bad_len_wire)     # Protocol length error frame
    stream.extend(b"\x01" * (USB_LINK_MAX_WIRE_FRAME + 20) + b"\x00")  # Overflow frame
    valid_wire_2 = frame_telem_health(seq=11, timestamp_us=25_000, rec=make_sample_telem_health())
    stream.extend(valid_wire_2)     # Valid frame #2 after recovery

    events = rx.feed_bytes(stream)
    statuses = [ev[0] for ev in events]
    assert statuses == [
        UsbLinkStatus.OK,
        UsbLinkStatus.ERR_CRC,
        UsbLinkStatus.ERR_LENGTH,
        UsbLinkStatus.ERR_OVERFLOW,
        UsbLinkStatus.OK,
    ]
    assert rx.frames_ok == 2
    assert rx.crc_errors == 1
    assert rx.proto_errors == 1
    assert rx.overflow_errors == 1


def test_tx_rate_gate_matches_host_gate() -> None:
    # Verify exact constants from S12_comms_in_loop/host_gate.py
    assert TTL_US == 100_000
    assert FIRMWARE_TTL_US == 150_000
    assert MIN_INTERVAL_US == 20_000
    assert MAX_CLOCK_ERROR_US == 5_000
    assert MAX_V == 0.20
    assert MAX_YAW_RATE == 0.50
    assert MAX_ACCEL == 0.40
    assert MAX_YAW_ACCEL == 1.0

    # 1. Default local_only rejects grant and packets
    g_local = TxRateGate(local_only=True)
    assert not g_local.local_grant(1, 1000, balance_ready=True, clock_error_us=100)
    pkt0 = Packet(Command.TWIST, 1, 1, 1000, 1000, 1, 0.1, 0.0, 0.0, 0.0, 0.0)
    assert g_local.receive(pkt0.encode(), 1000, clock_error_us=100).reason == "local_only"

    # 2. Un-granted gate returns no_local_grant
    gate = TxRateGate(local_only=False, ttl_us=150_000)
    assert gate.receive(pkt0.encode(), 1000, clock_error_us=100).reason == "no_local_grant"

    # 3. Grant session 10
    assert gate.local_grant(10, 100_000, balance_ready=True, clock_error_us=500)

    # First valid CMD_TWIST at t=100_000 us
    cmd1 = CmdTwist(
        session_id=10,
        sent_us=99_000,
        pose_us=98_500,
        pose_seq=1,
        v_cmd=0.15,
        yaw_rate_cmd=0.20,
        pose_x=0.0,
        pose_y=0.0,
        pose_yaw=0.0,
    )
    d1, wire1 = gate.encode_twist(cmd1, seq=1, now_us=100_000, clock_error_us=500)
    assert d1.accepted and d1.reason == "accepted"
    assert wire1 is not None
    assert d1.command_age_upper_us == 1500
    assert d1.pose_age_upper_us == 2000

    # 4. Rate ceiling violation: sending <20_000 us later (>50 Hz) -> "packet_rate"
    cmd_fast = CmdTwist(
        session_id=10,
        sent_us=110_000,
        pose_us=110_000,
        pose_seq=2,
        v_cmd=0.15,
        yaw_rate_cmd=0.20,
        pose_x=0.001,
        pose_y=0.0,
        pose_yaw=0.0,
    )
    d_fast, wire_fast = gate.encode_twist(cmd_fast, seq=2, now_us=110_000, clock_error_us=500)
    assert not d_fast.accepted and d_fast.reason == "packet_rate"
    assert wire_fast is None

    # 5. Sequence error (non-increasing seq) -> "sequence"
    d_seq = gate.check_twist(cmd_fast, seq=1, now_us=125_000, clock_error_us=500)
    assert not d_seq.accepted and d_seq.reason == "sequence"

    # 6. Velocity envelope violation -> "velocity_envelope"
    cmd_over_v = CmdTwist(
        session_id=10,
        sent_us=125_000,
        pose_us=125_000,
        pose_seq=2,
        v_cmd=0.25,  # > MAX_V (0.20)
        yaw_rate_cmd=0.10,
    )
    d_vel = gate.check_twist(cmd_over_v, seq=2, now_us=125_000, clock_error_us=500)
    assert not d_vel.accepted and d_vel.reason == "velocity_envelope"

    # 7. Negative velocity rejected -> "velocity_envelope"
    cmd_neg_v = CmdTwist(
        session_id=10,
        sent_us=125_000,
        pose_us=125_000,
        pose_seq=2,
        v_cmd=-0.01,
        yaw_rate_cmd=0.0,
    )
    assert gate.check_twist(cmd_neg_v, seq=2, now_us=125_000, clock_error_us=500).reason == "velocity_envelope"

    # 8. Non-finite float rejected -> "nonfinite"
    cmd_nan = CmdTwist(
        session_id=10,
        sent_us=125_000,
        pose_us=125_000,
        pose_seq=2,
        v_cmd=float("nan"),
        yaw_rate_cmd=0.0,
    )
    assert gate.check_twist(cmd_nan, seq=2, now_us=125_000, clock_error_us=500).reason == "nonfinite"

    # 9. Future timestamp beyond clock_error_us -> "future_stamp"
    cmd_future = CmdTwist(
        session_id=10,
        sent_us=126_000,
        pose_us=125_000,
        pose_seq=2,
        v_cmd=0.10,
        yaw_rate_cmd=0.0,
    )
    assert gate.check_twist(cmd_future, seq=2, now_us=125_000, clock_error_us=500).reason == "future_stamp"

    # 10. Pose sample changed with identical pose_seq -> "pose_sample_changed"
    cmd_changed = CmdTwist(
        session_id=10,
        sent_us=125_000,
        pose_us=98_500,
        pose_seq=1,
        v_cmd=0.10,
        yaw_rate_cmd=0.0,
        pose_x=0.05,  # Changed x with same pose_seq=1
    )
    assert gate.check_twist(cmd_changed, seq=2, now_us=125_000, clock_error_us=500).reason == "pose_sample_changed"

    # 11. Valid second packet at 25 Hz (40_000 us interval)
    cmd2 = CmdTwist(
        session_id=10,
        sent_us=140_000,
        pose_us=139_500,
        pose_seq=2,
        v_cmd=0.20,
        yaw_rate_cmd=-0.20,
        pose_x=0.005,
        pose_y=0.0,
        pose_yaw=0.01,
    )
    d2, wire2 = gate.encode_twist(cmd2, seq=2, now_us=140_000, clock_error_us=500)
    assert d2.accepted and wire2 is not None

    # 12. Slew-rate limiter tick() (1 ms step -> max dv = 0.40 * 0.001 = 0.0004)
    v_out, yaw_out = gate.tick(141_000, clock_error_us=500)
    assert math.isclose(v_out, 0.0004, rel_tol=1e-6)
    assert math.isclose(yaw_out, -0.001, rel_tol=1e-6)

    # 13. Relocalization jump trips fault="relocalization"
    cmd_jump = CmdTwist(
        session_id=10,
        sent_us=165_000,
        pose_us=165_000,
        pose_seq=3,
        v_cmd=0.10,
        yaw_rate_cmd=0.0,
        pose_x=0.50,  # Jump > 0.10 + 0.50 * dt
        pose_y=0.0,
        pose_yaw=0.01,
    )
    d_jump = gate.check_twist(cmd_jump, seq=3, now_us=165_000, clock_error_us=500)
    assert not d_jump.accepted and d_jump.reason == "relocalization"
    assert gate.fault == "relocalization"
    assert gate.session is None

    # 14. Reset and re-grant with a strictly higher session ID
    assert gate.local_reset(supervisor_disarmed=True)
    assert not gate.local_grant(10, 200_000, balance_ready=True, clock_error_us=500)  # Must be > 10
    assert gate.local_grant(11, 200_000, balance_ready=True, clock_error_us=500)


def test_loopback_transport_and_endpoints() -> None:
    # Self-loopback transport
    single = LoopbackTransport()
    assert single.is_open
    single.write(b"abc")
    single.write(b"def")
    assert single.available == 6
    assert single.read(3) == b"abc"
    assert single.read(10) == b"def"
    assert single.read() == b""

    # Paired full-duplex Pi <-> STM32 loopback endpoints
    pi_tx_gate = TxRateGate(local_only=False, ttl_us=150_000)
    stm32_rx_gate = HostGate(local_only=False, ttl_us=150_000)
    assert pi_tx_gate.local_grant(1, 100_000, balance_ready=True, clock_error_us=200)
    assert stm32_rx_gate.local_grant(1, 100_000, balance_ready=True, clock_error_us=200)

    pi_tr, stm32_tr = LoopbackTransport.create_pair()
    pi_ep = PiLinkEndpoint(pi_tr, tx_gate=pi_tx_gate)
    stm32_ep = PiLinkEndpoint(stm32_tr)

    # Pi sends gated CMD_TWIST at t=100_000
    twist = CmdTwist(
        session_id=1,
        sent_us=100_000,
        pose_us=99_800,
        pose_seq=1,
        v_cmd=0.12,
        yaw_rate_cmd=0.05,
        pose_x=0.0,
        pose_y=0.0,
        pose_yaw=0.0,
    )
    dec = pi_ep.send_twist_gated(twist, 100_000, clock_error_us=200)
    assert dec.accepted

    # STM32 receives and validates through HostGate
    rx_msgs = stm32_ep.poll_messages()
    assert len(rx_msgs) == 1
    rx_hdr, rx_twist = rx_msgs[0]
    assert rx_hdr.msg_type == int(UsbMsgType.CMD_TWIST)
    assert isinstance(rx_twist, CmdTwist)
    stm32_dec = stm32_rx_gate.receive(
         rx_twist,
        100_500,
        clock_error_us=200,
        seq=rx_hdr.seq,
    )
    assert stm32_dec.accepted

    # STM32 streams TELEM_FAST and TELEM_HEALTH back to Pi
    fast = make_sample_telem_fast()
    health = make_sample_telem_health()
    stm32_ep.send_message(fast, timestamp_us=101_000)
    stm32_ep.send_message(health, timestamp_us=101_500)

    pi_rx = pi_ep.poll_messages()
    assert len(pi_rx) == 2
    assert pi_rx[0][0].msg_type == int(UsbMsgType.TELEM_FAST)
    assert pi_rx[0][1].pack() == fast.pack()
    assert pi_rx[1][0].msg_type == int(UsbMsgType.TELEM_HEALTH)
    assert pi_rx[1][1] == health

    pi_tr.close()
    stm32_tr.close()
    assert not pi_tr.is_open


UNIT_TESTS = [
    ("test_crc32c_known_vectors", test_crc32c_known_vectors),
    ("test_cobs_vectors_and_edge_cases", test_cobs_vectors_and_edge_cases),
    ("test_header_and_versioning", test_header_and_versioning),
    ("test_catalog_and_all_message_roundtrips", test_catalog_and_all_message_roundtrips),
    ("test_telem_and_flight_record_decoders", test_telem_and_flight_record_decoders),
    ("test_corrupted_frames_and_streaming_decoder", test_corrupted_frames_and_streaming_decoder),
    ("test_tx_rate_gate_matches_host_gate", test_tx_rate_gate_matches_host_gate),
    ("test_loopback_transport_and_endpoints", test_loopback_transport_and_endpoints),
]
