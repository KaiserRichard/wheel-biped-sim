"""Cross-language tests: Python <-> C shared library (c_ref) for every message type."""
from __future__ import annotations

import ctypes
from ctypes import (
    POINTER,
    Structure,
    byref,
    c_float,
    c_int8,
    c_int16,
    c_size_t,
    c_uint8,
    c_uint16,
    c_uint32,
    c_uint64,
)
import math
from pathlib import Path

from pi_link import (
    BenchRawLog,
    CmdMode,
    CmdModeAction,
    CmdTwist,
    CobsStatus,
    FaultSnapshot,
    FlightRecord,
    FlightRingBuffer,
    LOG_BENCH_RAW_SIZE,
    LOG_FLIGHT_RECORD_SIZE,
    LOG_NUM_AXES,
    LOG_NUM_CAN_BUSES,
    LOG_NUM_RTOS_TASKS,
    LOG_TELEM_FAST_SIZE,
    LOG_TELEM_HEALTH_SIZE,
    LogStatus,
    MSG_CMD_MODE_SIZE,
    MSG_CMD_TWIST_SIZE,
    MSG_FAULT_SNAPSHOT_SIZE,
    MSG_PARAM_OP_SIZE,
    MSG_TIME_SYNC_REQ_SIZE,
    MSG_TIME_SYNC_RESP_SIZE,
    ParamOp,
    ParamOpCode,
    StreamingDecoder,
    TelemFast,
    TelemHealth,
    TimeSyncReq,
    TimeSyncResp,
    USB_LINK_MAX_PAYLOAD,
    USB_LINK_MAX_WIRE_FRAME,
    USB_LINK_VER_MAJOR,
    USB_LINK_VER_MINOR,
    UsbLinkHeader,
    UsbLinkStatus,
    UsbMsgType,
    cobs_decode,
    cobs_encode,
    crc32c_compute,
    decode_bench_raw,
    decode_flight_record,
    decode_message_frame,
    decode_telem_fast,
    decode_telem_health,
    encode_message_frame,
    pack_flight_record,
    usb_link_decode,
    usb_link_encode,
)


class CUsbLinkHeader(Structure):
    _fields_ = [
        ("ver_major", c_uint8),
        ("ver_minor", c_uint8),
        ("msg_type", c_uint8),
        ("flags", c_uint8),
        ("seq", c_uint32),
        ("timestamp_us", c_uint64),
        ("payload_len", c_uint16),
    ]


class CUsbLinkRxParser(Structure):
    _fields_ = [
        ("buf", c_uint8 * USB_LINK_MAX_WIRE_FRAME),
        ("len", c_size_t),
        ("overflowed", c_uint8),
        ("frames_ok", c_uint32),
        ("crc_errors", c_uint32),
        ("proto_errors", c_uint32),
        ("overflow_errors", c_uint32),
    ]


class CLogTelemFast(Structure):
    _fields_ = [
        ("fsm_state", c_uint8),
        ("s11_mode", c_uint8),
        ("s11_max_misses", c_uint8),
        ("flags", c_uint8),
        ("fault_mask", c_uint32),
        ("euler_rad", c_float * 3),
        ("gyro_rad_s", c_float * 3),
        ("v_est_m_s", c_float),
        ("x_est_m", c_float),
        ("leg_length_m", c_float * 2),
        ("leg_theta_rad", c_float * 2),
        ("motor_pos_rad", c_float * LOG_NUM_AXES),
        ("motor_vel_rad_s", c_float * LOG_NUM_AXES),
        ("motor_tor_cmd_nm", c_float * LOG_NUM_AXES),
        ("motor_tor_fb_nm", c_float * LOG_NUM_AXES),
        ("enc_age_us", c_uint16 * LOG_NUM_AXES),
        ("s9_trim_rad", c_float),
        ("gyro_bias_y_rad_s", c_float),
        ("v_bus_mv", c_uint16),
        ("i_bus_ma", c_int16),
    ]


class CLogTelemHealth(Structure):
    _fields_ = [
        ("can_tec", c_uint8 * LOG_NUM_CAN_BUSES),
        ("can_rec", c_uint8 * LOG_NUM_CAN_BUSES),
        ("can_busoff_cnt", c_uint16 * LOG_NUM_CAN_BUSES),
        ("can_rx_drop", c_uint16 * LOG_NUM_CAN_BUSES),
        ("motor_state", c_uint8 * LOG_NUM_AXES),
        ("t_mos_c", c_uint8 * LOG_NUM_AXES),
        ("t_coil_c", c_uint8 * LOG_NUM_AXES),
        ("rc_rssi_dbm", c_int8),
        ("rc_lq_pct", c_uint8),
        ("rc_age_ms", c_uint16),
        ("rc_switches", c_uint16),
        ("power_io_bits", c_uint16),
        ("reset_cause", c_uint8),
        ("cpu_load_pct", c_uint8),
        ("min_stack_words", c_uint16 * LOG_NUM_RTOS_TASKS),
    ]


class CLogBenchRaw(Structure):
    _fields_ = [
        ("imu_ts_us", c_uint64),
        ("gyro_rad_s", c_float * 3),
        ("accel_m_s2", c_float * 3),
        ("imu_temp_c", c_float),
        ("motor_pos_rad", c_float * LOG_NUM_AXES),
        ("motor_vel_rad_s", c_float * LOG_NUM_AXES),
        ("motor_rx_us", c_uint32 * LOG_NUM_AXES),
        ("fresh_mask", c_uint8),
        ("reserved", c_uint8 * 3),
        ("pitch_base_rad", c_float),
        ("v_base_m_s", c_float),
        ("pitch_s10_rad", c_float),
        ("v_s10_m_s", c_float),
    ]


class CLogFlightRecord(Structure):
    _fields_ = [
        ("timestamp_us", c_uint64),
        ("seq", c_uint32),
        ("fast", CLogTelemFast),
    ]


class CLogFlightRing(Structure):
    _fields_ = [
        ("storage", POINTER(CLogFlightRecord)),
        ("capacity", c_uint32),
        ("head", c_uint32),
        ("count", c_uint32),
        ("post_fault_ticks", c_uint32),
        ("post_fault_remaining", c_uint32),
        ("fault_triggered", c_uint8),
        ("frozen", c_uint8),
    ]


class CMsgCmdTwist(Structure):
    _fields_ = [
        ("session_id", c_uint64),
        ("sent_us", c_uint64),
        ("pose_us", c_uint64),
        ("pose_seq", c_uint32),
        ("v_cmd", c_float),
        ("yaw_rate_cmd", c_float),
        ("height_cmd", c_float),
        ("roll_cmd", c_float),
        ("pose_x", c_float),
        ("pose_y", c_float),
        ("pose_yaw", c_float),
    ]


class CMsgCmdMode(Structure):
    _fields_ = [
        ("session_id", c_uint64),
        ("action", c_uint8),
        ("arg", c_uint32),
    ]


class CMsgTimeSyncReq(Structure):
    _fields_ = [
        ("pi_t1_us", c_uint64),
    ]


class CMsgParamOp(Structure):
    _fields_ = [
        ("op", c_uint8),
        ("param_id", c_uint16),
        ("value", c_float),
    ]


class CMsgTimeSyncResp(Structure):
    _fields_ = [
        ("pi_t1_us", c_uint64),
        ("stm32_rx_us", c_uint64),
        ("stm32_tx_us", c_uint64),
        ("clock_err_bound_us", c_uint32),
    ]


class CMsgFaultSnapshot(Structure):
    _fields_ = [
        ("trigger_reason", c_uint32),
        ("fault_timestamp_us", c_uint64),
        ("record_index", c_uint16),
        ("total_records", c_uint16),
        ("record", CLogFlightRecord),
    ]


def load_c_ref_lib() -> ctypes.CDLL:
    root = Path(__file__).resolve().parent.parent
    candidates = [
        root / "build" / "libc_ref.so",
        root / "build" / "c_ref.dll",
        root / "build" / "libc_ref.dylib",
    ]
    for path in candidates:
        if path.exists():
            lib = ctypes.CDLL(str(path))
            _bind_signatures(lib)
            return lib
    raise FileNotFoundError(
        f"Shared library not found in {[str(p) for p in candidates]}; run cmake --build build first."
    )


def _bind_signatures(lib: ctypes.CDLL) -> None:
    u8_p = POINTER(c_uint8)
    sz_p = POINTER(c_size_t)

    lib.crc32c_init.argtypes = []
    lib.crc32c_init.restype = c_uint32
    lib.crc32c_update.argtypes = [c_uint32, u8_p, c_size_t]
    lib.crc32c_update.restype = c_uint32
    lib.crc32c_final.argtypes = [c_uint32]
    lib.crc32c_final.restype = c_uint32
    lib.crc32c_compute.argtypes = [u8_p, c_size_t]
    lib.crc32c_compute.restype = c_uint32

    lib.cobs_encode.argtypes = [u8_p, c_size_t, u8_p, c_size_t, sz_p]
    lib.cobs_encode.restype = ctypes.c_int
    lib.cobs_decode.argtypes = [u8_p, c_size_t, u8_p, c_size_t, sz_p]
    lib.cobs_decode.restype = ctypes.c_int

    lib.usb_link_encode.argtypes = [POINTER(CUsbLinkHeader), u8_p, u8_p, c_size_t, sz_p]
    lib.usb_link_encode.restype = ctypes.c_int
    lib.usb_link_decode.argtypes = [u8_p, c_size_t, POINTER(CUsbLinkHeader), u8_p, c_size_t]
    lib.usb_link_decode.restype = ctypes.c_int

    lib.usb_link_rx_init.argtypes = [POINTER(CUsbLinkRxParser)]
    lib.usb_link_rx_init.restype = None
    lib.usb_link_rx_feed.argtypes = [
        POINTER(CUsbLinkRxParser),
        c_uint8,
        POINTER(CUsbLinkHeader),
        u8_p,
        c_size_t,
    ]
    lib.usb_link_rx_feed.restype = ctypes.c_int

    # log_packer.h functions
    lib.log_pack_telem_fast.argtypes = [POINTER(CLogTelemFast), u8_p, c_size_t, sz_p]
    lib.log_pack_telem_fast.restype = ctypes.c_int
    lib.log_unpack_telem_fast.argtypes = [u8_p, c_size_t, POINTER(CLogTelemFast)]
    lib.log_unpack_telem_fast.restype = ctypes.c_int
    lib.log_frame_telem_fast.argtypes = [
        c_uint32,
        c_uint64,
        c_uint8,
        POINTER(CLogTelemFast),
        u8_p,
        c_size_t,
        sz_p,
    ]
    lib.log_frame_telem_fast.restype = ctypes.c_int

    lib.log_pack_telem_health.argtypes = [POINTER(CLogTelemHealth), u8_p, c_size_t, sz_p]
    lib.log_pack_telem_health.restype = ctypes.c_int
    lib.log_unpack_telem_health.argtypes = [u8_p, c_size_t, POINTER(CLogTelemHealth)]
    lib.log_unpack_telem_health.restype = ctypes.c_int
    lib.log_frame_telem_health.argtypes = [
        c_uint32,
        c_uint64,
        c_uint8,
        POINTER(CLogTelemHealth),
        u8_p,
        c_size_t,
        sz_p,
    ]
    lib.log_frame_telem_health.restype = ctypes.c_int

    lib.log_pack_bench_raw.argtypes = [POINTER(CLogBenchRaw), u8_p, c_size_t, sz_p]
    lib.log_pack_bench_raw.restype = ctypes.c_int
    lib.log_unpack_bench_raw.argtypes = [u8_p, c_size_t, POINTER(CLogBenchRaw)]
    lib.log_unpack_bench_raw.restype = ctypes.c_int
    lib.log_frame_bench_raw.argtypes = [
        c_uint32,
        c_uint64,
        c_uint8,
        POINTER(CLogBenchRaw),
        u8_p,
        c_size_t,
        sz_p,
    ]
    lib.log_frame_bench_raw.restype = ctypes.c_int

    lib.log_pack_flight_record.argtypes = [POINTER(CLogFlightRecord), u8_p, c_size_t, sz_p]
    lib.log_pack_flight_record.restype = ctypes.c_int
    lib.log_unpack_flight_record.argtypes = [u8_p, c_size_t, POINTER(CLogFlightRecord)]
    lib.log_unpack_flight_record.restype = ctypes.c_int

    lib.log_ring_init.argtypes = [
        POINTER(CLogFlightRing),
        POINTER(CLogFlightRecord),
        c_uint32,
        c_uint32,
    ]
    lib.log_ring_init.restype = ctypes.c_int
    lib.log_ring_push.argtypes = [POINTER(CLogFlightRing), POINTER(CLogFlightRecord)]
    lib.log_ring_push.restype = ctypes.c_int
    lib.log_ring_trigger_fault.argtypes = [POINTER(CLogFlightRing)]
    lib.log_ring_trigger_fault.restype = None
    lib.log_ring_count.argtypes = [POINTER(CLogFlightRing)]
    lib.log_ring_count.restype = c_uint32
    lib.log_ring_is_frozen.argtypes = [POINTER(CLogFlightRing)]
    lib.log_ring_is_frozen.restype = c_uint8
    lib.log_ring_get.argtypes = [POINTER(CLogFlightRing), c_uint32, POINTER(CLogFlightRecord)]
    lib.log_ring_get.restype = ctypes.c_int

    # c_msg_packers.h functions for remaining catalog messages
    for name, c_cls in [
        ("cmd_twist", CMsgCmdTwist),
        ("cmd_mode", CMsgCmdMode),
        ("time_sync_req", CMsgTimeSyncReq),
        ("param_op", CMsgParamOp),
        ("time_sync_resp", CMsgTimeSyncResp),
        ("fault_snapshot", CMsgFaultSnapshot),
    ]:
        pack_fn = getattr(lib, f"msg_pack_{name}")
        pack_fn.argtypes = [POINTER(c_cls), u8_p, c_size_t, sz_p]
        pack_fn.restype = ctypes.c_int

        unpack_fn = getattr(lib, f"msg_unpack_{name}")
        unpack_fn.argtypes = [u8_p, c_size_t, POINTER(c_cls)]
        unpack_fn.restype = ctypes.c_int

        frame_fn = getattr(lib, f"msg_frame_{name}")
        frame_fn.argtypes = [c_uint32, c_uint64, c_uint8, POINTER(c_cls), u8_p, c_size_t, sz_p]
        frame_fn.restype = ctypes.c_int


def _to_u8_array(data: bytes) -> ctypes.Array[c_uint8]:
    arr_t = c_uint8 * max(1, len(data))
    arr = arr_t()
    for i, b in enumerate(data):
        arr[i] = b
    return arr


def _c_decode_wire_and_stream(
    lib: ctypes.CDLL, wire: bytes
) -> tuple[CUsbLinkHeader, bytes]:
    """Decode a wire frame in C using BOTH usb_link_decode and byte-by-byte usb_link_rx_feed."""
    wire_arr = _to_u8_array(wire)
    out_hdr = CUsbLinkHeader()
    out_payload = (c_uint8 * USB_LINK_MAX_PAYLOAD)()

    st = lib.usb_link_decode(
        wire_arr,
        len(wire),
        byref(out_hdr),
        out_payload,
        USB_LINK_MAX_PAYLOAD,
    )
    assert st == int(UsbLinkStatus.OK), f"C usb_link_decode failed with {st}"
    decoded_payload = bytes(out_payload[: out_hdr.payload_len])

    # Also test C byte-by-byte streaming parser usb_link_rx_feed
    rx = CUsbLinkRxParser()
    lib.usb_link_rx_init(byref(rx))
    stream_hdr = CUsbLinkHeader()
    stream_payload = (c_uint8 * USB_LINK_MAX_PAYLOAD)()

    for idx, b in enumerate(wire):
        st_feed = lib.usb_link_rx_feed(
            byref(rx),
            b,
            byref(stream_hdr),
            stream_payload,
            USB_LINK_MAX_PAYLOAD,
        )
        if idx < len(wire) - 1:
            assert st_feed == int(UsbLinkStatus.NEED_MORE)
        else:
            assert st_feed == int(UsbLinkStatus.OK)

    assert rx.frames_ok == 1
    assert rx.crc_errors == 0
    assert rx.proto_errors == 0
    assert stream_hdr.ver_major == out_hdr.ver_major
    assert stream_hdr.ver_minor == out_hdr.ver_minor
    assert stream_hdr.msg_type == out_hdr.msg_type
    assert stream_hdr.flags == out_hdr.flags
    assert stream_hdr.seq == out_hdr.seq
    assert stream_hdr.timestamp_us == out_hdr.timestamp_us
    assert stream_hdr.payload_len == out_hdr.payload_len
    assert bytes(stream_payload[: stream_hdr.payload_len]) == decoded_payload

    return out_hdr, decoded_payload


def _populate_c_telem_fast(py_rec: TelemFast) -> CLogTelemFast:
    c_rec = CLogTelemFast()
    c_rec.fsm_state = py_rec.fsm_state
    c_rec.s11_mode = py_rec.s11_mode
    c_rec.s11_max_misses = py_rec.s11_max_misses
    c_rec.flags = py_rec.flags
    c_rec.fault_mask = py_rec.fault_mask
    for i in range(3):
        c_rec.euler_rad[i] = py_rec.euler_rad[i]
        c_rec.gyro_rad_s[i] = py_rec.gyro_rad_s[i]
    c_rec.v_est_m_s = py_rec.v_est_m_s
    c_rec.x_est_m = py_rec.x_est_m
    for i in range(2):
        c_rec.leg_length_m[i] = py_rec.leg_length_m[i]
        c_rec.leg_theta_rad[i] = py_rec.leg_theta_rad[i]
    for i in range(LOG_NUM_AXES):
        c_rec.motor_pos_rad[i] = py_rec.motor_pos_rad[i]
        c_rec.motor_vel_rad_s[i] = py_rec.motor_vel_rad_s[i]
        c_rec.motor_tor_cmd_nm[i] = py_rec.motor_tor_cmd_nm[i]
        c_rec.motor_tor_fb_nm[i] = py_rec.motor_tor_fb_nm[i]
        c_rec.enc_age_us[i] = py_rec.enc_age_us[i]
    c_rec.s9_trim_rad = py_rec.s9_trim_rad
    c_rec.gyro_bias_y_rad_s = py_rec.gyro_bias_y_rad_s
    c_rec.v_bus_mv = py_rec.v_bus_mv
    c_rec.i_bus_ma = py_rec.i_bus_ma
    return c_rec


def _assert_c_telem_fast_eq(c_rec: CLogTelemFast, py_rec: TelemFast) -> None:
    assert c_rec.fsm_state == py_rec.fsm_state
    assert c_rec.s11_mode == py_rec.s11_mode
    assert c_rec.s11_max_misses == py_rec.s11_max_misses
    assert c_rec.flags == py_rec.flags
    assert c_rec.fault_mask == py_rec.fault_mask
    for i in range(3):
        assert math.isclose(c_rec.euler_rad[i], py_rec.euler_rad[i], rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(c_rec.gyro_rad_s[i], py_rec.gyro_rad_s[i], rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(c_rec.v_est_m_s, py_rec.v_est_m_s, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(c_rec.x_est_m, py_rec.x_est_m, rel_tol=1e-6, abs_tol=1e-6)
    for i in range(2):
        assert math.isclose(c_rec.leg_length_m[i], py_rec.leg_length_m[i], rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(c_rec.leg_theta_rad[i], py_rec.leg_theta_rad[i], rel_tol=1e-6, abs_tol=1e-6)
    for i in range(LOG_NUM_AXES):
        assert math.isclose(c_rec.motor_pos_rad[i], py_rec.motor_pos_rad[i], rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(c_rec.motor_vel_rad_s[i], py_rec.motor_vel_rad_s[i], rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(c_rec.motor_tor_cmd_nm[i], py_rec.motor_tor_cmd_nm[i], rel_tol=1e-6, abs_tol=1e-6)
        assert math.isclose(c_rec.motor_tor_fb_nm[i], py_rec.motor_tor_fb_nm[i], rel_tol=1e-6, abs_tol=1e-6)
        assert c_rec.enc_age_us[i] == py_rec.enc_age_us[i]
    assert math.isclose(c_rec.s9_trim_rad, py_rec.s9_trim_rad, rel_tol=1e-6, abs_tol=1e-6)
    assert math.isclose(c_rec.gyro_bias_y_rad_s, py_rec.gyro_bias_y_rad_s, rel_tol=1e-6, abs_tol=1e-6)
    assert c_rec.v_bus_mv == py_rec.v_bus_mv
    assert c_rec.i_bus_ma == py_rec.i_bus_ma


def test_cross_crc32c_and_cobs() -> None:
    lib = load_c_ref_lib()

    vectors = [
        b"",
        b"123456789",
        b"\x00",
        b"\x00" * 64,
        b"\xFF" * 64,
        bytes(range(1, 255)),
        bytes(range(1, 255)) + b"\x00\xAA\x00\xBB",
        bytes(i & 0xFF for i in range(278)),
    ]

    for vec in vectors:
        arr = _to_u8_array(vec)
        c_crc = lib.crc32c_compute(arr, len(vec))
        py_crc = crc32c_compute(vec)
        assert c_crc == py_crc, f"CRC mismatch for len={len(vec)}: C=0x{c_crc:08X} vs Py=0x{py_crc:08X}"

        # Python COBS encode -> C COBS decode
        py_enc = cobs_encode(vec)
        enc_arr = _to_u8_array(py_enc)
        c_dec_buf = (c_uint8 * max(1, len(vec) + 16))()
        c_dec_len = c_size_t(0)
        st = lib.cobs_decode(enc_arr, len(py_enc), c_dec_buf, len(c_dec_buf), byref(c_dec_len))
        assert st == int(CobsStatus.OK)
        assert bytes(c_dec_buf[: c_dec_len.value]) == vec

        # C COBS encode -> Python COBS decode
        c_enc_buf = (c_uint8 * 300)()
        c_enc_len = c_size_t(0)
        st_enc = lib.cobs_encode(arr, len(vec), c_enc_buf, len(c_enc_buf), byref(c_enc_len))
        assert st_enc == int(CobsStatus.OK)
        c_enc_bytes = bytes(c_enc_buf[: c_enc_len.value])
        assert c_enc_bytes == py_enc
        assert cobs_decode(c_enc_bytes) == vec


def test_cross_cmd_twist_0x01() -> None:
    lib = load_c_ref_lib()
    py_msg = CmdTwist(
        session_id=0x1122334455667788,
        sent_us=1_250_000,
        pose_us=1_248_500,
        pose_seq=42,
        v_cmd=0.18,
        yaw_rate_cmd=-0.35,
        height_cmd=0.145,
        roll_cmd=0.012,
        pose_x=2.34,
        pose_y=-0.89,
        pose_yaw=1.57,
    )

    # 1. Python encode -> C decode
    py_wire = encode_message_frame(py_msg, seq=101, timestamp_us=1_250_000, flags=1)
    c_hdr, c_payload = _c_decode_wire_and_stream(lib, py_wire)
    assert c_hdr.msg_type == int(UsbMsgType.CMD_TWIST)
    assert c_hdr.seq == 101
    assert c_hdr.timestamp_us == 1_250_000
    assert c_hdr.payload_len == MSG_CMD_TWIST_SIZE

    c_struct = CMsgCmdTwist()
    st = lib.msg_unpack_cmd_twist(_to_u8_array(c_payload), len(c_payload), byref(c_struct))
    assert st == int(LogStatus.OK)
    assert c_struct.session_id == py_msg.session_id
    assert c_struct.sent_us == py_msg.sent_us
    assert c_struct.pose_us == py_msg.pose_us
    assert c_struct.pose_seq == py_msg.pose_seq
    assert math.isclose(c_struct.v_cmd, py_msg.v_cmd, rel_tol=1e-6)
    assert math.isclose(c_struct.yaw_rate_cmd, py_msg.yaw_rate_cmd, rel_tol=1e-6)
    assert math.isclose(c_struct.height_cmd, py_msg.height_cmd, rel_tol=1e-6)
    assert math.isclose(c_struct.roll_cmd, py_msg.roll_cmd, rel_tol=1e-6)
    assert math.isclose(c_struct.pose_x, py_msg.pose_x, rel_tol=1e-6)
    assert math.isclose(c_struct.pose_y, py_msg.pose_y, rel_tol=1e-6)
    assert math.isclose(c_struct.pose_yaw, py_msg.pose_yaw, rel_tol=1e-6)

    # 2. C encode -> Python decode
    c_wire_buf = (c_uint8 * USB_LINK_MAX_WIRE_FRAME)()
    c_wire_len = c_size_t(0)
    st_f = lib.msg_frame_cmd_twist(
        202,
        2_500_000,
        0,
        byref(c_struct),
        c_wire_buf,
        USB_LINK_MAX_WIRE_FRAME,
        byref(c_wire_len),
    )
    assert st_f == int(LogStatus.OK)
    c_wire = bytes(c_wire_buf[: c_wire_len.value])

    py_hdr, py_decoded = decode_message_frame(c_wire)
    assert py_hdr.msg_type == int(UsbMsgType.CMD_TWIST)
    assert py_hdr.seq == 202
    assert py_hdr.timestamp_us == 2_500_000
    assert py_decoded.pack() == py_msg.pack()


def test_cross_cmd_mode_0x02() -> None:
    lib = load_c_ref_lib()
    py_msg = CmdMode(
        session_id=0xAABBCCDDEEFF0011,
        action=int(CmdModeAction.REQ_JUMP),
        arg=0x00001388,
    )

    # 1. Python encode -> C decode
    py_wire = encode_message_frame(py_msg, seq=12, timestamp_us=300_000, flags=0)
    c_hdr, c_payload = _c_decode_wire_and_stream(lib, py_wire)
    assert c_hdr.msg_type == int(UsbMsgType.CMD_MODE)
    assert c_hdr.payload_len == MSG_CMD_MODE_SIZE

    c_struct = CMsgCmdMode()
    st = lib.msg_unpack_cmd_mode(_to_u8_array(c_payload), len(c_payload), byref(c_struct))
    assert st == int(LogStatus.OK)
    assert c_struct.session_id == py_msg.session_id
    assert c_struct.action == py_msg.action
    assert c_struct.arg == py_msg.arg

    # 2. C encode -> Python decode
    c_wire_buf = (c_uint8 * USB_LINK_MAX_WIRE_FRAME)()
    c_wire_len = c_size_t(0)
    st_f = lib.msg_frame_cmd_mode(
        13,
        305_000,
        1,
        byref(c_struct),
        c_wire_buf,
        USB_LINK_MAX_WIRE_FRAME,
        byref(c_wire_len),
    )
    assert st_f == int(LogStatus.OK)
    c_wire = bytes(c_wire_buf[: c_wire_len.value])

    py_hdr, py_decoded = decode_message_frame(c_wire)
    assert py_hdr.msg_type == int(UsbMsgType.CMD_MODE)
    assert py_hdr.seq == 13
    assert py_decoded == py_msg


def test_cross_time_sync_req_0x03() -> None:
    lib = load_c_ref_lib()
    py_msg = TimeSyncReq(pi_t1_us=99_887_766_554_433)

    # 1. Python encode -> C decode
    py_wire = encode_message_frame(py_msg, seq=31, timestamp_us=99_887_766_554_433)
    c_hdr, c_payload = _c_decode_wire_and_stream(lib, py_wire)
    assert c_hdr.msg_type == int(UsbMsgType.TIME_SYNC_REQ)
    assert c_hdr.payload_len == MSG_TIME_SYNC_REQ_SIZE

    c_struct = CMsgTimeSyncReq()
    st = lib.msg_unpack_time_sync_req(_to_u8_array(c_payload), len(c_payload), byref(c_struct))
    assert st == int(LogStatus.OK)
    assert c_struct.pi_t1_us == py_msg.pi_t1_us

    # 2. C encode -> Python decode
    c_wire_buf = (c_uint8 * USB_LINK_MAX_WIRE_FRAME)()
    c_wire_len = c_size_t(0)
    st_f = lib.msg_frame_time_sync_req(
        32,
        99_887_766_554_433,
        0,
        byref(c_struct),
        c_wire_buf,
        USB_LINK_MAX_WIRE_FRAME,
        byref(c_wire_len),
    )
    assert st_f == int(LogStatus.OK)
    c_wire = bytes(c_wire_buf[: c_wire_len.value])

    py_hdr, py_decoded = decode_message_frame(c_wire)
    assert py_hdr.msg_type == int(UsbMsgType.TIME_SYNC_REQ)
    assert py_hdr.seq == 32
    assert py_decoded == py_msg


def test_cross_param_op_0x04() -> None:
    lib = load_c_ref_lib()
    py_msg = ParamOp(op=int(ParamOpCode.SET), param_id=0x0105, value=5.125)

    # 1. Python encode -> C decode
    py_wire = encode_message_frame(py_msg, seq=41, timestamp_us=410_000)
    c_hdr, c_payload = _c_decode_wire_and_stream(lib, py_wire)
    assert c_hdr.msg_type == int(UsbMsgType.PARAM_OP)
    assert c_hdr.payload_len == MSG_PARAM_OP_SIZE

    c_struct = CMsgParamOp()
    st = lib.msg_unpack_param_op(_to_u8_array(c_payload), len(c_payload), byref(c_struct))
    assert st == int(LogStatus.OK)
    assert c_struct.op == py_msg.op
    assert c_struct.param_id == py_msg.param_id
    assert math.isclose(c_struct.value, py_msg.value, rel_tol=1e-6)

    # 2. C encode -> Python decode
    c_wire_buf = (c_uint8 * USB_LINK_MAX_WIRE_FRAME)()
    c_wire_len = c_size_t(0)
    st_f = lib.msg_frame_param_op(
        42,
        420_000,
        0,
        byref(c_struct),
        c_wire_buf,
        USB_LINK_MAX_WIRE_FRAME,
        byref(c_wire_len),
    )
    assert st_f == int(LogStatus.OK)
    c_wire = bytes(c_wire_buf[: c_wire_len.value])

    py_hdr, py_decoded = decode_message_frame(c_wire)
    assert py_hdr.msg_type == int(UsbMsgType.PARAM_OP)
    assert py_hdr.seq == 42
    assert py_decoded.pack() == py_msg.pack()


def test_cross_telem_fast_0x81() -> None:
    lib = load_c_ref_lib()
    py_msg = TelemFast(
        fsm_state=4,
        s11_mode=2,
        s11_max_misses=1,
        flags=0x03,
        fault_mask=0x00000020,
        euler_rad=(0.025, -0.012, -0.785),
        gyro_rad_s=(-0.05, 0.14, -0.22),
        v_est_m_s=0.195,
        x_est_m=3.14,
        leg_length_m=(0.138, 0.139),
        leg_theta_rad=(-0.021, -0.019),
        motor_pos_rad=(0.45, -0.44, 25.1, -0.46, 0.43, 24.9),
        motor_vel_rad_s=(0.22, -0.21, 5.6, -0.23, 0.20, 5.5),
        motor_tor_cmd_nm=(2.4, -2.3, 0.85, -2.45, 2.35, 0.82),
        motor_tor_fb_nm=(2.38, -2.29, 0.84, -2.41, 2.33, 0.81),
        enc_age_us=(280, 295, 310, 285, 300, 312),
        s9_trim_rad=0.0085,
        gyro_bias_y_rad_s=-0.0021,
        v_bus_mv=23850,
        i_bus_ma=-5120,
    )

    # 1. Python encode -> C decode (usb_link_decode + log_unpack_telem_fast)
    py_wire = encode_message_frame(py_msg, seq=810, timestamp_us=8_100_000)
    c_hdr, c_payload = _c_decode_wire_and_stream(lib, py_wire)
    assert c_hdr.msg_type == int(UsbMsgType.TELEM_FAST)
    assert c_hdr.payload_len == LOG_TELEM_FAST_SIZE

    c_struct = CLogTelemFast()
    st = lib.log_unpack_telem_fast(_to_u8_array(c_payload), len(c_payload), byref(c_struct))
    assert st == int(LogStatus.OK)
    _assert_c_telem_fast_eq(c_struct, py_msg)

    # 2. C encode (log_frame_telem_fast) -> Python decode
    c_wire_buf = (c_uint8 * USB_LINK_MAX_WIRE_FRAME)()
    c_wire_len = c_size_t(0)
    st_f = lib.log_frame_telem_fast(
        811,
        8_105_000,
        0,
        byref(c_struct),
        c_wire_buf,
        USB_LINK_MAX_WIRE_FRAME,
        byref(c_wire_len),
    )
    assert st_f == int(LogStatus.OK)
    c_wire = bytes(c_wire_buf[: c_wire_len.value])

    # Verify byte-by-byte streaming decode in Python
    py_rx = StreamingDecoder()
    events = py_rx.feed_bytes(c_wire)
    assert len(events) == 1
    st_py, py_hdr, py_payload = events[0]
    assert st_py == UsbLinkStatus.OK
    assert py_hdr is not None and py_payload is not None
    assert py_hdr.msg_type == int(UsbMsgType.TELEM_FAST)
    assert py_hdr.seq == 811
    dec_fast = decode_telem_fast(py_payload)
    assert dec_fast.pack() == py_msg.pack()


def test_cross_telem_health_0x82() -> None:
    lib = load_c_ref_lib()
    py_msg = TelemHealth(
        can_tec=(1, 2, 0),
        can_rec=(0, 3, 1),
        can_busoff_cnt=(0, 0, 0),
        can_rx_drop=(2, 5, 1),
        motor_state=(1, 1, 1, 1, 1, 1),
        t_mos_c=(36, 37, 41, 35, 38, 42),
        t_coil_c=(44, 45, 51, 43, 46, 52),
        rc_rssi_dbm=-58,
        rc_lq_pct=99,
        rc_age_ms=4,
        rc_switches=0x0007,
        power_io_bits=0x007F,
        reset_cause=0x02,
        cpu_load_pct=19,
        min_stack_words=(290, 610, 210, 450, 240),
    )

    # 1. Python encode -> C decode (usb_link_decode + log_unpack_telem_health)
    py_wire = encode_message_frame(py_msg, seq=820, timestamp_us=8_200_000)
    c_hdr, c_payload = _c_decode_wire_and_stream(lib, py_wire)
    assert c_hdr.msg_type == int(UsbMsgType.TELEM_HEALTH)
    assert c_hdr.payload_len == LOG_TELEM_HEALTH_SIZE

    c_struct = CLogTelemHealth()
    st = lib.log_unpack_telem_health(_to_u8_array(c_payload), len(c_payload), byref(c_struct))
    assert st == int(LogStatus.OK)
    assert tuple(c_struct.can_tec) == py_msg.can_tec
    assert tuple(c_struct.can_rec) == py_msg.can_rec
    assert tuple(c_struct.can_busoff_cnt) == py_msg.can_busoff_cnt
    assert tuple(c_struct.can_rx_drop) == py_msg.can_rx_drop
    assert tuple(c_struct.motor_state) == py_msg.motor_state
    assert tuple(c_struct.t_mos_c) == py_msg.t_mos_c
    assert tuple(c_struct.t_coil_c) == py_msg.t_coil_c
    assert c_struct.rc_rssi_dbm == py_msg.rc_rssi_dbm
    assert c_struct.rc_lq_pct == py_msg.rc_lq_pct
    assert c_struct.rc_age_ms == py_msg.rc_age_ms
    assert c_struct.rc_switches == py_msg.rc_switches
    assert c_struct.power_io_bits == py_msg.power_io_bits
    assert c_struct.reset_cause == py_msg.reset_cause
    assert c_struct.cpu_load_pct == py_msg.cpu_load_pct
    assert tuple(c_struct.min_stack_words) == py_msg.min_stack_words

    # 2. C encode (log_frame_telem_health) -> Python decode
    c_wire_buf = (c_uint8 * USB_LINK_MAX_WIRE_FRAME)()
    c_wire_len = c_size_t(0)
    st_f = lib.log_frame_telem_health(
        821,
        8_250_000,
        0,
        byref(c_struct),
        c_wire_buf,
        USB_LINK_MAX_WIRE_FRAME,
        byref(c_wire_len),
    )
    assert st_f == int(LogStatus.OK)
    c_wire = bytes(c_wire_buf[: c_wire_len.value])

    py_hdr, py_decoded = decode_message_frame(c_wire)
    assert py_hdr.msg_type == int(UsbMsgType.TELEM_HEALTH)
    assert py_hdr.seq == 821
    assert py_decoded == py_msg
    assert decode_telem_health(py_decoded.pack()) == py_msg


def test_cross_time_sync_resp_0x83() -> None:
    lib = load_c_ref_lib()
    py_msg = TimeSyncResp(
        pi_t1_us=10_000_000,
        stm32_rx_us=10_000_350,
        stm32_tx_us=10_000_390,
        clock_err_bound_us=420,
    )

    # 1. Python encode -> C decode
    py_wire = encode_message_frame(py_msg, seq=830, timestamp_us=10_000_390)
    c_hdr, c_payload = _c_decode_wire_and_stream(lib, py_wire)
    assert c_hdr.msg_type == int(UsbMsgType.TIME_SYNC_RESP)
    assert c_hdr.payload_len == MSG_TIME_SYNC_RESP_SIZE

    c_struct = CMsgTimeSyncResp()
    st = lib.msg_unpack_time_sync_resp(_to_u8_array(c_payload), len(c_payload), byref(c_struct))
    assert st == int(LogStatus.OK)
    assert c_struct.pi_t1_us == py_msg.pi_t1_us
    assert c_struct.stm32_rx_us == py_msg.stm32_rx_us
    assert c_struct.stm32_tx_us == py_msg.stm32_tx_us
    assert c_struct.clock_err_bound_us == py_msg.clock_err_bound_us

    # 2. C encode -> Python decode
    c_wire_buf = (c_uint8 * USB_LINK_MAX_WIRE_FRAME)()
    c_wire_len = c_size_t(0)
    st_f = lib.msg_frame_time_sync_resp(
        831,
        10_500_390,
        0,
        byref(c_struct),
        c_wire_buf,
        USB_LINK_MAX_WIRE_FRAME,
        byref(c_wire_len),
    )
    assert st_f == int(LogStatus.OK)
    c_wire = bytes(c_wire_buf[: c_wire_len.value])

    py_hdr, py_decoded = decode_message_frame(c_wire)
    assert py_hdr.msg_type == int(UsbMsgType.TIME_SYNC_RESP)
    assert py_hdr.seq == 831
    assert py_decoded == py_msg


def test_cross_fault_snapshot_0x84_and_flight_record() -> None:
    lib = load_c_ref_lib()
    fast = TelemFast(
        fsm_state=6,
        s11_mode=3,
        s11_max_misses=4,
        flags=0x01,
        fault_mask=0x00000080,
        euler_rad=(0.53, 0.02, 0.15),
        gyro_rad_s=(1.2, -0.4, 0.05),
        v_est_m_s=0.04,
        x_est_m=0.88,
        leg_length_m=(0.140, 0.140),
        leg_theta_rad=(0.11, 0.10),
        motor_pos_rad=(0.2, -0.2, 8.1, -0.2, 0.2, 8.0),
        motor_vel_rad_s=(0.0, 0.0, 0.5, 0.0, 0.0, 0.5),
        motor_tor_cmd_nm=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        motor_tor_fb_nm=(0.1, -0.1, 0.05, -0.1, 0.1, 0.05),
        enc_age_us=(450, 460, 10200, 455, 465, 470),
        s9_trim_rad=0.011,
        gyro_bias_y_rad_s=-0.0015,
        v_bus_mv=23500,
        i_bus_ma=-200,
    )
    py_frec = FlightRecord(timestamp_us=55_000_000, seq=27500, fast=fast)
    py_snap = FaultSnapshot(
        trigger_reason=0x00000080,
        fault_timestamp_us=55_000_000,
        record_index=199,
        total_records=250,
        record=py_frec,
    )

    # A. Direct FlightRecord cross check (Python pack_flight_record <-> C log_pack/unpack_flight_record)
    py_frec_bytes = pack_flight_record(py_frec)
    assert len(py_frec_bytes) == LOG_FLIGHT_RECORD_SIZE
    c_frec = CLogFlightRecord()
    st_u = lib.log_unpack_flight_record(
        _to_u8_array(py_frec_bytes), len(py_frec_bytes), byref(c_frec)
    )
    assert st_u == int(LogStatus.OK)
    assert c_frec.timestamp_us == py_frec.timestamp_us
    assert c_frec.seq == py_frec.seq
    _assert_c_telem_fast_eq(c_frec.fast, fast)

    c_frec_buf = (c_uint8 * LOG_FLIGHT_RECORD_SIZE)()
    c_frec_len = c_size_t(0)
    st_p = lib.log_pack_flight_record(
        byref(c_frec), c_frec_buf, LOG_FLIGHT_RECORD_SIZE, byref(c_frec_len)
    )
    assert st_p == int(LogStatus.OK)
    assert c_frec_len.value == LOG_FLIGHT_RECORD_SIZE
    c_frec_bytes = bytes(c_frec_buf[: c_frec_len.value])
    assert c_frec_bytes == py_frec_bytes
    assert decode_flight_record(c_frec_bytes).pack() == py_frec_bytes

    # B. 0x84 FAULT_SNAPSHOT frame: Python encode -> C decode
    py_wire = encode_message_frame(py_snap, seq=840, timestamp_us=55_002_000)
    c_hdr, c_payload = _c_decode_wire_and_stream(lib, py_wire)
    assert c_hdr.msg_type == int(UsbMsgType.FAULT_SNAPSHOT)
    assert c_hdr.payload_len == MSG_FAULT_SNAPSHOT_SIZE

    c_snap = CMsgFaultSnapshot()
    st_su = lib.msg_unpack_fault_snapshot(
        _to_u8_array(c_payload), len(c_payload), byref(c_snap)
    )
    assert st_su == int(LogStatus.OK)
    assert c_snap.trigger_reason == py_snap.trigger_reason
    assert c_snap.fault_timestamp_us == py_snap.fault_timestamp_us
    assert c_snap.record_index == py_snap.record_index
    assert c_snap.total_records == py_snap.total_records
    assert c_snap.record.timestamp_us == py_frec.timestamp_us
    assert c_snap.record.seq == py_frec.seq
    _assert_c_telem_fast_eq(c_snap.record.fast, fast)

    # C. 0x84 FAULT_SNAPSHOT frame: C encode -> Python decode
    c_wire_buf = (c_uint8 * USB_LINK_MAX_WIRE_FRAME)()
    c_wire_len = c_size_t(0)
    st_sf = lib.msg_frame_fault_snapshot(
        841,
        55_004_000,
        0,
        byref(c_snap),
        c_wire_buf,
        USB_LINK_MAX_WIRE_FRAME,
        byref(c_wire_len),
    )
    assert st_sf == int(LogStatus.OK)
    c_wire = bytes(c_wire_buf[: c_wire_len.value])

    py_hdr, py_decoded = decode_message_frame(c_wire)
    assert py_hdr.msg_type == int(UsbMsgType.FAULT_SNAPSHOT)
    assert py_hdr.seq == 841
    assert py_decoded.pack() == py_snap.pack()

    # D. Cross-verify C LogFlightRing vs Python FlightRingBuffer
    cap = 6
    post_ticks = 2
    py_ring = FlightRingBuffer(capacity=cap, post_fault_ticks=post_ticks)
    c_storage = (CLogFlightRecord * cap)()
    c_ring = CLogFlightRing()
    assert lib.log_ring_init(byref(c_ring), c_storage, cap, post_ticks) == int(LogStatus.OK)

    for tick in range(1, 10):
        rec_py = FlightRecord(timestamp_us=tick * 2000, seq=tick, fast=fast)
        py_ring.push(rec_py)

        rec_c = CLogFlightRecord()
        rec_c.timestamp_us = tick * 2000
        rec_c.seq = tick
        rec_c.fast = _populate_c_telem_fast(fast)
        assert lib.log_ring_push(byref(c_ring), byref(rec_c)) == int(LogStatus.OK)

        if tick == 5:
            py_ring.trigger_fault()
            lib.log_ring_trigger_fault(byref(c_ring))

    assert py_ring.is_frozen == bool(lib.log_ring_is_frozen(byref(c_ring)))
    assert py_ring.count == lib.log_ring_count(byref(c_ring)) == 6

    for idx in range(py_ring.count):
        out_c = CLogFlightRecord()
        assert lib.log_ring_get(byref(c_ring), idx, byref(out_c)) == int(LogStatus.OK)
        out_py = py_ring.get(idx)
        assert out_c.seq == out_py.seq
        assert out_c.timestamp_us == out_py.timestamp_us


def test_cross_bench_raw_log_0x85() -> None:
    lib = load_c_ref_lib()
    py_msg = BenchRawLog(
        imu_ts_us=77_665_544,
        gyro_rad_s=(0.015, -0.025, 0.035),
        accel_m_s2=(-0.22, 0.11, 9.805),
        imu_temp_c=38.25,
        motor_pos_rad=(0.15, -0.15, 4.2, -0.16, 0.16, 4.1),
        motor_vel_rad_s=(0.55, -0.55, 2.1, -0.54, 0.54, 2.05),
        motor_rx_us=(77665100, 77665110, 77665120, 77665105, 77665115, 77665125),
        fresh_mask=0x3F,
        reserved=(0, 0, 0),
        pitch_base_rad=0.019,
        v_base_m_s=0.145,
        pitch_s10_rad=0.020,
        v_s10_m_s=0.148,
    )

    # 1. Python encode -> C decode (usb_link_decode + log_unpack_bench_raw)
    py_wire = encode_message_frame(py_msg, seq=850, timestamp_us=77_665_544, flags=2)
    c_hdr, c_payload = _c_decode_wire_and_stream(lib, py_wire)
    assert c_hdr.msg_type == int(UsbMsgType.BENCH_RAW_LOG)
    assert c_hdr.flags == 2
    assert c_hdr.payload_len == LOG_BENCH_RAW_SIZE

    c_struct = CLogBenchRaw()
    st = lib.log_unpack_bench_raw(_to_u8_array(c_payload), len(c_payload), byref(c_struct))
    assert st == int(LogStatus.OK)
    assert c_struct.imu_ts_us == py_msg.imu_ts_us
    for i in range(3):
        assert math.isclose(c_struct.gyro_rad_s[i], py_msg.gyro_rad_s[i], rel_tol=1e-6)
        assert math.isclose(c_struct.accel_m_s2[i], py_msg.accel_m_s2[i], rel_tol=1e-6)
    assert math.isclose(c_struct.imu_temp_c, py_msg.imu_temp_c, rel_tol=1e-6)
    for i in range(LOG_NUM_AXES):
        assert math.isclose(c_struct.motor_pos_rad[i], py_msg.motor_pos_rad[i], rel_tol=1e-6)
        assert math.isclose(c_struct.motor_vel_rad_s[i], py_msg.motor_vel_rad_s[i], rel_tol=1e-6)
        assert c_struct.motor_rx_us[i] == py_msg.motor_rx_us[i]
    assert c_struct.fresh_mask == py_msg.fresh_mask
    assert math.isclose(c_struct.pitch_base_rad, py_msg.pitch_base_rad, rel_tol=1e-6)
    assert math.isclose(c_struct.v_base_m_s, py_msg.v_base_m_s, rel_tol=1e-6)
    assert math.isclose(c_struct.pitch_s10_rad, py_msg.pitch_s10_rad, rel_tol=1e-6)
    assert math.isclose(c_struct.v_s10_m_s, py_msg.v_s10_m_s, rel_tol=1e-6)

    # 2. C encode (log_frame_bench_raw) -> Python decode
    c_wire_buf = (c_uint8 * USB_LINK_MAX_WIRE_FRAME)()
    c_wire_len = c_size_t(0)
    st_f = lib.log_frame_bench_raw(
        851,
        77_667_544,
        2,
        byref(c_struct),
        c_wire_buf,
        USB_LINK_MAX_WIRE_FRAME,
        byref(c_wire_len),
    )
    assert st_f == int(LogStatus.OK)
    c_wire = bytes(c_wire_buf[: c_wire_len.value])

    py_hdr, py_decoded = decode_message_frame(c_wire)
    assert py_hdr.msg_type == int(UsbMsgType.BENCH_RAW_LOG)
    assert py_hdr.seq == 851
    assert py_hdr.flags == 2
    assert py_decoded.pack() == py_msg.pack()
    assert decode_bench_raw(py_decoded.pack()).pack() == py_msg.pack()


def test_cross_corrupted_and_error_parity() -> None:
    lib = load_c_ref_lib()
    msg = TimeSyncReq(pi_t1_us=12345678)
    valid_wire = encode_message_frame(msg, seq=1, timestamp_us=1000)

    # Bit flip -> both C and Python return ERR_CRC
    bad_crc = bytearray(valid_wire)
    bad_crc[4] ^= 0x04
    if bad_crc[4] == 0x00:
        bad_crc[4] = 0x01
    st_py, _, _ = usb_link_decode(bad_crc)
    c_hdr = CUsbLinkHeader()
    c_pay = (c_uint8 * USB_LINK_MAX_PAYLOAD)()
    st_c = lib.usb_link_decode(
        _to_u8_array(bytes(bad_crc)),
        len(bad_crc),
        byref(c_hdr),
        c_pay,
        USB_LINK_MAX_PAYLOAD,
    )
    assert st_py == UsbLinkStatus.ERR_CRC
    assert st_c == int(UsbLinkStatus.ERR_CRC)

    # Bad major version -> both C and Python return ERR_VERSION
    bad_ver = encode_message_frame(msg, seq=2, timestamp_us=2000, ver_major=9)
    st_py_v, _, _ = usb_link_decode(bad_ver)
    st_c_v = lib.usb_link_decode(
        _to_u8_array(bad_ver),
        len(bad_ver),
        byref(c_hdr),
        c_pay,
        USB_LINK_MAX_PAYLOAD,
    )
    assert st_py_v == UsbLinkStatus.ERR_VERSION
    assert st_c_v == int(UsbLinkStatus.ERR_VERSION)


CROSS_TESTS = [
    ("test_cross_crc32c_and_cobs", test_cross_crc32c_and_cobs),
    ("test_cross_cmd_twist_0x01", test_cross_cmd_twist_0x01),
    ("test_cross_cmd_mode_0x02", test_cross_cmd_mode_0x02),
    ("test_cross_time_sync_req_0x03", test_cross_time_sync_req_0x03),
    ("test_cross_param_op_0x04", test_cross_param_op_0x04),
    ("test_cross_telem_fast_0x81", test_cross_telem_fast_0x81),
    ("test_cross_telem_health_0x82", test_cross_telem_health_0x82),
    ("test_cross_time_sync_resp_0x83", test_cross_time_sync_resp_0x83),
    ("test_cross_fault_snapshot_0x84_and_flight_record", test_cross_fault_snapshot_0x84_and_flight_record),
    ("test_cross_bench_raw_log_0x85", test_cross_bench_raw_log_0x85),
    ("test_cross_corrupted_and_error_parity", test_cross_corrupted_and_error_parity),
]
