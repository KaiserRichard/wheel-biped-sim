"""Full message catalog, rates, payload codecs, and flight recorder from FIRMWARE_DESIGN_v2.md §7.3 & c_ref/log_packer.h."""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum
import struct
from typing import Any

from .protocol import (
    USB_LINK_VER_MAJOR,
    USB_LINK_VER_MINOR,
    UsbLinkHeader,
    UsbMsgType,
    decode_frame,
    encode_frame,
)

LOG_NUM_AXES: int = 6
LOG_NUM_CAN_BUSES: int = 3
LOG_NUM_RTOS_TASKS: int = 5

MSG_CMD_TWIST_SIZE: int = 56
MSG_CMD_TWIST_SHORT_SIZE: int = 48
MSG_CMD_MODE_SIZE: int = 13
MSG_TIME_SYNC_REQ_SIZE: int = 8
MSG_PARAM_OP_SIZE: int = 7
LOG_TELEM_FAST_SIZE: int = 176
LOG_TELEM_HEALTH_SIZE: int = 56
MSG_TIME_SYNC_RESP_SIZE: int = 28
LOG_FLIGHT_RECORD_SIZE: int = 12 + LOG_TELEM_FAST_SIZE  # 188 bytes
MSG_FAULT_SNAPSHOT_SIZE: int = 16 + LOG_FLIGHT_RECORD_SIZE  # 204 bytes
LOG_BENCH_RAW_SIZE: int = 128

DEFAULT_FLIGHT_RING_CAPACITY: int = 250  # 0.50 s at 500 Hz
DEFAULT_POST_FAULT_TICKS: int = 50       # 0.10 s at 500 Hz


class Direction(IntEnum):
    PI_TO_STM32 = 1
    STM32_TO_PI = 2


class CmdModeAction(IntEnum):
    NOP = 0
    REQ_BALANCE = 1
    REQ_JUMP = 2
    REQ_STOP = 3
    RESET_FAULT = 4


class ParamOpCode(IntEnum):
    GET = 0
    SET = 1
    ACK = 2
    NACK = 3


class LogStatus(IntEnum):
    OK = 0
    ERR_NULL_PTR = 1
    ERR_BUFFER_TOO_SMALL = 2
    ERR_BAD_LENGTH = 3
    ERR_OUT_OF_RANGE = 4
    ERR_LINK_ENCODE = 5


_CMD_TWIST_STRUCT = struct.Struct("<QQQIfffffff")
_CMD_TWIST_52_STRUCT = struct.Struct("<QQIfffffff")
_CMD_MODE_STRUCT = struct.Struct("<QBI")
_TIME_SYNC_REQ_STRUCT = struct.Struct("<Q")
_PARAM_OP_STRUCT = struct.Struct("<BHf")
_TELEM_FAST_STRUCT = struct.Struct("<BBBBI3f3fff2f2f6f6f6f6f6HffHh")
_TELEM_HEALTH_STRUCT = struct.Struct("<3B3B3H3H6B6B6BbBHHHBB5H")
_TIME_SYNC_RESP_STRUCT = struct.Struct("<QQQI")
_BENCH_RAW_STRUCT = struct.Struct("<Q3f3ff6f6f6IB3Bffff")
_FLIGHT_REC_PREFIX_STRUCT = struct.Struct("<QI")
_FAULT_SNAPSHOT_PREFIX_STRUCT = struct.Struct("<IQHH")


@dataclass(frozen=True)
class CmdTwist:
    """0x01 CMD_TWIST (Pi -> STM32, 25 Hz nominal, <=50 Hz max ceiling)."""

    session_id: int
    sent_us: int
    pose_us: int
    pose_seq: int
    v_cmd: float
    yaw_rate_cmd: float
    height_cmd: float = 0.14
    roll_cmd: float = 0.0
    pose_x: float = 0.0
    pose_y: float = 0.0
    pose_yaw: float = 0.0

    MSG_TYPE = UsbMsgType.CMD_TWIST

    def pack(self) -> bytes:
        return _CMD_TWIST_STRUCT.pack(
            self.session_id,
            self.sent_us,
            self.pose_us,
            self.pose_seq,
            self.v_cmd,
            self.yaw_rate_cmd,
            self.height_cmd,
            self.roll_cmd,
            self.pose_x,
            self.pose_y,
            self.pose_yaw,
        )

    @classmethod
    def unpack(
        cls,
        src: bytes | bytearray | memoryview,
        *,
        header_timestamp_us: int = 0,
    ) -> CmdTwist:
        mv = memoryview(src)
        if len(mv) == MSG_CMD_TWIST_SIZE:
            (
                session_id,
                sent_us,
                pose_us,
                pose_seq,
                v_cmd,
                yaw_rate_cmd,
                height_cmd,
                roll_cmd,
                pose_x,
                pose_y,
                pose_yaw,
            ) = _CMD_TWIST_STRUCT.unpack(mv)
            return cls(
                session_id=session_id,
                sent_us=sent_us,
                pose_us=pose_us,
                pose_seq=pose_seq,
                v_cmd=v_cmd,
                yaw_rate_cmd=yaw_rate_cmd,
                height_cmd=height_cmd,
                roll_cmd=roll_cmd,
                pose_x=pose_x,
                pose_y=pose_y,
                pose_yaw=pose_yaw,
            )
        if len(mv) == MSG_CMD_TWIST_SHORT_SIZE:
            (
                session_id,
                pose_us,
                pose_seq,
                v_cmd,
                yaw_rate_cmd,
                height_cmd,
                roll_cmd,
                pose_x,
                pose_y,
                pose_yaw,
            ) = _CMD_TWIST_52_STRUCT.unpack(mv)
            return cls(
                session_id=session_id,
                sent_us=header_timestamp_us,
                pose_us=pose_us,
                pose_seq=pose_seq,
                v_cmd=v_cmd,
                yaw_rate_cmd=yaw_rate_cmd,
                height_cmd=height_cmd,
                roll_cmd=roll_cmd,
                pose_x=pose_x,
                pose_y=pose_y,
                pose_yaw=pose_yaw,
            )
        raise ValueError(f"CMD_TWIST bad length: {len(mv)} (expected {MSG_CMD_TWIST_SIZE})")


@dataclass(frozen=True)
class CmdMode:
    """0x02 CMD_MODE (Pi -> STM32, Event / 2 Hz)."""

    session_id: int
    action: int = int(CmdModeAction.NOP)
    arg: int = 0

    MSG_TYPE = UsbMsgType.CMD_MODE

    def pack(self) -> bytes:
        return _CMD_MODE_STRUCT.pack(self.session_id, int(self.action), self.arg)

    @classmethod
    def unpack(cls, src: bytes | bytearray | memoryview) -> CmdMode:
        if len(src) != MSG_CMD_MODE_SIZE:
            raise ValueError(f"CMD_MODE bad length: {len(src)} (expected {MSG_CMD_MODE_SIZE})")
        session_id, action, arg = _CMD_MODE_STRUCT.unpack(src)
        return cls(session_id=session_id, action=action, arg=arg)


@dataclass(frozen=True)
class TimeSyncReq:
    """0x03 TIME_SYNC_REQ (Pi -> STM32, 1-2 Hz)."""

    pi_t1_us: int

    MSG_TYPE = UsbMsgType.TIME_SYNC_REQ

    def pack(self) -> bytes:
        return _TIME_SYNC_REQ_STRUCT.pack(self.pi_t1_us)

    @classmethod
    def unpack(cls, src: bytes | bytearray | memoryview) -> TimeSyncReq:
        if len(src) != MSG_TIME_SYNC_REQ_SIZE:
            raise ValueError(
                f"TIME_SYNC_REQ bad length: {len(src)} (expected {MSG_TIME_SYNC_REQ_SIZE})"
            )
        (pi_t1_us,) = _TIME_SYNC_REQ_STRUCT.unpack(src)
        return cls(pi_t1_us=pi_t1_us)


@dataclass(frozen=True)
class ParamOp:
    """0x04 PARAM_OP (Pi -> STM32, On demand when DISARMED only)."""

    op: int
    param_id: int
    value: float

    MSG_TYPE = UsbMsgType.PARAM_OP

    def pack(self) -> bytes:
        return _PARAM_OP_STRUCT.pack(int(self.op), self.param_id, self.value)

    @classmethod
    def unpack(cls, src: bytes | bytearray | memoryview) -> ParamOp:
        if len(src) != MSG_PARAM_OP_SIZE:
            raise ValueError(f"PARAM_OP bad length: {len(src)} (expected {MSG_PARAM_OP_SIZE})")
        op, param_id, value = _PARAM_OP_STRUCT.unpack(src)
        return cls(op=op, param_id=param_id, value=value)


@dataclass(frozen=True)
class TelemFast:
    """0x81 TELEM_FAST (STM32 -> Pi, 200 Hz, 176-byte wire payload matching LogTelemFast)."""

    fsm_state: int = 0
    s11_mode: int = 0
    s11_max_misses: int = 0
    flags: int = 0
    fault_mask: int = 0
    euler_rad: tuple[float, float, float] = (0.0, 0.0, 0.0)
    gyro_rad_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    v_est_m_s: float = 0.0
    x_est_m: float = 0.0
    leg_length_m: tuple[float, float] = (0.14, 0.14)
    leg_theta_rad: tuple[float, float] = (0.0, 0.0)
    motor_pos_rad: tuple[float, float, float, float, float, float] = (0.0,) * 6
    motor_vel_rad_s: tuple[float, float, float, float, float, float] = (0.0,) * 6
    motor_tor_cmd_nm: tuple[float, float, float, float, float, float] = (0.0,) * 6
    motor_tor_fb_nm: tuple[float, float, float, float, float, float] = (0.0,) * 6
    enc_age_us: tuple[int, int, int, int, int, int] = (0,) * 6
    s9_trim_rad: float = 0.0
    gyro_bias_y_rad_s: float = 0.0
    v_bus_mv: int = 24000
    i_bus_ma: int = 0

    MSG_TYPE = UsbMsgType.TELEM_FAST

    def __post_init__(self) -> None:
        object.__setattr__(self, "euler_rad", tuple(float(x) for x in self.euler_rad))
        object.__setattr__(self, "gyro_rad_s", tuple(float(x) for x in self.gyro_rad_s))
        object.__setattr__(self, "leg_length_m", tuple(float(x) for x in self.leg_length_m))
        object.__setattr__(self, "leg_theta_rad", tuple(float(x) for x in self.leg_theta_rad))
        object.__setattr__(self, "motor_pos_rad", tuple(float(x) for x in self.motor_pos_rad))
        object.__setattr__(self, "motor_vel_rad_s", tuple(float(x) for x in self.motor_vel_rad_s))
        object.__setattr__(self, "motor_tor_cmd_nm", tuple(float(x) for x in self.motor_tor_cmd_nm))
        object.__setattr__(self, "motor_tor_fb_nm", tuple(float(x) for x in self.motor_tor_fb_nm))
        object.__setattr__(self, "enc_age_us", tuple(int(x) for x in self.enc_age_us))
        if (
            len(self.euler_rad) != 3
            or len(self.gyro_rad_s) != 3
            or len(self.leg_length_m) != 2
            or len(self.leg_theta_rad) != 2
            or len(self.motor_pos_rad) != LOG_NUM_AXES
            or len(self.motor_vel_rad_s) != LOG_NUM_AXES
            or len(self.motor_tor_cmd_nm) != LOG_NUM_AXES
            or len(self.motor_tor_fb_nm) != LOG_NUM_AXES
            or len(self.enc_age_us) != LOG_NUM_AXES
        ):
            raise ValueError("TelemFast array field lengths do not match specification")

    @property
    def pitch_rad(self) -> float:
        return self.euler_rad[0]

    @property
    def roll_rad(self) -> float:
        return self.euler_rad[1]

    @property
    def yaw_rad(self) -> float:
        return self.euler_rad[2]

    @property
    def v_bus_v(self) -> float:
        return self.v_bus_mv * 1e-3

    @property
    def i_bus_a(self) -> float:
        return self.i_bus_ma * 1e-3

    def pack(self) -> bytes:
        return _TELEM_FAST_STRUCT.pack(
            self.fsm_state,
            self.s11_mode,
            self.s11_max_misses,
            self.flags,
            self.fault_mask,
            *self.euler_rad,
            *self.gyro_rad_s,
            self.v_est_m_s,
            self.x_est_m,
            *self.leg_length_m,
            *self.leg_theta_rad,
            *self.motor_pos_rad,
            *self.motor_vel_rad_s,
            *self.motor_tor_cmd_nm,
            *self.motor_tor_fb_nm,
            *self.enc_age_us,
            self.s9_trim_rad,
            self.gyro_bias_y_rad_s,
            self.v_bus_mv,
            self.i_bus_ma,
        )

    @classmethod
    def unpack(cls, src: bytes | bytearray | memoryview) -> TelemFast:
        if len(src) != LOG_TELEM_FAST_SIZE:
            raise ValueError(
                f"TELEM_FAST bad length: {len(src)} (expected {LOG_TELEM_FAST_SIZE})"
            )
        vals = _TELEM_FAST_STRUCT.unpack(src)
        return cls(
            fsm_state=vals[0],
            s11_mode=vals[1],
            s11_max_misses=vals[2],
            flags=vals[3],
            fault_mask=vals[4],
            euler_rad=vals[5:8],
            gyro_rad_s=vals[8:11],
            v_est_m_s=vals[11],
            x_est_m=vals[12],
            leg_length_m=vals[13:15],
            leg_theta_rad=vals[15:17],
            motor_pos_rad=vals[17:23],
            motor_vel_rad_s=vals[23:29],
            motor_tor_cmd_nm=vals[29:35],
            motor_tor_fb_nm=vals[35:41],
            enc_age_us=vals[41:47],
            s9_trim_rad=vals[47],
            gyro_bias_y_rad_s=vals[48],
            v_bus_mv=vals[49],
            i_bus_ma=vals[50],
        )


@dataclass(frozen=True)
class TelemHealth:
    """0x82 TELEM_HEALTH (STM32 -> Pi, 20 Hz, 56-byte wire payload matching LogTelemHealth)."""

    can_tec: tuple[int, int, int] = (0, 0, 0)
    can_rec: tuple[int, int, int] = (0, 0, 0)
    can_busoff_cnt: tuple[int, int, int] = (0, 0, 0)
    can_rx_drop: tuple[int, int, int] = (0, 0, 0)
    motor_state: tuple[int, int, int, int, int, int] = (1,) * 6
    t_mos_c: tuple[int, int, int, int, int, int] = (25,) * 6
    t_coil_c: tuple[int, int, int, int, int, int] = (25,) * 6
    rc_rssi_dbm: int = -50
    rc_lq_pct: int = 100
    rc_age_ms: int = 4
    rc_switches: int = 0
    power_io_bits: int = 0
    reset_cause: int = 0
    cpu_load_pct: int = 16
    min_stack_words: tuple[int, int, int, int, int] = (256, 512, 192, 384, 192)

    MSG_TYPE = UsbMsgType.TELEM_HEALTH

    def __post_init__(self) -> None:
        object.__setattr__(self, "can_tec", tuple(int(x) for x in self.can_tec))
        object.__setattr__(self, "can_rec", tuple(int(x) for x in self.can_rec))
        object.__setattr__(self, "can_busoff_cnt", tuple(int(x) for x in self.can_busoff_cnt))
        object.__setattr__(self, "can_rx_drop", tuple(int(x) for x in self.can_rx_drop))
        object.__setattr__(self, "motor_state", tuple(int(x) for x in self.motor_state))
        object.__setattr__(self, "t_mos_c", tuple(int(x) for x in self.t_mos_c))
        object.__setattr__(self, "t_coil_c", tuple(int(x) for x in self.t_coil_c))
        object.__setattr__(self, "min_stack_words", tuple(int(x) for x in self.min_stack_words))
        if (
            len(self.can_tec) != LOG_NUM_CAN_BUSES
            or len(self.can_rec) != LOG_NUM_CAN_BUSES
            or len(self.can_busoff_cnt) != LOG_NUM_CAN_BUSES
            or len(self.can_rx_drop) != LOG_NUM_CAN_BUSES
            or len(self.motor_state) != LOG_NUM_AXES
            or len(self.t_mos_c) != LOG_NUM_AXES
            or len(self.t_coil_c) != LOG_NUM_AXES
            or len(self.min_stack_words) != LOG_NUM_RTOS_TASKS
        ):
            raise ValueError("TelemHealth array field lengths do not match specification")

    def pack(self) -> bytes:
        return _TELEM_HEALTH_STRUCT.pack(
            *self.can_tec,
            *self.can_rec,
            *self.can_busoff_cnt,
            *self.can_rx_drop,
            *self.motor_state,
            *self.t_mos_c,
            *self.t_coil_c,
            self.rc_rssi_dbm,
            self.rc_lq_pct,
            self.rc_age_ms,
            self.rc_switches,
            self.power_io_bits,
            self.reset_cause,
            self.cpu_load_pct,
            *self.min_stack_words,
        )

    @classmethod
    def unpack(cls, src: bytes | bytearray | memoryview) -> TelemHealth:
        if len(src) != LOG_TELEM_HEALTH_SIZE:
            raise ValueError(
                f"TELEM_HEALTH bad length: {len(src)} (expected {LOG_TELEM_HEALTH_SIZE})"
            )
        vals = _TELEM_HEALTH_STRUCT.unpack(src)
        return cls(
            can_tec=vals[0:3],
            can_rec=vals[3:6],
            can_busoff_cnt=vals[6:9],
            can_rx_drop=vals[9:12],
            motor_state=vals[12:18],
            t_mos_c=vals[18:24],
            t_coil_c=vals[24:30],
            rc_rssi_dbm=vals[30],
            rc_lq_pct=vals[31],
            rc_age_ms=vals[32],
            rc_switches=vals[33],
            power_io_bits=vals[34],
            reset_cause=vals[35],
            cpu_load_pct=vals[36],
            min_stack_words=vals[37:42],
        )


@dataclass(frozen=True)
class TimeSyncResp:
    """0x83 TIME_SYNC_RESP (STM32 -> Pi, 1-2 Hz, 28-byte payload)."""

    pi_t1_us: int
    stm32_rx_us: int
    stm32_tx_us: int
    clock_err_bound_us: int

    MSG_TYPE = UsbMsgType.TIME_SYNC_RESP

    def pack(self) -> bytes:
        return _TIME_SYNC_RESP_STRUCT.pack(
            self.pi_t1_us,
            self.stm32_rx_us,
            self.stm32_tx_us,
            self.clock_err_bound_us,
        )

    @classmethod
    def unpack(cls, src: bytes | bytearray | memoryview) -> TimeSyncResp:
        if len(src) != MSG_TIME_SYNC_RESP_SIZE:
            raise ValueError(
                f"TIME_SYNC_RESP bad length: {len(src)} (expected {MSG_TIME_SYNC_RESP_SIZE})"
            )
        pi_t1_us, stm32_rx_us, stm32_tx_us, clock_err_bound_us = (
            _TIME_SYNC_RESP_STRUCT.unpack(src)
        )
        return cls(
            pi_t1_us=pi_t1_us,
            stm32_rx_us=stm32_rx_us,
            stm32_tx_us=stm32_tx_us,
            clock_err_bound_us=clock_err_bound_us,
        )

    def estimate_offset_and_rtt_us(self, pi_t4_us: int) -> tuple[int, int, int]:
        """Compute NTP 4-timestamp (offset_us, rtt_us, half_rtt_bound_us) given Pi RX time pi_t4_us."""
        rtt_us = (pi_t4_us - self.pi_t1_us) - (self.stm32_tx_us - self.stm32_rx_us)
        offset_us = ((self.stm32_rx_us - self.pi_t1_us) + (self.stm32_tx_us - pi_t4_us)) // 2
        half_rtt_bound = max(0, (rtt_us + 1) // 2)
        return offset_us, rtt_us, max(self.clock_err_bound_us, half_rtt_bound)


@dataclass(frozen=True)
class FlightRecord:
    """188-byte flight-recorder record matching LogFlightRecord in c_ref/log_packer.h."""

    timestamp_us: int
    seq: int
    fast: TelemFast = field(default_factory=TelemFast)

    def pack(self) -> bytes:
        return _FLIGHT_REC_PREFIX_STRUCT.pack(self.timestamp_us, self.seq) + self.fast.pack()

    @classmethod
    def unpack(cls, src: bytes | bytearray | memoryview) -> FlightRecord:
        if len(src) != LOG_FLIGHT_RECORD_SIZE:
            raise ValueError(
                f"FlightRecord bad length: {len(src)} (expected {LOG_FLIGHT_RECORD_SIZE})"
            )
        mv = memoryview(src)
        timestamp_us, seq = _FLIGHT_REC_PREFIX_STRUCT.unpack_from(mv, 0)
        fast = TelemFast.unpack(mv[12:])
        return cls(timestamp_us=timestamp_us, seq=seq, fast=fast)


@dataclass(frozen=True)
class FaultSnapshot:
    """0x84 FAULT_SNAPSHOT (STM32 -> Pi, sent on FAULT/ESTOP)."""

    trigger_reason: int
    fault_timestamp_us: int
    record_index: int = 0
    total_records: int = 1
    record: FlightRecord = field(
        default_factory=lambda: FlightRecord(timestamp_us=0, seq=0, fast=TelemFast())
    )

    MSG_TYPE = UsbMsgType.FAULT_SNAPSHOT

    def pack(self) -> bytes:
        return (
            _FAULT_SNAPSHOT_PREFIX_STRUCT.pack(
                self.trigger_reason,
                self.fault_timestamp_us,
                self.record_index,
                self.total_records,
            )
            + self.record.pack()
        )

    @classmethod
    def unpack(cls, src: bytes | bytearray | memoryview) -> FaultSnapshot:
        mv = memoryview(src)
        if len(mv) == LOG_FLIGHT_RECORD_SIZE:
            rec = FlightRecord.unpack(mv)
            return cls(
                trigger_reason=rec.fast.fault_mask,
                fault_timestamp_us=rec.timestamp_us,
                record_index=0,
                total_records=1,
                record=rec,
            )
        if len(mv) != MSG_FAULT_SNAPSHOT_SIZE:
            raise ValueError(
                f"FAULT_SNAPSHOT bad length: {len(mv)} (expected {MSG_FAULT_SNAPSHOT_SIZE} or {LOG_FLIGHT_RECORD_SIZE})"
            )
        trigger_reason, fault_timestamp_us, record_index, total_records = (
            _FAULT_SNAPSHOT_PREFIX_STRUCT.unpack_from(mv, 0)
        )
        rec = FlightRecord.unpack(mv[16:])
        return cls(
            trigger_reason=trigger_reason,
            fault_timestamp_us=fault_timestamp_us,
            record_index=record_index,
            total_records=total_records,
            record=rec,
        )


@dataclass(frozen=True)
class BenchRawLog:
    """0x85 BENCH_RAW_LOG (STM32 -> Pi, 500 Hz bench mode, 128-byte payload matching LogBenchRaw)."""

    imu_ts_us: int = 0
    gyro_rad_s: tuple[float, float, float] = (0.0, 0.0, 0.0)
    accel_m_s2: tuple[float, float, float] = (0.0, 0.0, 9.81)
    imu_temp_c: float = 30.0
    motor_pos_rad: tuple[float, float, float, float, float, float] = (0.0,) * 6
    motor_vel_rad_s: tuple[float, float, float, float, float, float] = (0.0,) * 6
    motor_rx_us: tuple[int, int, int, int, int, int] = (0,) * 6
    fresh_mask: int = 0x3F
    reserved: tuple[int, int, int] = (0, 0, 0)
    pitch_base_rad: float = 0.0
    v_base_m_s: float = 0.0
    pitch_s10_rad: float = 0.0
    v_s10_m_s: float = 0.0

    MSG_TYPE = UsbMsgType.BENCH_RAW_LOG

    def __post_init__(self) -> None:
        object.__setattr__(self, "gyro_rad_s", tuple(float(x) for x in self.gyro_rad_s))
        object.__setattr__(self, "accel_m_s2", tuple(float(x) for x in self.accel_m_s2))
        object.__setattr__(self, "motor_pos_rad", tuple(float(x) for x in self.motor_pos_rad))
        object.__setattr__(self, "motor_vel_rad_s", tuple(float(x) for x in self.motor_vel_rad_s))
        object.__setattr__(self, "motor_rx_us", tuple(int(x) for x in self.motor_rx_us))
        object.__setattr__(self, "reserved", tuple(int(x) for x in self.reserved))
        if (
            len(self.gyro_rad_s) != 3
            or len(self.accel_m_s2) != 3
            or len(self.motor_pos_rad) != LOG_NUM_AXES
            or len(self.motor_vel_rad_s) != LOG_NUM_AXES
            or len(self.motor_rx_us) != LOG_NUM_AXES
            or len(self.reserved) != 3
        ):
            raise ValueError("BenchRawLog array field lengths do not match specification")

    def pack(self) -> bytes:
        return _BENCH_RAW_STRUCT.pack(
            self.imu_ts_us,
            *self.gyro_rad_s,
            *self.accel_m_s2,
            self.imu_temp_c,
            *self.motor_pos_rad,
            *self.motor_vel_rad_s,
            *self.motor_rx_us,
            self.fresh_mask,
            *self.reserved,
            self.pitch_base_rad,
            self.v_base_m_s,
            self.pitch_s10_rad,
            self.v_s10_m_s,
        )

    @classmethod
    def unpack(cls, src: bytes | bytearray | memoryview) -> BenchRawLog:
        if len(src) != LOG_BENCH_RAW_SIZE:
            raise ValueError(
                f"BENCH_RAW_LOG bad length: {len(src)} (expected {LOG_BENCH_RAW_SIZE})"
            )
        vals = _BENCH_RAW_STRUCT.unpack(src)
        return cls(
            imu_ts_us=vals[0],
            gyro_rad_s=vals[1:4],
            accel_m_s2=vals[4:7],
            imu_temp_c=vals[7],
            motor_pos_rad=vals[8:14],
            motor_vel_rad_s=vals[14:20],
            motor_rx_us=vals[20:26],
            fresh_mask=vals[26],
            reserved=vals[27:30],
            pitch_base_rad=vals[30],
            v_base_m_s=vals[31],
            pitch_s10_rad=vals[32],
            v_s10_m_s=vals[33],
        )


class FlightRingBuffer:
    """Pre-fault / post-fault circular flight recorder matching LogFlightRing in c_ref/log_packer.c."""

    def __init__(
        self,
        capacity: int = DEFAULT_FLIGHT_RING_CAPACITY,
        post_fault_ticks: int = DEFAULT_POST_FAULT_TICKS,
    ) -> None:
        if capacity <= 0:
            raise ValueError("capacity must be > 0")
        if post_fault_ticks < 0:
            raise ValueError("post_fault_ticks must be >= 0")
        self.capacity: int = capacity
        self.post_fault_ticks: int = post_fault_ticks
        self._storage: list[FlightRecord | None] = [None] * capacity
        self.head: int = 0
        self.count: int = 0
        self.post_fault_remaining: int = 0
        self.fault_triggered: bool = False
        self.frozen: bool = False

    def push(self, rec: FlightRecord) -> None:
        if self.frozen:
            return
        self._storage[self.head] = rec
        self.head = (self.head + 1) % self.capacity
        if self.count < self.capacity:
            self.count += 1
        if self.fault_triggered:
            if self.post_fault_remaining > 0:
                self.post_fault_remaining -= 1
            if self.post_fault_remaining == 0:
                self.frozen = True

    def trigger_fault(self) -> None:
        if self.fault_triggered:
            return
        self.fault_triggered = True
        self.post_fault_remaining = self.post_fault_ticks
        if self.post_fault_remaining == 0:
            self.frozen = True

    @property
    def is_frozen(self) -> bool:
        return self.frozen

    def get(self, logical_idx: int) -> FlightRecord:
        if not (0 <= logical_idx < self.count):
            raise IndexError(f"logical_idx {logical_idx} out of range (count={self.count})")
        oldest = self.head if (self.count == self.capacity) else 0
        phys_idx = (oldest + logical_idx) % self.capacity
        rec = self._storage[phys_idx]
        assert rec is not None
        return rec

    def records(self) -> list[FlightRecord]:
        return [self.get(i) for i in range(self.count)]

    def pack_dump(self) -> bytes:
        return b"".join(rec.pack() for rec in self.records())


@dataclass(frozen=True)
class MessageSpec:
    """Catalog metadata entry for a protocol message in FIRMWARE_DESIGN_v2.md Section 7.3."""

    msg_type: UsbMsgType
    name: str
    direction: Direction
    nominal_rate_hz: float | None
    max_rate_hz: float | None
    min_interval_us: int | None
    payload_size: int
    cls: type


MESSAGE_CATALOG: dict[UsbMsgType, MessageSpec] = {
    UsbMsgType.CMD_TWIST: MessageSpec(
        msg_type=UsbMsgType.CMD_TWIST,
        name="CMD_TWIST",
        direction=Direction.PI_TO_STM32,
        nominal_rate_hz=25.0,
        max_rate_hz=50.0,
        min_interval_us=20_000,
        payload_size=MSG_CMD_TWIST_SIZE,
        cls=CmdTwist,
    ),
    UsbMsgType.CMD_MODE: MessageSpec(
        msg_type=UsbMsgType.CMD_MODE,
        name="CMD_MODE",
        direction=Direction.PI_TO_STM32,
        nominal_rate_hz=2.0,
        max_rate_hz=2.0,
        min_interval_us=500_000,
        payload_size=MSG_CMD_MODE_SIZE,
        cls=CmdMode,
    ),
    UsbMsgType.TIME_SYNC_REQ: MessageSpec(
        msg_type=UsbMsgType.TIME_SYNC_REQ,
        name="TIME_SYNC_REQ",
        direction=Direction.PI_TO_STM32,
        nominal_rate_hz=1.0,
        max_rate_hz=2.0,
        min_interval_us=500_000,
        payload_size=MSG_TIME_SYNC_REQ_SIZE,
        cls=TimeSyncReq,
    ),
    UsbMsgType.PARAM_OP: MessageSpec(
        msg_type=UsbMsgType.PARAM_OP,
        name="PARAM_OP",
        direction=Direction.PI_TO_STM32,
        nominal_rate_hz=None,
        max_rate_hz=None,
        min_interval_us=None,
        payload_size=MSG_PARAM_OP_SIZE,
        cls=ParamOp,
    ),
    UsbMsgType.TELEM_FAST: MessageSpec(
        msg_type=UsbMsgType.TELEM_FAST,
        name="TELEM_FAST",
        direction=Direction.STM32_TO_PI,
        nominal_rate_hz=200.0,
        max_rate_hz=200.0,
        min_interval_us=5_000,
        payload_size=LOG_TELEM_FAST_SIZE,
        cls=TelemFast,
    ),
    UsbMsgType.TELEM_HEALTH: MessageSpec(
        msg_type=UsbMsgType.TELEM_HEALTH,
        name="TELEM_HEALTH",
        direction=Direction.STM32_TO_PI,
        nominal_rate_hz=20.0,
        max_rate_hz=20.0,
        min_interval_us=50_000,
        payload_size=LOG_TELEM_HEALTH_SIZE,
        cls=TelemHealth,
    ),
    UsbMsgType.TIME_SYNC_RESP: MessageSpec(
        msg_type=UsbMsgType.TIME_SYNC_RESP,
        name="TIME_SYNC_RESP",
        direction=Direction.STM32_TO_PI,
        nominal_rate_hz=1.0,
        max_rate_hz=2.0,
        min_interval_us=500_000,
        payload_size=MSG_TIME_SYNC_RESP_SIZE,
        cls=TimeSyncResp,
    ),
    UsbMsgType.FAULT_SNAPSHOT: MessageSpec(
        msg_type=UsbMsgType.FAULT_SNAPSHOT,
        name="FAULT_SNAPSHOT",
        direction=Direction.STM32_TO_PI,
        nominal_rate_hz=None,
        max_rate_hz=None,
        min_interval_us=None,
        payload_size=MSG_FAULT_SNAPSHOT_SIZE,
        cls=FaultSnapshot,
    ),
    UsbMsgType.BENCH_RAW_LOG: MessageSpec(
        msg_type=UsbMsgType.BENCH_RAW_LOG,
        name="BENCH_RAW_LOG",
        direction=Direction.STM32_TO_PI,
        nominal_rate_hz=500.0,
        max_rate_hz=500.0,
        min_interval_us=2_000,
        payload_size=LOG_BENCH_RAW_SIZE,
        cls=BenchRawLog,
    ),
}


# Explicit pack / decode helpers matching c_ref/log_packer.h and prompt requirements
def pack_telem_fast(rec: TelemFast) -> bytes:
    return rec.pack()


def decode_telem_fast(data: bytes | bytearray | memoryview) -> TelemFast:
    return TelemFast.unpack(data)


unpack_telem_fast = decode_telem_fast


def frame_telem_fast(
    seq: int, timestamp_us: int, rec: TelemFast, flags: int = 0
) -> bytes:
    return encode_frame(
        UsbMsgType.TELEM_FAST,
        rec.pack(),
        seq=seq,
        timestamp_us=timestamp_us,
        flags=flags,
    )


def pack_telem_health(rec: TelemHealth) -> bytes:
    return rec.pack()


def decode_telem_health(data: bytes | bytearray | memoryview) -> TelemHealth:
    return TelemHealth.unpack(data)


unpack_telem_health = decode_telem_health


def frame_telem_health(
    seq: int, timestamp_us: int, rec: TelemHealth, flags: int = 0
) -> bytes:
    return encode_frame(
        UsbMsgType.TELEM_HEALTH,
        rec.pack(),
        seq=seq,
        timestamp_us=timestamp_us,
        flags=flags,
    )


def pack_bench_raw(rec: BenchRawLog) -> bytes:
    return rec.pack()


def decode_bench_raw(data: bytes | bytearray | memoryview) -> BenchRawLog:
    return BenchRawLog.unpack(data)


unpack_bench_raw = decode_bench_raw


def frame_bench_raw(
    seq: int, timestamp_us: int, rec: BenchRawLog, flags: int = 0
) -> bytes:
    return encode_frame(
        UsbMsgType.BENCH_RAW_LOG,
        rec.pack(),
        seq=seq,
        timestamp_us=timestamp_us,
        flags=flags,
    )


def pack_flight_record(rec: FlightRecord) -> bytes:
    return rec.pack()


def decode_flight_record(data: bytes | bytearray | memoryview) -> FlightRecord:
    return FlightRecord.unpack(data)


unpack_flight_record = decode_flight_record


def decode_flight_dump(data: bytes | bytearray | memoryview) -> list[FlightRecord]:
    """Decode one or more concatenated 188-byte FlightRecord structs."""
    mv = memoryview(data)
    if len(mv) % LOG_FLIGHT_RECORD_SIZE != 0:
        raise ValueError(
            f"Flight dump length {len(mv)} is not a multiple of {LOG_FLIGHT_RECORD_SIZE}"
        )
    count = len(mv) // LOG_FLIGHT_RECORD_SIZE
    return [
        FlightRecord.unpack(mv[i * LOG_FLIGHT_RECORD_SIZE : (i + 1) * LOG_FLIGHT_RECORD_SIZE])
        for i in range(count)
    ]


def decode_payload(
    msg_type: int | UsbMsgType,
    payload: bytes | bytearray | memoryview,
    *,
    header_timestamp_us: int = 0,
) -> Any:
    """Decode raw payload bytes into the typed catalog dataclass for msg_type."""
    mt = UsbMsgType(int(msg_type))
    if mt == UsbMsgType.CMD_TWIST:
        return CmdTwist.unpack(payload, header_timestamp_us=header_timestamp_us)
    spec = MESSAGE_CATALOG[mt]
    return spec.cls.unpack(payload)


def encode_message_frame(
    msg: Any,
    *,
    seq: int = 0,
    timestamp_us: int = 0,
    flags: int = 0,
    ver_major: int = USB_LINK_VER_MAJOR,
    ver_minor: int = USB_LINK_VER_MINOR,
) -> bytes:
    """Pack a catalog message dataclass and encode it into a COBS+CRC-32C USB wire frame."""
    return encode_frame(
        int(msg.MSG_TYPE),
        msg.pack(),
        seq=seq,
        timestamp_us=timestamp_us,
        flags=flags,
        ver_major=ver_major,
        ver_minor=ver_minor,
    )


def decode_message_frame(
    wire_buf: bytes | bytearray | memoryview,
) -> tuple[UsbLinkHeader, Any]:
    """Decode a COBS+CRC-32C USB wire frame into (UsbLinkHeader, typed message object)."""
    hdr, payload = decode_frame(wire_buf)
    msg = decode_payload(hdr.msg_type, payload, header_timestamp_us=hdr.timestamp_us)
    return hdr, msg
