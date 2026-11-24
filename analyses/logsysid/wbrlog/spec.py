"""WBR real-robot log format, version 1. Single source of truth for LOG_SPEC.md.

Every number in LOG_SPEC.md sections 3 to 6 is produced from this file
(tests/test_spec.py checks the tables against it).

Tags used in comments: SOURCE (read in a file), INFERRED, ASSUMED, UNKNOWN.
"""
import struct

FORMAT_VERSION = 1
MAGIC = b"\xA5\x5A"
# Frame: magic(2) type(u8) len(u16) seq(u16) t_us(u32) payload(len) crc16(u16)
FRAME_HEAD = struct.Struct("<2sBHHI")
FRAME_OVERHEAD = FRAME_HEAD.size + 2  # 13 bytes
MAX_PAYLOAD = 4096

# Studio axis order (SOURCE wbr-studio/params/sensor_params.m P.axes)
STUDIO_AXES = ("L_hip3", "L_hip4", "R_hip3", "R_hip4", "L_wheel", "R_wheel")

# ---------------------------------------------------------------- records
# name -> (type id, struct format of the fixed payload, field names)
RECORDS = {
    "META":     (0x01, None, ("json",)),
    "SYNC":     (0x02, struct.Struct("<IBQ"), ("sync_count", "source", "host_time_ns")),
    "EVENT":    (0x03, struct.Struct("<HH12s"), ("code", "arg", "label")),
    "IMU":      (0x10, struct.Struct("<6hH"), ("gx", "gy", "gz", "ax", "ay", "az", "imu_seq")),
    "IMU_TEMP": (0x11, struct.Struct("<h"), ("temp_raw",)),
    "CAN_TX":   (0x20, struct.Struct("<BBHB8s"), ("bus", "flags", "can_id", "dlc", "data")),
    "CAN_RX":   (0x21, struct.Struct("<BBHB8s"), ("bus", "flags", "can_id", "dlc", "data")),
    "CTRL":     (0x30, struct.Struct("<IIHH32f"), None),   # fields: CTRL_FIELDS
    "EST":      (0x31, struct.Struct("<12f"), None),       # fields: EST_FIELDS
    "TIMING":   (0x40, struct.Struct("<HHHH"), ("loop_period_us", "exec_us", "log_drop_count", "can_tx_err")),
}
TYPE_TO_NAME = {v[0]: k for k, v in RECORDS.items()}

# CTRL: header ints then 32 float32 in this order. Native DM-jump variables,
# not Studio variables; the importer maps them (wbrlog/importer.py, LOG_SPEC 5).
CTRL_HEAD_FIELDS = ("ctrl_tick", "imu_t_us_used", "imu_seq_used", "flags")
CTRL_FIELDS = (
    "theta_R", "theta_L",            # rad, vmc->theta (SOURCE VMC_calc.c line 83)
    "d_theta_R", "d_theta_L",        # rad/s
    "L0_R", "L0_L",                  # m, virtual leg length
    "d_L0_R", "d_L0_L",              # m/s
    "alpha_R", "alpha_L",            # rad, vmc->alpha = pi/2 - phi0 (VMC_calc.c line 72)
    "d_alpha_R", "d_alpha_L",        # rad/s
    "pitch", "pitch_gyro",           # rad, rad/s: INS.Pitch, INS.Gyro[0] (chassisR_task.c 158-159)
    "roll", "yaw_total", "yaw_gyro", # rad, rad, rad/s: INS.Roll, INS.YawTotalAngle, INS.Gyro[2]
    "x_filter", "v_filter",          # m, m/s (observe_task.c line 87-88)
    "F0_R", "F0_L",                  # N, virtual leg force
    "Tp_R", "Tp_L",                  # N*m, virtual hip torque
    "tau_req_R0", "tau_req_R1",      # N*m, joint torque BEFORE the +-7 N*m clamp
    "tau_req_L0", "tau_req_L1",
    "wheel_req_R", "wheel_req_L",    # N*m, wheel torque BEFORE the +-2 N*m clamp
    "leg_set", "turn_T", "roll_out", # m, N*m, N (roll PID output)
)
assert len(CTRL_FIELDS) == 32

# EST: exactly the Studio estimator diagnostic (11 values, SOURCE
# m13_packet_update.m line 21) plus the INS accel norm. NaN when the firmware
# has no such quantity.
EST_FIELDS = ("ready", "pitch", "roll", "yaw", "body_speed", "L_L", "L_R",
              "gbias_x", "gbias_y", "gbias_z", "accel_bias", "accel_norm")
assert len(EST_FIELDS) == 12

SYNC_SOURCES = {0: "gpio_pulse", 1: "host_ping", 2: "power_on"}

# ------------------------------------------------------- rate profiles
# record -> (rate Hz, count per period). CAN rates: DM-jump sends one frame
# per motor every 3 ms (SOURCE chassisR_task.c lines 79-84, osDelay(1) x3).
FW_PERIOD_S = 0.003
PROFILES = {
    # Full balance logging over USB CDC (CtrBoard-H7 has USB OTG HS with
    # internal FS PHY, SOURCE h7-advisory-fw docs/hardware.md).
    "FULL": {
        "IMU": 1000.0, "IMU_TEMP": 1.0, "SYNC": 1.0,
        "CAN_TX": 6 / FW_PERIOD_S, "CAN_RX": 6 / FW_PERIOD_S,
        "CTRL": 1 / FW_PERIOD_S, "EST": 1 / FW_PERIOD_S, "TIMING": 1 / FW_PERIOD_S,
    },
    # Stationary IMU (Allan variance), fits any UART.
    "IMU_ONLY": {"IMU": 1000.0, "IMU_TEMP": 1.0, "SYNC": 1.0},
    # One motor on the stand, commanded at 1 kHz by a bench build.
    "ONE_MOTOR": {"CAN_TX": 1000.0, "CAN_RX": 1000.0, "TIMING": 1000.0, "SYNC": 1.0, "EVENT": 2.0},
}
LINKS_BYTES_PER_S = {
    # INFERRED usable throughput; measure with the SYNC/drop counters (EXPERIMENTS B0).
    "USB_FS_CDC": 800_000.0,      # 12 Mbit/s raw; ~0.8 MB/s practical (INFERRED)
    "UART_2M": 200_000.0,         # 2 Mbaud 8N1 = 10 bits/byte
    "UART_921600": 92_160.0,
    "UART_460800": 46_080.0,
}
META_BYTES = 2000  # typical META JSON size, sent once


def record_size(name):
    fmt = RECORDS[name][1]
    payload = META_BYTES if fmt is None else fmt.size
    return FRAME_OVERHEAD + payload


def budget(profile):
    """Return (rows, total bytes/s) for a rate profile."""
    rows = []
    total = 0.0
    for name, rate in PROFILES[profile].items():
        size = record_size(name)
        bps = size * rate
        rows.append((name, size, rate, bps))
        total += bps
    return rows, total


# ----------------------------------------------------- default metadata
# Every value here must be confirmed on the robot (EXPERIMENTS R0/R1).
def default_meta():
    return {
        "format_version": FORMAT_VERSION,
        "board": "UNKNOWN (STM32H723-class; CtrBoard-H7 or DM-MC02 not confirmed)",
        "firmware": "UNKNOWN (DM-jump 6855ef8 or newer)",
        "clock": {"t_us_source": "32-bit TIM at 1 MHz (not bare DWT CYCCNT)",
                  "wrap_s": 2**32 / 1e6},
        "imu": {
            "model": "IMU sensor",
            # Ranges: UNKNOWN on the robot. Course driver uses +-3 g / 250 dps
            # (unbundled sensor-driver reference, lines 39-40); balance firmware usually
            # needs more. Log what is actually written to the registers.
            "gyro_range_dps": 2000.0,
            "accel_range_g": 6.0,
            "odr_hz": 1000.0,
            "R_BS": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],   # sensor->body, measured in R1
            "temp_scale": 0.125, "temp_offset_c": 23.0,
        },
        # One entry per Studio axis. CAN ids and buses UNKNOWN until asked
        # (unbundled hardware planning reference, question 4). Ranges must be read from each motor with
        # the Damiao tool; the values below are INFERRED placeholders.
        "motors": [
            {"axis": "L_hip3", "bus": 2, "can_id": 0x03, "model": "DM4310", "sign": 1, "offset_rad": 0.0, "pmax": 12.5, "vmax": 30.0, "tmax": 10.0, "clamp_nm": 7.0},
            {"axis": "L_hip4", "bus": 2, "can_id": 0x04, "model": "DM4310", "sign": 1, "offset_rad": 0.0, "pmax": 12.5, "vmax": 30.0, "tmax": 10.0, "clamp_nm": 7.0},
            {"axis": "R_hip3", "bus": 1, "can_id": 0x01, "model": "DM4310", "sign": 1, "offset_rad": 0.0, "pmax": 12.5, "vmax": 30.0, "tmax": 10.0, "clamp_nm": 7.0},
            {"axis": "R_hip4", "bus": 1, "can_id": 0x02, "model": "DM4310", "sign": 1, "offset_rad": 0.0, "pmax": 12.5, "vmax": 30.0, "tmax": 10.0, "clamp_nm": 7.0},
            {"axis": "L_wheel", "bus": 2, "can_id": 0x06, "model": "DM6215", "sign": 1, "offset_rad": 0.0, "pmax": 12.5, "vmax": 45.0, "tmax": 10.0, "clamp_nm": 2.0},
            {"axis": "R_wheel", "bus": 1, "can_id": 0x05, "model": "DM6215", "sign": 1, "offset_rad": 0.0, "pmax": 12.5, "vmax": 45.0, "tmax": 10.0, "clamp_nm": 2.0},
        ],
        # Native CTRL -> Studio sign table (LOG_SPEC 5.3). Defaults from DM-jump:
        # the left side negates pitch and gyro (VMC_calc.c 109-110).
        "ctrl_map": {"pitch_sign": 1, "alpha_sign": 1, "roll_sign": 1, "yaw_sign": 1,
                     "x_sign": 1, "mirror_left": True},
        "provenance": "measurement_domain=hardware",
    }
