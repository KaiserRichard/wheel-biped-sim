"""Firmware-in-the-Loop (FIL) test for balance_ctrl on MuJoCo CAD plant.

Loads the compiled C11 shared library (build/libwbr_balance_ctrl.so) via ctypes,
runs the full 500 Hz balance_ctrl pipeline (3-bus can_sched + damiao_mit +
s11_packet_policy + s10_estimator + safety_fsm + S9 brake/trim + VMC + S7/S5)
closed-loop against MuJoCo and verifies:
  1. 12.0 s balanced-robot stand (nominal CAD CoM, com_mm = 0) + 6 N / 30 ms push:
     |drift_x| < 0.10 m and |final_pitch - 0.0 deg| < 1.5 deg.
  2. 12.0 s perturbed draw_001.xml (com_mm = -9.51 mm) stand + 6 N / 30 ms push:
     S9 trim converges (> 5.0 deg), |v_est| < 0.01 m/s, |drift_x| < 0.35 m, and
     |final_pitch - (-1.317 deg)| < 1.5 deg (regression guard against 0.30 m deadlock).
  3. 10.0 s radio-loss policy c (RADIO_LOSS_BRAKE_BALANCE) while standing + push:
     BALANCING -> CONTROLLED_STOP -> REST upright with zero body contact.
  4. 13.46 s S11 G2 driving mission on nominal CAD CoM (com_mm = 0):
     quintic ramp 1.0..3.0 s to 0.20 m/s, 6 N / 30 ms push at 3.60 s,
     0.30 rad/s yaw turn slewed at 1.0 rad/s^2 from 4.20 s, host loss at 5.46 s ->
     0.60 s brake -> REST, compared against S11 near-nominal reference (018_s11_s9).
  5. 13.46 s S11 G2 driving mission on perturbed draw_001.xml (com_mm = -9.51 mm):
     same mission profile, compared against S11 reference trace 001_s11_s9.csv/.json.
  6. 12.0 s radio-loss policy c (RADIO_LOSS_BRAKE_BALANCE) while driving at 0.20 m/s
     on draw_001.xml: radio lost at t = 4.0 s -> brake -> REST -> keep balancing.
"""

import ctypes
import math
from pathlib import Path
import sys
from typing import Callable

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
MODEL_XML = ROOT / "sim_model" / "draw_001.xml"
LIB_PATH = ROOT / "build" / "libwbr_balance_ctrl.so"

DT = 0.001
WHEEL_RADIUS = 0.0603
LEG_REF_M = 0.14
GRAVITY = 9.80665

# Traceable S11 reference values:
# - Nominal CAD chassis CoM x: S11_packet_tolerance/b05_copy/model_assets/wbr_full.xml:105
#   (also stored verbatim at sim_model/draw_001.xml:113 as model_base_imu_site).
NOMINAL_CAD_CHASSIS_COM_X_M = 0.00265373721167746
# - Nominal balanced CAD equilibrium pitch at L = 0.14 m:
#   S11_packet_tolerance/b05_copy/model_repo/17_cad_robot/results/gains.json (x0[4] = 0.0).
NOMINAL_REF_EQ_PITCH_DEG = 0.0
# - Perturbed draw_001 (sample_id=1, com_mm=-9.509 mm) steady-state pitch in S11:
#   S11_packet_tolerance/out/traces/001_s11_s9.csv:333 (pitch_deg = -1.31654 deg).
DRAW001_REF_EQ_PITCH_DEG = -1.317

# Traceable S11 G2 mission parameters (S11_packet_tolerance/run_g1.py:21-31, speed_design.py:45-56):
S11_G2_DURATION_S = 13.46
S11_G2_HOST_LOSS_S = 5.46
S11_G2_PUSH_START_S = 3.60
S11_G2_PUSH_DUR_S = 0.030
S11_G2_PUSH_FORCE_N = 6.0

# S11 reference summary numbers for side-by-side reporting:
# - S11_packet_tolerance/out/cases/001_s11_s9.json & out/traces/001_s11_s9.csv (draw_001)
S11_REF_DRAW001 = {
    "pre_push_mean_v_m_s": 0.2126,
    "speed_at_latch_m_s": 0.2317,
    "pre_loss_mean_yaw_rate_rad_s": 0.2930,
    "peak_abs_pitch_deg": 8.199,
    "stop_path_m": 0.5521,
    "path_to_end_m": 0.5755,
    "tail_mean_pitch_deg": -1.330,
    "final_pitch_deg": -1.317,
    "s9_trim_deg": 5.906,
}
# - S11_packet_tolerance/out/cases/018_s11_s9.json (closest near-nominal draw: com=-0.024 mm, damp=0.0251)
S11_REF_NEAR_NOMINAL_018 = {
    "pre_push_mean_v_m_s": 0.1906,
    "speed_at_latch_m_s": 0.1975,
    "pre_loss_mean_yaw_rate_rad_s": 0.2923,
    "peak_abs_pitch_deg": 1.303,
    "stop_path_m": 0.1467,
    "path_to_end_m": 0.1737,
    "tail_mean_pitch_deg": 0.068,
    "final_pitch_deg": 0.065,
    "s9_trim_deg": 0.178,
}

RADIO_LOSS_TORQUE_OFF = 0
RADIO_LOSS_BRAKE_BALANCE = 1

SAFETY_STATE_BALANCING = 3
SAFETY_STATE_CONTROLLED_STOP = 4
SAFETY_STATE_REST = 5

SAFETY_REASON_HOST_LOSS = 5
SAFETY_REASON_RADIO_LOSS_BRAKE = 7


def g2_speed_profile(t_s: float) -> tuple[float, float, float]:
    """Exact S11 G2 speed/accel/jerk profile (S11_packet_tolerance/speed_design.py:45-56)."""
    if t_s <= 1.0:
        return 0.0, 0.0, 0.0
    if t_s < 3.0:
        z = (t_s - 1.0) / 2.0
        v = 0.20 * z * z * z * (10.0 - 15.0 * z + 6.0 * z * z)
        a = (0.20 / 2.0) * 30.0 * z * z * (1.0 - z) * (1.0 - z)
        j = (0.20 / 4.0) * 60.0 * z * (1.0 - z) * (1.0 - 2.0 * z)
        return v, a, j
    return 0.20, 0.0, 0.0


def g2_yaw_profile(t_s: float) -> float:
    """Exact S11 G2 slewed yaw-rate command (run_g1.py:26-28, run_g2.py:249-250)."""
    if t_s < 4.20:
        return 0.0
    return min(0.30, (t_s - 4.20) * 1.0)


def load_library(path: Path) -> ctypes.CDLL:
    if not path.is_file():
        raise FileNotFoundError(f"Shared library not found: {path}")
    lib = ctypes.CDLL(str(path))

    lib.balance_ctrl_state_size.argtypes = []
    lib.balance_ctrl_state_size.restype = ctypes.c_size_t

    lib.balance_ctrl_fil_init.argtypes = [
        ctypes.c_void_p,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_uint8,
    ]
    lib.balance_ctrl_fil_init.restype = None

    lib.balance_ctrl_fil_imu_1khz.argtypes = [
        ctypes.c_void_p,
        ctypes.c_int32,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
    ]
    lib.balance_ctrl_fil_imu_1khz.restype = None

    lib.balance_ctrl_fil_arm_and_handoff.argtypes = [
        ctypes.c_void_p,
        ctypes.c_uint32,
        ctypes.c_int32,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
    ]
    lib.balance_ctrl_fil_arm_and_handoff.restype = ctypes.c_int32

    lib.balance_ctrl_fil_step_500hz.argtypes = [
        ctypes.c_void_p,
        ctypes.c_uint32,
        ctypes.c_int32,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_uint8,
        ctypes.c_uint8,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
    ]
    lib.balance_ctrl_fil_step_500hz.restype = ctypes.c_int32

    lib.balance_ctrl_fil_step_500hz_ex.argtypes = [
        ctypes.c_void_p,
        ctypes.c_uint32,
        ctypes.c_int32,
        ctypes.POINTER(ctypes.c_double),
        ctypes.POINTER(ctypes.c_double),
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_float,
        ctypes.c_uint8,
        ctypes.c_uint8,
        ctypes.c_uint8,
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
        ctypes.POINTER(ctypes.c_float),
    ]
    lib.balance_ctrl_fil_step_500hz_ex.restype = ctypes.c_int32
    return lib


class RobotRig:
    """Thin MuJoCo wrapper for draw_001.xml matching S11 CAD sign & pose conventions."""

    def __init__(self, xml_path: Path, chassis_com_x_m: float | None = None):
        self.m = mujoco.MjModel.from_xml_path(str(xml_path))
        self.d = mujoco.MjData(self.m)
        if chassis_com_x_m is not None:
            chassis_id = int(self.m.body("chassis").id)
            self.m.body_ipos[chassis_id, 0] = float(chassis_com_x_m)
            mujoco.mj_setConst(self.m, self.d)
        self.sign = np.array([-1.0, 1.0, 1.0], dtype=np.float64)
        self.sign6 = np.tile(self.sign, 2)
        self.ja = {}
        self.jv = {}
        self.act = {}
        self.wheel = {}
        self.offset = {}
        self.hips = {}
        for s, side in zip("LR", ("left", "right")):
            names = {
                "hip_front": f"{side}_front_motor",
                "hip_rear": f"{side}_rear_motor",
                "knee_front": f"{side}_front_knee",
                "knee_rear": f"{side}_rear_knee",
                "wheel": f"{side}_wheel",
            }
            self.ja[s] = {k: int(self.m.joint(v).qposadr[0]) for k, v in names.items()}
            self.jv[s] = {k: int(self.m.joint(v).dofadr[0]) for k, v in names.items()}
            self.act[s] = [
                int(self.m.actuator(names[k] + "_torque").id)
                for k in ("hip_front", "hip_rear", "wheel")
            ]
            self.wheel[s] = int(self.m.body(f"{side}_wheel").id)
            hf = self.m.body_pos[self.m.body(f"{side}_front_upper").id]
            hr = self.m.body_pos[self.m.body(f"{side}_rear_upper").id]
            self.hips[s] = 0.5 * (hf + hr)
            off = []
            for b, sgn in (("front", 1.0), ("rear", -1.0)):
                v = self.m.body_pos[self.m.body(f"{side}_{b}_lower").id]
                off.append(math.atan2(sgn * float(v[0]), -float(v[2])))
            self.offset[s] = np.array(off + [0.0], dtype=np.float64)
        self.hip_mid = np.mean(list(self.hips.values()), axis=0)
        self.hip_mid[1] = 0.0
        self.axis = [a for s in "LR" for a in self.act[s]]
        self.qidx = [self.ja[s][k] for s in "LR" for k in ("hip_front", "hip_rear", "wheel")]
        self.vidx = [self.jv[s][k] for s in "LR" for k in ("hip_front", "hip_rear", "wheel")]
        self.offset6 = np.concatenate([self.offset[s] for s in "LR"])
        self.floor_id = int(self.m.geom("floor").id)
        self.wheel_geoms = {
            int(self.m.geom("left_wheel_collision").id),
            int(self.m.geom("right_wheel_collision").id),
        }
        self.total_mass = float(self.m.body_mass.sum())

    def reset_standing(self, length_m: float = LEG_REF_M) -> None:
        mujoco.mj_resetData(self.m, self.d)
        q = self.m.qpos0.copy()
        s = 0.0
        theta = -0.018
        phi = 0.0
        R = np.array(
            [[math.cos(phi), 0.0, math.sin(phi)],
             [0.0, 1.0, 0.0],
             [-math.sin(phi), 0.0, math.cos(phi)]],
            dtype=np.float64,
        )
        q[:3] = np.array(
            [s + length_m * math.sin(theta), 0.0, WHEEL_RADIUS + length_m * math.cos(theta)]
        ) - R @ self.hip_mid
        q[3:7] = [math.cos(phi / 2.0), 0.0, math.sin(phi / 2.0), 0.0]

        for side, word in zip("LR", ("left", "right")):
            foot = self.hips[side] + np.array(
                [-length_m * math.sin(theta - phi), 0.0, -length_m * math.cos(theta - phi)]
            )
            for b, sgn in (("front", 1.0), ("rear", -1.0)):
                u_id = self.m.body(f"{word}_{b}_upper").id
                l_id = self.m.body(f"{word}_{b}_lower").id
                hip = self.m.body_pos[u_id]
                v = self.m.body_pos[l_id][[0, 2]]
                if b == "front":
                    end = self.m.body_pos[self.wheel[side]][[0, 2]]
                else:
                    end = self.m.site_pos[self.m.site(f"{word}_rear_closure").id][[0, 2]]
                target = (foot - hip)[[0, 2]]
                dist = float(np.linalg.norm(target))
                a = float(np.linalg.norm(v))
                c = float(np.linalg.norm(end))
                cos_arg = max(-1.0, min(1.0, (a * a + dist * dist - c * c) / (2.0 * a * dist)))
                angle = math.atan2(sgn * float(target[0]), -float(target[1])) + math.acos(cos_arg)
                upper = np.array([sgn * a * math.sin(angle), -a * math.cos(angle)])
                du = -(math.atan2(float(v[0]), float(v[1])) - math.atan2(float(upper[0]), float(upper[1])))
                lower = target - upper
                total = math.atan2(float(lower[0]), float(lower[1])) - math.atan2(float(end[0]), float(end[1]))
                total = (total + math.pi) % (2.0 * math.pi) - math.pi
                du = (du + math.pi) % (2.0 * math.pi) - math.pi
                q[self.ja[side][f"hip_{b}"]] = du
                q[self.ja[side][f"knee_{b}"]] = total - du
                if b == "front":
                    q[self.ja[side]["wheel"]] = s / WHEEL_RADIUS - phi - total

        self.d.qpos[:] = q
        self.d.qvel[:] = 0.0
        self.d.ctrl[:] = 0.0
        mujoco.mj_forward(self.m, self.d)

    def read_encoders(self):
        angles = self.sign6 * self.d.qpos[self.qidx] + self.offset6
        rates = self.sign6 * self.d.qvel[self.vidx]
        return angles, rates

    def read_imu(self):
        gyro = self.d.sensor("torso_gyro").data.copy()
        acc = self.d.sensor("torso_acc").data.copy()
        return gyro, acc

    def chassis_metrics(self):
        rot = self.d.xmat[1].reshape(3, 3)
        heading = rot[:2, 0] / max(1e-12, math.hypot(float(rot[0, 0]), float(rot[1, 0])))
        pitch_deg = math.degrees(math.atan2(float(rot[:2, 2] @ heading), float(rot[2, 2])))
        roll_deg = math.degrees(math.asin(max(-1.0, min(1.0, float(rot[2, 1])))))
        height_m = float(self.d.qpos[2] + (rot @ self.hip_mid)[2])
        body_contact = False
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            if c.geom1 == self.floor_id:
                other = c.geom2
            elif c.geom2 == self.floor_id:
                other = c.geom1
            else:
                continue
            if other not in self.wheel_geoms:
                body_contact = True
                break
        return pitch_deg, roll_deg, height_m, body_contact

    def motion_metrics(self):
        rot = self.d.xmat[1].reshape(3, 3)
        heading = rot[:2, 0] / max(1e-12, math.hypot(float(rot[0, 0]), float(rot[1, 0])))
        obj_vel = np.zeros(6, dtype=np.float64)
        speed_m_s = 0.0
        for side in "LR":
            mujoco.mj_objectVelocity(
                self.m,
                self.d,
                mujoco.mjtObj.mjOBJ_BODY,
                self.wheel[side],
                obj_vel,
                0,
            )
            speed_m_s += 0.5 * float(obj_vel[3:5] @ heading)
        yaw_rate_rad_s = float((rot @ self.d.qvel[3:6])[2])
        hip_xy = (self.d.qpos[:3] + rot @ self.hip_mid)[:2].copy()
        return speed_m_s, yaw_rate_rad_s, hip_xy


def run_episode(
    lib: ctypes.CDLL,
    duration_s: float,
    push_start_s: float,
    push_dur_s: float,
    push_force_n: float,
    radio_loss_policy: int,
    radio_loss_s: float | None = None,
    chassis_com_x_m: float | None = None,
    v_profile_fn: Callable[[float], tuple[float, float, float]] | None = None,
    yaw_profile_fn: Callable[[float], float] | None = None,
    host_loss_s: float | None = None,
) -> dict:
    rig = RobotRig(MODEL_XML, chassis_com_x_m=chassis_com_x_m)
    rig.reset_standing(LEG_REF_M)
    x0_m = float(rig.d.qpos[0])
    _, _, prev_hip_xy = rig.motion_metrics()

    state_buf = ctypes.create_string_buffer(lib.balance_ctrl_state_size())
    lib.balance_ctrl_fil_init(state_buf, rig.total_mass, LEG_REF_M, radio_loss_policy)

    # Stationary S10 estimator pre-warmup (101+ samples at 1 kHz)
    rot0 = rig.d.xmat[1].reshape(3, 3)
    acc0 = rot0.T @ np.array([0.0, 0.0, GRAVITY], dtype=np.float64)
    gyro0 = np.zeros(3, dtype=np.float64)
    c_gyro = (ctypes.c_double * 3)(*gyro0)
    c_acc = (ctypes.c_double * 3)(*acc0)

    for t_ms in range(-120, -10):
        lib.balance_ctrl_fil_imu_1khz(state_buf, t_ms, c_gyro, c_acc)

    angles0, rates0 = rig.read_encoders()
    c_ang = (ctypes.c_double * 6)(*angles0)
    c_rat = (ctypes.c_double * 6)(*rates0)

    # Arm and handoff through DISARMED -> PRECHARGE -> ARMED -> BALANCING
    # spanning enc_tick_ms = -10..-2, keeping IMU ticks synchronized.
    for t_ms in range(-10, 0):
        lib.balance_ctrl_fil_imu_1khz(state_buf, t_ms, c_gyro, c_acc)
    state_after_arm = lib.balance_ctrl_fil_arm_and_handoff(
        state_buf, 10000, -10, c_ang, c_rat
    )
    assert state_after_arm == SAFETY_STATE_BALANCING, (
        f"Expected BALANCING ({SAFETY_STATE_BALANCING}), got {state_after_arm}"
    )

    out_torq = (ctypes.c_float * 6)()
    out_diag = (ctypes.c_float * 10)()
    out_drive_diag = (ctypes.c_float * 4)()
    applied_ctrl = np.zeros(6, dtype=np.float64)

    n_steps = int(round(duration_s / DT))
    push_start_k = int(round(push_start_s / DT))
    push_end_k = push_start_k + int(round(push_dur_s / DT))
    radio_loss_k = int(round(radio_loss_s / DT)) if radio_loss_s is not None else None
    host_loss_k = int(round(host_loss_s / DT)) if host_loss_s is not None else None
    loss_candidates = [v for v in (host_loss_k, radio_loss_k) if v is not None]
    loss_k = min(loss_candidates) if loss_candidates else None
    use_ex = (
        v_profile_fn is not None
        or yaw_profile_fn is not None
        or host_loss_s is not None
    )

    max_abs_pitch = 0.0
    max_abs_roll = 0.0
    max_abs_drift_x = 0.0
    max_abs_x_err_m = 0.0
    min_height = float("inf")
    body_hit = False
    saw_controlled_stop = False
    saw_rest = False
    saw_host_loss_reason = False
    saw_radio_loss_brake_reason = False
    final_state = state_after_arm
    final_pitch = 0.0

    path_m = 0.0
    path_at_latch_m: float | None = None
    stop_path_m: float | None = None
    speed_at_latch_m_s = 0.0
    pre_loss_peak_v_m_s = 0.0
    pre_push_v_samples: list[float] = []
    pre_loss_yaw_samples: list[float] = []
    tail_pitch_samples: list[float] = []
    prev_x_err_m = 0.0
    latch_pos_jump_m: float | None = None

    for k in range(n_steps):
        t = k * DT
        push = push_force_n if (push_force_n != 0.0 and push_start_k <= k < push_end_k) else 0.0
        rig.d.xfrc_applied[1, 0] = push
        mujoco.mj_forward(rig.m, rig.d)

        if loss_k is not None and k == loss_k:
            speed_at_latch_m_s, _, _ = rig.motion_metrics()
            path_at_latch_m = path_m

        gyro, acc = rig.read_imu()
        for i in range(3):
            c_gyro[i] = float(gyro[i])
            c_acc[i] = float(acc[i])
        lib.balance_ctrl_fil_imu_1khz(state_buf, k, c_gyro, c_acc)

        if k % 2 == 0:
            angles, rates = rig.read_encoders()
            for i in range(6):
                c_ang[i] = float(angles[i])
                c_rat[i] = float(rates[i])
            r_lost = 1 if (radio_loss_k is not None and k >= radio_loss_k) else 0
            h_lost = 1 if (host_loss_k is not None and k >= host_loss_k) else 0
            inhibit_trim = 1 if push != 0.0 else 0
            tick_us = 20000 + k * 1000
            if use_ex:
                v_cmd, a_cmd, j_cmd = v_profile_fn(t) if v_profile_fn is not None else (0.0, 0.0, 0.0)
                yaw_cmd = yaw_profile_fn(t) if yaw_profile_fn is not None else 0.0
                final_state = lib.balance_ctrl_fil_step_500hz_ex(
                    state_buf,
                    tick_us,
                    k,
                    c_ang,
                    c_rat,
                    float(v_cmd),
                    float(yaw_cmd),
                    float(a_cmd),
                    float(j_cmd),
                    h_lost,
                    r_lost,
                    inhibit_trim,
                    out_torq,
                    out_diag,
                    out_drive_diag,
                )
                x_err_eff = float(out_drive_diag[2])
                reason = int(round(float(out_drive_diag[3])))
                max_abs_x_err_m = max(max_abs_x_err_m, abs(x_err_eff))
                if reason == SAFETY_REASON_HOST_LOSS:
                    saw_host_loss_reason = True
                elif reason == SAFETY_REASON_RADIO_LOSS_BRAKE:
                    saw_radio_loss_brake_reason = True
                if loss_k is not None and k >= loss_k and latch_pos_jump_m is None:
                    latch_pos_jump_m = abs(x_err_eff - prev_x_err_m)
                prev_x_err_m = x_err_eff
            else:
                final_state = lib.balance_ctrl_fil_step_500hz(
                    state_buf,
                    tick_us,
                    k,
                    c_ang,
                    c_rat,
                    0.0,
                    0.0,
                    r_lost,
                    inhibit_trim,
                    out_torq,
                    out_diag,
                )
            if final_state == SAFETY_STATE_CONTROLLED_STOP:
                saw_controlled_stop = True
            elif final_state == SAFETY_STATE_REST:
                saw_rest = True
            req = np.array([out_torq[i] for i in range(6)], dtype=np.float64)
            applied_ctrl = rig.sign6 * req

        rig.d.ctrl[rig.axis] = applied_ctrl
        mujoco.mj_step(rig.m, rig.d)

        pitch_deg, roll_deg, height_m, hit = rig.chassis_metrics()
        speed_m_s, yaw_rate_rad_s, hip_xy = rig.motion_metrics()
        path_m += math.hypot(float(hip_xy[0] - prev_hip_xy[0]), float(hip_xy[1] - prev_hip_xy[1]))
        prev_hip_xy = hip_xy

        if 3300 <= k < 3600:
            pre_push_v_samples.append(speed_m_s)
        if 5100 <= k < 5400:
            pre_loss_yaw_samples.append(yaw_rate_rad_s)
        if loss_k is None or k < loss_k:
            pre_loss_peak_v_m_s = max(pre_loss_peak_v_m_s, abs(speed_m_s))
        if path_at_latch_m is not None and stop_path_m is None and final_state == SAFETY_STATE_REST:
            stop_path_m = path_m - path_at_latch_m
        if k >= n_steps - 250:
            tail_pitch_samples.append(pitch_deg)

        max_abs_pitch = max(max_abs_pitch, abs(pitch_deg))
        max_abs_roll = max(max_abs_roll, abs(roll_deg))
        max_abs_drift_x = max(max_abs_drift_x, abs(float(rig.d.qpos[0]) - x0_m))
        min_height = min(min_height, height_m)
        body_hit = body_hit or hit
        final_pitch = pitch_deg

        if rig.d.warning.number.any():
            raise RuntimeError(f"MuJoCo numerical warning at t={t:.3f}s")

    drift_x_m = float(rig.d.qpos[0]) - x0_m
    path_to_end_m = (path_m - path_at_latch_m) if path_at_latch_m is not None else 0.0
    if stop_path_m is None:
        stop_path_m = path_to_end_m
    return {
        "mass_kg": rig.total_mass,
        "duration_s": duration_s,
        "max_abs_pitch_deg": max_abs_pitch,
        "max_abs_roll_deg": max_abs_roll,
        "final_pitch_deg": final_pitch,
        "tail_mean_pitch_deg": float(np.mean(tail_pitch_samples)) if tail_pitch_samples else final_pitch,
        "drift_x_m": drift_x_m,
        "max_abs_drift_x_m": max_abs_drift_x,
        "max_abs_x_err_m": max_abs_x_err_m,
        "latch_pos_jump_m": latch_pos_jump_m if latch_pos_jump_m is not None else 0.0,
        "pre_push_mean_v_m_s": float(np.mean(pre_push_v_samples)) if pre_push_v_samples else 0.0,
        "speed_at_latch_m_s": speed_at_latch_m_s,
        "pre_loss_peak_v_m_s": pre_loss_peak_v_m_s,
        "pre_loss_mean_yaw_rate_rad_s": float(np.mean(pre_loss_yaw_samples)) if pre_loss_yaw_samples else 0.0,
        "stop_path_m": stop_path_m,
        "path_to_end_m": path_to_end_m,
        "min_height_m": min_height,
        "body_contact": body_hit,
        "final_state": final_state,
        "saw_controlled_stop": saw_controlled_stop,
        "saw_rest": saw_rest,
        "saw_host_loss_reason": saw_host_loss_reason,
        "saw_radio_loss_brake_reason": saw_radio_loss_brake_reason,
        "est_pitch_deg": math.degrees(float(out_diag[0])),
        "est_v_m_s": float(out_diag[2]),
        "L0_left_m": float(out_diag[3]),
        "s9_trim_deg": math.degrees(float(out_diag[6])),
    }


def main() -> int:
    lib = load_library(LIB_PATH)

    # Case 1: 12.0 s balanced-robot stand (nominal CAD CoM, com_mm = 0) + 6 N / 30 ms push
    res1 = run_episode(
        lib,
        duration_s=12.0,
        push_start_s=3.0,
        push_dur_s=0.030,
        push_force_n=6.0,
        radio_loss_policy=RADIO_LOSS_BRAKE_BALANCE,
        radio_loss_s=None,
        chassis_com_x_m=NOMINAL_CAD_CHASSIS_COM_X_M,
    )
    print(
        f"[FIL Case 1 - 12s Balanced Stand + 6N/30ms Push (com_mm=0)] "
        f"mass={res1['mass_kg']:.3f}kg "
        f"drift_x={res1['drift_x_m']:+.4f}m max_|drift|={res1['max_abs_drift_x_m']:.4f}m "
        f"max_|pitch|={res1['max_abs_pitch_deg']:.3f}deg "
        f"max_|roll|={res1['max_abs_roll_deg']:.3f}deg "
        f"final_pitch={res1['final_pitch_deg']:.3f}deg "
        f"est_pitch={res1['est_pitch_deg']:.3f}deg "
        f"s9_trim={res1['s9_trim_deg']:.3f}deg "
        f"v_est={res1['est_v_m_s']:.4f}m/s "
        f"L0={res1['L0_left_m']:.4f}m "
        f"state={res1['final_state']} body_hit={res1['body_contact']}"
    )
    assert abs(res1["drift_x_m"]) < 0.10, (
        f"Balanced-robot drift exceeded 0.10 m: {res1['drift_x_m']:+.4f} m"
    )
    assert res1["max_abs_drift_x_m"] < 0.10, (
        f"Balanced-robot peak drift exceeded 0.10 m: {res1['max_abs_drift_x_m']:.4f} m"
    )
    assert abs(res1["final_pitch_deg"] - NOMINAL_REF_EQ_PITCH_DEG) < 1.5, (
        f"Final |pitch - {NOMINAL_REF_EQ_PITCH_DEG:.3f}| >= 1.5 deg: {res1['final_pitch_deg']:.3f} deg"
    )
    assert abs(res1["est_v_m_s"]) < 0.01, f"Residual speed exceeded 0.01 m/s: {res1['est_v_m_s']:.4f}"
    assert res1["max_abs_pitch_deg"] < 15.0, f"Pitch exceeded 15 deg: {res1['max_abs_pitch_deg']}"
    assert res1["max_abs_roll_deg"] < 15.0, f"Roll exceeded 15 deg: {res1['max_abs_roll_deg']}"
    assert not res1["body_contact"], "Body contacted floor during Case 1"
    assert res1["final_state"] == SAFETY_STATE_BALANCING, f"Unexpected state: {res1['final_state']}"

    # Case 2: 12.0 s perturbed draw_001.xml (com_mm = -9.51 mm) + 6 N / 30 ms push
    # Regression test for the 0.30 m standing clamp deadlock that trapped s9_trim at 0.80 deg.
    res2 = run_episode(
        lib,
        duration_s=12.0,
        push_start_s=3.0,
        push_dur_s=0.030,
        push_force_n=6.0,
        radio_loss_policy=RADIO_LOSS_BRAKE_BALANCE,
        radio_loss_s=None,
        chassis_com_x_m=None,
    )
    print(
        f"[FIL Case 2 - 12s Perturbed draw_001 Stand + 6N/30ms Push (com_mm=-9.51mm)] "
        f"mass={res2['mass_kg']:.3f}kg "
        f"drift_x={res2['drift_x_m']:+.4f}m "
        f"max_|pitch|={res2['max_abs_pitch_deg']:.3f}deg "
        f"final_pitch={res2['final_pitch_deg']:.3f}deg "
        f"est_pitch={res2['est_pitch_deg']:.3f}deg "
        f"s9_trim={res2['s9_trim_deg']:.3f}deg "
        f"v_est={res2['est_v_m_s']:.4f}m/s "
        f"L0={res2['L0_left_m']:.4f}m "
        f"state={res2['final_state']} body_hit={res2['body_contact']}"
    )
    assert res2["s9_trim_deg"] > 5.0, f"S9 trim failed to learn (deadlock?): {res2['s9_trim_deg']:.3f} deg"
    assert abs(res2["final_pitch_deg"] - DRAW001_REF_EQ_PITCH_DEG) < 1.5, (
        f"Final |pitch - {DRAW001_REF_EQ_PITCH_DEG:.3f}| >= 1.5 deg: {res2['final_pitch_deg']:.3f} deg"
    )
    assert abs(res2["est_v_m_s"]) < 0.01, f"Residual creep exceeded 0.01 m/s: {res2['est_v_m_s']:.4f}"
    assert abs(res2["drift_x_m"]) < 0.35, f"Perturbed drift exceeded 0.35 m: {res2['drift_x_m']:+.4f} m"
    assert res2["max_abs_pitch_deg"] < 15.0, f"Pitch exceeded 15 deg: {res2['max_abs_pitch_deg']}"
    assert res2["max_abs_roll_deg"] < 15.0, f"Roll exceeded 15 deg: {res2['max_abs_roll_deg']}"
    assert not res2["body_contact"], "Body contacted floor during Case 2"
    assert res2["final_state"] == SAFETY_STATE_BALANCING, f"Unexpected state: {res2['final_state']}"

    # Case 3: 10.0 s standing + push + RADIO_LOSS_BRAKE_BALANCE (policy c) at t = 2.0 s
    res3 = run_episode(
        lib,
        duration_s=10.0,
        push_start_s=3.5,
        push_dur_s=0.030,
        push_force_n=4.0,
        radio_loss_policy=RADIO_LOSS_BRAKE_BALANCE,
        radio_loss_s=2.0,
        chassis_com_x_m=None,
    )
    print(
        f"[FIL Case 3 - 10s RADIO_LOSS_BRAKE_BALANCE Stand + Push] "
        f"drift_x={res3['drift_x_m']:+.4f}m "
        f"max_|pitch|={res3['max_abs_pitch_deg']:.3f}deg "
        f"max_|roll|={res3['max_abs_roll_deg']:.3f}deg "
        f"final_pitch={res3['final_pitch_deg']:.3f}deg "
        f"est_pitch={res3['est_pitch_deg']:.3f}deg "
        f"s9_trim={res3['s9_trim_deg']:.3f}deg "
        f"v_est={res3['est_v_m_s']:.4f}m/s "
        f"saw_stop={res3['saw_controlled_stop']} saw_rest={res3['saw_rest']} "
        f"final_state={res3['final_state']} body_hit={res3['body_contact']}"
    )
    assert res3["max_abs_pitch_deg"] < 15.0, f"Pitch exceeded 15 deg: {res3['max_abs_pitch_deg']}"
    assert res3["max_abs_roll_deg"] < 15.0, f"Roll exceeded 15 deg: {res3['max_abs_roll_deg']}"
    assert abs(res3["final_pitch_deg"] - DRAW001_REF_EQ_PITCH_DEG) < 1.5, (
        f"Policy c final |pitch - {DRAW001_REF_EQ_PITCH_DEG:.3f}| >= 1.5 deg: {res3['final_pitch_deg']:.3f} deg"
    )
    assert not res3["body_contact"], "Body contacted floor during Case 3"
    assert res3["saw_controlled_stop"], "Expected CONTROLLED_STOP on radio loss under policy c"
    assert res3["saw_rest"], "Expected REST confirmation after brake + dwell under policy c"
    assert res3["final_state"] == SAFETY_STATE_REST, f"Expected REST, got {res3['final_state']}"

    # Case 4: 13.46 s S11 G2 driving mission on nominal CAD CoM (com_mm = 0)
    res4 = run_episode(
        lib,
        duration_s=S11_G2_DURATION_S,
        push_start_s=S11_G2_PUSH_START_S,
        push_dur_s=S11_G2_PUSH_DUR_S,
        push_force_n=S11_G2_PUSH_FORCE_N,
        radio_loss_policy=RADIO_LOSS_BRAKE_BALANCE,
        radio_loss_s=None,
        chassis_com_x_m=NOMINAL_CAD_CHASSIS_COM_X_M,
        v_profile_fn=g2_speed_profile,
        yaw_profile_fn=g2_yaw_profile,
        host_loss_s=S11_G2_HOST_LOSS_S,
    )
    print(
        f"[FIL Case 4 - 13.46s S11 G2 Drive + Host Loss (Nominal com_mm=0)] "
        f"pre_push_v={res4['pre_push_mean_v_m_s']:.4f}m/s (ref018={S11_REF_NEAR_NOMINAL_018['pre_push_mean_v_m_s']:.4f}) "
        f"v_latch={res4['speed_at_latch_m_s']:.4f}m/s (ref018={S11_REF_NEAR_NOMINAL_018['speed_at_latch_m_s']:.4f}) "
        f"yaw_pre={res4['pre_loss_mean_yaw_rate_rad_s']:.4f}rad/s "
        f"max_|pitch|={res4['max_abs_pitch_deg']:.3f}deg (ref018={S11_REF_NEAR_NOMINAL_018['peak_abs_pitch_deg']:.3f}) "
        f"stop_path={res4['stop_path_m']:.4f}m path_end={res4['path_to_end_m']:.4f}m "
        f"tail_pitch={res4['tail_mean_pitch_deg']:.3f}deg final_pitch={res4['final_pitch_deg']:.3f}deg "
        f"s9_trim={res4['s9_trim_deg']:.3f}deg max_|x_err|={res4['max_abs_x_err_m']:.4f}m "
        f"latch_jump={res4['latch_pos_jump_m']:.5f}m state={res4['final_state']} body_hit={res4['body_contact']}"
    )
    assert abs(res4["pre_push_mean_v_m_s"] - 0.20) < 0.03, (
        f"Case 4 speed tracking error too large: {res4['pre_push_mean_v_m_s']:.4f} m/s"
    )
    assert abs(res4["speed_at_latch_m_s"] - 0.20) < 0.04, (
        f"Case 4 speed at host loss too far from 0.20 m/s: {res4['speed_at_latch_m_s']:.4f} m/s"
    )
    assert abs(res4["pre_loss_mean_yaw_rate_rad_s"] - 0.30) < 0.06, (
        f"Case 4 yaw tracking error too large: {res4['pre_loss_mean_yaw_rate_rad_s']:.4f} rad/s"
    )
    assert res4["max_abs_pitch_deg"] < 10.0, f"Case 4 peak pitch exceeded 10 deg: {res4['max_abs_pitch_deg']:.3f}"
    assert res4["max_abs_roll_deg"] < 10.0, f"Case 4 peak roll exceeded 10 deg: {res4['max_abs_roll_deg']:.3f}"
    assert res4["stop_path_m"] < 0.40, f"Case 4 stop path exceeded 0.40 m: {res4['stop_path_m']:.4f} m"
    assert abs(res4["final_pitch_deg"] - NOMINAL_REF_EQ_PITCH_DEG) < 1.5, (
        f"Case 4 rest pitch |{res4['final_pitch_deg']:.3f} - {NOMINAL_REF_EQ_PITCH_DEG:.3f}| >= 1.5 deg"
    )
    assert abs(res4["est_v_m_s"]) < 0.01, f"Case 4 residual speed exceeded 0.01 m/s: {res4['est_v_m_s']:.4f}"
    assert res4["latch_pos_jump_m"] < 0.005, f"Case 4 host-loss latch jump: {res4['latch_pos_jump_m']:.5f} m"
    assert res4["saw_host_loss_reason"], "Case 4 expected SAFETY_REASON_HOST_LOSS"
    assert res4["saw_controlled_stop"] and res4["saw_rest"], "Case 4 expected CONTROLLED_STOP -> REST"
    assert res4["final_state"] == SAFETY_STATE_REST, f"Case 4 expected REST, got {res4['final_state']}"
    assert not res4["body_contact"], "Body contacted floor during Case 4"

    # Case 5: 13.46 s S11 G2 driving mission on perturbed draw_001.xml (com_mm = -9.51 mm)
    res5 = run_episode(
        lib,
        duration_s=S11_G2_DURATION_S,
        push_start_s=S11_G2_PUSH_START_S,
        push_dur_s=S11_G2_PUSH_DUR_S,
        push_force_n=S11_G2_PUSH_FORCE_N,
        radio_loss_policy=RADIO_LOSS_BRAKE_BALANCE,
        radio_loss_s=None,
        chassis_com_x_m=None,
        v_profile_fn=g2_speed_profile,
        yaw_profile_fn=g2_yaw_profile,
        host_loss_s=S11_G2_HOST_LOSS_S,
    )
    print(
        f"[FIL Case 5 - 13.46s S11 G2 Drive + Host Loss (draw_001 com_mm=-9.51mm)] "
        f"pre_push_v={res5['pre_push_mean_v_m_s']:.4f}m/s (ref001={S11_REF_DRAW001['pre_push_mean_v_m_s']:.4f}) "
        f"v_latch={res5['speed_at_latch_m_s']:.4f}m/s (ref001={S11_REF_DRAW001['speed_at_latch_m_s']:.4f}) "
        f"yaw_pre={res5['pre_loss_mean_yaw_rate_rad_s']:.4f}rad/s "
        f"max_|pitch|={res5['max_abs_pitch_deg']:.3f}deg (ref001={S11_REF_DRAW001['peak_abs_pitch_deg']:.3f}) "
        f"stop_path={res5['stop_path_m']:.4f}m (ref001={S11_REF_DRAW001['stop_path_m']:.4f}) "
        f"path_end={res5['path_to_end_m']:.4f}m "
        f"tail_pitch={res5['tail_mean_pitch_deg']:.3f}deg final_pitch={res5['final_pitch_deg']:.3f}deg "
        f"s9_trim={res5['s9_trim_deg']:.3f}deg max_|x_err|={res5['max_abs_x_err_m']:.4f}m "
        f"latch_jump={res5['latch_pos_jump_m']:.5f}m state={res5['final_state']} body_hit={res5['body_contact']}"
    )
    assert abs(res5["pre_push_mean_v_m_s"] - 0.20) < 0.05, (
        f"Case 5 speed tracking error too large: {res5['pre_push_mean_v_m_s']:.4f} m/s"
    )
    assert abs(res5["speed_at_latch_m_s"] - 0.20) < 0.05, (
        f"Case 5 speed at host loss too far from 0.20 m/s: {res5['speed_at_latch_m_s']:.4f} m/s"
    )
    assert abs(res5["pre_loss_mean_yaw_rate_rad_s"] - 0.30) < 0.08, (
        f"Case 5 yaw tracking error too large: {res5['pre_loss_mean_yaw_rate_rad_s']:.4f} rad/s"
    )
    assert res5["max_abs_pitch_deg"] < 15.0, f"Case 5 peak pitch exceeded 15 deg: {res5['max_abs_pitch_deg']:.3f}"
    assert res5["max_abs_roll_deg"] < 15.0, f"Case 5 peak roll exceeded 15 deg: {res5['max_abs_roll_deg']:.3f}"
    assert res5["stop_path_m"] < 0.90, f"Case 5 stop path exceeded 0.90 m: {res5['stop_path_m']:.4f} m"
    assert res5["s9_trim_deg"] > 4.5, f"Case 5 S9 trim failed to converge: {res5['s9_trim_deg']:.3f} deg"
    assert abs(res5["final_pitch_deg"] - DRAW001_REF_EQ_PITCH_DEG) < 1.5, (
        f"Case 5 rest pitch |{res5['final_pitch_deg']:.3f} - {DRAW001_REF_EQ_PITCH_DEG:.3f}| >= 1.5 deg"
    )
    assert abs(res5["est_v_m_s"]) < 0.01, f"Case 5 residual speed exceeded 0.01 m/s: {res5['est_v_m_s']:.4f}"
    assert res5["latch_pos_jump_m"] < 0.005, f"Case 5 host-loss latch jump: {res5['latch_pos_jump_m']:.5f} m"
    assert res5["saw_host_loss_reason"], "Case 5 expected SAFETY_REASON_HOST_LOSS"
    assert res5["saw_controlled_stop"] and res5["saw_rest"], "Case 5 expected CONTROLLED_STOP -> REST"
    assert res5["final_state"] == SAFETY_STATE_REST, f"Case 5 expected REST, got {res5['final_state']}"
    assert not res5["body_contact"], "Body contacted floor during Case 5"

    # Case 6: 12.0 s RADIO_LOSS_BRAKE_BALANCE (policy c) while driving at 0.20 m/s on draw_001.xml
    res6 = run_episode(
        lib,
        duration_s=12.0,
        push_start_s=0.0,
        push_dur_s=0.0,
        push_force_n=0.0,
        radio_loss_policy=RADIO_LOSS_BRAKE_BALANCE,
        radio_loss_s=4.0,
        chassis_com_x_m=None,
        v_profile_fn=g2_speed_profile,
        yaw_profile_fn=None,
        host_loss_s=None,
    )
    print(
        f"[FIL Case 6 - 12s RADIO_LOSS_BRAKE_BALANCE While Driving 0.20m/s (draw_001)] "
        f"pre_loss_v={res6['pre_push_mean_v_m_s']:.4f}m/s v_latch={res6['speed_at_latch_m_s']:.4f}m/s "
        f"max_|pitch|={res6['max_abs_pitch_deg']:.3f}deg max_|roll|={res6['max_abs_roll_deg']:.3f}deg "
        f"stop_path={res6['stop_path_m']:.4f}m path_end={res6['path_to_end_m']:.4f}m "
        f"final_pitch={res6['final_pitch_deg']:.3f}deg s9_trim={res6['s9_trim_deg']:.3f}deg "
        f"v_est={res6['est_v_m_s']:.4f}m/s latch_jump={res6['latch_pos_jump_m']:.5f}m "
        f"saw_stop={res6['saw_controlled_stop']} saw_rest={res6['saw_rest']} "
        f"final_state={res6['final_state']} body_hit={res6['body_contact']}"
    )
    assert abs(res6["pre_push_mean_v_m_s"] - 0.20) < 0.05, (
        f"Case 6 pre-loss cruise speed error too large: {res6['pre_push_mean_v_m_s']:.4f} m/s"
    )
    assert abs(res6["speed_at_latch_m_s"] - 0.20) < 0.05, (
        f"Case 6 speed at radio loss too far from 0.20 m/s: {res6['speed_at_latch_m_s']:.4f} m/s"
    )
    assert res6["max_abs_pitch_deg"] < 15.0, f"Case 6 peak pitch exceeded 15 deg: {res6['max_abs_pitch_deg']:.3f}"
    assert res6["max_abs_roll_deg"] < 15.0, f"Case 6 peak roll exceeded 15 deg: {res6['max_abs_roll_deg']:.3f}"
    assert res6["stop_path_m"] < 0.90, f"Case 6 stop path exceeded 0.90 m: {res6['stop_path_m']:.4f} m"
    assert abs(res6["final_pitch_deg"] - DRAW001_REF_EQ_PITCH_DEG) < 1.5, (
        f"Case 6 rest pitch |{res6['final_pitch_deg']:.3f} - {DRAW001_REF_EQ_PITCH_DEG:.3f}| >= 1.5 deg"
    )
    assert abs(res6["est_v_m_s"]) < 0.01, f"Case 6 residual speed exceeded 0.01 m/s: {res6['est_v_m_s']:.4f}"
    assert res6["latch_pos_jump_m"] < 0.005, f"Case 6 radio-loss latch jump: {res6['latch_pos_jump_m']:.5f} m"
    assert res6["saw_radio_loss_brake_reason"], "Case 6 expected SAFETY_REASON_RADIO_LOSS_BRAKE"
    assert res6["saw_controlled_stop"] and res6["saw_rest"], "Case 6 expected CONTROLLED_STOP -> REST"
    assert res6["final_state"] == SAFETY_STATE_REST, f"Case 6 expected REST, got {res6['final_state']}"
    assert not res6["body_contact"], "Body contacted floor during Case 6"

    print(
        "FIL PASSED (6/6): balanced stand, draw_001 stand, standing policy c, "
        "S11 G2 nominal drive, S11 G2 draw_001 drive, and driving policy c REST all verified."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
