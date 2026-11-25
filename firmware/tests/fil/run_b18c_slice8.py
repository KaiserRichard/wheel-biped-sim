"""Slice 8 B18c reproduction + full firmware telemetry diagnostic harness.

Runs the full 19-case B18c benchmark matrix (plus two step-physics sensitivity
probes: mu=0.8 on step5mm and a chamfered 5 mm ramp) against this workspace's
compiled build/libwbr_balance_ctrl.so and sim_model/draw_001.xml.

Unlike b18c_ref/run_b18c.py (which discarded outt/diag and omitted safety_reason),
this script calls balance_ctrl_fil_step_500hz_ex() and records:
  - safety_state, safety_reason, s11_mode, torque_enable
  - est_pitch_deg, est_roll_deg, est_v_m_s, x_err_m, L0_L_m, L0_R_m
  - wheel & hip torque commands, wheel & hip angular rates, body_contact
  - exact first-fault timestamp and a 500 Hz pre/post-fault window CSV.
"""

from pathlib import Path
import os
import ctypes
import csv
import math
import sys
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str((ROOT / "tests" / "fil").resolve()))
import run_fil as fil  # noqa: E402

OUT_DIR = Path(os.environ.get("WBR_B18C_OUT", str(ROOT / "tests" / "fil" / "b18c_out")))
MODELS_DIR = OUT_DIR / "models"
BASE_XML = ROOT / "sim_model" / "draw_001.xml"
MESH_DIR = (ROOT / "sim_model" / "meshes").resolve()

SAFETY_STATE_NAMES = {
    0: "DISARMED",
    1: "PRECHARGE",
    2: "ARMED",
    3: "BALANCING",
    4: "CONTROLLED_STOP",
    5: "REST",
    6: "FAULT",
    7: "ESTOP",
    8: "STEP_ASSIST",
}

SAFETY_REASON_NAMES = {
    0: "NONE",
    1: "POWER_ON",
    2: "PRECHARGE_START",
    3: "PRECHARGE_OK",
    4: "STAND_HANDOFF",
    5: "HOST_LOSS",
    6: "VOLUNTARY_STOP",
    7: "RADIO_LOSS_BRAKE",
    8: "REST_CONFIRMED",
    9: "LOCAL_REGRANT",
    10: "CLEAN_DISARM",
    11: "MANUAL_RESET",
    12: "BOOT_SELF_TEST",
    13: "BOR3",
    14: "CAN_BUS_OFF",
    15: "CLAMP_FAULT",
    16: "LINK_NOT_DISCHARGED",
    17: "PRECHARGE_TIMEOUT",
    18: "MIRROR_MISMATCH",
    19: "DRIVE_ENABLE_FAIL",
    20: "TILT_LIMIT",
    21: "S11_FAULT",
    22: "HARD_UV",
    23: "PERSISTENT_OV",
    24: "DRIVE_FAULT",
    25: "WDG_FAULT",
    26: "IMU_EST_INVALID",
    27: "RADIO_LOSS_FAULT",
    28: "RADIO_ESTOP_SW",
    29: "ARM_SW_DROPPED",
    30: "ESTOP_HW",
    31: "S11_DEGRADED",
    32: "STEP_ASSIST_ABORT",  # FW6FIX D3
    33: "STOP_TIMEOUT",       # FW6FIX D2
}

S11_MODE_NAMES = {
    0: "S11_FRESH",
    1: "S11_PREDICT",
    2: "S11_LIMIT",
    3: "S11_TIMEOUT",
    4: "S11_INVALID",
    5: "S11_DRIVE_FAULT",
    6: "S11_STUCK",
    7: "S11_DISARMED",
    8: "S11_ARMING",
}

COM_OVERRIDE = {"nominal_com0_5p06kg": fil.NOMINAL_CAD_CHASSIS_COM_X_M}
MASS_TAG = {
    "nominal_com0_5p06kg": "[COMPUTED]",
    "draw_001_5p06kg": "[COMPUTED]",
    "assumed_6p2kg": "[ASSUMED]",
}


def build_models() -> dict[str, Path]:
    MODELS_DIR.mkdir(parents=True, exist_ok=True)
    model_xml: dict[str, Path] = {}

    # 1. draw_001 (5.06 kg, XML CoM)
    tree = ET.parse(BASE_XML)
    tree.getroot().find("compiler").set("meshdir", str(MESH_DIR))
    p_draw = MODELS_DIR / "draw_001_5p06.xml"
    tree.write(p_draw, encoding="unicode")
    model_xml["draw_001_5p06kg"] = p_draw

    # 2. nominal_com0 (5.06 kg, CoM at model_base_imu_site)
    nom = ET.parse(BASE_XML)
    nroot = nom.getroot()
    chassis = next(b for b in nroot.iter("body") if b.get("name") == "chassis")
    chassis.find("inertial").set(
        "pos", "0.00265373721167746 1.5332899978765e-07 0.0344707443228917"
    )
    nroot.find("compiler").set("meshdir", str(MESH_DIR))
    p_nom = MODELS_DIR / "nominal_com0.xml"
    nom.write(p_nom, encoding="unicode")
    model_xml["nominal_com0_5p06kg"] = p_nom

    # 3. assumed_6p2kg (+1.144426 kg placeholder battery box)
    mass = sum(float(el.get("mass", "0")) for el in nroot.iter("inertial"))
    added = 6.2 - mass
    body = ET.SubElement(
        chassis, "body", {"name": "assumed_battery_mass_lump", "pos": "0 0 0.029"}
    )
    ET.SubElement(
        body,
        "inertial",
        {
            "mass": f"{added:.12f}",
            "pos": "0 0 0",
            "diaginertia": (
                f"{added / 12.0 * (0.06**2 + 0.04**2):.12g} "
                f"{added / 12.0 * (0.12**2 + 0.04**2):.12g} "
                f"{added / 12.0 * (0.12**2 + 0.06**2):.12g}"
            ),
        },
    )
    p_62 = MODELS_DIR / "nominal_6p2_assumed.xml"
    nom.write(p_62, encoding="unicode")
    model_xml["assumed_6p2kg"] = p_62

    # 4. Step 5 mm and 10 mm vertical-box fields (exact B18c geometry)
    for label in ("nominal_com0_5p06kg", "draw_001_5p06kg", "assumed_6p2kg"):
        base = model_xml[label]
        for height_mm in (5, 10):
            stree = ET.parse(base)
            world = stree.getroot().find("worldbody")
            heights = [height_mm / 1000.0] * 20
            x0, patch_len = 0.20, 0.15
            pitch = (4.36 - patch_len) / (len(heights) - 1)
            for i, h in enumerate(heights):
                x = x0 + i * pitch
                ET.SubElement(
                    world,
                    "geom",
                    {
                        "name": f"step_{i + 1:02d}",
                        "type": "box",
                        "pos": f"{x + patch_len / 2:.6f} 0 {h / 2:.6f}",
                        "size": f"{patch_len / 2:.6f} .35 {h / 2:.6f}",
                        "friction": "0.37884506146961616 0.005 0.0001",
                        "solref": ".008 1",
                        "solimp": ".95 .99 .001",
                    },
                )
            target = MODELS_DIR / f"{label}_step{height_mm}mm.xml"
            stree.write(target, encoding="unicode")
            model_xml[f"{label}_step{height_mm}mm"] = target

    # 5. draw_001 mu=0.8 lateral sensitivity model
    mu_tree = ET.parse(model_xml["draw_001_5p06kg"])
    for geom in mu_tree.getroot().iter("geom"):
        if geom.get("name") in ("left_wheel_collision", "right_wheel_collision"):
            geom.set("friction", "0.8 0.005 0.0001")
    p_mu08 = MODELS_DIR / "draw_001_5p06_mu0p8.xml"
    mu_tree.write(p_mu08, encoding="unicode")
    model_xml["draw_001_mu0p8"] = p_mu08

    # 6. Diagnostic probe: nominal step5mm with wheel/step friction mu=0.8
    #    (where mu=0.8 > tan(alpha)=0.435 so static climbing is physically feasible)
    probe_mu = ET.parse(model_xml["nominal_com0_5p06kg_step5mm"])
    for geom in probe_mu.getroot().iter("geom"):
        gname = geom.get("name", "")
        if gname in ("left_wheel_collision", "right_wheel_collision", "floor") or gname.startswith("step_"):
            geom.set("friction", "0.8 0.005 0.0001")
    p_probe_mu = MODELS_DIR / "nominal_com0_5p06kg_step5mm_mu0p8.xml"
    probe_mu.write(p_probe_mu, encoding="unicode")
    model_xml["nominal_step5mm_mu0p8_probe"] = p_probe_mu

    return model_xml


def setup_rig(lib: ctypes.CDLL, xml_path: Path, label: str):
    rig = fil.RobotRig(xml_path, chassis_com_x_m=COM_OVERRIDE.get(label))
    rig.reset_standing(fil.LEG_REF_M)
    state = ctypes.create_string_buffer(lib.balance_ctrl_state_size())
    lib.balance_ctrl_fil_init(state, rig.total_mass, fil.LEG_REF_M, fil.RADIO_LOSS_BRAKE_BALANCE)
    rot0 = rig.d.xmat[1].reshape(3, 3)
    acc0 = rot0.T @ np.array([0.0, 0.0, fil.GRAVITY], dtype=np.float64)
    gyro0 = np.zeros(3, dtype=np.float64)
    cgyro = (ctypes.c_double * 3)(*gyro0)
    cacc = (ctypes.c_double * 3)(*acc0)
    for tick in range(-120, -10):
        lib.balance_ctrl_fil_imu_1khz(state, tick, cgyro, cacc)
    angles, rates = rig.read_encoders()
    cang = (ctypes.c_double * 6)(*angles)
    crate = (ctypes.c_double * 6)(*rates)
    for tick in range(-10, 0):
        lib.balance_ctrl_fil_imu_1khz(state, tick, cgyro, cacc)
    handoff = lib.balance_ctrl_fil_arm_and_handoff(state, 10000, -10, cang, crate)
    if handoff != fil.SAFETY_STATE_BALANCING:
        raise RuntimeError(f"handoff state={handoff}")
    return rig, state, cgyro, cacc, cang, crate


def run_case(
    lib: ctypes.CDLL,
    label: str,
    scenario: str,
    duration: float,
    xml_path: Path,
    push_axis: str | None = None,
    mu_tag: str = "0.378845",
    observer=None,
):
    rig, state, cgyro, cacc, cang, crate = setup_rig(lib, xml_path, label)
    x0, y0 = float(rig.d.qpos[0]), float(rig.d.qpos[1])

    outt = (ctypes.c_float * 6)()
    diag = (ctypes.c_float * 10)()
    drive_diag = (ctypes.c_float * 4)()
    applied = np.zeros(6, dtype=np.float64)
    state_now = fil.SAFETY_STATE_BALANCING
    reason_now = 0
    s11_now = 0
    torque_en = 1

    trace_10ms = []
    ring_500hz = []
    first_fault = None
    post_fault_left = 0
    fault_window = []

    wall0 = time.monotonic()
    n_steps = int(round(duration / fil.DT))
    last_rates = np.zeros(6, dtype=np.float64)
    last_torq = np.zeros(6, dtype=np.float64)
    torque_feedback = (ctypes.c_float * 6)()

    for k in range(n_steps):
        t = k * fil.DT
        force_on = push_axis is not None and 2.0 <= t < 2.1
        rig.d.xfrc_applied[1, :] = 0.0
        if force_on:
            rig.d.xfrc_applied[1, 0 if push_axis == "sagittal" else 1] = 30.0
        mujoco.mj_forward(rig.m, rig.d)

        gyro, acc = rig.read_imu()
        for j in range(3):
            cgyro[j], cacc[j] = float(gyro[j]), float(acc[j])
        lib.balance_ctrl_fil_imu_1khz(state, k, cgyro, cacc)

        if k % 2 == 0:
            for j in range(6):
                torque_feedback[j] = float(last_torq[j])
            if hasattr(lib, "balance_ctrl_fil_torque_feedback"):
                lib.balance_ctrl_fil_torque_feedback(state, torque_feedback)
            angles, rates = rig.read_encoders()
            last_rates = rates.copy()
            for j in range(6):
                cang[j], crate[j] = float(angles[j]), float(rates[j])
            target = 0.10 if (scenario == "run_3m" or scenario.startswith("step")) else 0.0
            state_now = lib.balance_ctrl_fil_step_500hz_ex(
                state,
                20000 + k * 1000,
                k,
                cang,
                crate,
                float(target),
                0.0,
                0.0,
                0.0,
                0,
                0,
                1 if force_on else 0,
                outt,
                diag,
                drive_diag,
            )
            last_torq = np.array([float(outt[j]) for j in range(6)], dtype=np.float64)
            applied = rig.sign6 * last_torq
            s11_now = int(round(float(diag[8])))
            torque_en = int(round(float(diag[9])))
            reason_now = int(round(float(drive_diag[3])))

        rig.d.ctrl[rig.axis] = applied
        mujoco.mj_step(rig.m, rig.d)
        pitch, roll, height, body_hit = rig.chassis_metrics()

        if k % 2 == 0:
            max_hip_tau = float(max(abs(last_torq[0]), abs(last_torq[1]), abs(last_torq[3]), abs(last_torq[4])))
            max_hip_rate = float(max(abs(last_rates[0]), abs(last_rates[1]), abs(last_rates[3]), abs(last_rates[4])))
            rec = (
                t,
                float(rig.d.qpos[0] - x0),
                float(rig.d.qpos[1] - y0),
                pitch,
                roll,
                height,
                int(body_hit),
                int(force_on),
                state_now,
                reason_now,
                s11_now,
                torque_en,
                math.degrees(float(diag[0])),
                math.degrees(float(diag[1])),
                float(diag[2]),
                float(drive_diag[2]),
                float(diag[3]),
                float(diag[4]),
                float(last_torq[2]),
                float(last_torq[5]),
                max_hip_tau,
                float(last_rates[2]),
                float(last_rates[5]),
                max_hip_rate,
            )
            if observer is not None:
                observer(state, rig, rec)
            if first_fault is None:
                ring_500hz.append(rec)
                if len(ring_500hz) > 100:
                    ring_500hz.pop(0)
                if state_now == 6:
                    first_fault = rec
                    fault_window.extend(ring_500hz)
                    post_fault_left = 25
            elif post_fault_left > 0:
                fault_window.append(rec)
                post_fault_left -= 1

            if k % 10 == 0:
                trace_10ms.append(rec)

        if rig.d.warning.number.any():
            raise RuntimeError(f"MuJoCo warning at t={t:.3f}s")

    wall = time.monotonic() - wall0
    a = np.asarray(trace_10ms, dtype=float)
    t_arr = a[:, 0]
    row = {
        "model": label,
        "mass_kg": rig.total_mass,
        "scenario": scenario,
        "mu": mu_tag,
        "duration_s": duration,
        "final_state": state_now,
        "final_state_name": SAFETY_STATE_NAMES.get(state_now, str(state_now)),
        "first_fault_t_s": "" if first_fault is None else f"{first_fault[0]:.4f}",
        "first_fault_reason": "" if first_fault is None else SAFETY_REASON_NAMES.get(first_fault[9], str(first_fault[9])),
        "first_fault_s11": "" if first_fault is None else S11_MODE_NAMES.get(first_fault[10], str(first_fault[10])),
        "first_fault_pitch_deg": "" if first_fault is None else f"{first_fault[3]:.3f}",
        "first_fault_roll_deg": "" if first_fault is None else f"{first_fault[4]:.3f}",
        "first_fault_est_v_m_s": "" if first_fault is None else f"{first_fault[14]:.4f}",
        "first_fault_wheel_tau_nm": "" if first_fault is None else f"{0.5 * (first_fault[18] + first_fault[19]):.3f}",
        "first_fault_max_wheel_rate": "" if first_fault is None else f"{max(abs(first_fault[21]), abs(first_fault[22])):.3f}",
        "body_contact": bool(np.any(a[:, 6])),
        "max_abs_pitch_deg": float(np.max(np.abs(a[:, 3]))),
        "max_abs_roll_deg": float(np.max(np.abs(a[:, 4]))),
        "drift_x_m": float(a[-1, 1]),
        "drift_y_m": float(a[-1, 2]),
        "wall_s": round(wall, 3),
        "claim_tag": MASS_TAG.get(label, "[COMPUTED]"),
    }
    if push_axis:
        pre = a[(t_arr >= 1.0) & (t_arr < 2.0), 3]
        win = a[(t_arr >= 2.0) & (t_arr <= 5.0)]
        row["push_peak_pitch_deg"] = float(np.max(np.abs(win[:, 3] - float(np.mean(pre)))))
        row["push_peak_roll_deg"] = float(
            np.max(np.abs(win[:, 4] - float(np.mean(a[(t_arr >= 1.0) & (t_arr < 2.0), 4]))))
        )
    else:
        row["push_peak_pitch_deg"] = ""
        row["push_peak_roll_deg"] = ""
    if scenario.startswith("step"):
        row["pitch_rms_deg"] = float(np.sqrt(np.mean(a[:, 3] ** 2)))
    else:
        row["pitch_rms_deg"] = ""
    return row, trace_10ms, fault_window


def main() -> int:
    lib = fil.load_library(fil.LIB_PATH)
    model_xml = build_models()
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    rows = []
    traces = []
    fault_windows = []

    for label in ("nominal_com0_5p06kg", "draw_001_5p06kg", "assumed_6p2kg"):
        scenarios = [
            ("stand", 12.0, model_xml[label], None, "0.378845"),
            ("push_sagittal", 12.0, model_xml[label], "sagittal", "0.378845"),
            ("push_lateral", 12.0, model_xml[label], "lateral", "0.378845"),
            ("run_3m", 30.0, model_xml[label], None, "0.378845"),
            ("step5mm", 43.6, model_xml[f"{label}_step5mm"], None, "0.378845"),
            ("step10mm", 43.6, model_xml[f"{label}_step10mm"], None, "0.378845"),
        ]
        for scenario, duration, xml, axis, mu_tag in scenarios:
            row, tr, fw = run_case(lib, label, scenario, duration, xml, axis, mu_tag)
            rows.append(row)
            traces.extend((label, scenario, *r) for r in tr)
            fault_windows.extend((label, scenario, *r) for r in fw)
            print(
                f"{label:20s} {scenario:22s} state={row['final_state_name']:10s} "
                f"fault_t={row['first_fault_t_s']:7s} reason={row['first_fault_reason']:10s} "
                f"s11={row['first_fault_s11']:11s} body_hit={str(row['body_contact']):5s} "
                f"max_p={row['max_abs_pitch_deg']:6.2f} max_r={row['max_abs_roll_deg']:5.2f}",
                flush=True,
            )

    # B18c case 19: draw_001 push_lateral mu=0.8
    for label, scenario, dur, xml_key, axis, mu_tag in (
        ("draw_001_5p06kg", "push_lateral_mu0p8", 12.0, "draw_001_mu0p8", "lateral", "0.8"),
        ("nominal_com0_5p06kg", "step5mm_mu0p8_probe", 15.0, "nominal_step5mm_mu0p8_probe", None, "0.8"),
    ):
        row, tr, fw = run_case(lib, label, scenario, dur, model_xml[xml_key], axis, mu_tag)
        rows.append(row)
        traces.extend((label, scenario, *r) for r in tr)
        fault_windows.extend((label, scenario, *r) for r in fw)
        print(
            f"{label:20s} {scenario:22s} state={row['final_state_name']:10s} "
            f"fault_t={row['first_fault_t_s']:7s} reason={row['first_fault_reason']:10s} "
            f"s11={row['first_fault_s11']:11s} body_hit={str(row['body_contact']):5s} "
            f"max_p={row['max_abs_pitch_deg']:6.2f} max_r={row['max_abs_roll_deg']:5.2f}",
            flush=True,
        )

    trace_header = [
        "model", "scenario", "time_s", "x_m", "y_m", "pitch_deg", "roll_deg",
        "body_height_m", "body_contact", "push_active",
        "safety_state", "safety_reason", "s11_mode", "torque_enable",
        "est_pitch_deg", "est_roll_deg", "est_v_m_s", "x_err_m", "L0_L_m", "L0_R_m",
        "tau_L_wheel_nm", "tau_R_wheel_nm", "max_abs_hip_tau_nm",
        "rate_L_wheel_rad_s", "rate_R_wheel_rad_s", "max_abs_hip_rate_rad_s",
    ]
    with (OUT_DIR / "results_slice8.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    with (OUT_DIR / "traces_slice8.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(trace_header)
        w.writerows(traces)
    with (OUT_DIR / "fault_windows.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(trace_header)
        w.writerows(fault_windows)

    print(f"DONE slice8 B18c diagnostic: cases={len(rows)}, fault_rows={len(fault_windows)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
