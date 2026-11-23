/*
 * Wheel-Biped Safety Supervisor State Machine
 *
 * Implements FIRMWARE_DESIGN_v2.md Section 6 (6.1 Policy Summary, 6.2 State Diagram,
 * 6.3 Transition & Action Matrix).
 *
 * Pure C11, zero HAL/RTOS dependencies, zero dynamic memory, fixed-width types.
 */

#ifndef WBR_SAFETY_FSM_H
#define WBR_SAFETY_FSM_H

#include <stdint.h>

/* Compile-time ExpressLRS RF-loss policy switch (FIRMWARE_DESIGN_v2.md Section 6.1).
 * 0 (default, S12 recommendation): latched immediate CAN torque-off (-> FAULT).
 * 1 (owner option, S14 policy c): 0.60 s local brake -> REST (keep balancing). */
#define SAFETY_RADIO_LOSS_TORQUE_OFF    0u
#define SAFETY_RADIO_LOSS_BRAKE_BALANCE 1u

#ifndef RADIO_LOSS_POLICY
#define RADIO_LOSS_POLICY SAFETY_RADIO_LOSS_TORQUE_OFF
#endif

typedef enum {
    SAFETY_STATE_DISARMED = 0,
    SAFETY_STATE_PRECHARGE = 1,
    SAFETY_STATE_ARMED = 2,
    SAFETY_STATE_BALANCING = 3,
    SAFETY_STATE_CONTROLLED_STOP = 4,
    SAFETY_STATE_REST = 5,
    SAFETY_STATE_FAULT = 6,
    SAFETY_STATE_ESTOP = 7
} SafetyState;

typedef enum {
    SAFETY_REASON_NONE = 0,
    SAFETY_REASON_POWER_ON,
    SAFETY_REASON_PRECHARGE_START,
    SAFETY_REASON_PRECHARGE_OK,
    SAFETY_REASON_STAND_HANDOFF,
    SAFETY_REASON_HOST_LOSS,
    SAFETY_REASON_VOLUNTARY_STOP,
    SAFETY_REASON_RADIO_LOSS_BRAKE,
    SAFETY_REASON_REST_CONFIRMED,
    SAFETY_REASON_LOCAL_REGRANT,
    SAFETY_REASON_CLEAN_DISARM,
    SAFETY_REASON_MANUAL_RESET,
    SAFETY_REASON_BOOT_SELF_TEST,
    SAFETY_REASON_BOR3,
    SAFETY_REASON_CAN_BUS_OFF,
    SAFETY_REASON_CLAMP_FAULT,
    SAFETY_REASON_LINK_NOT_DISCHARGED,
    SAFETY_REASON_PRECHARGE_TIMEOUT,
    SAFETY_REASON_MIRROR_MISMATCH,
    SAFETY_REASON_DRIVE_ENABLE_FAIL,
    SAFETY_REASON_TILT_LIMIT,
    SAFETY_REASON_S11_FAULT,
    SAFETY_REASON_HARD_UV,
    SAFETY_REASON_PERSISTENT_OV,
    SAFETY_REASON_DRIVE_FAULT,
    SAFETY_REASON_WDG_FAULT,
    SAFETY_REASON_IMU_EST_INVALID,
    SAFETY_REASON_RADIO_LOSS_FAULT,
    SAFETY_REASON_RADIO_ESTOP_SW,
    SAFETY_REASON_ARM_SW_DROPPED,
    SAFETY_REASON_ESTOP_HW
} SafetyReason;

typedef struct {
    uint8_t  radio_loss_policy;       /* SAFETY_RADIO_LOSS_TORQUE_OFF or _BRAKE_BALANCE */
    uint32_t precharge_timeout_ms;    /* Default 600 ms (S13) */
    uint32_t radio_timeout_ms;        /* Default 150 ms (S12) */
    uint32_t host_timeout_ms;         /* Default 150 ms (S12) */
    uint32_t brake_duration_ms;       /* Default 600 ms (S8/S9/supervisor.py) */
    uint32_t rest_dwell_ms;           /* Default 250 ms (supervisor.py) */
    uint32_t tilt_dwell_ms;           /* Default 20 ms (supervisor.py) */
    uint32_t ov_deglitch_ms;          /* Default 20 ms persistent OV deglitch */
    float    link_discharged_max_v;   /* Default 5.0 V (S13 bleeder threshold) */
    float    arm_vbus_min_v;          /* Default 20.5 V (V1.2 >20 V + margin) */
    float    mild_sag_v;              /* Default 18.5 V (jump inhibit / set-down) */
    float    hard_uv_v;               /* Default 15.0 V (drive UV floor) */
    float    regen_taper_start_v;     /* Default 25.8 V (FW regen taper start) */
    float    regen_taper_full_v;      /* Default 26.8 V (FW regen taper to 0) */
    float    persistent_ov_v;         /* Default 28.0 V (sustained OV trip) */
    float    immediate_ov_v;          /* Default 32.0 V (drive OV immediate trip) */
    float    stand_pitch_max_deg;     /* Default 5.0 deg */
    float    tilt_limit_deg;          /* Default 30.0 deg (lab mode: 12.0 deg) */
    float    rest_pitch_max_deg;      /* Default 5.0 deg */
    float    rest_vel_max_m_s;        /* Default 0.02 m/s */
    float    rest_pitch_rate_max_deg_s; /* Default 2.0 deg/s */
} SafetyConfig;

typedef struct {
    uint32_t dt_ms;              /* Tick period in ms (e.g. 2 for 500 Hz) */

    /* Hardware stop chain & power path (S13) */
    uint8_t  estop_open;         /* 1 = PD7 wired NC mushroom or S13 safety-radio stop open */
    uint8_t  boot_self_test_ok;  /* 1 = boot self-test passed */
    uint8_t  bor3_fault;         /* 1 = brown-out reset fault active */
    uint8_t  can_bus_off;        /* 1 = any FDCAN bus-off latched */
    uint8_t  clamp_fault;        /* 1 = regen chopper / thermal clamp fault */
    uint8_t  dump_continuity_ok; /* 1 = pre-arm dump resistor continuity ok */
    uint8_t  k1_closed;          /* 1 = K1 main contactor mirror closed */
    uint8_t  k2_closed;          /* 1 = K2 return contactor mirror closed */
    uint8_t  kpre_closed;        /* 1 = Kpre precharge contactor mirror closed */
    uint8_t  precharge_dv_ok;    /* 1 = |V_batt - V_bus| <= 1.0 V */
    uint8_t  mirror_mismatch;    /* 1 = contactor mirror mismatch detected */
    uint8_t  hard_uv_fault;      /* 1 = logic PG loss / drive UV fault */
    uint8_t  wdg_fault;          /* 1 = critical task heartbeat/WDG fault */
    float    v_bus_v;            /* Measured downstream motor link voltage (V) */
    float    v_batt_ocv_v;       /* Measured upstream battery voltage (V) */

    /* Drives, S11 packet policy, IMU/estimator, contact */
    uint8_t  drives_healthy_6;   /* 1 = all 6 motors replying with healthy status */
    uint8_t  drive_enable_fail;  /* 1 = drive rejected/failed 0xFC enable */
    uint8_t  s11_fault;          /* 1 = S11 packet guard latched fault */
    uint8_t  s11_limited;        /* 1 = S11 authority < 1.0 (inhibit trim/jump) */
    uint8_t  imu_est_valid;      /* 1 = IMU & state estimator healthy/converged */
    uint8_t  ground_left;        /* 1 = left wheel grounded */
    uint8_t  ground_right;       /* 1 = right wheel grounded */
    float    pitch_deg;
    float    roll_deg;
    float    pitch_rate_deg_s;
    float    v_est_m_s;

    /* Pilot ExpressLRS radio, Pi 5 host gate, local operator buttons */
    uint32_t radio_age_ms;       /* Time since last fresh qualified CRSF frame */
    uint8_t  radio_arm_sw;       /* 1 = pilot radio arm switch HIGH */
    uint8_t  radio_estop_sw;     /* 1 = deliberate pilot radio e-stop switch active */
    uint32_t host_age_ms;        /* Time since last valid Pi 5 host command */
    uint8_t  host_valid;         /* 1 = host_gate currently valid & granted */
    float    v_target_m_s;       /* Requested linear velocity target */
    float    yaw_target_rad_s;   /* Requested yaw rate target */
    uint8_t  stand_handoff_req;  /* 1 = explicit stand handoff request */
    uint8_t  voluntary_stop_req; /* 1 = non-safety voluntary stop request */
    uint8_t  local_arm_req;      /* 1 = separate local hardware/operator arm */
    uint8_t  local_reset_req;    /* 1 = local manual reset / latch acknowledge */
    uint8_t  hw_reset_ack;       /* 1 = local hardware stop-latch reset confirmed */
    uint8_t  local_regrant_req;  /* 1 = explicit local operator re-grant from REST */
} SafetyInputs;

typedef struct {
    SafetyState  state;
    SafetyReason reason;
    uint8_t      permit_out;       /* External hardware permit pin command */
    uint8_t      torque_enable;    /* 1 = active balance/stop/rest torque allowed */
    uint8_t      can_enable_cmd;   /* 1 = send CAN 0xFF..FC enable (only after K1 closed) */
    uint8_t      can_disable_cmd;  /* 1 = send CAN 0xFF..FD disable */
    uint8_t      can_abort_tx;     /* 1 = abort pending ordinary CAN TX mailboxes */
    uint8_t      host_lost;        /* 1 = S9 brake position mode latched */
    uint8_t      host_revoke;      /* 1 = revoke host_gate authority */
    uint8_t      inhibit_trim;     /* 1 = inhibit S9 balance-point trim learning */
    uint8_t      inhibit_jump;     /* 1 = inhibit jump requests */
    uint8_t      mild_sag_setdown; /* 1 = mild loaded sag: keep balance ON, set down */
    uint8_t      rest_confirmed;   /* 1 = REST state confirmed after brake + dwell */
    float        v_ref_m_s;
    float        yaw_ref_rad_s;
    float        regen_scale;      /* 1.0 normal, tapered to 0.0 above 25.8 V */
} SafetyOutput;

typedef struct {
    SafetyConfig config;
    SafetyState  state;
    SafetyReason reason;
    uint32_t     precharge_elapsed_ms;
    uint32_t     brake_elapsed_ms;
    uint32_t     rest_dwell_ms;
    uint32_t     tilt_dwell_ms;
    uint32_t     ov_dwell_ms;
    float        v_ref_m_s;
    float        yaw_ref_rad_s;
    float        brake_start_v_m_s;
    float        brake_start_yaw_rad_s;
    uint8_t      host_lost_latched;
} SafetyFsm;

SafetyConfig safety_default_config(void);
void safety_fsm_init(SafetyFsm *fsm);
void safety_fsm_init_with_config(SafetyFsm *fsm, const SafetyConfig *cfg);
void safety_fsm_step(SafetyFsm *fsm, const SafetyInputs *in, SafetyOutput *out);

#endif /* WBR_SAFETY_FSM_H */
