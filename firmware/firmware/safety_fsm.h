/*
 * Wheel-Biped Safety Supervisor State Machine
 *
 * Implements FIRMWARE_DESIGN_v2.md Section 6 (6.1 Policy Summary, 6.2 State Diagram,
 * 6.3 Transition & Action Matrix) and wires verified S11 packet-policy states
 * (S11_DISARMED, S11_ARMING, S11_FRESH, stale S11_PREDICT / S11_LIMIT / S11_TIMEOUT,
 * frozen S11_STUCK, S11_INVALID, and S11_DRIVE_FAULT) to their documented safe actions.
 *
 * Pure C11, zero HAL/RTOS dependencies, zero dynamic memory, fixed-width types.
 */

#ifndef WBR_SAFETY_FSM_H
#define WBR_SAFETY_FSM_H

#include "s11_packet_policy.h"

#include <stdint.h>

/* Compile-time ExpressLRS RF-loss policy switch (FIRMWARE_DESIGN_v2.md Section 6.1).
 * 0 (option, old S12 default): latched immediate CAN torque-off (-> FAULT).
 * 1 (default since 23 Nov 2026, owner decision from S14 policy c: 0/34 falls vs 34/34):
 *   0.60 s local brake -> REST (keep balancing in place). Tilt, wired e-stop and
 *   drive/CAN/power faults still cut torque at once under either policy. */
#define RADIO_LOSS_TORQUE_OFF           0u
#define RADIO_LOSS_BRAKE_BALANCE        1u
#define SAFETY_RADIO_LOSS_TORQUE_OFF    RADIO_LOSS_TORQUE_OFF
#define SAFETY_RADIO_LOSS_BRAKE_BALANCE RADIO_LOSS_BRAKE_BALANCE

#ifndef RADIO_LOSS_POLICY
#define RADIO_LOSS_POLICY RADIO_LOSS_BRAKE_BALANCE
#endif

typedef enum {
    SAFETY_STATE_DISARMED = 0,
    SAFETY_STATE_PRECHARGE = 1,
    SAFETY_STATE_ARMED = 2,
    SAFETY_STATE_BALANCING = 3,
    SAFETY_STATE_CONTROLLED_STOP = 4,
    SAFETY_STATE_REST = 5,
    SAFETY_STATE_FAULT = 6,
    SAFETY_STATE_ESTOP = 7,
    SAFETY_STATE_STEP_ASSIST = 8
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
    SAFETY_REASON_ESTOP_HW,
    SAFETY_REASON_S11_DEGRADED, /* FW6: single frozen encoder -> controlled stop */
    SAFETY_REASON_STEP_ASSIST_ABORT, /* FW6FIX D3: run-up abort (no room, stall, timeout,
                                      * pitch, saturation, legs lost) -> controlled stop */
    SAFETY_REASON_STOP_TIMEOUT  /* FW6FIX D2: CONTROLLED_STOP did not reach REST within
                                 * stop_timeout_ms -> REST (torque kept) + host alarm */
} SafetyReason;

typedef struct {
    uint8_t  radio_loss_policy;       /* SAFETY_RADIO_LOSS_TORQUE_OFF or _BRAKE_BALANCE */
    uint8_t  diagnostic_only;        /* FW10: stay DISARMED, no power-path arming.
                                       * Unknown power inputs remain unknown. Wired
                                       * stop, CAN, watchdog and reset faults apply. */
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
    float    degraded_pitch_fault_deg;  /* Default 15.0 deg: S11_DEGRADED stop escalates to FAULT */
    uint32_t stop_timeout_ms;         /* Default 20000 ms (FW6FIX D2): ~2x the slowest healthy
                                       * stop-to-REST seen in FIL (draw_001, 0.3 m/s: 10.1 s) */
    uint8_t  degraded_settle_trim;    /* Default 1 (FW7): after the brake ramp of a
                                       * CONTROLLED_STOP and in REST, a pure single-axis
                                       * S11_DEGRADED substitution (s11_degraded_only) no
                                       * longer inhibits the S9 balance-point trim.
                                       * 0 = FW6FIX behaviour (trim frozen while degraded). */
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
    uint8_t      drives_healthy_6;   /* 1 = all 6 motors replying with healthy status */
    uint8_t      drive_enable_fail;  /* 1 = drive rejected/failed 0xFC enable */
    uint8_t      s11_fault;          /* 1 = S11 packet guard latched fault */
    uint8_t      s11_limited;        /* 1 = S11 authority < 1.0 (inhibit trim/jump) */
    uint8_t      s11_degraded;       /* 1 = S11_DEGRADED: one axis frozen, data substituted */
    uint8_t      s11_degraded_only;  /* FW7: 1 = the only S11/CAN limit is the DEGRADED
                                      * substitution (full authority, no stale packet, no
                                      * CAN error-passive). Default 0 = not claimed. */
    enum S11Mode s11_mode;          /* Verified S11 mode (default 0 = S11_FRESH) */
    uint8_t      s11_frame_bound;    /* 1 = populated via safety_fsm_bind_s11_frame() */
    float        s11_authority;      /* S11 authority scale when s11_frame_bound == 1 */
    uint8_t      imu_est_valid;      /* 1 = IMU & state estimator healthy/converged */
    uint8_t      ground_left;        /* 1 = left wheel grounded */
    uint8_t      ground_right;       /* 1 = right wheel grounded */
    float        pitch_deg;
    float        roll_deg;
    float        pitch_rate_deg_s;
    float        v_est_m_s;

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
    uint8_t  step_assist_req;    /* Qualified blocked drive with available legs */
    uint8_t  step_assist_done;   /* Onboard passage confirmation */
    uint8_t  step_assist_failed; /* Pitch, saturation, unavailable legs or timeout */
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
    uint8_t      radio_lost;       /* 1 = ExpressLRS RF timeout active (> radio_timeout_ms) */
    uint8_t      stop_timeout_alarm; /* 1 = FW6FIX D2: stop timed out, REST not confirmed;
                                      * host must inspect (cleared by disarm/regrant/fault) */
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
    uint8_t      stop_timeout_latched; /* FW6FIX D2 alarm latch */
} SafetyFsm;

SafetyConfig safety_default_config(void);
SafetyConfig safety_config_with_radio_policy(uint8_t radio_loss_policy);
void safety_fsm_init(SafetyFsm *fsm);
void safety_fsm_init_with_config(SafetyFsm *fsm, const SafetyConfig *cfg);

/* Wire an S11PacketFrame (ARMING, FRESH, PREDICT, LIMIT, TIMEOUT, STUCK, INVALID,
 * DRIVE_FAULT, DISARMED) directly into SafetyInputs. */
void safety_fsm_bind_s11_frame(SafetyInputs *in, const S11PacketFrame *s11_frame);

void safety_fsm_step(SafetyFsm *fsm, const SafetyInputs *in, SafetyOutput *out);

/* Convenience step binding S11PacketFrame states before evaluating safety_fsm_step(). */
void safety_fsm_step_with_s11(SafetyFsm *fsm, const SafetyInputs *in,
                              const S11PacketFrame *s11_frame, SafetyOutput *out);

#endif /* WBR_SAFETY_FSM_H */
