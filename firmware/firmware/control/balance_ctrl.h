/*
 * Unified 500 Hz Balance Control Pipeline (balance_ctrl)
 *
 * Executes one deterministic 500 Hz (2.0 ms) control tick in the exact 8-step order
 * specified in FIRMWARE_DESIGN_v2.md Section 3.2:
 *   1. Snapshot inputs (IMU, 6x CAN RX slots & ages, radio, host, power/estop GPIOs)
 *   2. S11 packet-age policy (extrapolation <= 4 ms, 75% authority limit, fault latch)
 *   3. Estimator update (S10 complementary estimator fed ONLY genuine fresh encoders)
 *   4. Safety supervisor FSM (including RADIO_LOSS_TORQUE_OFF / RADIO_LOSS_BRAKE_BALANCE)
 *   5. S9 host-loss brake (called every enabled tick) & quiet balance-point trim
 *   6. VMC + S7/S5 predictor + yaw/sync/roll/height PIDs + jump FSM + clamps + regen taper
 *   7. Parallel 3-bus CAN TX dispatch & s5_commit()
 *   8. Flight recorder push & balance_heartbeat increment
 *
 * Adapted from S11_packet_tolerance & DM-jump (MIT License preserved in
 * firmware/DM_JUMP_LICENSE.txt).
 * Pure C11, zero HAL/RTOS dependencies, zero dynamic memory, fixed-width types.
 */

#ifndef WBR_CONTROL_BALANCE_CTRL_H
#define WBR_CONTROL_BALANCE_CTRL_H

#include "can_sched.h"
#include "log_packer.h"
#include "s11_packet_policy.h"
#include "safety_fsm.h"
#include "control/h5_balance.h"
#include "control/s10_estimator.h"

#include <stddef.h>
#include <stdint.h>

#define BALANCE_CTRL_DT_S     S4_DT
#define BALANCE_CTRL_DT_MS    2u
#define BALANCE_CTRL_MAX_IMU  4u

typedef struct {
    H5BalanceConfig h5_cfg;
    SafetyConfig    safety_cfg;
    S11Config       s11_cfg;
    CanSchedConfig  can_cfg;
    S10FiveBarGeom  fivebar_geom;
    double          wheel_radius_m;    /* Default 0.0603 m */
    double          imu_lever_m[3];    /* Body-frame IMU offset from chassis origin */
    uint8_t         use_s10_estimator; /* 1 = S10 estimator (default), 0 = external attitude/vel */
    uint8_t         s10_robust;        /* 1 = S10 robust moving bias & bounded speed recapture */
    uint8_t         enable_step_assist;
    float           step_height_m;       /* Conservative configured edge height, no truth input */
    /* FW6FIX D6: the STEP3 mode-0 bilateral leg pulse is removed (STEP3RC showed it
     * cannot climb). The run-up below (the former mode 1) is the only step assist. */
    float           step_backoff_m;      /* mode 1 reverse distance from stall point */
    float           step_back_m_s;       /* mode 1 reverse speed */
    float           step_charge_m_s;     /* mode 1 charge speed at the edge */
    float           step_acc_m_s2;       /* mode 1 reference slew limit */
    float           step_pass_m;         /* mode 1 forward progress past stall point = passage */
    uint32_t        step_timeout_ms;     /* mode 1 whole-manoeuvre timeout */
    uint32_t        step_refractory_ms;  /* mode 1 blocked-detector hold-off after passage */
    float           step_handback_scale; /* mode 1 hand-back slew as fraction of step_acc */
    float           step_carry_m;        /* legacy FIL knob, unused since FW6 (see step_declaration) */
} BalanceCtrlConfig;

typedef struct {
    int32_t tick_ms;
    double  gyro_rad_s[3];
    double  accel_m_s2[3];
} BalanceImuSample;

typedef struct {
    float   pitch_rad;
    float   pitch_rate_rad_s;
    float   roll_rad;
    float   roll_rate_rad_s;
    float   yaw_rad;
    float   yaw_rate_rad_s;
    float   v_est_m_s;
    uint8_t air;
    uint8_t valid;
} BalanceExtState;

enum {
    BALANCE_STEP_DECL_NONE = 0,   /* blocked drive -> CONTROLLED_STOP (F3b) */
    BALANCE_STEP_DECL_CURB = 1,   /* isolated curb, h <= 12 mm: carry 0 */
    /* FW6FIX D1 (interim): REFUSED. A link/host/frozen-encoder stop during a field
     * carry can leave a wheel in the gap between boxes and fall (FW6REVIEW: 52/119).
     * A field declaration is treated as no declaration (F3b controlled stop before
     * the field) and is flagged in step_decl_refused. Re-enabling it needs an owner
     * safety decision on the field-safe stop. */
    BALANCE_STEP_DECL_FIELD = 2
};
#define BALANCE_STEP_MAX_HEIGHT_M   0.0121f

typedef struct {
    uint32_t         tick_us;          /* Monotonic 500 Hz tick timestamp (us) */
    int32_t          enc_tick_ms;      /* Acquisition tick (ms) for S10 estimator */

    /* Optional pending 1 kHz IMU samples to consume before S10 frame read. */
    uint8_t          imu_sample_count; /* 0..BALANCE_CTRL_MAX_IMU */
    BalanceImuSample imu_samples[BALANCE_CTRL_MAX_IMU];
    BalanceExtState  ext_state;        /* Used when cfg.use_s10_estimator == 0 */

    /* Encoder source: 1 = snapshot from embedded CanSched (default), 0 = use direct enc_snap. */
    uint8_t          use_can_sched_rx;
    CanSchedRxSnapshot direct_enc_snap;

    /* Hardware stop chain & power path (S13) */
    uint8_t  estop_open;
    uint8_t  boot_self_test_ok;
    uint8_t  bor3_fault;
    uint8_t  clamp_fault;
    uint8_t  dump_continuity_ok;
    uint8_t  k1_closed;
    uint8_t  k2_closed;
    uint8_t  kpre_closed;
    uint8_t  precharge_dv_ok;
    uint8_t  mirror_mismatch;
    uint8_t  hard_uv_fault;
    uint8_t  wdg_fault;
    uint8_t  drive_enable_fail;
    float    v_bus_v;
    float    v_batt_ocv_v;
    int16_t  i_bus_ma;

    /* Pilot radio, Pi 5 host gate, operator controls, motion targets */
    uint32_t radio_age_ms;
    uint8_t  radio_arm_sw;
    uint8_t  radio_estop_sw;
    uint32_t host_age_ms;
    uint8_t  host_valid;
    float    v_target_m_s;
    float    yaw_target_rad_s;
    float    accel_ref_m_s2;
    float    jerk_ref_m_s3;
    uint8_t  stand_handoff_req;
    uint8_t  voluntary_stop_req;
    uint8_t  jump_req;
    uint8_t  local_arm_req;
    uint8_t  local_reset_req;
    uint8_t  hw_reset_ack;
    uint8_t  local_regrant_req;
    uint8_t  external_inhibit_trim;

    /* FW6 (STEP3RC A6 item 3): host terrain declaration for the mode-1 run-up.
     * The firmware cannot sense terrain; without a declaration a blocked drive
     * makes the existing F3b controlled stop. */
    uint8_t  step_declaration;     /* BALANCE_STEP_DECL_NONE / _CURB / _FIELD */
    float    step_decl_height_m;   /* declared edge height; <= 0 uses cfg.step_height_m */
    float    step_field_length_m;  /* FIELD length; unused while FIELD is refused (FW6FIX D1) */
} BalanceTickInput;

typedef struct {
    S11PacketFrame   s11_frame;
    S10Frame         est_frame;
    SafetyOutput     safety_out;
    H5BalanceOutput  h5_out;
    CanSchedHealth   can_health;
    float            torque_cmd[6];     /* Final 6-axis motor commands sent to CAN TX */
    uint8_t          s10_fed_fresh_enc; /* 1 iff genuine fresh encoders were fed to S10 */
    uint8_t          flight_rec_pushed;
    uint8_t          step_decl_refused; /* FW6FIX D1: host declared FIELD this tick (refused);
                                         * also flight-record flag 0x10 */
    uint32_t         balance_heartbeat;
} BalanceTickOutput;

typedef struct {
    BalanceCtrlConfig cfg;
    CanSched          can_sched;
    S11PacketPolicy   s11;
    S10Estimator      s10;
    SafetyFsm         safety;
    H5BalanceState    h5;
    LogFlightRing    *flight_ring; /* Optional caller-owned static ring buffer */
    uint32_t          balance_heartbeat;
    uint32_t          tick_count;
    float             progress_ref_m_s;
    uint32_t          blocked_drive_ms;
    uint32_t          step_elapsed_ms;
    uint32_t          step_limit_ms;
    uint32_t          step_damping_ms;
    uint8_t           step_phase; /* 0 idle, 4 reverse, 5 settle, 6 charge, 7 hand-back */
    uint8_t           step_clipped;
    float             step_start_x_m;
    float             step_direction;
    float             fil_torque_fb[6];
    float             step_v_m_s;    /* mode 1 assist speed reference (slewed) */
    float             step_a_m_s2;   /* mode 1 assist accel feed-forward */
    uint32_t          step_phase_ms; /* time in current mode-1 phase */
    float             drive_lag_m;   /* filtered steady x_set - x_est while driving */
    float             step_backed_m; /* achieved reverse distance (mode 1) */
    uint8_t           step_charged;  /* charge speed reached (mode 1) */
    uint8_t           step_abort;    /* 0 none, 1 no run-up room, 2 charge stall, 3 timeout */
    uint32_t          step_refractory_ms; /* blocked detector hold-off after passage */
    float             step_carry_eff_m;   /* carry for this manoeuvre: 0 (curb only, FW6FIX D1) */
    uint32_t          step_timeout_eff_ms; /* timeout for this manoeuvre */
    uint32_t          step_decl_refused_n; /* FW6FIX D1: ticks with a refused FIELD declaration */
    uint8_t           fil_step_decl;      /* FIL-only declaration passed to the tick */
    float             fil_step_decl_h_m;
    float             fil_step_field_m;
} BalanceCtrl;

BalanceCtrlConfig balance_ctrl_default_config(void);
void balance_ctrl_init(BalanceCtrl *ctrl, const BalanceCtrlConfig *cfg);
void balance_ctrl_attach_flight_ring(BalanceCtrl *ctrl, LogFlightRing *ring);

/* Feed one 1 kHz IMU sample directly into the S10 estimator (for 1 kHz imu_task). */
void balance_ctrl_feed_imu_1khz(BalanceCtrl *ctrl, int32_t tick_ms,
                                const double gyro_rad_s[3],
                                const double accel_m_s2[3]);

/* Execute one complete 500 Hz tick in exact FIRMWARE_DESIGN_v2.md Section 3.2 order. */
void balance_ctrl_tick(BalanceCtrl *ctrl, const BalanceTickInput *in, BalanceTickOutput *out);

/* Flat zero-malloc C ABI helpers exported for Python/ctypes MuJoCo FIL testing. */
size_t  balance_ctrl_state_size(void);
void balance_ctrl_fil_torque_feedback(void *state_buf, const float torque_nm[6]);
void balance_ctrl_fil_step_diag(void *state_buf, float diag[8]);
void balance_ctrl_fil_step_config(void *state_buf, const float p[11]);
void balance_ctrl_fil_disable_stuck(void *state_buf);
void balance_ctrl_fil_step_declare(void *state_buf, uint8_t declaration,
                                   float height_m, float field_length_m);
void balance_ctrl_fil_s11_diag(void *state_buf, float diag[10]);
void balance_ctrl_fil_s11_hip_mirror(void *state_buf, uint8_t enable);
/* FW6FIX FIL diag: [0] refused FIELD-declaration ticks, [1] stop-timeout alarm latch,
 * [2] safety reason, [3] last run-up abort code. */
void balance_ctrl_fil_alarm_diag(void *state_buf, float diag[4]);
void    balance_ctrl_fil_init(void *state_buf, float mass_kg, float length_ref_m,
                              uint8_t radio_loss_policy);
void    balance_ctrl_fil_imu_1khz(void *state_buf, int32_t tick_ms,
                                  const double gyro_rad_s[3],
                                  const double accel_m_s2[3]);
int32_t balance_ctrl_fil_arm_and_handoff(void *state_buf, uint32_t tick_us, int32_t enc_tick_ms,
                                         const double raw_angles[6], const double raw_rates[6]);
int32_t balance_ctrl_fil_step_500hz(void *state_buf, uint32_t tick_us, int32_t enc_tick_ms,
                                    const double raw_angles[6], const double raw_rates[6],
                                    float v_target_m_s, float yaw_target_rad_s,
                                    uint8_t radio_lost, uint8_t external_inhibit_trim,
                                    float out_torques[6], float out_diag[10]);
/* Extended FIL step supporting driving feedforward (accel/jerk), explicit host loss, and
 * optional 4-element drive diagnostics: [v_ref_m_s, yaw_ref_rad_s, x_err_eff_m, safety_reason]. */
int32_t balance_ctrl_fil_step_500hz_ex(void *state_buf, uint32_t tick_us, int32_t enc_tick_ms,
                                       const double raw_angles[6], const double raw_rates[6],
                                       float v_target_m_s, float yaw_target_rad_s,
                                       float accel_ref_m_s2, float jerk_ref_m_s3,
                                       uint8_t host_lost, uint8_t radio_lost,
                                       uint8_t external_inhibit_trim,
                                       float out_torques[6], float out_diag[10],
                                       float out_drive_diag[4]);

#endif /* WBR_CONTROL_BALANCE_CTRL_H */
