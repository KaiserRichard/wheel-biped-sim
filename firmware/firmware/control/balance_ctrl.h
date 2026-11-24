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
