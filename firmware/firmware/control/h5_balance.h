/*
 * H5 Unified Balance Kernel, S7/S5 Predictor, S8/S9 Brake & Trim, VMC, and Jump FSM
 *
 * Ported from S11_packet_tolerance (h5_balance.h, g2_adapter.c, firmware_copy/src/
 * chassisR_task.c, chassisL_task.c) into reentrant pure C11 with zero global state.
 *
 * Chassis control loop & jump state machine structure adapted from DM-jump:
 *   Copyright (c) 2024-2026 Trần Nguyên Bình
 *   Project: DM-jump (https://github.com/nguyenbinh-shark/DM-jump), MIT License
 *   Reference: dmBots/wheel-legged (https://github.com/dmBots/wheel-legged)
 *   See firmware/DM_JUMP_LICENSE.txt for the full MIT license notice.
 *
 * Pure C11, zero HAL/RTOS dependencies, zero dynamic memory.
 */

#ifndef WBR_CONTROL_H5_BALANCE_H
#define WBR_CONTROL_H5_BALANCE_H

#include "VMC_calc.h"
#include "pid.h"
#include "s4_gains.h"
#include "s5_predictor.h"
#include "s7_wheel_loss.h"
#include "s8_stop_position.h"
#include "s9_balance_point.h"

#include <stdbool.h>
#include <stdint.h>

#define H5_HIP_TORQUE_LIMIT_NM   7.0f
#define H5_WHEEL_TORQUE_LIMIT_NM 2.0f
#define H5_FORCE_LIMIT_N         100.0f
#define H5_GRAVITY_FEEDFORWARD_N 17.658f /* DM-jump Mg constant used in jump FSM */

typedef struct {
    float        mass_kg;
    float        trim_mass_kg;
    float        length_ref_m;
    enum s7_mode s7_mode;
    float        stop_position_limit_m; /* Default S8_STOP_POSITION_LIMIT_M (0.60 m) */
    float        sync_kp;               /* Default S4_SYNC_KP (15.0) */
    float        sync_kd;               /* Default S4_SYNC_KD (0.35) */
    uint8_t      use_s9_blend;          /* 1 = S9 continuous brake blend, 0 = S8 reset */
    uint8_t      enable_s9_trim;        /* 1 = quiet balance-point pitch trim learning */
    uint8_t      enable_s7_loss_adapt;  /* 1 = online wheel-loss damping adaptation in S7_V1 */
} H5BalanceConfig;

typedef struct {
    H5BalanceConfig cfg;
    vmc_leg_t       left;
    vmc_leg_t       right;
    PidTypeDef      leg_l_pid;
    PidTypeDef      leg_r_pid;
    s5_predictor_t  s5;
    s7_loss_t       s7_loss;
    s9_brake_t      s9_brake;
    s9_trim_t       s9_trim;
    bool            s8_was_host_lost;
    float           stop_position_error_m;
    float           s5_trim_values[3];
    float           h5_support;
    float           h5_leg_ref;
    float           h5_length;
    float           h5_theta_error;
    float           h5_theta_rate;
    float           h5_tp_trim;
    float           h5_yaw_torque;
    float           yaw_integral;
    float           s9_pitch_reference;
    float           s9_lean_reference;
    float           x_est_m;
    float           x_set_m;
    float           x_filter_eff;
    float           x_set_eff;
    uint8_t         s9_learning;
    uint8_t         right_air_flag;
    uint8_t         left_air_flag;
    uint8_t         recover_flag;
    uint8_t         jump_flag;
    uint8_t         jump_status;
    uint32_t        jump_time_l;
    uint32_t        jump_time_r;
} H5BalanceState;

typedef struct {
    float   pitch_rad;
    float   pitch_rate_rad_s;
    float   roll_rad;
    float   roll_rate_rad_s;
    float   yaw_rad;
    float   yaw_rate_rad_s;
    float   x_m;
    float   v_m_s;
    float   x_set_m;
    float   v_set_m_s;
    float   yaw_ref_rad_s;
    float   accel_ref_m_s2;
    float   jerk_ref_m_s3;
    float   angle[6]; /* [L_front, L_rear, L_wheel, R_front, R_rear, R_wheel] */
    float   rate[6];
    uint8_t air;
    uint8_t host_lost;
    uint8_t inhibit_trim;
    uint8_t inhibit_jump;
    uint8_t jump_req;
    uint8_t step_extra_damping; /* post-run-up height damping (FW6FIX D6: leg-pulse inputs removed) */
    uint8_t mild_sag_setdown;
    float   regen_scale; /* 1.0 normal, tapered to 0.0 above 25.8 V */
} H5BalanceInput;

typedef struct {
    float   torque_cmd[6];      /* [L_front, L_rear, L_wheel, R_front, R_rear, R_wheel] */
    float   pre_limit_torque[6];
    float   s5_effort[S5_EFFORTS];
    float   s5_feedback[S5_EFFORTS]; /* Deviation effort (excluding feedforward) for S5 predictor history */
    float   leg_length[2];      /* [left.L0, right.L0] */
    float   leg_theta[2];       /* [left.theta, right.theta] */
    float   theta_error_rad;
    float   stop_position_error_m;
    float   s9_pitch_ref_rad;
    float   s9_brake_blend_m;
    float   damping_est;
    uint8_t standing;
    uint8_t both_air;
    uint8_t s9_learning;
    uint8_t clipped_axes;
} H5BalanceOutput;

H5BalanceConfig h5_balance_default_config(void);
void h5_balance_init(H5BalanceState *st, const H5BalanceConfig *cfg);

/* Step 5 of FIRMWARE_DESIGN_v2.md Section 3.2: S9 brake tracking & quiet trim update.
 * Must be called on EVERY enabled 500 Hz tick before h5_balance_step_compute(). */
void h5_balance_step_brake_and_trim(H5BalanceState *st, const H5BalanceInput *in);

/* Step 6 of FIRMWARE_DESIGN_v2.md Section 3.2: VMC + S7/S5 + yaw/sync/roll/height + jump + clamps. */
void h5_balance_step_compute(H5BalanceState *st, const H5BalanceInput *in, H5BalanceOutput *out);

/* Step 7 predictor commit: records sent s5_effort (or clears history when !standing). */
void h5_balance_commit(H5BalanceState *st, const float s5_effort[S5_EFFORTS],
                       const float torque_cmd[6], uint8_t standing);

/* Reset brake/trim/odometry/predictor history when torque is disabled. */
void h5_balance_reset_disabled(H5BalanceState *st);

#endif /* WBR_CONTROL_H5_BALANCE_H */
