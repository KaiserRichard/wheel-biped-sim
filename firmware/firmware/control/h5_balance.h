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
#define H5_BODY_PRED_HIST        16u     /* PUSH10 Tp history, 500 Hz ticks (32 ms) */

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
    uint8_t      enable_push_window;
    float        push_rate_rad_s;
    float        push_accel_rad_s2;
    float        push_confirm_s;
    float        push_arm_s;
    float        push_fade_in_s;
    float        push_hold_s;
    float        push_fade_out_s;
    float        push_quiet_rate_rad_s; /* onset accepted only while lpf(|IMU sensor pitch rate|) < this ... */
    float        push_quiet_s;          /* ... with this low-pass time constant (rejects un-masked fault rocking) */
    /* PUSH10 body-pitch hip loop, added to the aggregate leg effort (Tp) while
     * on the ground. Tp += kp_eff * (pitch - washout(pitch)) + body_kd * lpf(gyro).
     * kp_eff = body_kp + (body_kp_hi - body_kp) * min(1, |e| / body_sched_rad)
     * when body_sched_rad > 0. All-zero gains disable the loop. */
    float        body_kp;               /* N*m/rad, aggregate (both legs) */
    float        body_kp_hi;            /* N*m/rad, scheduled ceiling (used if body_sched_rad > 0) */
    float        body_sched_rad;        /* rad, |e| at which kp reaches body_kp_hi; 0 = no schedule */
    float        body_kd;               /* N*m*s/rad on low-passed pitch rate */
    float        body_rate_fc_hz;       /* first-order gyro low-pass corner; 0 = unfiltered */
    float        body_washout_s;        /* pitch washout time constant; 0 = no washout (absolute pitch) */
    /* Lag-aware (Smith / finite-horizon) correction of the body-pitch feedback:
     * adds the pitch and rate that Tp commands still in flight (model transport
     * delay + first-order actuator lag) will produce, using the identified
     * body-pitch model  I*p'' + c*p' + k*p = -Tp.  body_pred_inertia <= 0
     * disables the predictor. */
    float        body_pred_delay_s;     /* modelled command-to-actuator transport delay, s (<= 30 ms) */
    float        body_pred_lag_s;       /* modelled first-order actuator lag, s */
    float        body_pred_inertia;     /* effective body-pitch inertia seen by aggregate Tp, kg*m^2 */
    float        body_pred_damp;        /* model damping c, N*m*s/rad */
    float        body_pred_stiff;       /* model stiffness k, N*m/rad */
    /* PUSH11 CoM-offset lean trim (all 0 = off). A CoM offset needs a steady
     * leg lean; without a lean reference the S7 rows hold that lean with a
     * standing position error. com_lean_ki integrates the standing position
     * error into a leg-lean reference (obs theta) while the robot is standing
     * still and commanded still, clamped to +/-com_lean_lim_rad. The constant
     * terms are fixed offsets for FIL diagnosis (oracle tests). */
    float        com_lean_ki;           /* rad/(m*s), lean-reference integral gain on x error */
    float        com_lean_lim_rad;      /* |lean reference| clamp, rad */
    float        com_learn_vmax;        /* learn only while |v - v_set| < this, m/s; 0 = no speed gate */
    float        com_learn_wmax;        /* learn only while |pitch rate| < this, rad/s; 0 = no gate */
    float        com_x_ff_m;            /* FIL oracle: constant x-error offset, m */
    float        com_lean_ff_rad;       /* FIL oracle: constant lean-reference offset, rad */
    float        com_pitch_ff_rad;      /* FIL oracle: constant pitch-reference offset, rad */
    float        com_tp_ff_nm;          /* FIL oracle: constant aggregate Tp feedforward, N*m */
    /* PUSH11 push-yield (yield_v_m_s = 0 = off). While standing, commanded still
     * and not host-lost, a speed error above yield_v_m_s starts a yield: the
     * position error fed to the S7 rows is frozen at its pre-push value, so the
     * rows answer the push with velocity damping only and do not add a
     * position-return lean on top of the push. The yield ends when the speed
     * error falls below release_v_m_s for release_s, or after yield_max_s at
     * the latest (0 = no limit); the frozen offset then
     * returns to zero with time constant return_s (0 = keep it, i.e. the robot
     * stays where the push left it). */
    float        yield_v_m_s;
    float        yield_release_v_m_s;
    float        yield_release_s;
    float        yield_return_s;
    float        yield_max_s;
    float        yield_v_scale;         /* speed-error scale while yielding, (0,1]; <= 0 = 1 */
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
    float           body_pitch_base;    /* PUSH10 washout state, rad */
    float           body_rate_f;        /* PUSH10 low-passed pitch rate, rad/s */
    float           body_tp;            /* PUSH10 last body-pitch effort, N*m */
    uint8_t         body_active;        /* PUSH10 1 once the loop state is seeded */
    float           body_u_hist[H5_BODY_PRED_HIST]; /* PUSH10 past body-loop Tp commands */
    uint32_t        body_u_idx;
    float           body_pred_act;      /* modelled applied torque of delayed commands */
    float           body_pred_dv;       /* in-flight pitch-rate correction, rad/s */
    float           body_pred_dp;       /* in-flight pitch correction, rad */
    float           fil_body_inj;       /* FIL-only additive Tp injection (margin tests), N*m */
    float           com_lean_i;         /* PUSH11 learned lean reference, rad */
    uint8_t         com_learning;       /* PUSH11 1 while the lean trim is integrating */
    uint8_t         yield_on;           /* PUSH11 1 while a push-yield is active */
    uint32_t        yield_quiet_ticks;  /* PUSH11 ticks below the release speed */
    uint32_t        yield_ticks;        /* PUSH11 ticks since the yield started */
    uint32_t        yield_events;       /* PUSH11 number of yields started */
    float           yield_x_obs;        /* PUSH11 position error frozen at yield start, m */
    float           yield_x_off;        /* PUSH11 offset removed from the S7 position error, m */
    float           push_weight;
    float           push_age_s;
    float           push_ready_s;
    float           push_evidence_s;
    float           push_quiet_lpf;     /* low-passed |pitch rate| before any onset, rad/s */
    float           push_previous_rate;
    float           push_previous_pitch;
    float           push_rate_residual;
    float           push_accel_residual;
    uint32_t        push_events;
    uint32_t        push_active_ticks;
    uint8_t         push_open;
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
    uint8_t push_inhibit; /* supervisor state, host loss, or any S11 suspect mask */
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
