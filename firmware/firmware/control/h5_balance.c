/*
 * H5 Unified Balance Kernel, S7/S5 Predictor, S8/S9 Brake & Trim, VMC, and Jump FSM
 *
 * Ported from S11_packet_tolerance (h5_balance.h, g2_adapter.c, firmware_copy/src/
 * chassisR_task.c, chassisL_task.c) with exact numerical tables and control laws.
 *
 * Chassis control loop & jump state machine structure adapted from DM-jump:
 *   Copyright (c) 2024-2026 Trần Nguyên Bình
 *   Project: DM-jump (https://github.com/nguyenbinh-shark/DM-jump), MIT License
 *   Reference: dmBots/wheel-legged (https://github.com/dmBots/wheel-legged)
 *   See firmware/DM_JUMP_LICENSE.txt for the full MIT license notice.
 */

#include "h5_balance.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static const float s_poly_coefficient[12][4] = {
#include "python_gain_table.inc"
};

static float clamp_f(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static float apply_regen_taper(float torque_nm, float vel_rad_s, float regen_scale)
{
    if (regen_scale >= 1.0f || !isfinite(regen_scale)) {
        return torque_nm;
    }
    const float scale = clamp_f(regen_scale, 0.0f, 1.0f);
    if (torque_nm * vel_rad_s < 0.0f) {
        return torque_nm * scale;
    }
    return torque_nm;
}

H5BalanceConfig h5_balance_default_config(void)
{
    H5BalanceConfig c;
    c.mass_kg               = 5.06f;
    c.trim_mass_kg          = 5.06f;
    c.length_ref_m          = 0.14f;
    c.s7_mode               = S7_V1;
    c.stop_position_limit_m = S8_STOP_POSITION_LIMIT_M;
    c.sync_kp               = S4_SYNC_KP;
    c.sync_kd               = S4_SYNC_KD;
    c.use_s9_blend          = 1u;
    c.enable_s9_trim        = 1u;
    c.enable_s7_loss_adapt  = 1u;
    c.enable_push_window    = 1u;
    c.push_rate_rad_s       = 0.04f;
    c.push_accel_rad_s2     = 8.0f;
    c.push_confirm_s       = 0.004f;
    c.push_arm_s           = 1.5f;
    c.push_fade_in_s       = 0.004f;
    c.push_hold_s          = 3.0f;
    c.push_fade_out_s      = 2.0f;
    c.push_quiet_rate_rad_s = 0.04f;
    c.push_quiet_s         = 0.5f;
    c.body_kp               = 55.0f;
    c.body_kp_hi            = 0.0f;
    c.body_sched_rad        = 0.0f;
    c.body_kd               = 0.4f;
    c.body_rate_fc_hz       = 0.0f;
    c.body_washout_s        = 2.0f;
    c.body_pred_delay_s     = 0.005f;
    c.body_pred_lag_s       = 0.003f;
    c.body_pred_inertia     = 0.0075f;
    c.body_pred_damp        = 0.49f;
    c.body_pred_stiff       = 5.4f;
    c.com_lean_ki           = 0.0f;
    c.com_lean_lim_rad      = 0.0f;
    c.com_learn_vmax        = 0.0f;
    c.com_learn_wmax        = 0.0f;
    c.com_x_ff_m            = 0.0f;
    c.com_lean_ff_rad       = 0.0f;
    c.com_pitch_ff_rad      = 0.0f;
    c.com_tp_ff_nm          = 0.0f;
    c.yield_v_m_s           = 0.4f;
    c.yield_release_v_m_s   = 0.08f;
    c.yield_release_s       = 0.2f;
    c.yield_return_s        = 2.0f;
    c.yield_max_s           = 1.0f;
    c.yield_v_scale         = 0.7f;
    return c;
}

void h5_balance_init(H5BalanceState *st, const H5BalanceConfig *cfg)
{
    if (st == NULL) {
        return;
    }
    memset(st, 0, sizeof(*st));
    st->cfg = (cfg != NULL) ? *cfg : h5_balance_default_config();

    st->s5.mass_kg = st->cfg.mass_kg;
    st->s5.length_ref_m = st->cfg.length_ref_m;
    st->s7_loss.damping = 0.0255f;

    s5_predictor_t trim_schedule;
    memset(&trim_schedule, 0, sizeof(trim_schedule));
    trim_schedule.mass_kg = st->cfg.trim_mass_kg;
    trim_schedule.length_ref_m = st->cfg.length_ref_m;
    s5_trim(&trim_schedule, st->s5_trim_values);
    st->h5_support = st->s5_trim_values[1];
    st->h5_leg_ref = st->cfg.length_ref_m;

    VMC_init(&st->left);
    VMC_init(&st->right);

    const float leg_gains[3] = { LEG_PID_KP, LEG_PID_KI, LEG_PID_KD };
    PID_init(&st->leg_l_pid, (uint8_t)PID_POSITION, leg_gains, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);
    PID_init(&st->leg_r_pid, (uint8_t)PID_POSITION, leg_gains, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);
}

void h5_balance_reset_disabled(H5BalanceState *st)
{
    if (st == NULL) {
        return;
    }
    const float zero_effort[S5_EFFORTS] = { 0.0f, 0.0f };
    s5_commit(&st->s5, zero_effort, false);
    st->s7_loss.quiet_ticks = 0u;
    st->s7_loss.previous_request = 0.0f;
    st->yaw_integral = 0.0f;
    st->jump_flag = 0u;
    st->jump_status = 0u;
    st->jump_time_l = 0u;
    st->jump_time_r = 0u;
    st->s8_was_host_lost = false;
    st->stop_position_error_m = 0.0f;
    memset(&st->s9_brake, 0, sizeof(st->s9_brake));
    st->s9_learning = 0u;
    st->x_est_m = 0.0f;
    st->x_set_m = 0.0f;
    st->x_filter_eff = 0.0f;
    st->x_set_eff = 0.0f;
    st->push_open = 0u;
    st->push_weight = 0.0f;
    st->push_ready_s = 0.0f;
    st->push_evidence_s = 0.0f;
    st->push_quiet_lpf = 0.0f;
    st->body_active = 0u;
    st->body_tp = 0.0f;
    st->body_pred_dp = 0.0f;
    st->body_pred_dv = 0.0f;
    st->body_pred_act = 0.0f;
    st->yield_on = 0u;
    st->yield_x_off = 0.0f;
}

static float push_smooth(float t)
{
    const float z = clamp_f(t, 0.0f, 1.0f);
    return z * z * (3.0f - 2.0f * z);
}

static void push_window(H5BalanceState *st, const H5BalanceInput *in, bool standing)
{
    /* Conservative command veto, including settling after motion. This version
     * only arms at commanded standstill. No simulator push flag is an input.
     * The IMU sensor rate and its derivative are residuals against the quiet
     * commanded motion model; acceleration/jerk veto prevents interpreting
     * commanded pitch transients as disturbances. */
    const bool eligible = st->cfg.enable_push_window != 0u && standing
        && in->push_inhibit == 0u && in->host_lost == 0u
        && fabsf(in->v_set_m_s) <= 0.015f
        && fabsf(in->accel_ref_m_s2) < 0.06f
        && fabsf(in->jerk_ref_m_s3) < 0.1f
        && fabsf(in->yaw_ref_rad_s) < 0.02f;
    st->push_rate_residual = fabsf(in->pitch_rate_rad_s);
    /* Positive only when rate magnitude grows coherently: this rejects slow
     * encoder-induced drift and alternating differentiated IMU sensor noise. */
    const float rate_delta = in->pitch_rate_rad_s - st->push_previous_rate;
    st->push_accel_residual = (rate_delta * in->pitch_rate_rad_s > 0.0f)
        ? fabsf(rate_delta) / S4_DT : 0.0f;
    st->push_previous_rate = in->pitch_rate_rad_s;
    if (!eligible) {
        /* Safety exclusions take priority over a normal fade-out. */
        st->push_open = 0u;
        st->push_weight = 0.0f;
        st->push_ready_s = 0.0f;
        st->push_evidence_s = 0.0f;
        st->push_quiet_lpf = 0.0f;
    } else {
        st->push_ready_s += S4_DT;
        if (st->push_open == 0u && st->push_ready_s >= st->cfg.push_arm_s) {
            const bool evidence = st->push_rate_residual > st->cfg.push_rate_rad_s
                && st->push_accel_residual > st->cfg.push_accel_rad_s2;
            /* A push strikes a quiet robot: the slow (push_quiet_s) low-pass of
             * |IMU sensor pitch rate| barely moves during a 10-20 ms onset, while
             * sustained rocking (e.g. an encoder fault S11 has not masked) raises it. */
            const bool quiet = st->push_quiet_lpf < st->cfg.push_quiet_rate_rad_s;
            st->push_evidence_s = (evidence && quiet) ? st->push_evidence_s + S4_DT : 0.0f;
            if (st->push_evidence_s >= st->cfg.push_confirm_s) {
                st->push_open = 1u;
                st->push_age_s = 0.0f;
                st->push_events++;
                st->body_pitch_base = st->push_previous_pitch;
                st->body_active = 0u;
            }
        }
        if (st->push_open != 0u) {
            st->push_age_s += S4_DT;
            const float release = st->cfg.push_fade_in_s + st->cfg.push_hold_s;
            if (st->push_age_s < st->cfg.push_fade_in_s) {
                st->push_weight = push_smooth(st->push_age_s / st->cfg.push_fade_in_s);
            } else if (st->push_age_s <= release) {
                st->push_weight = 1.0f;
            } else if (st->push_age_s < release + st->cfg.push_fade_out_s) {
                st->push_weight = 1.0f - push_smooth(
                    (st->push_age_s - release) / st->cfg.push_fade_out_s);
            } else {
                st->push_open = 0u;
                st->push_weight = 0.0f;
                st->push_ready_s = 0.0f;
                st->push_evidence_s = 0.0f;
            }
        }
        if (st->push_open == 0u) {
            st->push_quiet_lpf += (S4_DT / (st->cfg.push_quiet_s + S4_DT))
                                * (st->push_rate_residual - st->push_quiet_lpf);
        }
    }
    st->push_previous_pitch = in->pitch_rad;
    if (st->push_open == 0u) {
        st->body_active = 0u;
        st->body_pred_dp = 0.0f;
        st->body_pred_dv = 0.0f;
        st->body_pred_act = 0.0f;
        st->yield_on = 0u;
        st->yield_x_off = 0.0f;
    } else {
        st->push_active_ticks++;
    }
}

void h5_balance_step_brake_and_trim(H5BalanceState *st, const H5BalanceInput *in)
{
    if (st == NULL || in == NULL) {
        return;
    }

    const bool host_lost = (in->host_lost != 0u);
    if (st->cfg.use_s9_blend != 0u) {
        st->stop_position_error_m = s9_brake_position(
            &st->s9_brake, host_lost,
            in->x_m - in->x_set_m,
            in->v_m_s - in->v_set_m_s,
            S4_DT, st->cfg.stop_position_limit_m);
    } else {
        s8_stop_position(&st->stop_position_error_m, &st->s8_was_host_lost,
                         host_lost, in->v_m_s - in->v_set_m_s,
                         S4_DT, st->cfg.stop_position_limit_m);
    }

    if (host_lost) {
        st->x_filter_eff = st->stop_position_error_m;
        st->x_set_eff = 0.0f;
    } else {
        st->x_filter_eff = in->x_m;
        st->x_set_eff = in->x_set_m;
    }

    if (st->cfg.enable_s9_trim != 0u) {
        const bool quiet = (in->inhibit_trim == 0u)
            && (in->air == 0u)
            && (st->right_air_flag == 0u)
            && (st->left_air_flag == 0u)
            && (st->jump_flag == 0u)
            && (st->recover_flag == 0u)
            && (fabsf(in->v_m_s) < 0.065f)
            && (fabsf(in->v_m_s - in->v_set_m_s) < 0.045f)
            && (fabsf(in->v_set_m_s) <= 0.015f)
            && (fabsf(in->yaw_ref_rad_s) < 0.02f)
            && (fabsf(in->yaw_rate_rad_s) < 0.075f)
            && (fabsf(in->pitch_rate_rad_s) < 0.13f)
            && (fabsf(in->accel_ref_m_s2) < 0.06f);
        st->s9_learning = (quiet && st->s9_trim.quiet_ticks >= S9_QUIET_TICKS) ? 1u : 0u;
        s9_update_trim(&st->s9_trim, quiet, st->x_filter_eff - st->x_set_eff, S4_DT);
        st->s9_pitch_reference = st->s9_trim.pitch_rad;
        st->s9_lean_reference = S9_LEAN_PER_PITCH * st->s9_pitch_reference;
    } else {
        st->s9_learning = 0u;
    }
}

static void h5_update_jump_forces(H5BalanceState *st, float roll_force,
                                  uint8_t inhibit_jump, uint8_t jump_req,
                                  uint8_t mild_sag_setdown)
{
    if (inhibit_jump == 0u && jump_req != 0u && st->jump_flag == 0u) {
        st->jump_flag = 1u;
        st->jump_status = 0u;
        st->jump_time_l = 0u;
        st->jump_time_r = 0u;
    }

    if (st->jump_flag != 0u) {
        float target_l0;
        if (st->jump_status == 0u) {
            target_l0 = 0.08f;
            if (st->right.L0 < 0.10f) {
                st->jump_time_r++;
            }
            if (st->left.L0 < 0.10f) {
                st->jump_time_l++;
            }
            if (st->jump_time_r >= 10u && st->jump_time_l >= 10u) {
                st->jump_time_r = 0u;
                st->jump_time_l = 0u;
                st->jump_status = 1u;
            }
        } else if (st->jump_status == 1u) {
            target_l0 = 0.21f;
            if (st->right.L0 > 0.18f) {
                st->jump_time_r++;
            }
            if (st->left.L0 > 0.18f) {
                st->jump_time_l++;
            }
            if (st->jump_time_r >= 10u && st->jump_time_l >= 10u) {
                st->jump_time_r = 0u;
                st->jump_time_l = 0u;
                st->jump_status = 2u;
            }
        } else if (st->jump_status == 2u) {
            target_l0 = st->h5_leg_ref;
            if (st->right.L0 < (st->h5_leg_ref + 0.01f)) {
                st->jump_time_r++;
            }
            if (st->left.L0 < (st->h5_leg_ref + 0.01f)) {
                st->jump_time_l++;
            }
            if (st->jump_time_r >= 10u && st->jump_time_l >= 10u) {
                st->jump_time_r = 0u;
                st->jump_time_l = 0u;
                st->jump_status = 3u;
            }
        } else {
            target_l0 = st->h5_leg_ref;
            st->jump_flag = 0u;
            st->jump_status = 0u;
            st->jump_time_l = 0u;
            st->jump_time_r = 0u;
        }

        const float cos_r = fmaxf(0.25f, cosf(st->right.theta));
        const float cos_l = fmaxf(0.25f, cosf(st->left.theta));
        st->right.F0 = H5_GRAVITY_FEEDFORWARD_N / cos_r + PID_Calc(&st->leg_r_pid, st->right.L0, target_l0);
        st->left.F0  = H5_GRAVITY_FEEDFORWARD_N / cos_l + PID_Calc(&st->leg_l_pid, st->left.L0, target_l0);
    } else {
        /* Normal standing balance; gently crouch toward 0.115 m on mild bus-voltage sag. */
        float leg_target = st->h5_leg_ref;
        if (mild_sag_setdown != 0u && leg_target > 0.115f) {
            leg_target = 0.115f;
        }
        st->right.F0 = st->h5_support + S4_HEIGHT_KP * (leg_target - st->right.L0)
                     - S4_HEIGHT_KD * st->right.d_L0 - roll_force;
        st->left.F0  = st->h5_support + S4_HEIGHT_KP * (leg_target - st->left.L0)
                     - S4_HEIGHT_KD * st->left.d_L0 + roll_force;
    }
}

void h5_balance_step_compute(H5BalanceState *st, const H5BalanceInput *in, H5BalanceOutput *out)
{
    if (st == NULL || in == NULL || out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));

    /* Map to DM-jump INS_t convention: Gyro[0]=pitch_rate, Gyro[1]=roll_rate, Gyro[2]=yaw_rate. */
    VmcInsState ins;
    ins.Pitch         = in->pitch_rad;
    ins.Gyro[0]       = in->pitch_rate_rad_s;
    ins.Roll          = in->roll_rad;
    ins.Gyro[1]       = in->roll_rate_rad_s;
    ins.YawTotalAngle = in->yaw_rad;
    ins.Gyro[2]       = in->yaw_rate_rad_s;
    ins.ins_flag      = 1u;

    /* Map 6-axis S11 order [L_f, L_r, L_w, R_f, R_r, R_w] -> VMC joint angles/rates
     * matching S11/run_g2.py:61-62 and chassisR/L_feedback_update. */
    st->right.phi1 = VMC_PI / 2.0f + in->angle[4];
    st->right.phi4 = VMC_PI / 2.0f - in->angle[3];
    st->right.qd1  = in->rate[4];
    st->right.qd4  = -in->rate[3];

    st->left.phi1  = VMC_PI / 2.0f + in->angle[1];
    st->left.phi4  = VMC_PI / 2.0f - in->angle[0];
    st->left.qd1   = in->rate[1];
    st->left.qd4   = -in->rate[0];

    if (ins.Pitch < (VMC_PI / 6.0f) && ins.Pitch > (-VMC_PI / 6.0f)) {
        st->recover_flag = 0u;
    }

    /* 1. Five-bar forward kinematics */
    VMC_calc_1_right(&st->right, &ins, S4_DT);
    VMC_calc_1_left(&st->left, &ins, S4_DT);

    st->h5_length = clamp_f(0.5f * (st->right.L0 + st->left.L0), 0.08f, 0.16f);
    const float theta_trim = st->s5_trim_values[0];
    st->h5_tp_trim = st->s5_trim_values[2];
    st->h5_theta_error = -0.5f * (st->right.theta + st->left.theta) - theta_trim;
    st->h5_theta_rate  = -0.5f * (st->right.d_theta + st->left.d_theta);

    const bool standing = (in->air == 0u)
        && !(st->right_air_flag != 0u && st->left_air_flag != 0u)
        && (st->jump_flag == 0u)
        && (st->recover_flag == 0u);
    if (!standing) {
        const float zero_effort[S5_EFFORTS] = { 0.0f, 0.0f };
        s5_commit(&st->s5, zero_effort, false);
        st->s7_loss.quiet_ticks = 0u;
    }

    push_window(st, in, standing);
    const bool push_active = st->push_open != 0u;
    const float push_w = st->push_weight;

    /* PUSH11: CoM-offset lean trim. Integrates the standing position error into
     * a leg-lean reference while standing still and commanded still; frozen
     * otherwise (driving, turning, airborne, jumping, inhibited trim). */
    const float com_x_err = st->x_filter_eff - st->x_set_eff;
    const bool commanded_still = standing && in->host_lost == 0u && in->inhibit_trim == 0u
        && fabsf(in->v_set_m_s) <= 0.015f && fabsf(in->accel_ref_m_s2) < 0.06f
        && fabsf(in->yaw_ref_rad_s) < 0.02f;

    /* PUSH11 push-yield (see H5BalanceConfig). */
    if (push_active && st->cfg.yield_v_m_s > 0.0f) {
        const float v_err = fabsf(in->v_m_s - in->v_set_m_s);
        if (st->yield_on == 0u) {
            if (commanded_still && v_err > st->cfg.yield_v_m_s) {
                st->yield_on = 1u;
                st->yield_quiet_ticks = 0u;
                st->yield_ticks = 0u;
                st->yield_events++;
                st->yield_x_obs = com_x_err - st->yield_x_off;
            } else if (st->cfg.yield_return_s > 0.0f) {
                st->yield_x_off -= (S4_DT / (st->cfg.yield_return_s + S4_DT)) * st->yield_x_off;
            }
        }
        if (st->yield_on != 0u) {
            if (!commanded_still) {
                /* any command, host loss or lift-off ends the yield and drops the offset */
                st->yield_on = 0u;
                st->yield_x_off = 0.0f;
            } else {
                st->yield_x_off = com_x_err - st->yield_x_obs;
                st->yield_quiet_ticks = (v_err < st->cfg.yield_release_v_m_s) ? (st->yield_quiet_ticks + 1u) : 0u;
                st->yield_ticks++;
                if ((float)st->yield_quiet_ticks * S4_DT >= st->cfg.yield_release_s
                    || (st->cfg.yield_max_s > 0.0f && (float)st->yield_ticks * S4_DT >= st->cfg.yield_max_s)) {
                    st->yield_on = 0u;
                }
            }
        } else if (!commanded_still) {
            st->yield_x_off = 0.0f;
        }
    } else {
        st->yield_on = 0u;
        st->yield_x_off = 0.0f;
    }

    st->com_learning = 0u;
    if (push_active && st->cfg.com_lean_ki > 0.0f && standing && in->inhibit_trim == 0u && st->yield_on == 0u
        && fabsf(in->v_set_m_s) <= 0.015f && fabsf(in->accel_ref_m_s2) < 0.06f
        && fabsf(in->yaw_ref_rad_s) < 0.02f
        && (st->cfg.com_learn_vmax <= 0.0f || fabsf(in->v_m_s - in->v_set_m_s) < st->cfg.com_learn_vmax)
        && (st->cfg.com_learn_wmax <= 0.0f || fabsf(ins.Gyro[0]) < st->cfg.com_learn_wmax)) {
        st->com_learning = 1u;
        /* sign from the FIL settle test: a robot held behind its set point
         * (x error < 0) by the S7 rows needs a positive lean reference; the
         * opposite sign runs away (x error pinned at the stop-position clamp) */
        st->com_lean_i = clamp_f(st->com_lean_i - st->cfg.com_lean_ki * com_x_err * S4_DT,
                                 -st->cfg.com_lean_lim_rad, st->cfg.com_lean_lim_rad);
    }
    const float com_lean = st->com_lean_i + st->cfg.com_lean_ff_rad;

    /* 2. Assemble 6-element standing observation and run S7/S5 predictor */
    float obs[S5_OBS] = {
        st->x_filter_eff - st->x_set_eff,
        in->v_m_s - in->v_set_m_s,
        standing ? (st->h5_theta_error - st->s9_lean_reference) : st->h5_theta_error,
        st->h5_theta_rate,
        standing ? (ins.Pitch - st->s9_pitch_reference) : ins.Pitch,
        ins.Gyro[0]
    };
    if (push_active) {
        obs[0] -= push_w * (st->cfg.com_x_ff_m + st->yield_x_off);
        if (st->yield_on != 0u && st->cfg.yield_v_scale > 0.0f) {
            obs[1] *= 1.0f + push_w * (st->cfg.yield_v_scale - 1.0f);
        }
        obs[2] -= push_w * com_lean;
        obs[4] -= push_w * st->cfg.com_pitch_ff_rad;
    }

    s7_reference_t s7_ref;
    s7_ref.speed_m_s        = in->v_set_m_s;
    s7_ref.accel_m_s2       = in->accel_ref_m_s2;
    s7_ref.jerk_m_s3        = in->jerk_ref_m_s3;
    s7_ref.wheel_rate_rad_s = 0.5f * (in->rate[2] + in->rate[5]);
    s7_ref.yaw_rad_s        = ins.Gyro[2];

    float s5_effort[S5_EFFORTS] = { 0.0f, 0.0f };
    float s5_fb[S5_EFFORTS] = { 0.0f, 0.0f };
    s7_efforts_ex(&st->s5, &st->s7_loss, st->cfg.s7_mode,
                  st->cfg.enable_s7_loss_adapt != 0u,
                  st->h5_length, obs, &s7_ref, standing,
                  in->inhibit_trim != 0u, s5_effort, s5_fb);

    /* PUSH10: delay-tolerant body-pitch loop on the hips (aggregate Tp). The
     * pitch term acts on washed-out pitch, so it changes no steady-state lean
     * or trim; the rate term uses a low-passed gyro. Same sign convention as
     * the S7 hip row (positive gain on pitch and pitch rate). */
    st->body_tp = 0.0f;
    if (push_active) {
        if (st->body_active == 0u) {
            st->body_rate_f = ins.Gyro[0];
            st->body_active = 1u;
            /* clear the body-loop Tp history so the in-flight correction starts at zero */
            for (uint32_t i = 0u; i < H5_BODY_PRED_HIST; ++i) {
                st->body_u_hist[i] = 0.0f;
            }
            st->body_pred_act = 0.0f;
            st->body_pred_dv = 0.0f;
            st->body_pred_dp = 0.0f;
        }
        if (st->cfg.body_washout_s > 0.0f) {
            st->body_pitch_base += (S4_DT / (st->cfg.body_washout_s + S4_DT))
                                 * (ins.Pitch - st->body_pitch_base);
        } else {
            st->body_pitch_base = st->s9_pitch_reference;
        }
        if (st->cfg.body_rate_fc_hz > 0.0f) {
            const float w = 6.2831853f * st->cfg.body_rate_fc_hz * S4_DT;
            st->body_rate_f += (w / (1.0f + w)) * (ins.Gyro[0] - st->body_rate_f);
        } else {
            st->body_rate_f = ins.Gyro[0];
        }
        /* Lag-aware prediction: commands u[k-1..] that have not yet acted.
         * d_act = u_undelayed - lag(u_delayed) drives the body-pitch model. */
        if (st->cfg.body_pred_inertia > 0.0f) {
            const uint32_t n = H5_BODY_PRED_HIST;
            float nd_f = st->cfg.body_pred_delay_s / S4_DT;
            nd_f = clamp_f(nd_f, 0.0f, (float)(n - 2u));
            const uint32_t nd = (uint32_t)(nd_f + 0.5f);
            const float u_last = st->body_u_hist[(st->body_u_idx + n - 1u) % n];
            const float u_del  = st->body_u_hist[(st->body_u_idx + n - 1u - nd) % n];
            const float a = (st->cfg.body_pred_lag_s > 0.0f)
                ? (1.0f - expf(-S4_DT / st->cfg.body_pred_lag_s)) : 1.0f;
            st->body_pred_act += a * (u_del - st->body_pred_act);
            const float d_act = u_last - st->body_pred_act;
            const float acc = (-d_act - st->cfg.body_pred_damp * st->body_pred_dv
                               - st->cfg.body_pred_stiff * st->body_pred_dp) / st->cfg.body_pred_inertia;
            st->body_pred_dv += S4_DT * acc;
            st->body_pred_dp += S4_DT * st->body_pred_dv;
        } else {
            st->body_pred_dv = 0.0f;
            st->body_pred_dp = 0.0f;
        }
        const float e = ins.Pitch - st->body_pitch_base + st->body_pred_dp;
        float kp = st->cfg.body_kp;
        if (st->cfg.body_sched_rad > 0.0f) {
            const float s = fminf(1.0f, fabsf(e) / st->cfg.body_sched_rad);
            kp += (st->cfg.body_kp_hi - st->cfg.body_kp) * s;
        }
        st->body_tp = push_w * (kp * e + st->cfg.body_kd * (st->body_rate_f + st->body_pred_dv));
    } else {
        st->body_active = 0u;
        st->body_pred_dv = 0.0f;
        st->body_pred_dp = 0.0f;
    }

    float leg_total = s5_effort[1];
    if (push_active) {
        leg_total += st->body_tp + push_w * (st->fil_body_inj + st->cfg.com_tp_ff_nm);
    }
    if (push_active) {
        /* the identified model already contains the S7 hip row, so only the
         * body-loop torque drives the prediction (never the FIL injection) */
        st->body_u_hist[st->body_u_idx] = st->body_tp;
        st->body_u_idx = (st->body_u_idx + 1u) % H5_BODY_PRED_HIST;
    }

    const float half_wheel = 0.5f * s5_effort[0];
    const float half_leg   = 0.5f * leg_total;

    /* 3. Radial Jacobian rate d_L0 (H5 Stage >= 2) */
    VMC_calc_2(&st->right);
    VMC_calc_2(&st->left);
    st->right.d_L0 = st->right.j11 * st->right.qd1 + st->right.j21 * st->right.qd4;
    st->left.d_L0  = st->left.j11 * st->left.qd1 + st->left.j21 * st->left.qd4;

    /* 4. Yaw PI (H5 Stage >= 5) */
    const float yaw_err = in->yaw_ref_rad_s - ins.Gyro[2];
    if (in->air == 0u) {
        st->yaw_integral = clamp_f(st->yaw_integral + S4_YAW_KI * S4_DT * yaw_err, -0.5f, 0.5f);
    }
    st->h5_yaw_torque = S4_YAW_KP * yaw_err + st->yaw_integral;
    const float turn_t = -st->h5_yaw_torque;

    /* 5. Anti-splay sync PD & Roll PD */
    const float leg_tp = st->cfg.sync_kp * (st->right.alpha - st->left.alpha)
                       + st->cfg.sync_kd * (st->right.d_alpha - st->left.d_alpha);
    const float roll_force = -(S4_ROLL_KP * ins.Roll + S4_ROLL_KD * ins.Gyro[1]);

    /* 6. Per-leg wheel & tangential torques */
    float wheel_r = half_wheel - turn_t;
    float wheel_l = half_wheel + turn_t;
    out->pre_limit_torque[0] = wheel_r;
    out->pre_limit_torque[1] = wheel_l;
    wheel_r = clamp_f(wheel_r, -H5_WHEEL_TORQUE_LIMIT_NM, H5_WHEEL_TORQUE_LIMIT_NM);
    wheel_l = clamp_f(wheel_l, -H5_WHEEL_TORQUE_LIMIT_NM, H5_WHEEL_TORQUE_LIMIT_NM);

    st->right.Tp = half_leg + leg_tp + st->h5_tp_trim;
    st->left.Tp  = half_leg - leg_tp + st->h5_tp_trim;

    /* 7. Height PD / Jump FSM */
    h5_update_jump_forces(st, roll_force, in->inhibit_jump, in->jump_req, in->mild_sag_setdown);

    /* Extra height damping for 150 ms after a run-up hand-back. (FW6FIX D6: the
     * mode-0 leg-pulse target, force ceiling and extension pulse are removed.) */
    if (in->step_extra_damping != 0u) {
        const float target = st->h5_leg_ref;
        const float kd = S4_HEIGHT_KD + 35.0f;
        const float cap = H5_FORCE_LIMIT_N;
        st->left.F0 = clamp_f(st->h5_support + S4_HEIGHT_KP * (target - st->left.L0)
                             - kd * st->left.d_L0, 0.0f, cap);
        st->right.F0 = clamp_f(st->h5_support + S4_HEIGHT_KP * (target - st->right.L0)
                              - kd * st->right.d_L0, 0.0f, cap);
    }

    /* 8. Ground contact detection & airborne suppression */
    st->right_air_flag = ground_detectionR(&st->right);
    st->left_air_flag  = ground_detectionL(&st->left);

    const float k6 = LQR_K_calc(s_poly_coefficient[6], st->h5_length);
    const float k7 = LQR_K_calc(s_poly_coefficient[7], st->h5_length);

    if (st->recover_flag == 0u) {
        if (st->right_air_flag != 0u && st->left_air_flag != 0u && st->right.leg_flag == 0u) {
            wheel_r = 0.0f;
            st->right.Tp = k6 * (-st->right.theta + 0.01726156f)
                         + k7 * (-st->right.d_theta) + leg_tp;
        } else {
            st->right.leg_flag = 0u;
        }
        if (st->left_air_flag != 0u && st->right_air_flag != 0u && st->left.leg_flag == 0u) {
            wheel_l = 0.0f;
            st->left.Tp = k6 * (-st->left.theta + 0.01726156f)
                        + k7 * (-st->left.d_theta) + leg_tp;
        } else {
            st->left.leg_flag = 0u;
        }
    } else {
        st->right.Tp = 0.0f;
        st->left.Tp  = 0.0f;
    }

    st->right.F0 = clamp_f(st->right.F0, -H5_FORCE_LIMIT_N, H5_FORCE_LIMIT_N);
    st->left.F0  = clamp_f(st->left.F0, -H5_FORCE_LIMIT_N, H5_FORCE_LIMIT_N);

    /* 9. Jacobian transpose mapping -> hip joint torques & saturation clamps */
    VMC_calc_2(&st->right);
    VMC_calc_2(&st->left);

    out->pre_limit_torque[2] = st->right.torque_set[0];
    out->pre_limit_torque[3] = st->right.torque_set[1];
    out->pre_limit_torque[4] = st->left.torque_set[0];
    out->pre_limit_torque[5] = st->left.torque_set[1];

    st->right.torque_set[0] = clamp_f(st->right.torque_set[0], -H5_HIP_TORQUE_LIMIT_NM, H5_HIP_TORQUE_LIMIT_NM);
    st->right.torque_set[1] = clamp_f(st->right.torque_set[1], -H5_HIP_TORQUE_LIMIT_NM, H5_HIP_TORQUE_LIMIT_NM);
    st->left.torque_set[0]  = clamp_f(st->left.torque_set[0],  -H5_HIP_TORQUE_LIMIT_NM, H5_HIP_TORQUE_LIMIT_NM);
    st->left.torque_set[1]  = clamp_f(st->left.torque_set[1],  -H5_HIP_TORQUE_LIMIT_NM, H5_HIP_TORQUE_LIMIT_NM);

    /* Output order [L_front, L_rear, L_wheel, R_front, R_rear, R_wheel] (g2_adapter.c:99-104) */
    out->torque_cmd[0] = -st->left.torque_set[1];
    out->torque_cmd[1] =  st->left.torque_set[0];
    out->torque_cmd[2] =  wheel_l;
    out->torque_cmd[3] = -st->right.torque_set[1];
    out->torque_cmd[4] =  st->right.torque_set[0];
    out->torque_cmd[5] =  wheel_r;

    /* 10. Smooth regenerative torque taper when V_bus > 25.8 V */
    for (uint32_t i = 0u; i < 6u; ++i) {
        out->torque_cmd[i] = apply_regen_taper(out->torque_cmd[i], in->rate[i], in->regen_scale);
    }

    uint8_t clipped = 0u;
    for (uint32_t i = 0u; i < 6u; ++i) {
        const float lim = (i < 2u) ? H5_WHEEL_TORQUE_LIMIT_NM : H5_HIP_TORQUE_LIMIT_NM;
        if (fabsf(out->pre_limit_torque[i]) > lim) {
            clipped++;
        }
    }

    const uint8_t both_air = (st->right_air_flag != 0u && st->left_air_flag != 0u) ? 1u : 0u;
    out->s5_effort[0] = s5_effort[0];
    out->s5_effort[1] = s5_effort[1];
    out->s5_feedback[0] = s5_fb[0];
    out->s5_feedback[1] = s5_fb[1];
    out->leg_length[0] = st->left.L0;
    out->leg_length[1] = st->right.L0;
    out->leg_theta[0] = st->left.theta;
    out->leg_theta[1] = st->right.theta;
    out->theta_error_rad = st->h5_theta_error;
    out->stop_position_error_m = st->stop_position_error_m;
    out->s9_pitch_ref_rad = st->s9_pitch_reference;
    out->s9_brake_blend_m = st->s9_brake.blend_m;
    out->damping_est = st->s7_loss.damping;
    out->both_air = both_air;
    out->standing = (in->air == 0u && both_air == 0u
                     && st->jump_flag == 0u && st->recover_flag == 0u) ? 1u : 0u;
    out->s9_learning = st->s9_learning;
    out->clipped_axes = clipped;
}

void h5_balance_commit(H5BalanceState *st, const float s5_effort[S5_EFFORTS],
                       const float torque_cmd[6], uint8_t standing)
{
    if (st == NULL || s5_effort == NULL || torque_cmd == NULL) {
        return;
    }
    s5_commit(&st->s5, s5_effort, standing != 0u);
    st->s7_loss.previous_request = 0.5f * (torque_cmd[2] + torque_cmd[5]);
}
