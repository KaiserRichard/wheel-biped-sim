/*
 * Unified 500 Hz Balance Control Pipeline (balance_ctrl)
 *
 * Implements the exact 8-step 500 Hz execution order of FIRMWARE_DESIGN_v2.md Section 3.2.
 * Pure C11, zero HAL/RTOS dependencies, zero dynamic memory.
 */

#include "control/balance_ctrl.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static const float RAD_TO_DEG_F = 57.29577951308232f;

static float clamp_f32(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static uint8_t s11_mode_is_fault(enum S11Mode m)
{
    return (m == S11_TIMEOUT || m == S11_INVALID
         || m == S11_DRIVE_FAULT || m == S11_STUCK) ? 1u : 0u;
}

static uint16_t clamp_u16_ms_to_us(double age_ms)
{
    if (!isfinite(age_ms) || age_ms <= 0.0) {
        return 0u;
    }
    const double us = age_ms * 1000.0;
    if (us >= 65535.0) {
        return 65535u;
    }
    return (uint16_t)us;
}

static uint16_t clamp_vbus_mv(float v_bus_v)
{
    if (!isfinite(v_bus_v) || v_bus_v <= 0.0f) {
        return 0u;
    }
    const float mv = v_bus_v * 1000.0f;
    if (mv >= 65535.0f) {
        return 65535u;
    }
    return (uint16_t)mv;
}

BalanceCtrlConfig balance_ctrl_default_config(void)
{
    BalanceCtrlConfig c;
    memset(&c, 0, sizeof(c));
    c.h5_cfg            = h5_balance_default_config();
    c.safety_cfg        = safety_default_config();
    c.s11_cfg           = s11_default_config();
    c.can_cfg           = can_sched_default_config();
    c.fivebar_geom      = s10_default_fivebar_geom();
    c.wheel_radius_m    = 0.0603;
    /* IMU site position from sim_model/draw_001.xml (model_base_imu_site). */
    c.imu_lever_m[0]    = 0.00265373721167746;
    c.imu_lever_m[1]    = 1.5332899978765e-07;
    c.imu_lever_m[2]    = 0.0344707443228917;
    c.use_s10_estimator = 1u;
    c.s10_robust        = 1u;
    c.enable_step_assist = 1u;
    c.step_height_m = 0.010f;
    c.step_backoff_m = 0.12f;
    c.step_back_m_s = 0.15f;
    /* FW6: 0.50 m/s (STEP3RC 0.45). At 0.45 the 6.2 kg model stalled at a later
     * box of the 20-box F3 field; 0.475-0.55 cross all slow cells (FW6 report). */
    c.step_charge_m_s = 0.50f;
    c.step_acc_m_s2 = 1.5f;
    c.step_pass_m = 0.06f;
    c.step_timeout_ms = 3000u;
    c.step_refractory_ms = 1000u;
    c.step_handback_scale = 0.5f;
    c.step_carry_m = 0.0f;
    return c;
}

void balance_ctrl_init(BalanceCtrl *ctrl, const BalanceCtrlConfig *cfg)
{
    if (ctrl == NULL) {
        return;
    }
    memset(ctrl, 0, sizeof(*ctrl));
    ctrl->cfg = (cfg != NULL) ? *cfg : balance_ctrl_default_config();

    can_sched_init(&ctrl->can_sched, &ctrl->cfg.can_cfg);
    s11_init_with_config(&ctrl->s11, &ctrl->cfg.s11_cfg);
    s10_init(&ctrl->s10, ctrl->cfg.wheel_radius_m, ctrl->cfg.imu_lever_m);
    ctrl->s10.robust = (ctrl->cfg.s10_robust != 0u) ? 1 : 0;
    safety_fsm_init_with_config(&ctrl->safety, &ctrl->cfg.safety_cfg);
    h5_balance_init(&ctrl->h5, &ctrl->cfg.h5_cfg);
}

void balance_ctrl_attach_flight_ring(BalanceCtrl *ctrl, LogFlightRing *ring)
{
    if (ctrl == NULL) {
        return;
    }
    ctrl->flight_ring = ring;
}

/* FW6: the frozen-encoder IMU transient gate runs on the 1 kHz IMU path; the
 * 500 Hz S11 tick uses its latest value (STEP3RC B6 item 1). */
static void balance_ctrl_frozen_gate(BalanceCtrl *ctrl, const double gyro_rad_s[3],
                                     const double accel_m_s2[3])
{
    const double acc_norm = sqrt(accel_m_s2[0] * accel_m_s2[0]
                                 + accel_m_s2[1] * accel_m_s2[1]
                                 + accel_m_s2[2] * accel_m_s2[2]);
    s11_frozen_imu_update(&ctrl->s11, gyro_rad_s[0], gyro_rad_s[1], acc_norm, 1.0);
}

void balance_ctrl_feed_imu_1khz(BalanceCtrl *ctrl, int32_t tick_ms,
                                const double gyro_rad_s[3],
                                const double accel_m_s2[3])
{
    if (ctrl == NULL || gyro_rad_s == NULL || accel_m_s2 == NULL) {
        return;
    }
    s10_imu(&ctrl->s10, tick_ms, gyro_rad_s, accel_m_s2);
    balance_ctrl_frozen_gate(ctrl, gyro_rad_s, accel_m_s2);
}

void balance_ctrl_tick(BalanceCtrl *ctrl, const BalanceTickInput *in, BalanceTickOutput *out)
{
    if (ctrl == NULL || in == NULL || out == NULL) {
        return;
    }
    memset(out, 0, sizeof(*out));

    /* =========================================================================
     * STEP 1: Snapshot Inputs (IMU, 6x CAN RX slots & ages, radio, host, power)
     * ========================================================================= */
    const uint8_t n_imu = (in->imu_sample_count <= BALANCE_CTRL_MAX_IMU)
                        ? in->imu_sample_count : BALANCE_CTRL_MAX_IMU;
    for (uint8_t i = 0u; i < n_imu; ++i) {
        s10_imu(&ctrl->s10, in->imu_samples[i].tick_ms,
                in->imu_samples[i].gyro_rad_s,
                in->imu_samples[i].accel_m_s2);
        balance_ctrl_frozen_gate(ctrl, in->imu_samples[i].gyro_rad_s,
                                 in->imu_samples[i].accel_m_s2);
    }

    CanSchedRxSnapshot rx_snap;
    if (in->use_can_sched_rx != 0u) {
        (void)can_sched_snapshot_rx(&ctrl->can_sched, in->tick_us, &rx_snap);
    } else {
        rx_snap = in->direct_enc_snap;
    }
    can_sched_get_health(&ctrl->can_sched, &out->can_health);

    /* Keep S11 armed during active states; disarm/reset when DISARMED. */
    if (ctrl->safety.state == SAFETY_STATE_DISARMED) {
        if (ctrl->s11.latched != S11_FRESH) {
            s11_reset(&ctrl->s11);
        } else {
            s11_disarm(&ctrl->s11);
        }
    } else if (ctrl->safety.state == SAFETY_STATE_PRECHARGE
               || ctrl->safety.state == SAFETY_STATE_ARMED
               || ctrl->safety.state == SAFETY_STATE_BALANCING
               || ctrl->safety.state == SAFETY_STATE_STEP_ASSIST
               || ctrl->safety.state == SAFETY_STATE_CONTROLLED_STOP
               || ctrl->safety.state == SAFETY_STATE_REST) {
        if (ctrl->s11.arm_state == S11_STATE_DISARMED && ctrl->s11.latched == S11_FRESH) {
            (void)s11_arm(&ctrl->s11);
        }
    }

    /* =========================================================================
     * STEP 2: S11 Packet-Age Policy (extrapolation <= 4 ms, 75% authority, latch)
     * ========================================================================= */
    if (isfinite(in->v_bus_v) && in->v_bus_v >= 12.0f && in->v_bus_v <= 36.0f) {
        (void)s11_set_bus_voltage(&ctrl->s11, (double)in->v_bus_v);
    }
    if (in->use_can_sched_rx != 0u) {
        s11_packets_snapshot(&ctrl->s11,
                             &rx_snap.s11_snap,
                             (double)BALANCE_CTRL_DT_MS,
                             &out->s11_frame);
    } else {
        s11_packets_ex(&ctrl->s11,
                       rx_snap.fresh_mask,
                       rx_snap.valid_mask,
                       rx_snap.healthy_mask,
                       rx_snap.age_ms,
                       rx_snap.angle,
                       rx_snap.rate,
                       (double)BALANCE_CTRL_DT_MS,
                       rx_snap.seq,
                       &out->s11_frame);
    }

    const uint8_t s11_fault = s11_mode_is_fault(out->s11_frame.mode);
    const uint8_t s11_limited = (out->s11_frame.mode == S11_LIMIT
                                 || out->s11_frame.mode == S11_DEGRADED
                                 || (out->s11_frame.valid != 0u && out->s11_frame.authority < 1.0))
                              ? 1u : 0u;

    /* FW6 degraded mode: every consumer (S10 estimator, H5 balance) sees the same
     * substituted data. A frozen wheel uses its partner's increment and rate. A
     * frozen hip, by default (frozen_hip_mirror = 1), copies the same-role hip of
     * the other leg; only if that hip is also frozen does it hold the geometry
     * captured at confirmation with zero rate. */
    if (out->s11_frame.valid != 0u && out->s11_frame.degraded_mask != 0u) {
        for (uint32_t i = 0u; i < 6u; ++i) {
            if ((out->s11_frame.degraded_mask & (1u << i)) != 0u) {
                rx_snap.angle[i] = out->s11_frame.angle[i];
                rx_snap.rate[i] = out->s11_frame.rate[i];
            }
        }
    }

    /* =========================================================================
     * STEP 3: Estimator Update (ONLY genuine fresh encoder packets into S10)
     * ========================================================================= */
    const uint8_t all_genuine_fresh =
        (rx_snap.fresh_mask == 0x3Fu
         && rx_snap.valid_mask == 0x3Fu
         && rx_snap.healthy_mask == 0x3Fu) ? 1u : 0u;

    if (ctrl->cfg.use_s10_estimator != 0u && all_genuine_fresh != 0u) {
        S10Encoder enc_pkt;
        if (s10_decode_fivebar(&ctrl->cfg.fivebar_geom,
                               rx_snap.angle, rx_snap.rate, &enc_pkt) != 0u) {
            s10_encoder(&ctrl->s10, in->enc_tick_ms, in->enc_tick_ms, &enc_pkt);
            out->s10_fed_fresh_enc = 1u;
        }
    }

    float est_pitch = 0.0f;
    float est_roll = 0.0f;
    float est_yaw = 0.0f;
    float est_roll_rate = 0.0f;
    float est_pitch_rate = 0.0f;
    float est_yaw_rate = 0.0f;
    float est_v = 0.0f;
    uint8_t est_air = 0u;
    uint8_t est_valid = 0u;

    if (ctrl->cfg.use_s10_estimator != 0u) {
        if (s10_ready(&ctrl->s10) != 0) {
            s10_frame(&ctrl->s10, &out->est_frame);
            est_pitch      = (float)out->est_frame.pitch;
            est_roll       = (float)out->est_frame.roll;
            est_yaw        = (float)out->est_frame.yaw;
            est_roll_rate  = (float)out->est_frame.rate[0];
            est_pitch_rate = (float)out->est_frame.rate[1];
            est_yaw_rate   = (float)out->est_frame.rate[2];
            est_v          = (float)out->est_frame.speed;
            est_air        = (out->est_frame.air != 0) ? 1u : 0u;
            est_valid      = (isfinite(est_pitch) && isfinite(est_roll) && isfinite(est_v)) ? 1u : 0u;
        }
    } else {
        est_pitch      = in->ext_state.pitch_rad;
        est_roll       = in->ext_state.roll_rad;
        est_yaw        = in->ext_state.yaw_rad;
        est_roll_rate  = in->ext_state.roll_rate_rad_s;
        est_pitch_rate = in->ext_state.pitch_rate_rad_s;
        est_yaw_rate   = in->ext_state.yaw_rate_rad_s;
        est_v          = in->ext_state.v_est_m_s;
        est_air        = in->ext_state.air;
        est_valid      = in->ext_state.valid;
        out->est_frame.pitch   = (double)est_pitch;
        out->est_frame.roll    = (double)est_roll;
        out->est_frame.yaw     = (double)est_yaw;
        out->est_frame.rate[0] = (double)est_roll_rate;
        out->est_frame.rate[1] = (double)est_pitch_rate;
        out->est_frame.rate[2] = (double)est_yaw_rate;
        out->est_frame.speed   = (double)est_v;
        out->est_frame.air     = (int32_t)est_air;
    }

    /* =========================================================================
     * STEP 4: Safety Supervisor FSM
     * ========================================================================= */
    SafetyInputs s_in;
    memset(&s_in, 0, sizeof(s_in));
    safety_fsm_bind_s11_frame(&s_in, &out->s11_frame);
    s_in.dt_ms              = BALANCE_CTRL_DT_MS;
    s_in.estop_open         = in->estop_open;
    s_in.boot_self_test_ok  = in->boot_self_test_ok;
    s_in.bor3_fault         = in->bor3_fault;
    s_in.can_bus_off        = out->can_health.any_bus_off;
    s_in.clamp_fault        = in->clamp_fault;
    s_in.dump_continuity_ok = in->dump_continuity_ok;
    s_in.k1_closed          = in->k1_closed;
    s_in.k2_closed          = in->k2_closed;
    s_in.kpre_closed        = in->kpre_closed;
    s_in.precharge_dv_ok    = in->precharge_dv_ok;
    s_in.mirror_mismatch    = in->mirror_mismatch;
    s_in.hard_uv_fault      = in->hard_uv_fault;
    s_in.wdg_fault          = in->wdg_fault;
    s_in.v_bus_v            = in->v_bus_v;
    s_in.v_batt_ocv_v       = in->v_batt_ocv_v;
    s_in.drives_healthy_6   = (out->s11_frame.mode == S11_DRIVE_FAULT) ? 0u : rx_snap.drives_healthy_6;
    s_in.drive_enable_fail  = in->drive_enable_fail;
    s_in.s11_fault          = s11_fault;
    s_in.s11_limited        = (s11_limited != 0u || out->can_health.inhibit_jump != 0u) ? 1u : 0u;
    s_in.imu_est_valid      = est_valid;
    s_in.ground_left        = (est_air == 0u) ? 1u : 0u;
    s_in.ground_right       = (est_air == 0u) ? 1u : 0u;
    s_in.pitch_deg          = est_pitch * RAD_TO_DEG_F;
    s_in.roll_deg           = est_roll * RAD_TO_DEG_F;
    s_in.pitch_rate_deg_s   = est_pitch_rate * RAD_TO_DEG_F;
    s_in.v_est_m_s          = est_v;
    s_in.radio_age_ms       = in->radio_age_ms;
    s_in.radio_arm_sw       = in->radio_arm_sw;
    s_in.radio_estop_sw     = in->radio_estop_sw;
    s_in.host_age_ms        = in->host_age_ms;
    s_in.host_valid         = in->host_valid;
    s_in.v_target_m_s       = in->v_target_m_s;
    s_in.yaw_target_rad_s   = in->yaw_target_rad_s;
    s_in.stand_handoff_req  = in->stand_handoff_req;
    /* Stop a blocked drive before reference integration causes breakaway slip.
     * Qualify previous progress to exclude startup/trim capture, then require
     * 100 ms below 25% of reference speed with > 30 mm signed position deficit.
     * Restrict to low reference speed so the 0.3/0.5 m/s grid retains crossings.
     * Reuse the latched S9 brake-to-REST path; no change to radio-loss policy.
     * Thresholds are development choices, not measured hardware constants. */
    const float v_ref = ctrl->safety.v_ref_m_s;
    if (ctrl->cfg.use_s10_estimator != 0u
        && ctrl->safety.state == SAFETY_STATE_BALANCING
        && fabsf(v_ref) > 0.05f && fabsf(v_ref) <= 0.15f
        && est_air == 0u) {
        if (ctrl->progress_ref_m_s * v_ref <= 0.0f) {
            ctrl->progress_ref_m_s = 0.0f;
            ctrl->blocked_drive_ms = 0u;
        }
        if (est_v * v_ref >= 0.5f * v_ref * v_ref) {
            ctrl->progress_ref_m_s = v_ref;
        }
        const float signed_error = (v_ref > 0.0f ? 1.0f : -1.0f)
                                   * (ctrl->h5.x_est_m - ctrl->h5.x_set_m);
        /* Steady drive lag: the wheel loop has no integrator, so an offset COM is
         * held by a standing position error. Remember it so rebases keep it. */
        if (ctrl->blocked_drive_ms == 0u && fabsf(est_v - v_ref) < 0.03f) {
            ctrl->drive_lag_m += (ctrl->h5.x_set_m - ctrl->h5.x_est_m - ctrl->drive_lag_m)
                                 * (BALANCE_CTRL_DT_S / 1.0f);
            ctrl->drive_lag_m = clamp_f32(ctrl->drive_lag_m, -0.03f, 0.03f);
        }
        if (ctrl->step_refractory_ms >= BALANCE_CTRL_DT_MS) {
            ctrl->step_refractory_ms -= BALANCE_CTRL_DT_MS;
        }
        if (ctrl->progress_ref_m_s != 0.0f
            && ctrl->step_refractory_ms == 0u
            && est_v * v_ref < 0.25f * v_ref * v_ref
            && signed_error < -0.03f) {
            if (ctrl->blocked_drive_ms < 100u) {
                ctrl->blocked_drive_ms += BALANCE_CTRL_DT_MS;
            }
        } else {
            ctrl->blocked_drive_ms = 0u;
        }
    } else {
        ctrl->progress_ref_m_s = 0.0f;
        ctrl->blocked_drive_ms = 0u;
    }
    /* FW6 host declaration (STEP3RC A6 item 3): the run-up is allowed only for a
     * declared curb whose declared edge height is within the validated band (a
     * charge into a taller obstacle falls, STEP3RC A5).
     * FW6FIX D1 (interim): a FIELD declaration is refused. It counts as no
     * declaration, so a blocked drive at a field makes the F3b controlled stop
     * before the field. The refusal is reported every tick it is asserted. */
    const float decl_h = (isfinite(in->step_decl_height_m) && in->step_decl_height_m > 0.0f)
                       ? in->step_decl_height_m : ctrl->cfg.step_height_m;
    if (in->step_declaration == (uint8_t)BALANCE_STEP_DECL_FIELD) {
        out->step_decl_refused = 1u;
        if (ctrl->step_decl_refused_n < UINT32_MAX) {
            ctrl->step_decl_refused_n++;
        }
    }
    const uint8_t runup_declared = (in->step_declaration == (uint8_t)BALANCE_STEP_DECL_CURB
        && decl_h <= BALANCE_STEP_MAX_HEIGHT_M) ? 1u : 0u;
    /* STEP_ASSIST is a torque-enabled supervisor state. The detector above is
     * unchanged; only its response differs. No simulated terrain truth is used. */
    const uint8_t legs_available = (ctrl->cfg.enable_step_assist != 0u
        && est_valid != 0u && s11_limited == 0u
        && ctrl->h5.jump_flag == 0u && in->jump_req == 0u
        && ctrl->h5.left.L0 >= 0.12f && ctrl->h5.right.L0 >= 0.12f
        && ctrl->h5.left.L0 <= 0.28f && ctrl->h5.right.L0 <= 0.28f
        && in->v_bus_v >= ctrl->safety.config.mild_sag_v
        && fabsf(v_ref) > 0.05f && fabsf(v_ref) <= 0.15f
        && fabsf(in->v_target_m_s) <= 0.15f
        && runup_declared != 0u) ? 1u : 0u;
    const SafetyState previous_state = ctrl->safety.state;
    const uint8_t blocked = ctrl->blocked_drive_ms >= 100u ? 1u : 0u;
    if (ctrl->step_damping_ms >= BALANCE_CTRL_DT_MS) {
        ctrl->step_damping_ms -= BALANCE_CTRL_DT_MS;
    }
    if (previous_state == SAFETY_STATE_STEP_ASSIST) {
        /* Back-off-and-charge: wheel-only manoeuvre, legs keep normal height PD.
         * Phase 4 reverse, 5 settle, 6 charge. Uses only odometry/estimator. */
        const float dir = ctrl->step_direction;
        const float rel = dir * (ctrl->h5.x_est_m - ctrl->step_start_x_m);
        ctrl->step_elapsed_ms += BALANCE_CTRL_DT_MS;
        ctrl->step_phase_ms += BALANCE_CTRL_DT_MS;
        ctrl->step_limit_ms = ctrl->step_clipped != 0u
                           ? ctrl->step_limit_ms + BALANCE_CTRL_DT_MS : 0u;
        float v_goal = 0.0f;
        if (ctrl->step_phase == 4u) {
            v_goal = -ctrl->cfg.step_back_m_s;
            /* Stop reversing at the configured distance, or if reverse motion
             * itself stalls (e.g. backing into the previous edge). */
            if (rel <= -ctrl->cfg.step_backoff_m
                || (ctrl->step_phase_ms >= 300u && dir * est_v > -0.02f)) {
                ctrl->step_backed_m = -rel;
                ctrl->step_phase = 5u;
                ctrl->step_phase_ms = 0u;
                /* Too little room to build speed: refuse to charge. */
                if (ctrl->step_backed_m < 0.6f * ctrl->cfg.step_backoff_m) {
                    ctrl->step_abort = 1u;
                }
            }
        }
        if (ctrl->step_phase == 5u) {
            v_goal = 0.0f;
            if (ctrl->step_phase_ms >= 200u && fabsf(ctrl->step_v_m_s) < 1e-4f) {
                ctrl->step_phase = 6u;
                ctrl->step_phase_ms = 0u;
            }
        }
        if (ctrl->step_phase == 6u && rel > ctrl->cfg.step_pass_m + ctrl->step_carry_eff_m
            && dir * est_v > 0.05f) {
            /* Passage: ramp the reference back to the commanded speed in-state so
             * x_set stays continuous (no rebase, no speed step at hand-back). */
            ctrl->step_phase = 7u;
            ctrl->step_phase_ms = 0u;
        }
        if (ctrl->step_phase == 7u) {
            v_goal = dir * v_ref;
        }
        if (ctrl->step_phase == 6u) {
            v_goal = ctrl->cfg.step_charge_m_s;
            if (dir * est_v >= 0.6f * ctrl->cfg.step_charge_m_s) {
                ctrl->step_charged = 1u;
            }
            /* Charge stalled at an edge (first, or a later one while carrying):
             * stop before the reference winds up. */
            /* A single sample below 20% aborts. FW6 tested 60-150 ms persistence:
             * it let the 6.2 kg model fall on the F3 field (incipient stall). */
            if (ctrl->step_charged != 0u
                && dir * est_v < 0.2f * ctrl->cfg.step_charge_m_s) {
                ctrl->step_abort = 2u;
            }
            if (ctrl->step_phase_ms >= 1500u && ctrl->step_charged == 0u) {
                ctrl->step_abort = 3u;
            }
        }
        /* Hand-back decelerates at half the run-up slew: a hard stop on the
         * plateau pitches an offset-COM robot back (-12.6 deg on draw_001). */
        const float dv_max = (ctrl->step_phase == 7u ? ctrl->cfg.step_handback_scale : 1.0f)
                             * ctrl->cfg.step_acc_m_s2 * BALANCE_CTRL_DT_S;
        const float v_rel = dir * ctrl->step_v_m_s;
        const float dv = clamp_f32(v_goal - v_rel, -dv_max, dv_max);
        ctrl->step_v_m_s = dir * (v_rel + dv);
        ctrl->step_a_m_s2 = dir * dv / BALANCE_CTRL_DT_S;
        s_in.step_assist_done = (ctrl->step_phase == 7u
            && fabsf(ctrl->step_v_m_s - v_ref) < 0.005f) ? 1u : 0u;
        s_in.step_assist_failed = (fabsf(s_in.pitch_deg) > 15.0f
            || ctrl->step_abort != 0u
            || ctrl->step_limit_ms > 100u
            || ctrl->step_elapsed_ms >= ctrl->step_timeout_eff_ms
            || legs_available == 0u || v_ref * dir <= 0.0f) ? 1u : 0u;
    }
    s_in.step_assist_req = (blocked != 0u && legs_available != 0u
                           && fabsf(s_in.pitch_deg) <= 15.0f) ? 1u : 0u;
    s_in.voluntary_stop_req = (in->voluntary_stop_req != 0u
        || (blocked != 0u && s_in.step_assist_req == 0u)) ? 1u : 0u;
    s_in.local_arm_req      = in->local_arm_req;
    s_in.local_reset_req    = in->local_reset_req;
    s_in.hw_reset_ack       = in->hw_reset_ack;
    s_in.local_regrant_req  = in->local_regrant_req;

    safety_fsm_step(&ctrl->safety, &s_in, &out->safety_out);
    if (ctrl->safety.state == SAFETY_STATE_STEP_ASSIST
        && previous_state != SAFETY_STATE_STEP_ASSIST) {
        ctrl->step_elapsed_ms = 0u;
        ctrl->step_limit_ms = 0u;
        ctrl->step_clipped = 0u;
        ctrl->step_phase = 4u;
        ctrl->step_phase_ms = 0u;
        ctrl->step_v_m_s = 0.0f;
        ctrl->step_a_m_s2 = 0.0f;
        ctrl->step_backed_m = 0.0f;
        ctrl->step_charged = 0u;
        ctrl->step_abort = 0u;
        ctrl->step_start_x_m = ctrl->h5.x_est_m;
        ctrl->step_direction = v_ref > 0.0f ? 1.0f : -1.0f;
        /* Curb only (FW6FIX D1): no field carry, nominal manoeuvre timeout. */
        ctrl->step_carry_eff_m = 0.0f;
        ctrl->step_timeout_eff_ms = ctrl->cfg.step_timeout_ms;
        /* Drop the stall deficit but keep the steady drive lag. */
        ctrl->h5.x_set_m = ctrl->h5.x_est_m + ctrl->drive_lag_m;
    } else if (previous_state == SAFETY_STATE_STEP_ASSIST
               && ctrl->safety.state != SAFETY_STATE_STEP_ASSIST) {
        ctrl->step_phase = 0u;
        ctrl->blocked_drive_ms = 0u;
        ctrl->progress_ref_m_s = 0.0f;
        if (ctrl->safety.state == SAFETY_STATE_BALANCING) {
            ctrl->step_damping_ms = 150u;
            ctrl->step_refractory_ms = ctrl->cfg.step_refractory_ms; /* plateau */
        }
    }

    /* Frozen-encoder ADVISORY: flag only, inhibit jumps (STEP3RC B5). */
    if (out->s11_frame.advisory_mask != 0u || out->s11_frame.degraded_mask != 0u) {
        out->safety_out.inhibit_jump = 1u;
    }
    /* Ensure CAN ErrorPassive always inhibits jumps per Section 4.3. */
    if (out->can_health.inhibit_jump != 0u) {
        out->safety_out.inhibit_jump = 1u;
    }

    /* =========================================================================
     * STEP 5: S9 Host-Loss Brake (every enabled tick) & Balance-Point Trim
     * STEP 6: VMC + S7/S5 Predictor + PIDs + Jump FSM + Clamps & Regen Taper
     * ========================================================================= */
    uint8_t active_control = 0u;
    if (out->safety_out.torque_enable != 0u && out->s11_frame.valid != 0u) {
        active_control = 1u;

        ctrl->h5.x_est_m += est_v * BALANCE_CTRL_DT_S;
        const uint8_t runup = (ctrl->safety.state == SAFETY_STATE_STEP_ASSIST
                               && ctrl->step_phase >= 4u) ? 1u : 0u;
        if (runup != 0u) {
            ctrl->h5.x_set_m += ctrl->step_v_m_s * BALANCE_CTRL_DT_S;
        } else if (ctrl->safety.state != SAFETY_STATE_STEP_ASSIST) {
            ctrl->h5.x_set_m += out->safety_out.v_ref_m_s * BALANCE_CTRL_DT_S;
        }
        const float pos_lim = ctrl->h5.cfg.stop_position_limit_m;
        const float x_err_clamped = clamp_f32(ctrl->h5.x_est_m - ctrl->h5.x_set_m,
                                              -pos_lim, pos_lim);
        ctrl->h5.x_est_m = ctrl->h5.x_set_m + x_err_clamped;
        const float x_clamped = ctrl->h5.x_est_m;

        H5BalanceInput h5_in;
        memset(&h5_in, 0, sizeof(h5_in));
        h5_in.pitch_rad        = est_pitch;
        h5_in.pitch_rate_rad_s = est_pitch_rate;
        h5_in.roll_rad         = est_roll;
        h5_in.roll_rate_rad_s  = est_roll_rate;
        h5_in.yaw_rad          = est_yaw;
        h5_in.yaw_rate_rad_s   = est_yaw_rate;
        h5_in.x_m              = x_clamped;
        h5_in.v_m_s            = est_v;
        h5_in.x_set_m          = ctrl->h5.x_set_m;
        h5_in.v_set_m_s        = runup != 0u ? ctrl->step_v_m_s : out->safety_out.v_ref_m_s;
        h5_in.yaw_ref_rad_s    = out->safety_out.yaw_ref_rad_s;
        h5_in.accel_ref_m_s2   = (out->safety_out.host_lost != 0u) ? 0.0f
                               : (runup != 0u ? ctrl->step_a_m_s2 : in->accel_ref_m_s2);
        h5_in.jerk_ref_m_s3    = (out->safety_out.host_lost != 0u) ? 0.0f : in->jerk_ref_m_s3;
        for (uint32_t i = 0u; i < 6u; ++i) {
            h5_in.angle[i] = (float)out->s11_frame.angle[i];
            h5_in.rate[i]  = (float)out->s11_frame.rate[i];
        }
        h5_in.air              = est_air;
        h5_in.host_lost        = out->safety_out.host_lost;
        h5_in.inhibit_trim     = (out->safety_out.inhibit_trim != 0u
                                  || in->external_inhibit_trim != 0u) ? 1u : 0u;
        h5_in.inhibit_jump     = out->safety_out.inhibit_jump;
        h5_in.jump_req         = in->jump_req;
        /* Post-hand-back height damping (150 ms); the mode-0 leg pulse is removed. */
        h5_in.step_extra_damping = (ctrl->step_damping_ms != 0u) ? 1u : 0u;
        h5_in.mild_sag_setdown = out->safety_out.mild_sag_setdown;
        h5_in.regen_scale      = out->safety_out.regen_scale;

        /* Step 5: S9 brake tracking & quiet trim update */
        h5_balance_step_brake_and_trim(&ctrl->h5, &h5_in);
        if (out->safety_out.host_lost != 0u) {
            /* Keep x_est_m synchronized with the active S9 brake error so an explicit
             * REST -> BALANCING re-grant hands off with zero step in (x_filter_eff - x_set_eff). */
            ctrl->h5.x_est_m = ctrl->h5.x_set_m + ctrl->h5.stop_position_error_m;
        }

        /* Step 6: VMC + S7/S5 + PIDs + Jump FSM + Clamps & Regen Taper */
        h5_balance_step_compute(&ctrl->h5, &h5_in, &out->h5_out);
        ctrl->step_clipped = out->h5_out.clipped_axes != 0u ? 1u : 0u;

        for (uint32_t i = 0u; i < 6u; ++i) {
            out->torque_cmd[i] = out->h5_out.torque_cmd[i];
        }
    } else {
        h5_balance_reset_disabled(&ctrl->h5);
        for (uint32_t i = 0u; i < 6u; ++i) {
            out->torque_cmd[i] = 0.0f;
        }
    }

    /* =========================================================================
     * STEP 7: Parallel 3-Bus CAN TX Dispatch & Predictor Commit
     * ========================================================================= */
    if (out->safety_out.can_abort_tx != 0u || out->safety_out.can_disable_cmd != 0u) {
        (void)can_sched_emergency_disable(&ctrl->can_sched, in->tick_us);
        const float zeros[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        const float zero_eff[S5_EFFORTS] = { 0.0f, 0.0f };
        h5_balance_commit(&ctrl->h5, zero_eff, zeros, 0u);
    } else if (out->safety_out.can_enable_cmd != 0u) {
        (void)can_sched_enqueue_enable_all(&ctrl->can_sched, in->tick_us);
        const float zeros[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        const float zero_eff[S5_EFFORTS] = { 0.0f, 0.0f };
        h5_balance_commit(&ctrl->h5, zero_eff, zeros, 0u);
    } else if (active_control != 0u) {
        (void)can_sched_enqueue_torques(&ctrl->can_sched, out->torque_cmd, in->tick_us);
        h5_balance_commit(&ctrl->h5, out->h5_out.s5_feedback, out->torque_cmd, out->h5_out.standing);
    } else if (out->safety_out.state == SAFETY_STATE_ARMED) {
        const float zeros[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        const float zero_eff[S5_EFFORTS] = { 0.0f, 0.0f };
        (void)can_sched_enqueue_torques(&ctrl->can_sched, zeros, in->tick_us);
        h5_balance_commit(&ctrl->h5, zero_eff, zeros, 0u);
    }

    /* Service any pending +1 ms repeat 0xFF..FD disable burst. */
    can_sched_service_tx(&ctrl->can_sched, in->tick_us);

    /* =========================================================================
     * STEP 8: Flight Recorder Push & Heartbeat
     * ========================================================================= */
    if (ctrl->flight_ring != NULL) {
        LogFlightRecord rec;
        memset(&rec, 0, sizeof(rec));
        rec.timestamp_us = (uint64_t)in->tick_us;
        rec.seq = ctrl->tick_count;
        rec.fast.fsm_state = (uint8_t)out->safety_out.state;
        rec.fast.s11_mode = (uint8_t)out->s11_frame.mode;
        rec.fast.s11_max_misses = (out->s11_frame.max_misses > 255u)
                                ? 255u : (uint8_t)out->s11_frame.max_misses;
        rec.fast.flags = (uint8_t)((out->safety_out.torque_enable != 0u ? 0x01u : 0u)
                                 | (out->safety_out.host_lost != 0u     ? 0x02u : 0u)
                                 | (out->safety_out.radio_lost != 0u    ? 0x04u : 0u)
                                 | (out->s10_fed_fresh_enc != 0u        ? 0x08u : 0u)
                                 | (out->step_decl_refused != 0u        ? 0x10u : 0u)
                                 | (out->safety_out.stop_timeout_alarm != 0u ? 0x20u : 0u));
        rec.fast.fault_mask = (uint32_t)out->safety_out.reason;
        rec.fast.euler_rad[0] = est_pitch;
        rec.fast.euler_rad[1] = est_roll;
        rec.fast.euler_rad[2] = est_yaw;
        rec.fast.gyro_rad_s[0] = est_roll_rate;
        rec.fast.gyro_rad_s[1] = est_pitch_rate;
        rec.fast.gyro_rad_s[2] = est_yaw_rate;
        rec.fast.v_est_m_s = est_v;
        rec.fast.x_est_m = ctrl->h5.x_est_m;
        rec.fast.leg_length_m[0] = out->h5_out.leg_length[0];
        rec.fast.leg_length_m[1] = out->h5_out.leg_length[1];
        rec.fast.leg_theta_rad[0] = out->h5_out.leg_theta[0];
        rec.fast.leg_theta_rad[1] = out->h5_out.leg_theta[1];
        for (uint32_t i = 0u; i < 6u; ++i) {
            rec.fast.motor_pos_rad[i]   = isfinite(rx_snap.angle[i]) ? (float)rx_snap.angle[i] : 0.0f;
            rec.fast.motor_vel_rad_s[i] = isfinite(rx_snap.rate[i])  ? (float)rx_snap.rate[i]  : 0.0f;
            rec.fast.motor_tor_cmd_nm[i] = out->torque_cmd[i];
            rec.fast.motor_tor_fb_nm[i]  = rx_snap.torque_fb[i];
            rec.fast.enc_age_us[i]       = clamp_u16_ms_to_us(rx_snap.age_ms[i]);
        }
        rec.fast.s9_trim_rad = ctrl->h5.s9_pitch_reference;
        rec.fast.gyro_bias_y_rad_s = (float)ctrl->s10.bias[1];
        rec.fast.v_bus_mv = clamp_vbus_mv(in->v_bus_v);
        rec.fast.i_bus_ma = in->i_bus_ma;

        if (out->safety_out.state == SAFETY_STATE_FAULT
            || out->safety_out.state == SAFETY_STATE_ESTOP) {
            log_ring_trigger_fault(ctrl->flight_ring);
        }
        if (log_ring_push(ctrl->flight_ring, &rec) == LOG_OK) {
            out->flight_rec_pushed = 1u;
        }
    }

    ctrl->balance_heartbeat++;
    ctrl->tick_count++;
    out->balance_heartbeat = ctrl->balance_heartbeat;
}

/* ============================================================================
 * Flat Zero-Malloc C ABI Helpers for Shared Library / Python ctypes FIL Tests
 * ============================================================================ */

static void fil_inject_can_feedback(BalanceCtrl *ctrl, uint32_t tick_us,
                                    const double raw_angles[6], const double raw_rates[6])
{
    for (uint8_t axis = 0u; axis < CAN_SCHED_NUM_AXES; ++axis) {
        DamiaoAxisConfig acfg;
        uint8_t bus_idx = 0u;
        if (damiao_get_axis_config(axis, &acfg) != DAMIAO_OK
            || can_sched_bus_for_axis(axis, &bus_idx) != CAN_SCHED_OK) {
            continue;
        }
        double pos = raw_angles[axis];
        if (acfg.motor_type == DAMIAO_MOTOR_DM6215) {
            pos = s11_wrap_angle(pos, 11.9);
        }

        DamiaoMitFeedback fb;
        memset(&fb, 0, sizeof(fb));
        fb.motor_id = (uint8_t)(axis + 1u);
        fb.status_nibble = ctrl->can_sched.cfg.expected_enabled_status;
        fb.pos = (float)pos;
        fb.vel = (float)raw_rates[axis];
        fb.torque = ctrl->fil_torque_fb[axis];
        fb.t_mos_c = 35.0f;
        fb.t_coil_c = 40.0f;

        CanFrame frame;
        memset(&frame, 0, sizeof(frame));
        frame.std_id = acfg.reply_can_id;
        frame.dlc = DAMIAO_CAN_DLC;
        if (damiao_pack_feedback(&acfg.limits, &fb, frame.data) == DAMIAO_OK) {
            (void)can_sched_rx_frame(&ctrl->can_sched, bus_idx, &frame, tick_us);
        }
    }
}

static void fil_drain_can_tx_torques(BalanceCtrl *ctrl, float out_torques[6])
{
    for (uint8_t i = 0u; i < 6u; ++i) {
        out_torques[i] = 0.0f;
    }
    for (uint8_t bus_idx = 0u; bus_idx < CAN_SCHED_NUM_BUSES; ++bus_idx) {
        CanFrame frame;
        while (can_sched_pop_tx(&ctrl->can_sched, bus_idx, &frame) == CAN_SCHED_OK) {
            for (uint8_t axis = 0u; axis < CAN_SCHED_NUM_AXES; ++axis) {
                DamiaoAxisConfig acfg;
                if (damiao_get_axis_config(axis, &acfg) == DAMIAO_OK
                    && frame.std_id == acfg.cmd_can_id) {
                    /* Ignore 0xFF..FC enable / 0xFF..FD disable frames for torque decode */
                    if (frame.data[0] == 0xFFu && frame.data[1] == 0xFFu
                        && frame.data[2] == 0xFFu && frame.data[3] == 0xFFu) {
                        out_torques[axis] = 0.0f;
                    } else {
                        DamiaoMitCmd cmd;
                        if (damiao_unpack_mit_cmd(&acfg.limits, frame.data, &cmd) == DAMIAO_OK) {
                            out_torques[axis] = cmd.torque;
                        }
                    }
                    break;
                }
            }
        }
    }
}

size_t balance_ctrl_state_size(void)
{
    return sizeof(BalanceCtrl);
}

void balance_ctrl_fil_torque_feedback(void *state_buf, const float torque_nm[6])
{
    BalanceCtrl *ctrl = (BalanceCtrl *)state_buf;
    if (ctrl != NULL && torque_nm != NULL) {
        memcpy(ctrl->fil_torque_fb, torque_nm, sizeof(ctrl->fil_torque_fb));
    }
}

void balance_ctrl_fil_step_config(void *state_buf, const float p[11])
{
    /* FIL-only: [mode, wheel_bias, backoff, back_v, charge_v, acc, pass, timeout_ms,
     *            refractory_ms, handback_scale, carry_m]
     * FW6FIX D6: p[0] (mode) and p[1] (mode-0 wheel bias) are ignored; the run-up
     * is the only step assist. The layout is kept for the existing FIL runners. */
    BalanceCtrl *ctrl = (BalanceCtrl *)state_buf;
    if (ctrl != NULL && p != NULL) {
        ctrl->cfg.step_backoff_m = p[2];
        ctrl->cfg.step_back_m_s = p[3];
        ctrl->cfg.step_charge_m_s = p[4];
        ctrl->cfg.step_acc_m_s2 = p[5];
        ctrl->cfg.step_pass_m = p[6];
        ctrl->cfg.step_timeout_ms = (uint32_t)p[7];
        ctrl->cfg.step_refractory_ms = (uint32_t)p[8];
        ctrl->cfg.step_handback_scale = p[9];
        ctrl->cfg.step_carry_m = p[10];
    }
}

void balance_ctrl_fil_disable_stuck(void *state_buf)
{
    /* FIL-only (STEP3RC part B): disable the S11 frozen-encoder detector to
     * measure the unprotected time-to-hazard after an injected freeze. */
    BalanceCtrl *ctrl = (BalanceCtrl *)state_buf;
    if (ctrl != NULL) {
        ctrl->s11.config.frozen_detect_enable = 0u;
    }
}

void balance_ctrl_fil_step_declare(void *state_buf, uint8_t declaration,
                                   float height_m, float field_length_m)
{
    /* FIL-only: host terrain declaration fed to every subsequent FIL tick. */
    BalanceCtrl *ctrl = (BalanceCtrl *)state_buf;
    if (ctrl != NULL) {
        ctrl->fil_step_decl = declaration;
        ctrl->fil_step_decl_h_m = height_m;
        ctrl->fil_step_field_m = field_length_m;
    }
}

void balance_ctrl_fil_s11_hip_mirror(void *state_buf, uint8_t enable)
{
    /* FIL-only: select the hip degraded-mode substitution (0 hold, 1 mirror). */
    BalanceCtrl *ctrl = (BalanceCtrl *)state_buf;
    if (ctrl != NULL) {
        ctrl->s11.config.frozen_hip_mirror = enable;
    }
}

void balance_ctrl_fil_s11_diag(void *state_buf, float diag[10])
{
    /* FIL-only: [0] CONFIRMED mask, [1] ADVISORY mask, [2] IMU gate, [3..8]
     * per-axis detector reason (S11FzReason), [9] latched S11 mode. */
    const BalanceCtrl *ctrl = (const BalanceCtrl *)state_buf;
    if (ctrl != NULL && diag != NULL) {
        diag[0] = (float)ctrl->s11.frozen_mask;
        diag[1] = (float)ctrl->s11.advisory_mask;
        diag[2] = (float)ctrl->s11.frozen_is_gated;
        for (uint32_t i = 0u; i < 6u; ++i) {
            diag[3u + i] = (float)ctrl->s11.axis[i].frozen.reason;
        }
        diag[9] = (float)ctrl->s11.latched;
    }
}

void balance_ctrl_fil_step_diag(void *state_buf, float diag[8])
{
    const BalanceCtrl *ctrl = (const BalanceCtrl *)state_buf;
    if (ctrl != NULL && diag != NULL) {
        diag[0] = (float)ctrl->step_phase;
        diag[1] = (float)ctrl->step_elapsed_ms;
        diag[2] = 0.0f;  /* was mode-0 landing flag (FW6FIX D6) */
        diag[3] = (float)ctrl->step_limit_ms;
        diag[4] = (float)ctrl->step_damping_ms;
        diag[5] = ctrl->drive_lag_m;
        diag[6] = (float)ctrl->step_abort;
        diag[7] = 0.0f;  /* was mode-0 hip torque (FW6FIX D6) */
    }
}

void balance_ctrl_fil_alarm_diag(void *state_buf, float diag[4])
{
    const BalanceCtrl *ctrl = (const BalanceCtrl *)state_buf;
    if (ctrl != NULL && diag != NULL) {
        diag[0] = (float)ctrl->step_decl_refused_n;
        diag[1] = (float)ctrl->safety.stop_timeout_latched;
        diag[2] = (float)ctrl->safety.reason;
        diag[3] = (float)ctrl->step_abort;
    }
}

void balance_ctrl_fil_init(void *state_buf, float mass_kg, float length_ref_m,
                           uint8_t radio_loss_policy)
{
    if (state_buf == NULL) {
        return;
    }
    BalanceCtrl *ctrl = (BalanceCtrl *)state_buf;
    BalanceCtrlConfig cfg = balance_ctrl_default_config();
    cfg.h5_cfg.mass_kg = clamp_f32(mass_kg, 3.72f, 6.40f);
    cfg.h5_cfg.trim_mass_kg = cfg.h5_cfg.mass_kg;
    cfg.h5_cfg.length_ref_m = length_ref_m;
    cfg.safety_cfg = safety_config_with_radio_policy(radio_loss_policy);
    balance_ctrl_init(ctrl, &cfg);
}

void balance_ctrl_fil_imu_1khz(void *state_buf, int32_t tick_ms,
                               const double gyro_rad_s[3],
                               const double accel_m_s2[3])
{
    if (state_buf == NULL) {
        return;
    }
    balance_ctrl_feed_imu_1khz((BalanceCtrl *)state_buf, tick_ms, gyro_rad_s, accel_m_s2);
}

int32_t balance_ctrl_fil_arm_and_handoff(void *state_buf, uint32_t tick_us, int32_t enc_tick_ms,
                                         const double raw_angles[6], const double raw_rates[6])
{
    if (state_buf == NULL || raw_angles == NULL || raw_rates == NULL) {
        return -1;
    }
    BalanceCtrl *ctrl = (BalanceCtrl *)state_buf;
    BalanceTickInput in;
    BalanceTickOutput out;

    /* Feed initial fresh encoder packet so S10 is ready */
    fil_inject_can_feedback(ctrl, tick_us, raw_angles, raw_rates);

    /* Tick 1: DISARMED -> PRECHARGE */
    memset(&in, 0, sizeof(in));
    in.tick_us            = tick_us;
    in.enc_tick_ms        = enc_tick_ms;
    in.use_can_sched_rx   = 1u;
    in.boot_self_test_ok  = 1u;
    in.dump_continuity_ok = 1u;
    in.v_bus_v            = 0.5f;
    in.v_batt_ocv_v       = 24.0f;
    in.radio_age_ms       = 4u;
    in.host_age_ms        = 10u;
    in.host_valid         = 1u;
    in.local_reset_req    = 1u;
    in.local_arm_req      = 1u;
    balance_ctrl_tick(ctrl, &in, &out);

    /* Ticks 2..4: PRECHARGE -> ARMED (3 fresh 6-axis groups arm S11 policy) */
    for (uint32_t step = 1u; step <= 3u; ++step) {
        const uint32_t t_step = tick_us + step * 2000u;
        fil_inject_can_feedback(ctrl, t_step, raw_angles, raw_rates);
        in.tick_us         = t_step;
        in.enc_tick_ms     = enc_tick_ms + (int32_t)(step * 2u);
        in.local_reset_req = 0u;
        in.local_arm_req   = 0u;
        in.k1_closed       = 1u;
        in.k2_closed       = 1u;
        in.kpre_closed     = 0u;
        in.precharge_dv_ok = 1u;
        in.v_bus_v         = 24.0f;
        in.radio_arm_sw    = 1u;
        balance_ctrl_tick(ctrl, &in, &out);
    }

    /* Drain any enable/zero frames */
    float dummy_torq[6];
    fil_drain_can_tx_torques(ctrl, dummy_torq);

    /* Final handoff tick: ARMED -> BALANCING */
    const uint32_t t_handoff = tick_us + 8000u;
    fil_inject_can_feedback(ctrl, t_handoff, raw_angles, raw_rates);
    in.tick_us           = t_handoff;
    in.enc_tick_ms       = enc_tick_ms + 8;
    in.stand_handoff_req = 1u;
    balance_ctrl_tick(ctrl, &in, &out);
    fil_drain_can_tx_torques(ctrl, dummy_torq);

    return (int32_t)out.safety_out.state;
}

int32_t balance_ctrl_fil_step_500hz_ex(void *state_buf, uint32_t tick_us, int32_t enc_tick_ms,
                                       const double raw_angles[6], const double raw_rates[6],
                                       float v_target_m_s, float yaw_target_rad_s,
                                       float accel_ref_m_s2, float jerk_ref_m_s3,
                                       uint8_t host_lost, uint8_t radio_lost,
                                       uint8_t external_inhibit_trim,
                                       float out_torques[6], float out_diag[10],
                                       float out_drive_diag[4])
{
    if (state_buf == NULL || raw_angles == NULL || raw_rates == NULL || out_torques == NULL) {
        return -1;
    }
    BalanceCtrl *ctrl = (BalanceCtrl *)state_buf;

    /* Round-trip encoder feedback through damiao_mit + 3-bus can_sched RX */
    fil_inject_can_feedback(ctrl, tick_us, raw_angles, raw_rates);

    BalanceTickInput in;
    memset(&in, 0, sizeof(in));
    in.tick_us               = tick_us;
    in.enc_tick_ms           = enc_tick_ms;
    in.use_can_sched_rx      = 1u;
    in.boot_self_test_ok     = 1u;
    in.dump_continuity_ok    = 1u;
    in.k1_closed             = 1u;
    in.k2_closed             = 1u;
    in.precharge_dv_ok       = 1u;
    in.v_bus_v               = 24.0f;
    in.v_batt_ocv_v          = 24.0f;
    in.radio_age_ms          = (radio_lost != 0u) ? 200u : 4u;
    in.radio_arm_sw          = (uint8_t)((radio_lost != 0u) ? 0u : 1u);
    in.host_age_ms           = (host_lost != 0u) ? 200u : 10u;
    in.host_valid            = (uint8_t)((host_lost != 0u) ? 0u : 1u);
    in.v_target_m_s          = v_target_m_s;
    in.yaw_target_rad_s      = yaw_target_rad_s;
    in.accel_ref_m_s2        = accel_ref_m_s2;
    in.jerk_ref_m_s3         = jerk_ref_m_s3;
    in.external_inhibit_trim = external_inhibit_trim;
    in.step_declaration      = ctrl->fil_step_decl;
    in.step_decl_height_m    = ctrl->fil_step_decl_h_m;
    in.step_field_length_m   = ctrl->fil_step_field_m;

    BalanceTickOutput out;
    balance_ctrl_tick(ctrl, &in, &out);

    /* Round-trip motor torque commands through 3-bus can_sched TX + damiao_mit */
    fil_drain_can_tx_torques(ctrl, out_torques);

    if (out_diag != NULL) {
        out_diag[0] = (float)out.est_frame.pitch;
        out_diag[1] = (float)out.est_frame.roll;
        out_diag[2] = (float)out.est_frame.speed;
        out_diag[3] = out.h5_out.leg_length[0];
        out_diag[4] = out.h5_out.leg_length[1];
        out_diag[5] = out.h5_out.theta_error_rad;
        out_diag[6] = out.h5_out.s9_pitch_ref_rad;
        out_diag[7] = (float)out.safety_out.state;
        out_diag[8] = (float)out.s11_frame.mode;
        out_diag[9] = (float)out.safety_out.torque_enable;
    }

    if (out_drive_diag != NULL) {
        out_drive_diag[0] = out.safety_out.v_ref_m_s;
        out_drive_diag[1] = out.safety_out.yaw_ref_rad_s;
        out_drive_diag[2] = ctrl->h5.x_filter_eff - ctrl->h5.x_set_eff;
        out_drive_diag[3] = (float)out.safety_out.reason;
    }

    return (int32_t)out.safety_out.state;
}

int32_t balance_ctrl_fil_step_500hz(void *state_buf, uint32_t tick_us, int32_t enc_tick_ms,
                                    const double raw_angles[6], const double raw_rates[6],
                                    float v_target_m_s, float yaw_target_rad_s,
                                    uint8_t radio_lost, uint8_t external_inhibit_trim,
                                    float out_torques[6], float out_diag[10])
{
    return balance_ctrl_fil_step_500hz_ex(state_buf, tick_us, enc_tick_ms,
                                          raw_angles, raw_rates,
                                          v_target_m_s, yaw_target_rad_s,
                                          0.0f, 0.0f, 0u, radio_lost,
                                          external_inhibit_trim,
                                          out_torques, out_diag, NULL);
}
