#ifdef NDEBUG
#undef NDEBUG
#endif
#include "control/balance_ctrl.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define S11_CHECK(expr)                                                   \
    do {                                                                  \
        if (!(expr)) {                                                    \
            fprintf(stderr, "%s:%d: %s: Assertion `%s' failed.\n",        \
                    __FILE__, __LINE__, __func__, #expr);                 \
            abort();                                                      \
        }                                                                 \
    } while (0)

#undef assert
#define assert(expr) S11_CHECK(expr)

static int near_f(float a, float b, float tol)
{
    return (fabsf(a - b) <= tol) ? 1 : 0;
}

static int near_d(double a, double b, double tol)
{
    return (fabs(a - b) <= tol) ? 1 : 0;
}

static void inject_all_6_can_fb(CanSched *sched, uint32_t rx_us,
                                const double angle[6], const double rate[6],
                                uint8_t status_nibble)
{
    for (uint8_t axis = 0u; axis < 6u; ++axis) {
        DamiaoAxisConfig acfg;
        uint8_t bus = 0u;
        assert(damiao_get_axis_config(axis, &acfg) == DAMIAO_OK);
        assert(can_sched_bus_for_axis(axis, &bus) == CAN_SCHED_OK);

        DamiaoMitFeedback fb;
        memset(&fb, 0, sizeof(fb));
        fb.motor_id = (uint8_t)(axis + 1u);
        fb.status_nibble = status_nibble;
        fb.pos = (float)angle[axis];
        fb.vel = (float)rate[axis];
        fb.torque = 0.0f;
        fb.t_mos_c = 36.0f;
        fb.t_coil_c = 42.0f;

        CanFrame frame;
        memset(&frame, 0, sizeof(frame));
        frame.std_id = acfg.reply_can_id;
        frame.dlc = DAMIAO_CAN_DLC;
        assert(damiao_pack_feedback(&acfg.limits, &fb, frame.data) == DAMIAO_OK);
        assert(can_sched_rx_frame(sched, bus, &frame, rx_us) == CAN_SCHED_OK);
    }
}

static void test_vmc_kinematics_and_pid(void)
{
    vmc_leg_t leg;
    VMC_init(&leg);
    VmcInsState ins;
    memset(&ins, 0, sizeof(ins));

    /* Sweep symmetric standing poses across workspace [0.09, 0.19] m and verify
     * finite FK, branch continuity, and virtual-work Jacobian consistency:
     *   tau^T * qdot == F0 * dL0 + Tp * (-d_alpha) */
    const float q_vals[5] = { 0.45f, 0.70f, 0.90f, 1.15f, 1.35f };
    for (uint32_t i = 0u; i < 5u; ++i) {
        const float q_front = q_vals[i];
        const float q_rear  = q_vals[i] + 0.05f;
        const float qd1 = 0.35f;
        const float qd4 = -0.25f;

        leg.phi1 = VMC_PI / 2.0f + q_rear;
        leg.phi4 = VMC_PI / 2.0f - q_front;
        leg.qd1 = qd1;
        leg.qd4 = qd4;
        leg.F0 = 22.5f;
        leg.Tp = 0.65f;

        VMC_calc_1_right(&leg, &ins, S4_DT);
        VMC_calc_2(&leg);
        assert(isfinite(leg.L0) && leg.L0 > 0.07f && leg.L0 < 0.21f);
        assert(isfinite(leg.torque_set[0]) && isfinite(leg.torque_set[1]));

        /* Finite-difference Jacobian check */
        const float eps = 1e-4f;
        vmc_leg_t p1 = leg;
        vmc_leg_t m1 = leg;
        p1.phi1 += eps; m1.phi1 -= eps;
        VMC_calc_1_right(&p1, &ins, S4_DT);
        VMC_calc_1_right(&m1, &ins, S4_DT);
        const float dL0_dq1 = (p1.L0 - m1.L0) / (2.0f * eps);
        const float dphi0_dq1 = (p1.phi0 - m1.phi0) / (2.0f * eps);

        vmc_leg_t p4 = leg;
        vmc_leg_t m4 = leg;
        p4.phi4 += eps; m4.phi4 -= eps;
        VMC_calc_1_right(&p4, &ins, S4_DT);
        VMC_calc_1_right(&m4, &ins, S4_DT);
        const float dL0_dq4 = (p4.L0 - m4.L0) / (2.0f * eps);
        const float dphi0_dq4 = (p4.phi0 - m4.phi0) / (2.0f * eps);

        const float dL0_fd = dL0_dq1 * qd1 + dL0_dq4 * qd4;
        const float dphi0_fd = dphi0_dq1 * qd1 + dphi0_dq4 * qd4;
        const float power_joint = leg.torque_set[0] * qd1 + leg.torque_set[1] * qd4;
        const float power_task  = leg.F0 * dL0_fd + leg.Tp * dphi0_fd;
        assert(near_f(power_joint, power_task, 5e-3f));
    }

    /* Ground detection threshold check */
    leg.F0 = 15.0f; leg.Tp = 0.0f; leg.theta = 0.0f; leg.L0 = 0.14f;
    assert(ground_detectionR(&leg) == 0u);
    leg.F0 = -5.0f;
    assert(ground_detectionL(&leg) == 1u);

    /* PID position and delta saturation checks */
    PidTypeDef pid;
    const float gains[3] = { 10.0f, 2.0f, 1.0f };
    PID_init(&pid, (uint8_t)PID_POSITION, gains, 15.0f, 4.0f);
    const float u = PID_Calc(&pid, 0.0f, 5.0f);
    assert(near_f(u, 15.0f, 1e-5f));
    assert(near_f(pid.Iout, 4.0f, 1e-5f));
    PID_clear(&pid);
    assert(near_f(pid.out, 0.0f, 1e-6f));
}

static void test_s5_s7_s8_s9_modules(void)
{
    /* S5 bilinear trim & feedback corner check */
    s5_predictor_t p;
    memset(&p, 0, sizeof(p));
    p.mass_kg = 5.10f;
    p.length_ref_m = 0.14f;
    float trim[3];
    s5_trim(&p, trim);
    assert(near_f(trim[0], s5_standing_trim[1][1][0], 1e-6f));
    assert(near_f(trim[1], s5_standing_trim[1][1][1], 1e-6f));
    assert(near_f(trim[2], s5_standing_trim[1][1][2], 1e-6f));

    /* S7_V1 vs S7_BASELINE paths + 3-tick command history commit/reset */
    s7_loss_t loss;
    memset(&loss, 0, sizeof(loss));
    loss.damping = 0.0255f;
    s7_reference_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.speed_m_s = 0.20f;
    ref.accel_m_s2 = 0.10f;

    float obs_v1[S5_OBS] = { 0.01f, 0.02f, 0.01f, 0.0f, 0.02f, 0.05f };
    float obs_base[S5_OBS];
    memcpy(obs_base, obs_v1, sizeof(obs_v1));
    float eff_v1[S5_EFFORTS] = { 0.0f, 0.0f };
    float eff_base[S5_EFFORTS] = { 0.0f, 0.0f };

    s7_efforts(&p, &loss, S7_V1, 0.14f, obs_v1, &ref, true, eff_v1);
    s7_efforts(&p, &loss, S7_BASELINE, 0.14f, obs_base, &ref, true, eff_base);
    assert(isfinite(eff_v1[0]) && isfinite(eff_v1[1]));
    assert(fabsf(eff_v1[0] - eff_base[0]) > 1e-3f); /* Feedforward + s7_gain_mid active */

    /* 3-tick history shift and clear on !standing */
    const float e1[2] = { 0.5f, -0.4f };
    const float e2[2] = { 0.2f, -0.1f };
    s5_commit(&p, e1, true);
    s5_commit(&p, e2, true);
    assert(near_f(p.command[0][0], 0.2f, 1e-6f));
    assert(near_f(p.command[1][0], 0.5f, 1e-6f));
    s5_commit(&p, e2, false);
    assert(near_f(p.command[0][0], 0.0f, 1e-6f) && near_f(p.command[1][0], 0.0f, 1e-6f));

    /* S9 continuous brake position: exact zero-jump continuity at latch */
    s9_brake_t brk;
    memset(&brk, 0, sizeof(brk));
    const float p_pre = s9_brake_position(&brk, false, 0.18f, 0.10f, S4_DT, S8_STOP_POSITION_LIMIT_M);
    assert(near_f(p_pre, 0.18f, 1e-6f));
    const float p_latch = s9_brake_position(&brk, true, 0.18f, 0.10f, S4_DT, S8_STOP_POSITION_LIMIT_M);
    assert(near_f(p_latch, p_pre, 1e-6f)); /* Zero jump! */
    assert(near_f(brk.blend_m, 0.18f, 1e-6f));
    const float p_next = s9_brake_position(&brk, true, 0.18f, 0.0f, S4_DT, S8_STOP_POSITION_LIMIT_M);
    assert(p_next < p_latch && p_next > 0.0f); /* Smooth 0.75 s decay */

    /* S9 quiet trim: requires 125 quiet ticks, then adapts toward -S9_TRIM_RAD_PER_M * pos */
    s9_trim_t trm;
    memset(&trm, 0, sizeof(trm));
    for (uint32_t i = 0u; i < S9_QUIET_TICKS; ++i) {
        s9_update_trim(&trm, true, 0.10f, S4_DT);
        assert(near_f(trm.pitch_rad, 0.0f, 1e-7f));
    }
    s9_update_trim(&trm, true, 0.10f, S4_DT);
    assert(trm.pitch_rad < 0.0f && fabsf(trm.pitch_rad) <= S9_TRIM_LIMIT_RAD);
}

static void test_s10_estimator(void)
{
    S10Estimator est;
    const double lever[3] = { 0.00265, 0.0, 0.03447 };
    s10_init(&est, 0.0603, lever);
    est.robust = 1;

    const S10FiveBarGeom geom = s10_default_fivebar_geom();
    const double q[6] = { 0.85, 0.85, 0.0, 0.85, 0.85, 0.0 };
    const double dq[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    S10Encoder enc;
    assert(s10_decode_fivebar(&geom, q, dq, &enc) == 1u);
    assert(enc.position[0][2] < -0.05 && enc.position[1][2] < -0.05);

    /* 101-sample startup with 0.02 rad/s pitch gyro bias and gravity along +z */
    const double gyro[3] = { 0.0, 0.02, 0.0 };
    const double force[3] = { 0.0, 0.0, 9.80665 };
    for (int32_t t = 0; t < S10_STARTUP; ++t) {
        s10_imu(&est, t, gyro, force);
        if ((t % 2) == 0) {
            s10_encoder(&est, t, t, &enc);
        }
    }
    assert(s10_ready(&est) == 1);
    assert(near_d(est.bias[1], 0.02, 1e-6));

    S10Frame fr;
    s10_frame(&est, &fr);
    assert(near_d(fr.pitch, 0.0, 1e-5));
    assert(near_d(fr.speed, 0.0, 1e-5));
    assert(fr.air == 0);

    /* Free-fall detection (< 0.30 g for >= 4 ms) */
    const double zero_f[3] = { 0.0, 0.0, 0.0 };
    for (int32_t t = S10_STARTUP; t < S10_STARTUP + 6; ++t) {
        s10_imu(&est, t, gyro, zero_f);
    }
    s10_frame(&est, &fr);
    assert(fr.air == 1);
}

static void test_balance_ctrl_pipeline_and_policies(void)
{
    BalanceCtrl ctrl;
    BalanceCtrlConfig cfg = balance_ctrl_default_config();
    cfg.safety_cfg = safety_config_with_radio_policy(RADIO_LOSS_BRAKE_BALANCE);
    balance_ctrl_init(&ctrl, &cfg);

    LogFlightRecord ring_buf[32];
    LogFlightRing ring;
    assert(log_ring_init(&ring, ring_buf, 32u, 4u) == LOG_OK);
    balance_ctrl_attach_flight_ring(&ctrl, &ring);

    const double q[6] = { 0.85, 0.85, 0.0, 0.85, 0.85, 0.0 };
    const double dq[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    const double gyro[3] = { 0.0, 0.0, 0.0 };
    const double accel[3] = { 0.0, 0.0, 9.80665 };

    /* 1. Converge S10 estimator via 1 kHz IMU pre-ticks (S10_STARTUP = 101 samples) */
    for (int32_t t = -110; t <= -10; ++t) {
        balance_ctrl_feed_imu_1khz(&ctrl, t, gyro, accel);
    }

    /* 2. Walk DISARMED -> PRECHARGE -> ARMED -> BALANCING */
    BalanceTickInput in;
    BalanceTickOutput out;
    memset(&in, 0, sizeof(in));
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
    in.tick_us            = 10000u;
    in.enc_tick_ms        = -10;

    inject_all_6_can_fb(&ctrl.can_sched, in.tick_us, q, dq, 0x1u);
    balance_ctrl_tick(&ctrl, &in, &out);
    assert(out.safety_out.state == SAFETY_STATE_PRECHARGE);
    assert(out.s10_fed_fresh_enc == 1u);
    assert(out.balance_heartbeat == 1u);
    assert(out.flight_rec_pushed == 1u);

    /* Close K1/K2 and complete 3 fresh 6-axis groups to arm S11 + enter ARMED */
    in.local_reset_req = 0u;
    in.local_arm_req   = 0u;
    in.k1_closed       = 1u;
    in.k2_closed       = 1u;
    in.precharge_dv_ok = 1u;
    in.v_bus_v         = 24.0f;
    in.radio_arm_sw    = 1u;
    for (uint32_t step = 1u; step <= 3u; ++step) {
        in.tick_us = 10000u + step * 2000u;
        in.enc_tick_ms = -10 + (int32_t)(step * 2u);
        balance_ctrl_feed_imu_1khz(&ctrl, in.enc_tick_ms - 1, gyro, accel);
        balance_ctrl_feed_imu_1khz(&ctrl, in.enc_tick_ms, gyro, accel);
        inject_all_6_can_fb(&ctrl.can_sched, in.tick_us, q, dq, 0x1u);
        balance_ctrl_tick(&ctrl, &in, &out);
    }
    assert(out.safety_out.state == SAFETY_STATE_ARMED);
    assert(out.s11_frame.valid == 1u);

    /* Stand handoff -> BALANCING */
    in.tick_us = 18000u;
    in.enc_tick_ms = -2;
    in.stand_handoff_req = 1u;
    balance_ctrl_feed_imu_1khz(&ctrl, -3, gyro, accel);
    balance_ctrl_feed_imu_1khz(&ctrl, -2, gyro, accel);
    inject_all_6_can_fb(&ctrl.can_sched, in.tick_us, q, dq, 0x1u);
    balance_ctrl_tick(&ctrl, &in, &out);
    assert(out.safety_out.state == SAFETY_STATE_BALANCING);
    assert(out.safety_out.torque_enable == 1u);
    assert(can_sched_tx_pending(&ctrl.can_sched, 0u) == 2u);
    assert(can_sched_tx_pending(&ctrl.can_sched, 1u) == 2u);
    assert(can_sched_tx_pending(&ctrl.can_sched, 2u) == 2u);

    /* 3. Verify standing position-error bound uses S8_STOP_POSITION_LIMIT_M (0.60 m)
     *    with anti-windup (no 0.30 m saturation deadlock, no unbounded x_est_m drift). */
    in.stand_handoff_req = 0u;
    ctrl.h5.x_est_m = -0.35f;
    ctrl.h5.x_set_m = 0.0f;
    in.tick_us = 20000u;
    in.enc_tick_ms = 0;
    balance_ctrl_feed_imu_1khz(&ctrl, -1, gyro, accel);
    balance_ctrl_feed_imu_1khz(&ctrl, 0, gyro, accel);
    inject_all_6_can_fb(&ctrl.can_sched, in.tick_us, q, dq, 0x1u);
    balance_ctrl_tick(&ctrl, &in, &out);
    assert(out.safety_out.state == SAFETY_STATE_BALANCING);
    assert(near_f(ctrl.h5.x_filter_eff - ctrl.h5.x_set_eff, -0.35f, 1e-3f));

    ctrl.h5.x_est_m = -0.85f;
    in.tick_us = 22000u;
    in.enc_tick_ms = 2;
    balance_ctrl_feed_imu_1khz(&ctrl, 1, gyro, accel);
    balance_ctrl_feed_imu_1khz(&ctrl, 2, gyro, accel);
    inject_all_6_can_fb(&ctrl.can_sched, in.tick_us, q, dq, 0x1u);
    balance_ctrl_tick(&ctrl, &in, &out);
    assert(near_f(ctrl.h5.x_est_m - ctrl.h5.x_set_m, -S8_STOP_POSITION_LIMIT_M, 1e-4f));
    assert(near_f(ctrl.h5.x_filter_eff - ctrl.h5.x_set_eff, -S8_STOP_POSITION_LIMIT_M, 1e-4f));
    ctrl.h5.x_est_m = 0.0f;

    /* 4. Verify Section 3.2 Step 3 rule: on a missed CAN tick (S11_PREDICT),
     *    S10 MUST NOT be fed extrapolated encoders, while balance control stays active! */
    in.tick_us = 24000u;
    in.enc_tick_ms = 4;
    balance_ctrl_feed_imu_1khz(&ctrl, 3, gyro, accel);
    balance_ctrl_feed_imu_1khz(&ctrl, 4, gyro, accel);
    /* No CAN RX injected on this tick -> S11_PREDICT */
    balance_ctrl_tick(&ctrl, &in, &out);
    assert(out.s11_frame.mode == S11_PREDICT);
    assert(out.s10_fed_fresh_enc == 0u);
    assert(out.safety_out.state == SAFETY_STATE_BALANCING);
    assert(out.safety_out.torque_enable == 1u);

    /* 5. Verify RADIO_LOSS_BRAKE_BALANCE enters CONTROLLED_STOP and keeps torque ON */
    in.tick_us = 26000u;
    in.enc_tick_ms = 6;
    in.radio_age_ms = 200u;
    in.radio_arm_sw = 0u;
    balance_ctrl_feed_imu_1khz(&ctrl, 5, gyro, accel);
    balance_ctrl_feed_imu_1khz(&ctrl, 6, gyro, accel);
    inject_all_6_can_fb(&ctrl.can_sched, in.tick_us, q, dq, 0x1u);
    balance_ctrl_tick(&ctrl, &in, &out);
    assert(out.safety_out.state == SAFETY_STATE_CONTROLLED_STOP);
    assert(out.safety_out.reason == SAFETY_REASON_RADIO_LOSS_BRAKE);
    assert(out.safety_out.torque_enable == 1u);

    /* 6. Verify regen taper scales regenerative torque at V_bus = 26.3 V (50%) */
    in.v_bus_v = 26.3f;
    in.tick_us = 28000u;
    in.enc_tick_ms = 8;
    balance_ctrl_feed_imu_1khz(&ctrl, 7, gyro, accel);
    balance_ctrl_feed_imu_1khz(&ctrl, 8, gyro, accel);
    inject_all_6_can_fb(&ctrl.can_sched, in.tick_us, q, dq, 0x1u);
    balance_ctrl_tick(&ctrl, &in, &out);
    assert(near_f(out.safety_out.regen_scale, 0.5f, 1e-4f));

    /* 7. Verify ESTOP immediately latches, dispatches 0xFF..FD, zeroes torque, triggers flight ring */
    in.estop_open = 1u;
    in.tick_us = 30000u;
    in.enc_tick_ms = 10;
    balance_ctrl_tick(&ctrl, &in, &out);
    assert(out.safety_out.state == SAFETY_STATE_ESTOP);
    assert(out.safety_out.torque_enable == 0u);
    for (uint32_t i = 0u; i < 6u; ++i) {
        assert(near_f(out.torque_cmd[i], 0.0f, 1e-6f));
    }
    CanFrame dis_frame;
    assert(can_sched_pop_tx(&ctrl.can_sched, 0u, &dis_frame) == CAN_SCHED_OK);
    assert(dis_frame.data[7] == DAMIAO_CMD_DISABLE_BYTE);
    assert(ring.fault_triggered == 1u);
}

static void drain_all_can_tx(CanSched *sched)
{
    for (uint8_t bus = 0u; bus < CAN_SCHED_NUM_BUSES; ++bus) {
        CanFrame frame;
        while (can_sched_pop_tx(sched, bus, &frame) == CAN_SCHED_OK) {
            /* discard */
        }
    }
}

static void test_driving_clamp_handoff_and_radio_loss_brake(void)
{
    const double q[6] = { 0.85, 0.85, 0.0, 0.85, 0.85, 0.0 };
    const double dq[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    const double gyro[3] = { 0.0, 0.0, 0.0 };
    const double accel[3] = { 0.0, 0.0, 9.80665 };

    /* Part A: Driving uses unified 0.60 m clamp + anti-windup, zero step across
     * v_target changes, and zero-step bidirectional handoff across
     * BALANCING -> CONTROLLED_STOP (HOST_LOSS) -> REST -> BALANCING. */
    BalanceCtrl ctrl;
    BalanceCtrlConfig cfg = balance_ctrl_default_config();
    cfg.safety_cfg = safety_config_with_radio_policy(RADIO_LOSS_BRAKE_BALANCE);
    balance_ctrl_init(&ctrl, &cfg);

    for (int32_t t = -110; t <= -10; ++t) {
        balance_ctrl_feed_imu_1khz(&ctrl, t, gyro, accel);
    }

    BalanceTickInput in;
    BalanceTickOutput out;
    memset(&in, 0, sizeof(in));
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
    in.tick_us            = 10000u;
    in.enc_tick_ms        = -10;

    inject_all_6_can_fb(&ctrl.can_sched, in.tick_us, q, dq, 0x1u);
    balance_ctrl_tick(&ctrl, &in, &out);
    drain_all_can_tx(&ctrl.can_sched);
    assert(out.safety_out.state == SAFETY_STATE_PRECHARGE);

    in.local_reset_req = 0u;
    in.local_arm_req   = 0u;
    in.k1_closed       = 1u;
    in.k2_closed       = 1u;
    in.precharge_dv_ok = 1u;
    in.v_bus_v         = 24.0f;
    in.radio_arm_sw    = 1u;
    for (uint32_t step = 1u; step <= 3u; ++step) {
        in.tick_us = 10000u + step * 2000u;
        in.enc_tick_ms = -10 + (int32_t)(step * 2u);
        balance_ctrl_feed_imu_1khz(&ctrl, in.enc_tick_ms - 1, gyro, accel);
        balance_ctrl_feed_imu_1khz(&ctrl, in.enc_tick_ms, gyro, accel);
        inject_all_6_can_fb(&ctrl.can_sched, in.tick_us, q, dq, 0x1u);
        balance_ctrl_tick(&ctrl, &in, &out);
        drain_all_can_tx(&ctrl.can_sched);
    }
    assert(out.safety_out.state == SAFETY_STATE_ARMED);

    in.tick_us = 18000u;
    in.enc_tick_ms = -2;
    in.stand_handoff_req = 1u;
    balance_ctrl_feed_imu_1khz(&ctrl, -3, gyro, accel);
    balance_ctrl_feed_imu_1khz(&ctrl, -2, gyro, accel);
    inject_all_6_can_fb(&ctrl.can_sched, in.tick_us, q, dq, 0x1u);
    balance_ctrl_tick(&ctrl, &in, &out);
    drain_all_can_tx(&ctrl.can_sched);
    assert(out.safety_out.state == SAFETY_STATE_BALANCING);
    in.stand_handoff_req = 0u;

    uint32_t tick_us = 20000u;
    int32_t enc_ms = 0;

    /* 1. Driving at 0.20 m/s allows -0.35 m CoM-offset equilibrium (no 0.30 m saturation)
     *    and stays continuous across v_target transitions 0.20 -> 0.0 -> 0.20 m/s. */
    ctrl.h5.x_est_m = -0.35f;
    ctrl.h5.x_set_m = 0.0f;
    in.v_target_m_s     = 0.20f;
    in.yaw_target_rad_s = 0.30f;
    in.accel_ref_m_s2   = 0.15f;
    in.jerk_ref_m_s3    = 0.05f;
    in.tick_us          = tick_us;
    in.enc_tick_ms      = enc_ms;
    balance_ctrl_feed_imu_1khz(&ctrl, enc_ms - 1, gyro, accel);
    balance_ctrl_feed_imu_1khz(&ctrl, enc_ms, gyro, accel);
    inject_all_6_can_fb(&ctrl.can_sched, tick_us, q, dq, 0x1u);
    balance_ctrl_tick(&ctrl, &in, &out);
    drain_all_can_tx(&ctrl.can_sched);
    assert(out.safety_out.state == SAFETY_STATE_BALANCING);
    assert(near_f(out.safety_out.v_ref_m_s, 0.20f, 1e-6f));
    assert(near_f(out.safety_out.yaw_ref_rad_s, 0.30f, 1e-6f));
    const float err_drive_1 = ctrl.h5.x_filter_eff - ctrl.h5.x_set_eff;
    assert(err_drive_1 < -0.30f);
    assert(near_f(err_drive_1, -0.35f - 0.20f * BALANCE_CTRL_DT_S, 1e-3f));

    tick_us += 2000u;
    enc_ms += 2;
    in.v_target_m_s     = 0.0f;
    in.yaw_target_rad_s = 0.0f;
    in.accel_ref_m_s2   = 0.0f;
    in.jerk_ref_m_s3    = 0.0f;
    in.tick_us          = tick_us;
    in.enc_tick_ms      = enc_ms;
    balance_ctrl_feed_imu_1khz(&ctrl, enc_ms - 1, gyro, accel);
    balance_ctrl_feed_imu_1khz(&ctrl, enc_ms, gyro, accel);
    inject_all_6_can_fb(&ctrl.can_sched, tick_us, q, dq, 0x1u);
    balance_ctrl_tick(&ctrl, &in, &out);
    drain_all_can_tx(&ctrl.can_sched);
    const float err_stand = ctrl.h5.x_filter_eff - ctrl.h5.x_set_eff;
    assert(near_f(err_stand, err_drive_1, 1e-3f));

    tick_us += 2000u;
    enc_ms += 2;
    in.v_target_m_s     = 0.20f;
    in.yaw_target_rad_s = 0.30f;
    in.tick_us          = tick_us;
    in.enc_tick_ms      = enc_ms;
    balance_ctrl_feed_imu_1khz(&ctrl, enc_ms - 1, gyro, accel);
    balance_ctrl_feed_imu_1khz(&ctrl, enc_ms, gyro, accel);
    inject_all_6_can_fb(&ctrl.can_sched, tick_us, q, dq, 0x1u);
    balance_ctrl_tick(&ctrl, &in, &out);
    drain_all_can_tx(&ctrl.can_sched);
    const float err_drive_2 = ctrl.h5.x_filter_eff - ctrl.h5.x_set_eff;
    assert(near_f(err_drive_2, err_stand - 0.20f * BALANCE_CTRL_DT_S, 1e-3f));

    /* 2. Anti-windup at +/-0.60 m while driving */
    tick_us += 2000u;
    enc_ms += 2;
    ctrl.h5.x_est_m = ctrl.h5.x_set_m - 0.85f;
    in.tick_us      = tick_us;
    in.enc_tick_ms  = enc_ms;
    balance_ctrl_feed_imu_1khz(&ctrl, enc_ms - 1, gyro, accel);
    balance_ctrl_feed_imu_1khz(&ctrl, enc_ms, gyro, accel);
    inject_all_6_can_fb(&ctrl.can_sched, tick_us, q, dq, 0x1u);
    balance_ctrl_tick(&ctrl, &in, &out);
    drain_all_can_tx(&ctrl.can_sched);
    assert(near_f(ctrl.h5.x_est_m - ctrl.h5.x_set_m, -S8_STOP_POSITION_LIMIT_M, 1e-4f));
    assert(near_f(ctrl.h5.x_filter_eff - ctrl.h5.x_set_eff, -S8_STOP_POSITION_LIMIT_M, 1e-4f));

    /* 3. Host loss while driving at 0.20 m/s, 0.30 rad/s -> zero-step latch into
     *    CONTROLLED_STOP -> 600 ms brake ramp -> 250 ms dwell -> REST -> zero-step re-grant. */
    tick_us += 2000u;
    enc_ms += 2;
    ctrl.h5.x_est_m = ctrl.h5.x_set_m - 0.35f;
    in.tick_us      = tick_us;
    in.enc_tick_ms  = enc_ms;
    balance_ctrl_feed_imu_1khz(&ctrl, enc_ms - 1, gyro, accel);
    balance_ctrl_feed_imu_1khz(&ctrl, enc_ms, gyro, accel);
    inject_all_6_can_fb(&ctrl.can_sched, tick_us, q, dq, 0x1u);
    balance_ctrl_tick(&ctrl, &in, &out);
    drain_all_can_tx(&ctrl.can_sched);
    const float pre_latch_err = ctrl.h5.x_filter_eff - ctrl.h5.x_set_eff;

    tick_us += 2000u;
    enc_ms += 2;
    in.host_valid  = 0u;
    in.host_age_ms = 200u;
    in.tick_us     = tick_us;
    in.enc_tick_ms = enc_ms;
    balance_ctrl_feed_imu_1khz(&ctrl, enc_ms - 1, gyro, accel);
    balance_ctrl_feed_imu_1khz(&ctrl, enc_ms, gyro, accel);
    inject_all_6_can_fb(&ctrl.can_sched, tick_us, q, dq, 0x1u);
    balance_ctrl_tick(&ctrl, &in, &out);
    drain_all_can_tx(&ctrl.can_sched);
    assert(out.safety_out.state == SAFETY_STATE_CONTROLLED_STOP);
    assert(out.safety_out.reason == SAFETY_REASON_HOST_LOSS);
    assert(out.safety_out.host_lost == 1u);
    assert(out.safety_out.torque_enable == 1u);
    assert(near_f(out.safety_out.v_ref_m_s, 0.20f, 1e-6f));
    assert(near_f(out.safety_out.yaw_ref_rad_s, 0.30f, 1e-6f));
    const float latch_err = ctrl.h5.x_filter_eff - ctrl.h5.x_set_eff;
    assert(near_f(latch_err, pre_latch_err, 1e-3f));

    for (uint32_t k = 1u; k <= 430u; ++k) {
        tick_us += 2000u;
        enc_ms += 2;
        in.tick_us     = tick_us;
        in.enc_tick_ms = enc_ms;
        balance_ctrl_feed_imu_1khz(&ctrl, enc_ms - 1, gyro, accel);
        balance_ctrl_feed_imu_1khz(&ctrl, enc_ms, gyro, accel);
        inject_all_6_can_fb(&ctrl.can_sched, tick_us, q, dq, 0x1u);
        balance_ctrl_tick(&ctrl, &in, &out);
        drain_all_can_tx(&ctrl.can_sched);
        assert(out.safety_out.torque_enable == 1u);
        if (k == 150u) {
            assert(out.safety_out.state == SAFETY_STATE_CONTROLLED_STOP);
            assert(near_f(out.safety_out.v_ref_m_s, 0.10f, 1e-3f));
            assert(near_f(out.safety_out.yaw_ref_rad_s, 0.15f, 1e-3f));
            assert(out.safety_out.inhibit_trim == 1u);
        } else if (k == 300u) {
            assert(near_f(out.safety_out.v_ref_m_s, 0.0f, 1e-6f));
            assert(near_f(out.safety_out.yaw_ref_rad_s, 0.0f, 1e-6f));
            assert(out.safety_out.inhibit_trim == 0u);
        }
    }
    assert(out.safety_out.state == SAFETY_STATE_REST);
    assert(out.safety_out.reason == SAFETY_REASON_REST_CONFIRMED);
    assert(out.safety_out.rest_confirmed == 1u);
    assert(out.safety_out.torque_enable == 1u);

    /* Explicit REST -> BALANCING re-grant must hand off with zero position-error step */
    const float pre_regrant_err = ctrl.h5.x_filter_eff - ctrl.h5.x_set_eff;
    assert(fabsf(pre_regrant_err) > 0.05f);
    tick_us += 2000u;
    enc_ms += 2;
    in.host_valid        = 1u;
    in.host_age_ms       = 10u;
    in.v_target_m_s      = 0.0f;
    in.yaw_target_rad_s  = 0.0f;
    in.local_regrant_req = 1u;
    in.tick_us           = tick_us;
    in.enc_tick_ms       = enc_ms;
    balance_ctrl_feed_imu_1khz(&ctrl, enc_ms - 1, gyro, accel);
    balance_ctrl_feed_imu_1khz(&ctrl, enc_ms, gyro, accel);
    inject_all_6_can_fb(&ctrl.can_sched, tick_us, q, dq, 0x1u);
    balance_ctrl_tick(&ctrl, &in, &out);
    drain_all_can_tx(&ctrl.can_sched);
    assert(out.safety_out.state == SAFETY_STATE_BALANCING);
    assert(out.safety_out.reason == SAFETY_REASON_LOCAL_REGRANT);
    assert(out.safety_out.host_lost == 0u);
    const float post_regrant_err = ctrl.h5.x_filter_eff - ctrl.h5.x_set_eff;
    assert(near_f(post_regrant_err, pre_regrant_err, 1e-3f));

    /* Part B: Extended FIL ABI (balance_ctrl_fil_step_500hz_ex) with driving at 0.20 m/s
     * followed by radio-loss policy c (RADIO_LOSS_BRAKE_BALANCE): brake -> REST -> keep balancing. */
    BalanceCtrl fil_ctrl;
    balance_ctrl_fil_init(&fil_ctrl, 5.056f, 0.14f, RADIO_LOSS_BRAKE_BALANCE);
    for (int32_t t = -120; t < 0; ++t) {
        balance_ctrl_fil_imu_1khz(&fil_ctrl, t, gyro, accel);
    }
    const int32_t armed_st = balance_ctrl_fil_arm_and_handoff(&fil_ctrl, 10000u, -10, q, dq);
    assert(armed_st == (int32_t)SAFETY_STATE_BALANCING);

    float torques[6];
    float diag[10];
    float drive_diag[4];
    uint32_t fil_us = 20000u;
    int32_t fil_ms = 0;

    for (uint32_t k = 0u; k < 5u; ++k) {
        balance_ctrl_fil_imu_1khz(&fil_ctrl, fil_ms, gyro, accel);
        balance_ctrl_fil_imu_1khz(&fil_ctrl, fil_ms + 1, gyro, accel);
        const int32_t st = balance_ctrl_fil_step_500hz_ex(
            &fil_ctrl, fil_us, fil_ms, q, dq,
            0.20f, 0.15f, 0.10f, 0.0f,
            0u, 0u, 0u, torques, diag, drive_diag);
        assert(st == (int32_t)SAFETY_STATE_BALANCING);
        assert(near_f(drive_diag[0], 0.20f, 1e-5f));
        assert(near_f(drive_diag[1], 0.15f, 1e-5f));
        assert(near_f(diag[9], 1.0f, 1e-5f));
        fil_us += 2000u;
        fil_ms += 2;
    }

    /* Drop radio while driving at 0.20 m/s under policy c */
    balance_ctrl_fil_imu_1khz(&fil_ctrl, fil_ms, gyro, accel);
    balance_ctrl_fil_imu_1khz(&fil_ctrl, fil_ms + 1, gyro, accel);
    int32_t st = balance_ctrl_fil_step_500hz_ex(
        &fil_ctrl, fil_us, fil_ms, q, dq,
        0.20f, 0.15f, 0.0f, 0.0f,
        0u, 1u, 0u, torques, diag, drive_diag);
    assert(st == (int32_t)SAFETY_STATE_CONTROLLED_STOP);
    assert(near_f(drive_diag[3], (float)SAFETY_REASON_RADIO_LOSS_BRAKE, 0.5f));
    assert(near_f(drive_diag[0], 0.20f, 1e-5f));
    assert(near_f(diag[9], 1.0f, 1e-5f));

    for (uint32_t k = 1u; k <= 430u; ++k) {
        fil_us += 2000u;
        fil_ms += 2;
        balance_ctrl_fil_imu_1khz(&fil_ctrl, fil_ms, gyro, accel);
        balance_ctrl_fil_imu_1khz(&fil_ctrl, fil_ms + 1, gyro, accel);
        st = balance_ctrl_fil_step_500hz_ex(
            &fil_ctrl, fil_us, fil_ms, q, dq,
            0.20f, 0.15f, 0.0f, 0.0f,
            0u, 1u, 0u, torques, diag, drive_diag);
        assert(near_f(diag[9], 1.0f, 1e-5f));
        if (k == 150u) {
            assert(st == (int32_t)SAFETY_STATE_CONTROLLED_STOP);
            assert(near_f(drive_diag[0], 0.10f, 2e-3f));
        }
    }
    assert(st == (int32_t)SAFETY_STATE_REST);
    assert(near_f(drive_diag[0], 0.0f, 1e-6f));
    assert(near_f(drive_diag[1], 0.0f, 1e-6f));
    assert(near_f(diag[9], 1.0f, 1e-5f));

    /* Legacy 11-arg wrapper remains ABI-compatible */
    fil_us += 2000u;
    fil_ms += 2;
    balance_ctrl_fil_imu_1khz(&fil_ctrl, fil_ms, gyro, accel);
    balance_ctrl_fil_imu_1khz(&fil_ctrl, fil_ms + 1, gyro, accel);
    st = balance_ctrl_fil_step_500hz(
        &fil_ctrl, fil_us, fil_ms, q, dq,
        0.0f, 0.0f, 1u, 0u, torques, diag);
    assert(st == (int32_t)SAFETY_STATE_REST);
}

static void test_s7_ff_history_and_loss_adapt(void)
{
    /* 1. Verify feedback-only S5 predictor commit delivers 100% of steady-state wheel_ff
     *    (zero self-attenuation from the 3-tick command history gains). */
    s5_predictor_t p;
    memset(&p, 0, sizeof(p));
    p.mass_kg = 5.056f;
    p.length_ref_m = 0.14f;

    s7_loss_t loss;
    memset(&loss, 0, sizeof(loss));
    loss.damping = 0.0255f;

    s7_reference_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.speed_m_s = 0.20f;
    ref.accel_m_s2 = 0.0f;
    ref.jerk_m_s3 = 0.0f;
    ref.wheel_rate_rad_s = 0.0f;
    ref.yaw_rad_s = 0.0f;

    const float radius = 0.0603f;
    const float rolling_torque = 0.5f * 0.005f * p.mass_kg * 9.81f * radius;
    const float rate_ref = ref.speed_m_s / radius;
    const float expected_wheel_ff = 2.0f * (0.0255f * rate_ref
                                            + rolling_torque * tanhf(rate_ref / 0.12f));

    float effort[S5_EFFORTS] = { 0.0f, 0.0f };
    float fb[S5_EFFORTS] = { 0.0f, 0.0f };
    for (uint32_t k = 0u; k < 20u; ++k) {
        float obs_zero[S5_OBS] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        s7_efforts_ex(&p, &loss, S7_V1, false, 0.14f, obs_zero, &ref, true, false, effort, fb);
        s5_commit(&p, fb, true);
    }
    assert(near_f(fb[0], 0.0f, 1e-6f));
    assert(near_f(fb[1], 0.0f, 1e-6f));
    assert(near_f(effort[0], expected_wheel_ff, 1e-5f));
    assert(near_f(effort[1], 0.0f, 1e-6f));

    /* 2. Verify acceleration-compensated online wheel-loss adaptation identifies
     *    draw_001 damping (b_true = 0.0410, crr = 0) during a smooth ramp and freezes
     *    when inhibit_loss is asserted (push / active brake). */
    memset(&p, 0, sizeof(p));
    p.mass_kg = 5.056f;
    p.length_ref_m = 0.14f;
    memset(&loss, 0, sizeof(loss));
    loss.damping = 0.0255f;

    const float b_true = 0.0410f;
    const float wheel_accel_gain = s7_accel[1][1][2]; /* ~0.3380 N*m/(m/s^2) */
    const float ramp_accel_rad_s2 = 2.5f;
    const float i_eq = 0.5f * wheel_accel_gain * radius;
    float omega = 2.0f;

    for (uint32_t k = 0u; k < 450u; ++k) {
        omega += ramp_accel_rad_s2 * S4_DT;
        ref.speed_m_s = 0.20f;
        ref.accel_m_s2 = ramp_accel_rad_s2 * radius;
        ref.wheel_rate_rad_s = omega;
        loss.previous_request = b_true * omega + i_eq * ramp_accel_rad_s2;

        float obs_ramp[S5_OBS] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        s7_efforts_ex(&p, &loss, S7_V1, true, 0.14f, obs_ramp, &ref, true, false, effort, fb);
    }
    /* Because crr = 0 in the synthetic plant while s7 uses crr_prior = 0.005,
     * loss.damping converges to b_true - rolling_torque / omega (~0.0385..0.0405)
     * so that total wheel_ff matches b_true * omega_ref closely. */
    assert(loss.damping > 0.036f && loss.damping < 0.042f);

    /* Assert inhibit_loss freezes damping even under an external push disturbance. */
    const float saved_damping = loss.damping;
    for (uint32_t k = 0u; k < 50u; ++k) {
        loss.previous_request = 0.0f;
        float obs_push[S5_OBS] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        s7_efforts_ex(&p, &loss, S7_V1, true, 0.14f, obs_push, &ref, true, true, effort, fb);
    }
    assert(near_f(loss.damping, saved_damping, 1e-7f));
}

int main(void)
{
    test_vmc_kinematics_and_pid();
    test_s5_s7_s8_s9_modules();
    test_s10_estimator();
    test_balance_ctrl_pipeline_and_policies();
    test_driving_clamp_handoff_and_radio_loss_brake();
    test_s7_ff_history_and_loss_adapt();
    printf("test_balance_ctrl: ALL PASSED\n");
    return 0;
}
