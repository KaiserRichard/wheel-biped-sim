/*
 * FW10 host tests: bring-up mode gating (torque impossible in BENCH), MOTOR_ID
 * wired-stop gate and limits, USB CDC EP0 logic, IMU sensor codec.
 * Simulation/design only.
 */

#include "bringup_mode.h"
#include "usb_cdc_ep0.h"
#include "imu_sensor_codec.h"
#include "control/balance_ctrl.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---------- helpers ---------- */

static uint32_t s_rng = 0x12345678u;
static uint32_t rnd(void)
{
    s_rng ^= s_rng << 13;
    s_rng ^= s_rng >> 17;
    s_rng ^= s_rng << 5;
    return s_rng;
}

static void inject_all_6_can_fb(CanSched *sched, uint32_t rx_us)
{
    const double q[6] = { 0.85, 0.85, 0.0, 0.85, 0.85, 0.0 };
    for (uint8_t axis = 0u; axis < 6u; ++axis) {
        DamiaoAxisConfig acfg;
        uint8_t bus = 0u;
        assert(damiao_get_axis_config(axis, &acfg) == DAMIAO_OK);
        assert(can_sched_bus_for_axis(axis, &bus) == CAN_SCHED_OK);
        DamiaoMitFeedback fb;
        memset(&fb, 0, sizeof(fb));
        fb.motor_id = (uint8_t)(axis + 1u);
        fb.status_nibble = 0x1u;
        fb.pos = (float)q[axis];
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

typedef struct {
    uint32_t frames;
    uint32_t non_disable;
    uint32_t passed_gate;
    uint32_t passed_non_disable;
} PopStats;

/* Drain every scheduler TX queue through a bring-up gate (the target's
 * send_hw() choke point applies exactly this check to every frame). */
static void drain_through_gate(CanSched *sched, const BringupCtx *gate, PopStats *st)
{
    for (uint8_t b = 0u; b < CAN_SCHED_NUM_BUSES; ++b) {
        CanFrame f;
        while (can_sched_pop_tx(sched, b, &f) == CAN_SCHED_OK) {
            const uint8_t dis = bringup_frame_is_disable(f.dlc, f.data);
            const uint8_t ok = bringup_tx_allowed(gate, b, f.std_id, f.dlc, f.data);
            st->frames++;
            st->non_disable += (dis == 0u) ? 1u : 0u;
            st->passed_gate += ok;
            st->passed_non_disable += (ok != 0u && dis == 0u) ? 1u : 0u;
        }
    }
}

/* The test_balance_ctrl.c arming recipe with every power-path input healthy
 * (as if F-02 were wired) and every operator request asserted: DISARMED ->
 * PRECHARGE -> ARMED -> BALANCING in RUN. `gate` is applied to the inputs. */
static void run_arming_recipe(const BringupCtx *gate, BalanceCtrl *ctrl,
                              BalanceTickOutput *last, uint32_t *torque_ticks,
                              uint32_t *max_state, PopStats *pops, const BringupCtx *tx_gate,
                              uint8_t charge_bus)
{
    BalanceCtrlConfig cfg = balance_ctrl_default_config();
    cfg.safety_cfg = safety_config_with_radio_policy(RADIO_LOSS_BRAKE_BALANCE);
    balance_ctrl_init(ctrl, &cfg);
    const double gyro[3] = { 0.0, 0.0, 0.0 };
    const double accel[3] = { 0.0, 0.0, 9.80665 };
    for (int32_t t = -110; t <= -10; ++t) {
        balance_ctrl_feed_imu_1khz(ctrl, t, gyro, accel);
    }
    BalanceTickInput base;
    memset(&base, 0, sizeof(base));
    base.use_can_sched_rx   = 1u;
    base.boot_self_test_ok  = 1u;
    base.dump_continuity_ok = 1u;
    base.v_bus_v            = 0.5f;
    base.v_batt_ocv_v       = 24.0f;
    base.radio_age_ms       = 4u;
    base.host_age_ms        = 10u;
    base.host_valid         = 1u;
    *torque_ticks = 0u;
    *max_state = 0u;
    for (uint32_t step = 0u; step < 400u; ++step) {
        BalanceTickInput in = base;
        in.tick_us = 10000u + step * 2000u;
        in.enc_tick_ms = -10 + (int32_t)(step * 2u);
        /* Step 0: reset + arm edge; afterwards contactors closed, arm switch,
         * stand handoff, motion command and jump all requested. */
        in.local_reset_req = (step == 0u) ? 1u : 0u;
        in.local_arm_req   = (step == 0u) ? 1u : 0u;
        in.hw_reset_ack    = (step == 0u) ? 1u : 0u;
        if (step >= 1u) {
            in.k1_closed = charge_bus;
            in.k2_closed = charge_bus;
            in.precharge_dv_ok = charge_bus;
            in.v_bus_v = charge_bus != 0u ? 24.0f : 0.5f;
            in.radio_arm_sw = 1u;
        }
        if (step >= 4u) {
            in.stand_handoff_req = 1u;
            in.v_target_m_s = 0.3f;
            in.jump_req = (step == 50u) ? 1u : 0u;
        }
        if (step > 0u) {
            balance_ctrl_feed_imu_1khz(ctrl, in.enc_tick_ms - 1, gyro, accel);
            balance_ctrl_feed_imu_1khz(ctrl, in.enc_tick_ms, gyro, accel);
        }
        inject_all_6_can_fb(&ctrl->can_sched, in.tick_us);
        bringup_gate_tick_input(gate, &in);
        balance_ctrl_tick(ctrl, &in, last);
        if (last->safety_out.torque_enable != 0u) {
            (*torque_ticks)++;
        }
        if ((uint32_t)last->safety_out.state > *max_state
            && last->safety_out.state != SAFETY_STATE_FAULT
            && last->safety_out.state != SAFETY_STATE_ESTOP) {
            *max_state = (uint32_t)last->safety_out.state;
        }
        drain_through_gate(&ctrl->can_sched, tx_gate, pops);
    }
}

/* ---------- bring-up gating ---------- */

static void test_bench_input_gate(void)
{
    BringupCtx ctx;
    bringup_init(&ctx, BRINGUP_MODE_BENCH);
    BalanceTickInput in;
    memset(&in, 0xFF, sizeof(in));
    in.v_target_m_s = 1.0f;
    in.yaw_target_rad_s = 1.0f;
    in.accel_ref_m_s2 = 1.0f;
    in.jerk_ref_m_s3 = 1.0f;
    bringup_gate_tick_input(&ctx, &in);
    assert(in.local_arm_req == 0u && in.local_reset_req == 0u && in.hw_reset_ack == 0u);
    assert(in.local_regrant_req == 0u && in.stand_handoff_req == 0u && in.jump_req == 0u);
    assert(in.radio_arm_sw == 0u && in.step_declaration == 0u);
    assert(in.v_target_m_s == 0.0f && in.yaw_target_rad_s == 0.0f);
    assert(in.accel_ref_m_s2 == 0.0f && in.jerk_ref_m_s3 == 0.0f);
    /* Unknown mode values fall back to BENCH (most restrictive). */
    bringup_init(&ctx, (BringupMode)7);
    assert(ctx.mode == BRINGUP_MODE_BENCH);
    /* RUN leaves the inputs untouched. */
    BringupCtx run;
    bringup_init(&run, BRINGUP_MODE_RUN);
    memset(&in, 0, sizeof(in));
    in.local_arm_req = 1u;
    in.radio_arm_sw = 1u;
    bringup_gate_tick_input(&run, &in);
    assert(in.local_arm_req == 1u && in.radio_arm_sw == 1u);
    /* Permit: BENCH never, whatever the FSM asks for. */
    BringupTickInput bin;
    memset(&bin, 0, sizeof(bin));
    assert(bringup_permit(&ctx, 1u, &bin) == 0u);
    assert(bringup_permit(&run, 1u, &bin) == 1u && bringup_permit(&run, 0u, &bin) == 0u);
}

/* End to end: the full balance pipeline with every arming input asserted and
 * the power path reported healthy. In BENCH the FSM never leaves DISARMED, no
 * torque/enable is ever requested, and no frame other than 0xFF..FD passes the
 * TX gate. */
static void test_bench_end_to_end(void)
{
    static BalanceCtrl ctrl;
    BringupCtx gate;
    bringup_init(&gate, BRINGUP_MODE_BENCH);
    BalanceTickOutput out;
    uint32_t torque_ticks = 0u;
    uint32_t max_state = 0u;
    PopStats pops;
    memset(&pops, 0, sizeof(pops));
    run_arming_recipe(&gate, &ctrl, &out, &torque_ticks, &max_state, &pops, &gate, 0u);
    assert(torque_ticks == 0u);
    assert(max_state == (uint32_t)SAFETY_STATE_DISARMED);
    printf("bench_end_to_end: final_state=%u reason=%u\n",
           (unsigned)out.safety_out.state, (unsigned)out.safety_out.reason);
    assert(out.safety_out.state == SAFETY_STATE_DISARMED);
    assert(out.safety_out.torque_enable == 0u && out.safety_out.can_enable_cmd == 0u);
    assert(bringup_check_tick_output(&gate, &out) == 0u && gate.bench_violations == 0u);
    assert(pops.passed_non_disable == 0u);
    printf("bench_end_to_end: torque_ticks=%u max_state=%u frames=%u non_disable=%u\n",
           (unsigned)torque_ticks, (unsigned)max_state, (unsigned)pops.frames,
           (unsigned)pops.non_disable);
    /* Original inconsistent fixture: charge the link without PRECHARGE.
     * The production supervisor must latch exactly this fault. */
    run_arming_recipe(&gate, &ctrl, &out, &torque_ticks, &max_state, &pops, &gate, 1u);
    assert(out.safety_out.state == SAFETY_STATE_FAULT);
    assert(out.safety_out.reason == SAFETY_REASON_LINK_NOT_DISCHARGED);
    assert(torque_ticks == 0u);
    printf("charged_bus_negative_control: final_state=%u reason=%u\n",
           (unsigned)out.safety_out.state, (unsigned)out.safety_out.reason);
}

/* Positive control: the same recipe in RUN reaches BALANCING with torque and
 * MIT frames (so the bench test above can detect torque), and every one of
 * those torque frames would be refused by a BENCH gate. */
static void test_run_positive_control(void)
{
    static BalanceCtrl ctrl;
    BringupCtx run;
    BringupCtx bench;
    bringup_init(&run, BRINGUP_MODE_RUN);
    bringup_init(&bench, BRINGUP_MODE_BENCH);
    BalanceTickOutput out;
    uint32_t torque_ticks = 0u;
    uint32_t max_state = 0u;
    PopStats run_pops;
    memset(&run_pops, 0, sizeof(run_pops));
    run_arming_recipe(&run, &ctrl, &out, &torque_ticks, &max_state, &run_pops, &run, 1u);
    assert(max_state >= (uint32_t)SAFETY_STATE_BALANCING);
    assert(torque_ticks > 100u);
    assert(run_pops.non_disable > 100u && run_pops.passed_non_disable == run_pops.non_disable);
    printf("run_positive_control: torque_ticks=%u max_state=%u frames=%u non_disable=%u\n",
           (unsigned)torque_ticks, (unsigned)max_state, (unsigned)run_pops.frames,
           (unsigned)run_pops.non_disable);
    /* Replay: same RUN trajectory, but the TX gate is BENCH. */
    PopStats bench_pops;
    memset(&bench_pops, 0, sizeof(bench_pops));
    run_arming_recipe(&run, &ctrl, &out, &torque_ticks, &max_state, &bench_pops, &bench, 1u);
    assert(bench_pops.non_disable > 100u && bench_pops.passed_non_disable == 0u);
    /* And the BENCH post-tick check flags a torque-enabled output. */
    assert(out.safety_out.torque_enable == 1u);
    assert(bringup_check_tick_output(&bench, &out) == 1u && bench.bench_violations == 1u);
    for (uint32_t i = 0u; i < 6u; ++i) {
        assert(out.torque_cmd[i] == 0.0f);
    }
}

/* 1,000,000 random 8-byte frames on random IDs/buses: BENCH passes exactly the
 * disable frames addressed to a rev4 motor command ID on its own bus. */
static void test_bench_tx_fuzz(void)
{
    BringupCtx ctx;
    bringup_init(&ctx, BRINGUP_MODE_BENCH);
    uint32_t passed = 0u;
    for (uint32_t i = 0u; i < 1000000u; ++i) {
        uint8_t d[8];
        for (uint32_t k = 0u; k < 8u; ++k) {
            d[k] = (uint8_t)rnd();
        }
        if ((i % 4u) == 0u) {
            memset(d, 0xFF, 7u); /* bias towards the special-command space */
            d[7] = (uint8_t)(0xF8u + (rnd() % 8u));
        }
        const uint16_t id = (uint16_t)((i % 2u) == 0u ? (1u + rnd() % 6u) : (rnd() & 0x7FFu));
        const uint8_t bus = (uint8_t)(rnd() % 3u);
        const uint8_t dlc = (uint8_t)((i % 16u) == 0u ? (rnd() % 9u) : 8u);
        const uint8_t ok = bringup_tx_allowed(&ctx, bus, id, dlc, d);
        if (ok != 0u) {
            passed++;
            assert(bringup_frame_is_disable(dlc, d) == 1u);
            assert(bringup_axis_for_cmd_id(bus, id) != BRINGUP_NO_AXIS);
        }
        /* Even an authorisation value poked into the context changes nothing. */
        ctx.tx_auth_axis = (uint8_t)(rnd() % 7u);
        if (bringup_frame_is_disable(dlc, d) == 0u) {
            assert(bringup_tx_allowed(&ctx, bus, id, dlc, d) == 0u);
        }
    }
    assert(passed > 1000u);
    printf("bench_tx_fuzz: 1000000 frames, %u passed (all disable)\n", (unsigned)passed);
}

/* 200,000 random bring-up ticks in BENCH (random requests, random wired-stop
 * state, FSM state): no output frame is ever anything but 0xFF..FD and the
 * permit is never granted. */
static void test_bench_step_fuzz(void)
{
    BringupCtx ctx;
    bringup_init(&ctx, BRINGUP_MODE_BENCH);
    uint32_t frames = 0u;
    for (uint32_t i = 0u; i < 200000u; ++i) {
        BringupTickInput in;
        memset(&in, 0, sizeof(in));
        in.now_us = i * 2000u;
        in.estop_pin_open = (uint8_t)(rnd() % 2u);
        in.estop_latched = (uint8_t)(rnd() % 2u);
        in.estop_pending = (uint8_t)(rnd() % 2u);
        in.fsm_disarmed = (uint8_t)(rnd() % 2u);
        in.req.valid = 1u;
        in.req.seq = rnd();
        in.req.rx_us = in.now_us;
        in.req.axis = (uint8_t)(rnd() % 8u);
        in.req.torque_nm = ((float)(rnd() % 2001u) - 1000.0f) / 1000.0f;
        in.req.duration_ms = (uint16_t)(rnd() % 400u);
        BringupTickOutput out;
        bringup_step(&ctx, &in, &out);
        assert(out.permit == 0u && bringup_permit(&ctx, 1u, &in) == 0u);
        assert(out.active_axis == BRINGUP_NO_AXIS && out.torque_nm == 0.0f);
        assert(ctx.tx_auth_axis == BRINGUP_NO_AXIS);
        for (uint8_t f = 0u; f < out.n_frames; ++f) {
            assert(bringup_frame_is_disable(out.frames[f].dlc, out.frames[f].data) == 1u);
            assert(bringup_tx_allowed(&ctx, out.frames[f].bus, out.frames[f].std_id,
                                      out.frames[f].dlc, out.frames[f].data) == 1u);
            frames++;
        }
    }
    assert(ctx.pulses_done == 0u && ctx.last_reject == BRINGUP_REJ_MODE);
    printf("bench_step_fuzz: 200000 ticks, %u disable-poll frames\n", (unsigned)frames);
}

static BringupTickInput healthy_input(uint32_t now_us, uint32_t seq, uint8_t axis,
                                      float torque, uint16_t dur)
{
    BringupTickInput in;
    memset(&in, 0, sizeof(in));
    in.now_us = now_us;
    in.fsm_disarmed = 1u;
    in.req.valid = 1u;
    in.req.seq = seq;
    in.req.rx_us = now_us;
    in.req.axis = axis;
    in.req.torque_nm = torque;
    in.req.duration_ms = dur;
    return in;
}

/* MOTOR_ID: each of the three wired-stop conditions and a non-DISARMED FSM
 * refuses a pulse; dropping any of them mid-pulse aborts it. */
static void test_motor_id_estop_gate(void)
{
    for (uint32_t cond = 0u; cond < 4u; ++cond) {
        BringupCtx ctx;
        bringup_init(&ctx, BRINGUP_MODE_MOTOR_ID);
        BringupTickInput in = healthy_input(1000u, 1u, 0u, 0.1f, 100u);
        if (cond == 0u) { in.estop_pin_open = 1u; }
        if (cond == 1u) { in.estop_latched = 1u; }
        if (cond == 2u) { in.estop_pending = 1u; }
        if (cond == 3u) { in.fsm_disarmed = 0u; }
        BringupTickOutput out;
        bringup_step(&ctx, &in, &out);
        assert(ctx.state == BRINGUP_ST_IDLE && ctx.tx_auth_axis == BRINGUP_NO_AXIS);
        assert(out.last_reject == (cond == 3u ? BRINGUP_REJ_FSM : BRINGUP_REJ_ESTOP));
        if (cond < 3u) {
            assert(out.permit == 0u);
        }
        for (uint8_t f = 0u; f < out.n_frames; ++f) {
            assert(bringup_frame_is_disable(out.frames[f].dlc, out.frames[f].data) == 1u);
        }

        /* Mid-pulse drop */
        bringup_init(&ctx, BRINGUP_MODE_MOTOR_ID);
        in = healthy_input(1000u, 1u, 4u, 0.15f, 200u);
        bringup_step(&ctx, &in, &out);           /* ENABLE */
        assert(ctx.state == BRINGUP_ST_ENABLE && out.permit == 1u);
        in.now_us = 3000u;
        bringup_step(&ctx, &in, &out);           /* PULSE */
        assert(ctx.state == BRINGUP_ST_PULSE && out.torque_nm == 0.15f);
        in.now_us = 5000u;
        if (cond == 0u) { in.estop_pin_open = 1u; }
        if (cond == 1u) { in.estop_latched = 1u; }
        if (cond == 2u) { in.estop_pending = 1u; }
        if (cond == 3u) { in.fsm_disarmed = 0u; }
        bringup_step(&ctx, &in, &out);
        assert(ctx.aborts == 1u && ctx.tx_auth_axis == BRINGUP_NO_AXIS);
        assert(out.active_axis == BRINGUP_NO_AXIS && out.torque_nm == 0.0f);
        uint32_t dis = 0u;
        for (uint8_t f = 0u; f < out.n_frames; ++f) {
            assert(bringup_frame_is_disable(out.frames[f].dlc, out.frames[f].data) == 1u);
            dis++;
        }
        assert(dis >= 6u);
    }
}

static void test_motor_id_limits(void)
{
    BringupCtx ctx;
    BringupTickOutput out;
    struct { uint8_t axis; float tq; uint16_t dur; uint32_t age_us; uint8_t why; } cases[] = {
        { 6u, 0.1f, 100u, 0u, BRINGUP_REJ_AXIS },
        { 0u, 0.21f, 100u, 0u, BRINGUP_REJ_TORQUE },
        { 0u, -0.21f, 100u, 0u, BRINGUP_REJ_TORQUE },
        { 0u, NAN, 100u, 0u, BRINGUP_REJ_TORQUE },
        { 0u, INFINITY, 100u, 0u, BRINGUP_REJ_TORQUE },
        { 0u, 0.1f, 0u, 0u, BRINGUP_REJ_DURATION },
        { 0u, 0.1f, 201u, 0u, BRINGUP_REJ_DURATION },
        { 0u, 0.1f, 100u, 100001u, BRINGUP_REJ_STALE },
        { 0u, 0.2f, 200u, 100000u, BRINGUP_REJ_NONE },
        { 5u, -0.2f, 1u, 0u, BRINGUP_REJ_NONE },
    };
    for (uint32_t i = 0u; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        bringup_init(&ctx, BRINGUP_MODE_MOTOR_ID);
        BringupTickInput in = healthy_input(500000u, 9u, cases[i].axis, cases[i].tq, cases[i].dur);
        in.req.rx_us = in.now_us - cases[i].age_us;
        bringup_step(&ctx, &in, &out);
        assert(out.last_reject == cases[i].why);
        assert((ctx.state == BRINGUP_ST_ENABLE) == (cases[i].why == BRINGUP_REJ_NONE));
    }
    /* RUN and BENCH images refuse every request with REJ_MODE. */
    bringup_init(&ctx, BRINGUP_MODE_BENCH);
    BringupTickInput in = healthy_input(1000u, 1u, 0u, 0.1f, 100u);
    bringup_step(&ctx, &in, &out);
    assert(out.last_reject == BRINGUP_REJ_MODE && ctx.state == BRINGUP_ST_IDLE);
    /* The same sequence number is consumed only once. */
    bringup_init(&ctx, BRINGUP_MODE_MOTOR_ID);
    in = healthy_input(1000u, 5u, 1u, 0.1f, 10u);
    bringup_step(&ctx, &in, &out);
    uint32_t t = 1000u;
    while (ctx.state != BRINGUP_ST_IDLE) {
        t += 2000u;
        in.now_us = t;
        bringup_step(&ctx, &in, &out);
        assert(t < 2000000u);
    }
    assert(ctx.pulses_done == 1u);
    for (uint32_t k = 0u; k < 100u; ++k) {
        t += 2000u;
        in.now_us = t;
        in.req.rx_us = t;
        bringup_step(&ctx, &in, &out);
        assert(ctx.state == BRINGUP_ST_IDLE);
    }
    assert(ctx.pulses_done == 1u);
}

/* Full pulse timeline: enable, MIT frames for exactly the duration, two
 * disables, 500 ms cool-down; the TX gate authorises only during the pulse. */
static void test_motor_id_pulse(void)
{
    BringupCtx ctx;
    bringup_init(&ctx, BRINGUP_MODE_MOTOR_ID);
    BringupTickInput in = healthy_input(1000u, 1u, 2u, 0.1f, 200u);
    BringupTickOutput out;
    uint32_t enables = 0u;
    uint32_t mits = 0u;
    uint32_t dis_axis = 0u;
    uint32_t t_first_mit = 0u;
    uint32_t t_last_mit = 0u;
    uint32_t t = 1000u;
    DamiaoAxisConfig cfg;
    assert(damiao_get_axis_config(2u, &cfg) == DAMIAO_OK);
    for (uint32_t k = 0u; k < 500u; ++k) {
        in.now_us = t;
        bringup_step(&ctx, &in, &out);
        for (uint8_t f = 0u; f < out.n_frames; ++f) {
            const BringupFrame *fr = &out.frames[f];
            const uint8_t axis = bringup_axis_for_cmd_id(fr->bus, fr->std_id);
            assert(axis != BRINGUP_NO_AXIS);
            /* Every frame the step emits passes the TX gate at emission time. */
            assert(bringup_tx_allowed(&ctx, fr->bus, fr->std_id, fr->dlc, fr->data) == 1u);
            if (bringup_frame_is_enable(fr->dlc, fr->data) != 0u) {
                assert(axis == 2u);
                enables++;
            } else if (bringup_frame_is_disable(fr->dlc, fr->data) != 0u) {
                dis_axis += (axis == 2u && ctx.state != BRINGUP_ST_IDLE) ? 1u : 0u;
            } else {
                DamiaoMitCmd c;
                assert(axis == 2u);
                assert(damiao_unpack_mit_cmd(&cfg.limits, fr->data, &c) == DAMIAO_OK);
                assert(c.kp == 0.0f && c.kd == 0.0f && fabsf(c.torque - 0.1f) < 0.01f);
                if (mits == 0u) {
                    t_first_mit = t;
                }
                t_last_mit = t;
                mits++;
            }
        }
        t += 2000u;
    }
    assert(enables == 1u && ctx.pulses_done == 1u && ctx.state == BRINGUP_ST_IDLE);
    /* 200 ms at 2 ms per tick: 100 MIT frames, last one < 200 ms after the first. */
    assert(mits == 100u);
    assert(t_last_mit - t_first_mit == 198000u);
    assert(dis_axis >= 2u);
    assert(ctx.tx_auth_axis == BRINGUP_NO_AXIS);
    /* After the pulse, a stored MIT frame for axis 2 is refused. */
    uint8_t d[8];
    DamiaoMitCmd c;
    memset(&c, 0, sizeof(c));
    c.torque = 0.1f;
    assert(damiao_pack_mit_cmd(&cfg.limits, &c, d) == DAMIAO_OK);
    assert(bringup_tx_allowed(&ctx, 1u, cfg.cmd_can_id, 8u, d) == 0u);
    /* Operator stop: a new request during a pulse ends it immediately. */
    bringup_init(&ctx, BRINGUP_MODE_MOTOR_ID);
    in = healthy_input(1000u, 1u, 2u, 0.1f, 200u);
    bringup_step(&ctx, &in, &out);
    in.now_us = 3000u;
    bringup_step(&ctx, &in, &out);
    assert(ctx.state == BRINGUP_ST_PULSE);
    in.now_us = 5000u;
    in.req.seq = 2u;
    in.req.rx_us = 5000u;
    bringup_step(&ctx, &in, &out);
    assert(ctx.state == BRINGUP_ST_DISABLE && ctx.tx_auth_axis == BRINGUP_NO_AXIS);
    assert(out.n_frames == 1u && bringup_frame_is_disable(8u, out.frames[0].data) == 1u);
}

/* MOTOR_ID TX gate: random frames while axis 3 is authorised. */
static void test_motor_id_tx_gate(void)
{
    BringupCtx ctx;
    bringup_init(&ctx, BRINGUP_MODE_MOTOR_ID);
    BringupTickInput in = healthy_input(1000u, 1u, 3u, 0.2f, 200u);
    BringupTickOutput out;
    bringup_step(&ctx, &in, &out);
    assert(ctx.tx_auth_axis == 3u);
    DamiaoAxisConfig cfg;
    assert(damiao_get_axis_config(3u, &cfg) == DAMIAO_OK);
    uint32_t mit_passed = 0u;
    for (uint32_t i = 0u; i < 300000u; ++i) {
        uint8_t d[8];
        for (uint32_t k = 0u; k < 8u; ++k) {
            d[k] = (uint8_t)rnd();
        }
        if ((i % 3u) == 0u) {
            d[3] = (uint8_t)(d[3] & 0xF0u); /* kp = 0 */
            d[4] = 0u;
            d[5] = 0u;                      /* kd = 0 */
            d[6] = (uint8_t)(d[6] & 0x0Fu);
        }
        const uint8_t bus = (uint8_t)(rnd() % 3u);
        const uint16_t id = (uint16_t)(1u + rnd() % 6u);
        const uint8_t ok = bringup_tx_allowed(&ctx, bus, id, 8u, d);
        if (ok == 0u || bringup_frame_is_disable(8u, d) != 0u
            || bringup_frame_is_enable(8u, d) != 0u) {
            continue;
        }
        DamiaoMitCmd c;
        assert(bringup_axis_for_cmd_id(bus, id) == 3u);
        assert(damiao_unpack_mit_cmd(&cfg.limits, d, &c) == DAMIAO_OK);
        assert(c.kp == 0.0f && c.kd == 0.0f);
        assert(fabsf(c.torque) <= BRINGUP_MOTOR_ID_TORQUE_CAP_NM + 0.01f);
        mit_passed++;
    }
    assert(mit_passed > 10u);
    printf("motor_id_tx_gate: 300000 frames, %u pure-torque frames to axis 3 passed\n",
           (unsigned)mit_passed);
}

/* ---------- USB CDC EP0 ---------- */

static void test_usb_descriptors(void)
{
    const uint8_t *dev = usb_cdc_device_descriptor();
    assert(dev[0] == 18u && dev[1] == 1u && dev[7] == 64u && dev[17] == 1u);
    assert(dev[4] == 0x02u);
    const uint8_t *cfg = usb_cdc_config_descriptor();
    const uint16_t total = (uint16_t)(cfg[2] | (cfg[3] << 8));
    assert(total == USB_CONFIG_DESC_LEN && cfg[4] == 2u);
    /* Walk the descriptor chain: lengths sum exactly, endpoints as declared. */
    uint32_t off = 0u;
    uint32_t n_if = 0u;
    uint32_t n_ep = 0u;
    uint8_t eps[3] = { 0u, 0u, 0u };
    while (off < total) {
        const uint8_t len = cfg[off];
        assert(len >= 2u && off + len <= total);
        if (cfg[off + 1u] == 0x04u) {
            n_if++;
        }
        if (cfg[off + 1u] == 0x05u) {
            assert(n_ep < 3u);
            eps[n_ep++] = cfg[off + 2u];
            assert(cfg[off + 4u] <= 64u);
        }
        off += len;
    }
    assert(off == total && n_if == 2u && n_ep == 3u);
    assert(eps[0] == USB_CDC_NOTIFY_EP && eps[1] == USB_CDC_DATA_OUT_EP
           && eps[2] == USB_CDC_DATA_IN_EP);
}

static void setup_pkt(uint8_t s[8], uint8_t bm, uint8_t req, uint16_t v, uint16_t i, uint16_t l)
{
    s[0] = bm;
    s[1] = req;
    s[2] = (uint8_t)(v & 0xFFu);
    s[3] = (uint8_t)(v >> 8);
    s[4] = (uint8_t)(i & 0xFFu);
    s[5] = (uint8_t)(i >> 8);
    s[6] = (uint8_t)(l & 0xFFu);
    s[7] = (uint8_t)(l >> 8);
}

static void test_usb_requests(void)
{
    UsbCdcEp0 u;
    const uint32_t uid[3] = { 0x0123ABCDu, 0u, 0xFFFFFFFFu };
    usb_cdc_ep0_init(&u, uid);
    uint8_t s[8];
    UsbEp0Action a;
    setup_pkt(s, 0x80u, 0x06u, 0x0303u, 0x0409u, 255u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_DATA_IN && a.len == 50u && a.zlp == 0u);
    assert(a.data[2] == '0' && a.data[4] == '1' && a.data[16] == 'D' && a.data[48] == 'F');
    /* Device descriptor clipped to the first 8 bytes (Windows' first request). */
    setup_pkt(s, 0x80u, 0x06u, 0x0100u, 0u, 8u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_DATA_IN && a.len == 8u && a.zlp == 0u);
    /* Config descriptor of exactly 64 bytes requested 255 -> ZLP rule only when
     * the clipped length is a multiple of 64 and shorter than wLength. */
    setup_pkt(s, 0x80u, 0x06u, 0x0200u, 0u, 64u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.len == 64u && a.zlp == 0u);
    setup_pkt(s, 0x80u, 0x06u, 0x0200u, 0u, 255u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.len == 67u && a.zlp == 0u);
    /* SET_ADDRESS, then GET_CONFIGURATION / SET_CONFIGURATION / GET_STATUS. */
    setup_pkt(s, 0x00u, 0x05u, 12u, 0u, 0u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_STATUS_IN && a.set_address == 1u && a.address == 12u);
    setup_pkt(s, 0x80u, 0x08u, 0u, 0u, 1u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_DATA_IN && a.len == 1u && a.data[0] == 0u);
    setup_pkt(s, 0x00u, 0x09u, 1u, 0u, 0u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_STATUS_IN && a.configure == 1u && u.config_value == 1u);
    setup_pkt(s, 0x00u, 0x09u, 2u, 0u, 0u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_STALL && u.config_value == 1u);
    setup_pkt(s, 0x80u, 0x00u, 0u, 0u, 2u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_DATA_IN && a.len == 2u && a.data[0] == 1u);
    setup_pkt(s, 0x02u, 0x01u, 0u, 0x81u, 0u);   /* CLEAR_FEATURE(HALT) EP 0x81 */
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_STATUS_IN && a.clear_halt_ep == 0x81u);
    setup_pkt(s, 0x02u, 0x01u, 0u, 0x85u, 0u);   /* unknown endpoint */
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_STALL);
    /* CDC class: line coding round trip, wrong interface stalls. */
    setup_pkt(s, 0x21u, 0x20u, 0u, 0u, 7u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_DATA_OUT && a.len == 7u);
    const uint8_t lc[7] = { 0x80u, 0x25u, 0u, 0u, 0u, 0u, 8u };
    assert(usb_cdc_ep0_out_data(&u, lc, 7u) == 1u);
    assert(usb_cdc_ep0_out_data(&u, lc, 7u) == 0u); /* not pending any more */
    setup_pkt(s, 0xA1u, 0x21u, 0u, 0u, 7u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_DATA_IN && memcmp(a.data, lc, 7u) == 0);
    setup_pkt(s, 0xA1u, 0x21u, 0u, 1u, 7u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_STALL);
    setup_pkt(s, 0x21u, 0x22u, 0x0003u, 0u, 0u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_STATUS_IN && u.ctrl_line_state == 3u);
    /* Device qualifier / other speed / vendor requests stall. */
    setup_pkt(s, 0x80u, 0x06u, 0x0600u, 0u, 10u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_STALL);
    setup_pkt(s, 0x80u, 0x06u, 0x0700u, 0u, 9u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_STALL);
    setup_pkt(s, 0xC0u, 0x01u, 0u, 0u, 4u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_STALL);
    /* SET_ADDRESS is refused once configured; bus reset returns to default. */
    setup_pkt(s, 0x00u, 0x05u, 3u, 0u, 0u);
    usb_cdc_ep0_setup(&u, s, &a);
    assert(a.kind == USB_EP0_ACT_STALL);
    usb_cdc_ep0_bus_reset(&u);
    assert(u.config_value == 0u && u.address == 0u);
    /* Random SETUP packets never crash and never produce an over-long reply. */
    for (uint32_t i = 0u; i < 200000u; ++i) {
        for (uint32_t k = 0u; k < 8u; ++k) {
            s[k] = (uint8_t)rnd();
        }
        usb_cdc_ep0_setup(&u, s, &a);
        const uint16_t wl = (uint16_t)(s[6] | (s[7] << 8));
        if (a.kind == USB_EP0_ACT_DATA_IN) {
            assert(a.len <= wl && a.len > 0u && a.data != NULL);
        }
        if (a.kind == USB_EP0_ACT_DATA_OUT) {
            assert(a.len == 7u);
        }
        assert(a.kind <= USB_EP0_ACT_DATA_OUT);
    }
}

static void test_usb_ep0_chunking(void)
{
    uint8_t buf[200];
    for (uint32_t i = 0u; i < sizeof(buf); ++i) {
        buf[i] = (uint8_t)i;
    }
    const uint16_t lens[] = { 1u, 18u, 63u, 64u, 65u, 67u, 128u, 129u, 200u };
    for (uint32_t c = 0u; c < sizeof(lens) / sizeof(lens[0]); ++c) {
        for (uint8_t zlp = 0u; zlp < 2u; ++zlp) {
            UsbEp0InXfer x;
            usb_ep0_in_begin(&x, buf, lens[c], zlp);
            uint32_t got = 0u;
            uint32_t zlps = 0u;
            uint32_t pkts = 0u;
            for (;;) {
                const uint8_t *p = NULL;
                uint16_t n = 0u;
                const UsbEp0InStep st = usb_ep0_in_next(&x, &p, &n);
                if (st == USB_EP0_IN_STATUS) {
                    break;
                }
                assert(st == USB_EP0_IN_SEND && n <= 64u);
                if (n == 0u) {
                    zlps++;
                } else {
                    assert(memcmp(p, &buf[got], n) == 0);
                    got += n;
                    pkts++;
                }
                assert(pkts < 10u);
            }
            assert(got == lens[c] && zlps == zlp);
            assert(pkts == (lens[c] + 63u) / 64u);
            assert(usb_ep0_in_next(&x, NULL, NULL) == USB_EP0_IN_IDLE);
        }
    }
}

/* ---------- IMU sensor codec ---------- */

static void test_imu_codec(void)
{
    ImuInitStep steps[IMU_INIT_MAX_STEPS];
    const uint32_t n = imu_codec_init_sequence(steps);
    assert(n > 0u && n <= IMU_INIT_MAX_STEPS);
    uint32_t checks = 0u;
    for (uint32_t i = 0u; i < n; ++i) {
        assert(steps[i].die <= IMU_DIE_GYRO && steps[i].reg <= 0x7Fu);
        checks += (steps[i].kind == IMU_STEP_CHECK) ? 1u : 0u;
    }
    assert(checks >= 4u); /* both chip IDs + config read-backs */
    assert(steps[0].kind == IMU_STEP_DUMMY_READ && steps[0].die == IMU_DIE_ACCEL);

    uint8_t tx[16];
    assert(imu_codec_build_read(IMU_DIE_ACCEL, 0x12u, 6u, tx, sizeof(tx)) == 8u);
    assert(tx[0] == 0x92u && tx[1] == 0u);
    assert(imu_codec_build_read(IMU_DIE_GYRO, 0x02u, 6u, tx, sizeof(tx)) == 7u);
    assert(tx[0] == 0x82u);
    assert(imu_codec_build_read(IMU_DIE_GYRO, 0x02u, 20u, tx, sizeof(tx)) == 0u);

    /* Scale: +32767 LSB = +6 g (accel), +2000 deg/s (gyro). */
    assert(fabs(imu_codec_accel_lsb_to_m_s2(32767) - 6.0 * 9.80665 * 32767.0 / 32768.0) < 1e-9);
    assert(fabs(imu_codec_gyro_lsb_to_rad_s(-32768) + 2000.0 * 3.14159265358979 / 180.0) < 1e-9);
    /* 1 g on +Z: 5461.33 LSB at +/-6 g. */
    const int16_t one_g = (int16_t)lround(32768.0 / 6.0);
    assert(fabs(imu_codec_accel_lsb_to_m_s2(one_g) - 9.80665) < 0.002);
    /* Temperature: 0x000 -> 23 C, raw +8 -> 24 C, 0x7FF.. negative wrap. */
    assert(imu_codec_temp_c(0x00u, 0x00u) == 23.0f);
    assert(imu_codec_temp_c(0x01u, 0x00u) == 24.0f);
    assert(imu_codec_temp_c(0xFFu, 0xE0u) == 22.875f);

    /* Decode + axis map: body x = -sensor y, body y = +sensor x, body z = sensor z. */
    const uint8_t acc6[6] = { 0x00u, 0x10u, 0x00u, 0xF0u, 0x55u, 0x15u }; /* 4096, -4096, 5461 */
    const uint8_t gyr6[6] = { 0x10u, 0x00u, 0xF0u, 0xFFu, 0x00u, 0x00u }; /* 16, -16, 0 */
    ImuAxisMap m;
    m.src[0] = 1u; m.sign[0] = -1;
    m.src[1] = 0u; m.sign[1] = 1;
    m.src[2] = 2u; m.sign[2] = 1;
    assert(imu_codec_axis_map_valid(&m) == 1u);
    ImuSample s;
    imu_codec_decode(&m, acc6, gyr6, 0u, 0u, &s);
    assert(s.accel_raw[0] == 4096 && s.accel_raw[1] == -4096 && s.accel_raw[2] == 5461);
    assert(fabs(s.accel_m_s2[0] - imu_codec_accel_lsb_to_m_s2(4096)) < 1e-12);
    assert(fabs(s.accel_m_s2[1] - imu_codec_accel_lsb_to_m_s2(4096)) < 1e-12);
    assert(fabs(s.gyro_rad_s[0] - imu_codec_gyro_lsb_to_rad_s(16)) < 1e-12);
    assert(fabs(s.gyro_rad_s[1] - imu_codec_gyro_lsb_to_rad_s(16)) < 1e-12);
    /* Invalid maps (duplicate source, bad sign) fall back to identity. */
    m.src[1] = 1u;
    assert(imu_codec_axis_map_valid(&m) == 0u);
    imu_codec_decode(&m, acc6, gyr6, 0u, 0u, &s);
    assert(fabs(s.accel_m_s2[1] - imu_codec_accel_lsb_to_m_s2(-4096)) < 1e-12);
    m = imu_codec_default_axis_map();
    m.sign[2] = 0;
    assert(imu_codec_axis_map_valid(&m) == 0u);
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        fprintf(stderr, "usage: %s <scenario>\n", argv[0]);
        return 2;
    }
    const char *t = argv[1];
    if (strcmp(t, "bench_input_gate") == 0) { test_bench_input_gate(); }
    else if (strcmp(t, "bench_end_to_end") == 0) { test_bench_end_to_end(); }
    else if (strcmp(t, "run_positive_control") == 0) { test_run_positive_control(); }
    else if (strcmp(t, "bench_tx_fuzz") == 0) { test_bench_tx_fuzz(); }
    else if (strcmp(t, "bench_step_fuzz") == 0) { test_bench_step_fuzz(); }
    else if (strcmp(t, "motor_id_estop_gate") == 0) { test_motor_id_estop_gate(); }
    else if (strcmp(t, "motor_id_limits") == 0) { test_motor_id_limits(); }
    else if (strcmp(t, "motor_id_pulse") == 0) { test_motor_id_pulse(); }
    else if (strcmp(t, "motor_id_tx_gate") == 0) { test_motor_id_tx_gate(); }
    else if (strcmp(t, "usb_descriptors") == 0) { test_usb_descriptors(); }
    else if (strcmp(t, "usb_requests") == 0) { test_usb_requests(); }
    else if (strcmp(t, "usb_ep0_chunking") == 0) { test_usb_ep0_chunking(); }
    else if (strcmp(t, "imu_codec") == 0) { test_imu_codec(); }
    else {
        fprintf(stderr, "unknown scenario %s\n", t);
        return 2;
    }
    printf("PASS %s\n", t);
    return 0;
}
