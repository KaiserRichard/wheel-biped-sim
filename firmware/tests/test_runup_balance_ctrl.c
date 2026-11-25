/* FW6 Part 1: mode-1 "back off and charge" run-up through the real balance_ctrl_tick()
 * (S11 policy, S10 estimator, blocked-drive detector, safety FSM, H5 controller).
 *
 * There is no dynamics plant: a kinematic wheel model feeds the encoders. The wheels
 * roll at v_act = r * omega, where v_act follows the run-up reference (perfect
 * tracking) unless a scenario imposes an edge stall, a charge stall, a speed cap or
 * a blocked reverse. This exercises the supervisor and phase logic, not stability
 * (that is the FIL's job).
 *
 *   R1 curb declared: blocked -> STEP_ASSIST, phases 4-5-6-7, hand-back to BALANCING
 *      with x_set continuous, hand-back slew <= 0.5 * 1.5 m/s^2, 1 s hold-off
 *   R2 field declared (L = 1.0 m): REFUSED (FW6FIX D1) -> F3b CONTROLLED_STOP -> REST
 *   R6-R9 aborts carry SAFETY_REASON_STEP_ASSIST_ABORT (FW6FIX D3)
 *   R3 no declaration: blocked -> CONTROLLED_STOP (F3b), never STEP_ASSIST
 *   R4 declared height 15 mm (> 12 mm gate): CONTROLLED_STOP, no run-up
 *   R5 legs/assist unavailable (bus below mild-sag 18.5 V): CONTROLLED_STOP
 *   R6 charge stall at the edge: abort 2 -> CONTROLLED_STOP -> REST, torque kept
 *   R7 charge speed never reached: abort 3 -> CONTROLLED_STOP
 *   R8 whole-manoeuvre timeout -> CONTROLLED_STOP
 *   R9 reverse blocked (< 60% of the back-off room): abort 1 -> CONTROLLED_STOP
 *   R10 radio loss / host loss during the run-up -> CONTROLLED_STOP (loss reason)
 */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "control/balance_ctrl.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(expr)                                                       \
    do {                                                                  \
        if (!(expr)) {                                                    \
            fprintf(stderr, "%s:%d: %s: check `%s' failed.\n",            \
                    __FILE__, __LINE__, __func__, #expr);                 \
            abort();                                                      \
        }                                                                 \
    } while (0)

enum Plant {
    PLANT_TRACK = 0,      /* follow the run-up reference */
    PLANT_CHARGE_STALL,   /* stop dead once charge speed is reached */
    PLANT_CHARGE_CAP,     /* cannot exceed 0.2 m/s in the charge */
    PLANT_REVERSE_BLOCK   /* cannot reverse */
};

typedef struct {
    BalanceCtrl        c;
    BalanceTickInput   in;
    BalanceTickOutput  out;
    double             ang[6];
    double             rate[6];
    uint32_t           seq;
    int32_t            t_ms;
    float              v_act;
    enum Plant         plant;
    uint8_t            edge_passed;  /* the kinematic edge stops the wheels until a charge */
    float              edge_x;       /* wheel odometry position of the edge */
    float              odo_x;
    uint8_t            phases_seen;  /* bit p for phase p */
    uint8_t            saw_assist;
} Sim;

static const double GYRO0[3] = { 0.0, 0.0, 0.0 };
static const double ACC0[3] = { 0.0, 0.0, 9.80665 };

static void sim_tick(Sim *s)
{
    const float r = (float)s->c.cfg.wheel_radius_m;
    s->odo_x += s->v_act * 0.002f;
    for (unsigned w = 2u; w <= 5u; w += 3u) {
        s->rate[w] = (double)(s->v_act / r);
        s->ang[w] = s11_wrap_angle(s->ang[w] + s->rate[w] * 0.002, 12.5);
    }
    balance_ctrl_feed_imu_1khz(&s->c, s->t_ms + 1, GYRO0, ACC0);
    balance_ctrl_feed_imu_1khz(&s->c, s->t_ms + 2, GYRO0, ACC0);
    s->t_ms += 2;
    s->seq += 1u;
    CanSchedRxSnapshot *e = &s->in.direct_enc_snap;
    memset(e, 0, sizeof(*e));
    e->fresh_mask = 0x3Fu;
    e->valid_mask = 0x3Fu;
    e->healthy_mask = 0x3Fu;
    e->drives_healthy_6 = 1u;
    for (unsigned i = 0u; i < 6u; ++i) {
        e->age_ms[i] = 1.0;
        e->angle[i] = s->ang[i];
        e->rate[i] = s->rate[i];
        e->seq[i] = s->seq;
    }
    s->in.enc_tick_ms = s->t_ms;
    s->in.tick_us = 100000u + (uint32_t)s->t_ms * 1000u;
    balance_ctrl_tick(&s->c, &s->in, &s->out);
    if (s->out.safety_out.state == SAFETY_STATE_STEP_ASSIST) {
        s->saw_assist = 1u;
        s->phases_seen |= (uint8_t)(1u << s->c.step_phase);
    }
}

/* Kinematic wheel speed for the next tick. */
static void sim_plant(Sim *s)
{
    const SafetyState st = s->out.safety_out.state;
    float v = 0.0f;
    if (st == SAFETY_STATE_STEP_ASSIST && s->c.step_phase >= 4u) {
        v = s->c.step_v_m_s;
        if (s->plant == PLANT_REVERSE_BLOCK && v < 0.0f) {
            v = 0.0f;
        }
        if (s->plant == PLANT_CHARGE_CAP && s->c.step_phase == 6u && v > 0.2f) {
            v = 0.2f;
        }
        if (s->plant == PLANT_CHARGE_STALL && s->c.step_phase == 6u && s->c.step_charged != 0u) {
            v = 0.0f;
        }
        if (s->c.step_phase >= 6u && v > 0.3f) {
            s->edge_passed = 1u;    /* arrived with momentum */
        }
    } else if (st == SAFETY_STATE_BALANCING || st == SAFETY_STATE_CONTROLLED_STOP
               || st == SAFETY_STATE_REST) {
        v = s->out.safety_out.v_ref_m_s;
    }
    if (s->edge_passed == 0u && v > 0.0f && s->odo_x >= s->edge_x) {
        v = 0.0f;                   /* slow drive is blocked at the edge */
    }
    s->v_act = v;
}

static void sim_start(Sim *s, void (*tweak)(BalanceCtrlConfig *))
{
    memset(s, 0, sizeof(*s));
    BalanceCtrlConfig cfg = balance_ctrl_default_config();
    cfg.safety_cfg = safety_config_with_radio_policy(RADIO_LOSS_BRAKE_BALANCE);
    if (tweak != NULL) {
        tweak(&cfg);
    }
    balance_ctrl_init(&s->c, &cfg);
    const double q[6] = { 0.85, 0.85, 0.0, 0.85, 0.85, 0.0 };
    memcpy(s->ang, q, sizeof(q));
    s->t_ms = -200;
    for (int32_t t = -320; t <= -200; ++t) {
        balance_ctrl_feed_imu_1khz(&s->c, t, GYRO0, ACC0);
    }
    BalanceTickInput *in = &s->in;
    in->use_can_sched_rx   = 0u;
    in->boot_self_test_ok  = 1u;
    in->dump_continuity_ok = 1u;
    in->v_bus_v            = 0.5f;
    in->v_batt_ocv_v       = 24.0f;
    in->radio_age_ms       = 4u;
    in->host_age_ms        = 10u;
    in->host_valid         = 1u;
    in->local_reset_req    = 1u;
    in->local_arm_req      = 1u;
    sim_tick(s);
    CHECK(s->out.safety_out.state == SAFETY_STATE_PRECHARGE);
    in->local_reset_req = 0u;
    in->local_arm_req   = 0u;
    in->k1_closed       = 1u;
    in->k2_closed       = 1u;
    in->precharge_dv_ok = 1u;
    in->v_bus_v         = 24.0f;
    in->radio_arm_sw    = 1u;
    for (unsigned k = 0u; k < 6u && s->out.safety_out.state != SAFETY_STATE_ARMED; ++k) {
        sim_tick(s);
    }
    CHECK(s->out.safety_out.state == SAFETY_STATE_ARMED);
    in->stand_handoff_req = 1u;
    sim_tick(s);
    CHECK(s->out.safety_out.state == SAFETY_STATE_BALANCING);
    in->stand_handoff_req = 0u;
    for (unsigned k = 0u; k < 100u; ++k) {   /* settle 200 ms */
        sim_tick(s);
    }
    s->edge_x = 0.15f;                       /* edge 15 cm ahead */
    s->odo_x = 0.0f;
    in->v_target_m_s = 0.10f;
}

/* Run until the supervisor leaves BALANCING/STEP_ASSIST or `ms` elapse. */
static SafetyState sim_run(Sim *s, uint32_t ms, uint8_t stop_on_rest)
{
    for (uint32_t t = 0u; t < ms; t += 2u) {
        sim_plant(s);
        sim_tick(s);
        const SafetyState st = s->out.safety_out.state;
        if (st == SAFETY_STATE_FAULT || st == SAFETY_STATE_ESTOP
            || (stop_on_rest != 0u && st == SAFETY_STATE_REST)) {
            return st;
        }
    }
    return s->out.safety_out.state;
}

static void declare_curb(Sim *s)
{
    s->in.step_declaration = (uint8_t)BALANCE_STEP_DECL_CURB;
    s->in.step_decl_height_m = 0.010f;
}

static SafetyState run_to_stop(Sim *s, uint32_t ms)
{
    for (uint32_t t = 0u; t < ms; t += 2u) {
        sim_plant(s);
        sim_tick(s);
        if (s->out.safety_out.state == SAFETY_STATE_CONTROLLED_STOP
            || s->out.safety_out.state == SAFETY_STATE_FAULT) {
            break;
        }
    }
    return s->out.safety_out.state;
}

static void test_r1_curb_runup_hand_back(void)
{
    Sim s;
    sim_start(&s, NULL);
    declare_curb(&s);
    float x_set_prev = s.c.h5.x_set_m;
    float v_prev = 0.0f;
    float max_handback_dv = 0.0f;
    uint8_t handed_back = 0u;
    uint8_t prev_assist = 0u;
    for (uint32_t t = 0u; t < 8000u && handed_back == 0u; t += 2u) {
        sim_plant(&s);
        x_set_prev = s.c.h5.x_set_m;
        const uint8_t phase_before = s.c.step_phase;
        v_prev = s.c.step_v_m_s;
        sim_tick(&s);
        const uint8_t assist = (s.out.safety_out.state == SAFETY_STATE_STEP_ASSIST) ? 1u : 0u;
        CHECK(s.out.safety_out.state == SAFETY_STATE_BALANCING || assist != 0u);
        CHECK(s.out.safety_out.torque_enable == 1u);
        if (assist != 0u && phase_before == 7u && s.c.step_phase == 7u) {
            const float dv = fabsf(s.c.step_v_m_s - v_prev);
            if (dv > max_handback_dv) max_handback_dv = dv;
        }
        if (prev_assist != 0u && assist == 0u) {
            handed_back = 1u;
            /* x_set continuous: advances by v_ref * dt only, no rebase */
            CHECK(fabsf((s.c.h5.x_set_m - x_set_prev) - 0.10f * 0.002f) < 1e-4f);
            CHECK(s.c.step_refractory_ms == s.c.cfg.step_refractory_ms);
        }
        prev_assist = assist;
    }
    CHECK(handed_back == 1u);
    CHECK(s.out.safety_out.state == SAFETY_STATE_BALANCING);
    CHECK((s.phases_seen & 0xF0u) == 0xF0u);   /* phases 4, 5, 6, 7 */
    CHECK((s.phases_seen & 0x0Eu) == 0u);      /* no mode-0 leg pulse */
    CHECK(max_handback_dv <= 0.5f * 1.5f * 0.002f + 1e-6f);
    CHECK(s.c.step_abort == 0u && s.c.step_carry_eff_m == 0.0f);
    /* hold-off: no new entry for 1 s while driving on */
    for (uint32_t t = 0u; t < 900u; t += 2u) {
        sim_plant(&s);
        sim_tick(&s);
        CHECK(s.out.safety_out.state == SAFETY_STATE_BALANCING);
    }
}

static void expect_stop_without_assist(Sim *s);

/* FW6FIX D1: a valid FIELD declaration (L = 1.0 m, 10 mm) is refused. The robot
 * keeps balancing up to the edge, never enters STEP_ASSIST, makes the F3b
 * controlled stop before the field and reaches REST; the refusal is reported. */
static void test_r2_field_refused(void)
{
    Sim s;
    sim_start(&s, NULL);
    s.in.step_declaration = (uint8_t)BALANCE_STEP_DECL_FIELD;
    s.in.step_decl_height_m = 0.010f;
    s.in.step_field_length_m = 1.0f;
    sim_tick(&s);
    CHECK(s.out.step_decl_refused == 1u);
    CHECK(s.out.safety_out.state == SAFETY_STATE_BALANCING);
    CHECK(s.c.step_decl_refused_n >= 1u);
    float diag[4] = { 0 };
    balance_ctrl_fil_alarm_diag(&s.c, diag);
    CHECK(diag[0] >= 1.0f && diag[1] == 0.0f);
    expect_stop_without_assist(&s);
    CHECK(s.c.step_carry_eff_m == 0.0f);
    CHECK(s.odo_x <= s.edge_x + 1e-3f);          /* stopped at the edge, not on the field */
    CHECK(s.out.step_decl_refused == 1u);
    /* a curb declaration on the same robot is accepted, so the refusal is
     * specific to FIELD */
    sim_start(&s, NULL);
    declare_curb(&s);
    sim_tick(&s);
    CHECK(s.out.step_decl_refused == 0u && s.c.step_decl_refused_n == 0u);
}

static void expect_stop_without_assist(Sim *s)
{
    CHECK(run_to_stop(s, 6000u) == SAFETY_STATE_CONTROLLED_STOP);
    CHECK(s->saw_assist == 0u);
    CHECK(s->out.safety_out.reason == SAFETY_REASON_VOLUNTARY_STOP);
    CHECK(s->out.safety_out.torque_enable == 1u);
    CHECK(sim_run(s, 3000u, 1u) == SAFETY_STATE_REST);
}

static void test_r3_no_declaration(void)
{
    Sim s;
    sim_start(&s, NULL);
    expect_stop_without_assist(&s);
}

static void test_r4_height_gate(void)
{
    Sim s;
    sim_start(&s, NULL);
    declare_curb(&s);
    s.in.step_decl_height_m = 0.015f;
    expect_stop_without_assist(&s);
    /* a field declaration without a valid length is also refused */
    sim_start(&s, NULL);
    s.in.step_declaration = (uint8_t)BALANCE_STEP_DECL_FIELD;
    s.in.step_field_length_m = 0.0f;
    expect_stop_without_assist(&s);
}

static void tweak_no_assist(BalanceCtrlConfig *c) { c->enable_step_assist = 0u; }

static void test_r5_legs_unavailable(void)
{
    Sim s;
    sim_start(&s, NULL);
    declare_curb(&s);
    s.in.v_bus_v = 18.0f;            /* below mild sag 18.5 V: no leg/drive margin */
    expect_stop_without_assist(&s);
    sim_start(&s, tweak_no_assist);   /* assist disabled in configuration */
    declare_curb(&s);
    expect_stop_without_assist(&s);
}

static void expect_assist_then_stop(Sim *s, uint8_t abort_code)
{
    CHECK(run_to_stop(s, 8000u) == SAFETY_STATE_CONTROLLED_STOP);
    CHECK(s->saw_assist == 1u);
    CHECK(s->c.step_phase == 0u);
    CHECK(s->out.safety_out.torque_enable == 1u);    /* balance torque kept */
    CHECK(s->out.safety_out.can_disable_cmd == 0u);
    /* FW6FIX D3: logged as a run-up abort, not as an operator stop */
    CHECK(s->out.safety_out.reason == SAFETY_REASON_STEP_ASSIST_ABORT);
    float diag[4] = { 0 };
    balance_ctrl_fil_alarm_diag(&s->c, diag);
    CHECK((int)diag[2] == (int)SAFETY_REASON_STEP_ASSIST_ABORT);
    CHECK((int)diag[3] == (int)abort_code);
    CHECK(sim_run(s, 3000u, 1u) == SAFETY_STATE_REST);
    CHECK(s->out.safety_out.stop_timeout_alarm == 0u);
}

/* Capture the abort code on the transition tick. */
static uint8_t run_capture_abort(Sim *s)
{
    uint8_t code = 0u;
    for (uint32_t t = 0u; t < 8000u; t += 2u) {
        sim_plant(s);
        sim_tick(s);
        if (s->c.step_abort != 0u) {
            code = s->c.step_abort;
        }
        if (s->out.safety_out.state == SAFETY_STATE_CONTROLLED_STOP) {
            break;
        }
    }
    return code;
}

static void test_r6_charge_stall(void)
{
    Sim s;
    sim_start(&s, NULL);
    declare_curb(&s);
    s.plant = PLANT_CHARGE_STALL;
    CHECK(run_capture_abort(&s) == 2u);
    expect_assist_then_stop(&s, 2u);
}

static void test_r7_charge_not_reached(void)
{
    Sim s;
    sim_start(&s, NULL);
    declare_curb(&s);
    s.plant = PLANT_CHARGE_CAP;      /* 0.2 m/s < 60% of the 0.5 m/s charge speed */
    CHECK(run_capture_abort(&s) == 3u);
    expect_assist_then_stop(&s, 3u);
}

static void tweak_short_timeout(BalanceCtrlConfig *c) { c->step_timeout_ms = 400u; }

static void test_r8_timeout(void)
{
    Sim s;
    sim_start(&s, tweak_short_timeout);
    declare_curb(&s);
    CHECK(run_capture_abort(&s) == 0u);
    CHECK(s.out.safety_out.state == SAFETY_STATE_CONTROLLED_STOP);
    CHECK(s.c.step_elapsed_ms >= 400u);
    expect_assist_then_stop(&s, 0u);
}

static void test_r9_reverse_blocked(void)
{
    Sim s;
    sim_start(&s, NULL);
    declare_curb(&s);
    s.plant = PLANT_REVERSE_BLOCK;
    CHECK(run_capture_abort(&s) == 1u);
    expect_assist_then_stop(&s, 1u);
}

static void test_r10_link_loss_priority(void)
{
    for (unsigned which = 0u; which < 2u; ++which) {
        Sim s;
        sim_start(&s, NULL);
        declare_curb(&s);
        for (uint32_t t = 0u; t < 6000u && s.out.safety_out.state != SAFETY_STATE_STEP_ASSIST;
             t += 2u) {
            sim_plant(&s);
            sim_tick(&s);
        }
        CHECK(s.out.safety_out.state == SAFETY_STATE_STEP_ASSIST);
        if (which == 0u) {
            s.in.radio_age_ms = 200u;   /* RF timeout, arm switch frame stale */
        } else {
            s.in.host_age_ms = 200u;
        }
        sim_plant(&s);
        sim_tick(&s);
        CHECK(s.out.safety_out.state == SAFETY_STATE_CONTROLLED_STOP);
        CHECK(s.out.safety_out.reason == (which == 0u ? SAFETY_REASON_RADIO_LOSS_BRAKE
                                                      : SAFETY_REASON_HOST_LOSS));
        CHECK(s.out.safety_out.torque_enable == 1u);
        CHECK(sim_run(&s, 3000u, 1u) == SAFETY_STATE_REST);
    }
}

int main(void)
{
    test_r1_curb_runup_hand_back();
    test_r2_field_refused();
    test_r3_no_declaration();
    test_r4_height_gate();
    test_r5_legs_unavailable();
    test_r6_charge_stall();
    test_r7_charge_not_reached();
    test_r8_timeout();
    test_r9_reverse_blocked();
    test_r10_link_loss_priority();
    printf("test_runup_balance_ctrl: 10 groups PASSED\n");
    return 0;
}
