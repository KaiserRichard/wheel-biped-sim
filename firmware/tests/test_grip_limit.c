/*
 * TRAC3 host tests: floor-grip estimate (slip history, demonstrated grip,
 * host hint, decay, ineligible evidence ignored) and the friction-aware
 * brake limiter (peak accel <= k*mu*g, stretched stop, bit-exact pass-through
 * when no low grip is known, within-limit commands, host loss), plus the
 * balance_ctrl initialisation / FIL hint wiring.
 */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "control/balance_ctrl.h"
#include "control/grip_limit.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(expr)                                                       \
    do {                                                                  \
        if (!(expr)) {                                                    \
            fprintf(stderr, "%s:%d: %s: Assertion `%s' failed.\n",        \
                    __FILE__, __LINE__, __func__, #expr);                 \
            abort();                                                      \
        }                                                                 \
    } while (0)

#define NEAR(a, b, tol) CHECK(fabsf((float)(a) - (float)(b)) <= (float)(tol))

static const float DT = 0.002f;

/* Host quintic speed profile: v0 -> 0 over T, starting at t = 0. */
static void quintic(float t, float v0, float T, float *v, float *a, float *j)
{
    float z = t / T;
    if (z < 0.0f) {
        z = 0.0f;
    }
    if (z > 1.0f) {
        z = 1.0f;
    }
    *v = v0 * (1.0f - z * z * z * (10.0f - 15.0f * z + 6.0f * z * z));
    *a = -v0 / T * 30.0f * z * z * (1.0f - z) * (1.0f - z);
    *j = (z > 0.0f && z < 1.0f) ? -v0 / (T * T) * 60.0f * z * (1.0f - z) * (1.0f - 2.0f * z)
                                : 0.0f;
}

/* Run a cruise + brake through the limiter; returns peak |a_out|, stop time
 * (first |v| < 1 mm/s), distance, min v and whether every sample passed
 * through bit-exactly. */
typedef struct {
    float a_peak;
    float t_stop;
    float dist;
    float v_min;
    int   exact;
} BrakeRun;

static BrakeRun run_brake(GripLimit *g, float v0, float T, uint8_t enable)
{
    BrakeRun r = { 0.0f, -1.0f, 0.0f, 1e9f, 1 };
    for (int k = -100; k < 2000; ++k) {
        const float t = (float)k * DT;
        float v = v0;
        float a = 0.0f;
        float j = 0.0f;
        if (t >= 0.0f) {
            quintic(t, v0, T, &v, &a, &j);
        }
        float vo;
        float ao;
        float jo;
        (void)grip_limit_ref(g, DT, enable, v, a, j, &vo, &ao, &jo);
        if (memcmp(&vo, &v, sizeof v) != 0 || memcmp(&ao, &a, sizeof a) != 0
            || memcmp(&jo, &j, sizeof j) != 0) {
            r.exact = 0;
        }
        if (t >= 0.0f) {
            r.dist += vo * DT;
            if (fabsf(ao) > r.a_peak) {
                r.a_peak = fabsf(ao);
            }
            if (vo < r.v_min) {
                r.v_min = vo;
            }
            if (r.t_stop < 0.0f && fabsf(vo) < 1e-3f && fabsf(v) < 1e-6f) {
                r.t_stop = t;
            }
        }
    }
    return r;
}

static void test_default_pass_through(void)
{
    GripLimit g;
    grip_limit_init(&g, NULL);
    NEAR(g.mu_used, 1.0f, 0.0f);
    NEAR(grip_limit_accel_max(&g), 0.0f, 0.0f);
    const BrakeRun r = run_brake(&g, 0.8f, 0.35f, 1u);
    CHECK(r.exact == 1);
    CHECK(g.limit_ticks_n == 0u);
    CHECK(r.a_peak > 4.2f);              /* 1.875 * 0.8 / 0.35 = 4.29 m/s^2 */
}

static void test_hint_stretches_brake(void)
{
    const float mus[3] = { 0.10f, 0.15f, 0.30f };
    for (int i = 0; i < 3; ++i) {
        GripLimit g;
        grip_limit_init(&g, NULL);
        grip_limit_set_hint(&g, mus[i]);
        NEAR(g.mu_used, mus[i], 0.0f);
        const float a_max = g.cfg.k_margin * mus[i] * 9.81f;
        NEAR(grip_limit_accel_max(&g), a_max, 1e-5f);
        const BrakeRun r = run_brake(&g, 0.8f, 0.35f, 1u);
        CHECK(r.exact == 0);
        CHECK(r.a_peak <= a_max * 1.0001f);
        CHECK(r.a_peak >= a_max * 0.95f);      /* the cap is actually used */
        CHECK(r.t_stop > 0.0f);                /* it does stop */
        CHECK(r.v_min > -0.01f);               /* no meaningful reversal */
        CHECK(r.dist >= 0.8f * 0.8f / (2.0f * a_max));  /* stretched */
        CHECK(r.dist <= 1.6f * 0.8f * 0.8f / (2.0f * a_max) + 0.1f);
        CHECK(g.active == 0u);                 /* rejoined the command */
    }
}

static void test_high_hint_and_gentle_commands_unchanged(void)
{
    GripLimit g;
    grip_limit_init(&g, NULL);
    grip_limit_set_hint(&g, 0.6f);             /* > mu_limit_on: no low grip */
    BrakeRun r = run_brake(&g, 0.8f, 0.35f, 1u);
    CHECK(r.exact == 1);

    /* TRAC4: the boundary is inclusive, so a 0.5 hint still limits. */
    grip_limit_init(&g, NULL);
    grip_limit_set_hint(&g, 0.5f);
    CHECK(grip_limit_accel_max(&g) > 0.0f);
    r = run_brake(&g, 0.8f, 0.35f, 1u);
    CHECK(r.exact == 0);
    CHECK(r.a_peak <= grip_limit_accel_max(&g) * 1.0001f);

    /* Low grip known, but a command already inside the limit: untouched. */
    grip_limit_init(&g, NULL);
    grip_limit_set_hint(&g, 0.3f);             /* a_max 0.48*0.3*g = 1.41 m/s^2 */
    r = run_brake(&g, 0.5f, 2.0f, 1u);         /* peak 0.47 m/s^2 */
    CHECK(r.exact == 1);
    CHECK(g.limit_ticks_n == 0u);
}

static void test_mid_command_drop_applies_at_once(void)
{
    GripLimit g;
    grip_limit_init(&g, NULL);
    grip_limit_set_hint(&g, 0.3f);             /* a_max 1.41 m/s^2 */
    float vo;
    float ao;
    float jo;
    float v = 0.0f;
    for (int k = 0; k < 100; ++k) {            /* 1.2 m/s^2: within the limit */
        v += 1.2f * DT;
        CHECK(grip_limit_ref(&g, DT, 1u, v, 1.2f, 0.0f, &vo, &ao, &jo) == 0u);
    }
    grip_limit_set_hint(&g, 0.1f);             /* new evidence: a_max 0.471 */
    v += 1.2f * DT;
    CHECK(grip_limit_ref(&g, DT, 1u, v, 1.2f, 0.0f, &vo, &ao, &jo) == 1u);
    CHECK(fabsf(ao) <= g.cfg.k_margin * 0.1f * 9.81f * 1.0001f);
    CHECK(vo < v);
}

static void test_host_loss_bypasses(void)
{
    GripLimit g;
    grip_limit_init(&g, NULL);
    grip_limit_set_hint(&g, 0.1f);
    const BrakeRun r = run_brake(&g, 0.8f, 0.35f, 0u);
    CHECK(r.exact == 1);
}

static void test_hint_validation(void)
{
    GripLimit g;
    grip_limit_init(&g, NULL);
    grip_limit_set_hint(&g, 0.2f);
    NEAR(g.mu_used, 0.2f, 0.0f);
    grip_limit_set_hint(&g, NAN);
    NEAR(g.mu_used, 1.0f, 0.0f);
    grip_limit_set_hint(&g, 0.2f);
    grip_limit_set_hint(&g, 0.0f);
    NEAR(g.mu_used, 1.0f, 0.0f);
    grip_limit_set_hint(&g, -0.3f);
    NEAR(g.mu_used, 1.0f, 0.0f);
    grip_limit_set_hint(&g, 1.5f);
    NEAR(g.mu_used, 1.0f, 0.0f);
}

static void feed(GripLimit *g, float seconds, uint8_t eligible, uint8_t slip, float accel)
{
    const int n = (int)(seconds / DT + 0.5f);
    for (int i = 0; i < n; ++i) {
        grip_limit_observe(g, DT, eligible, slip, accel);
    }
}

static void test_slip_history_estimate_and_decay(void)
{
    GripLimit g;
    grip_limit_init(&g, NULL);
    feed(&g, 0.5f, 1u, 0u, 1.0f);              /* driving at 1.0 m/s^2, no slip */
    NEAR(g.mu_used, 1.0f, 0.0f);               /* no slip: no low grip known */
    feed(&g, 0.05f, 1u, 1u, 3.0f);             /* slip; wheel accel ignored */
    CHECK(g.slip_events_n == 1u);
    NEAR(g.mu_slip, 1.0f / 9.81f, 0.01f);      /* utilisation just before slip */
    CHECK(grip_limit_accel_max(&g) > 0.0f);
    /* hold: unchanged for 5 s without wheel accel */
    const float held = g.mu_slip;
    feed(&g, 4.9f, 1u, 0u, 0.0f);
    NEAR(g.mu_slip, held, 1e-6f);
    /* then decays back to "no low grip known" */
    feed(&g, 1.0f, 1u, 0u, 0.0f);
    CHECK(g.mu_slip > held + 0.03f);
    feed(&g, 20.0f, 1u, 0u, 0.0f);
    NEAR(g.mu_slip, 1.0f, 0.0f);
    NEAR(grip_limit_accel_max(&g), 0.0f, 0.0f);

    /* Floor clamp: a slip with no prior utilisation gives mu_min. */
    grip_limit_init(&g, NULL);
    feed(&g, 0.01f, 1u, 1u, 0.0f);
    NEAR(g.mu_slip, 0.08f, 1e-6f);
    /* Cap: a slip at high utilisation is still treated as low grip. */
    grip_limit_init(&g, NULL);
    feed(&g, 0.5f, 1u, 0u, 5.0f);
    feed(&g, 0.01f, 1u, 1u, 5.0f);
    NEAR(g.mu_slip, 0.30f, 1e-6f);
}

static void test_demonstrated_grip_raises_estimate(void)
{
    GripLimit g;
    grip_limit_init(&g, NULL);
    feed(&g, 0.01f, 1u, 1u, 0.0f);             /* mu_slip = 0.08 */
    feed(&g, 0.1f, 1u, 0u, 3.0f);              /* shorter than the dwell */
    CHECK(g.mu_slip < 0.2f);
    feed(&g, 0.6f, 1u, 0u, 3.0f);              /* 0.31 g carried without slip */
    CHECK(g.mu_slip > 0.25f);
    CHECK(g.mu_slip < 0.32f);
}

static void test_ineligible_evidence_ignored(void)
{
    GripLimit g;
    grip_limit_init(&g, NULL);
    feed(&g, 1.0f, 0u, 1u, 3.0f);              /* e.g. S11 frozen/advisory mask */
    NEAR(g.mu_used, 1.0f, 0.0f);
    CHECK(g.slip_events_n == 0u);
    const BrakeRun r = run_brake(&g, 0.8f, 0.35f, 1u);
    CHECK(r.exact == 1);
}

static void test_balance_ctrl_wiring(void)
{
    static BalanceCtrl ctrl;
    memset(&ctrl, 0xA5, sizeof(ctrl));
    balance_ctrl_fil_init(&ctrl, 5.06f, 0.20f, 0u);
    NEAR(ctrl.grip.mu_used, 1.0f, 0.0f);
    NEAR(ctrl.grip.mu_hint, 0.0f, 0.0f);
    CHECK(ctrl.grip.active == 0u);
    NEAR(ctrl.fil_floor_hint, 0.0f, 0.0f);
    balance_ctrl_fil_floor_hint(&ctrl, 0.15f);
    NEAR(ctrl.fil_floor_hint, 0.15f, 0.0f);
    float d[8];
    balance_ctrl_fil_grip_diag(&ctrl, d);
    NEAR(d[0], 1.0f, 0.0f);
    NEAR(d[4], 0.0f, 0.0f);
}

/* TRAC4: full controller after the FIL handoff with a mu 0.10 hint, the
 * proactive limiter already active (stretched brake in progress) and the S10
 * slip alarm raised (as by a wheel lock). trac_preset seeds the TRAC2 governor
 * as already running (reactive case) or idle. */
static void trac4_setup(BalanceCtrl *ctrl, BalanceTickInput *in, uint8_t trac_preset)
{
    balance_ctrl_fil_init(ctrl, 5.06f, 0.20f, 0u);
    const double gyro[3] = {0.0, 0.0, 0.0};
    const double accel[3] = {0.0, 0.0, 9.80665};
    const double q[6] = {0.85, 0.85, 0.0, 0.85, 0.85, 0.0};
    const double dq[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (int32_t t = -110; t <= -10; ++t) {
        balance_ctrl_feed_imu_1khz(ctrl, t, gyro, accel);
    }
    CHECK(balance_ctrl_fil_arm_and_handoff(ctrl, 10000u, -10, q, dq)
          == SAFETY_STATE_BALANCING);
    memset(in, 0, sizeof(*in));
    in->tick_us = 20000u;
    in->boot_self_test_ok = 1u;
    in->dump_continuity_ok = 1u;
    in->k1_closed = 1u;
    in->k2_closed = 1u;
    in->precharge_dv_ok = 1u;
    in->v_bus_v = 24.0f;
    in->v_batt_ocv_v = 24.0f;
    in->radio_age_ms = 4u;
    in->radio_arm_sw = 1u;
    in->host_age_ms = 10u;
    in->host_valid = 1u;
    in->floor_mu_hint = 0.10f;
    in->direct_enc_snap.valid_mask = 0x3fu;
    in->direct_enc_snap.healthy_mask = 0x3fu;
    in->direct_enc_snap.drives_healthy_6 = 1u;
    memcpy(in->direct_enc_snap.angle, q, sizeof(q));
    ctrl->s10.alarm = 1;
    ctrl->s10.speed = 0.0;          /* est_v dragged down by the locked wheels */
    ctrl->grip.active = 1u;         /* limiter owns the reference ...          */
    ctrl->grip.v_m_s = 0.5f;        /* ... mid-way through a stretched brake   */
    ctrl->grip.a_m_s2 = -0.4f;
    ctrl->trac_active = trac_preset;
    ctrl->trac_v_m_s = 0.0f;
}

static void test_governor_does_not_restart_under_limiter(void)
{
    static BalanceCtrl ctrl;
    BalanceTickInput in;
    BalanceTickOutput out;

    /* Proactive limiter active, governor idle: the alarm must not start the
     * governor (no restart from est_v); the limiter reference is kept. */
    trac4_setup(&ctrl, &in, 0u);
    const uint32_t events0 = ctrl.trac_events_n;
    balance_ctrl_tick(&ctrl, &in, &out);
    CHECK(out.safety_out.state == SAFETY_STATE_BALANCING);
    CHECK(out.safety_out.torque_enable != 0u);
    CHECK(out.s11_frame.degraded_mask == 0u && out.s11_frame.advisory_mask == 0u);
    CHECK(ctrl.s10.alarm != 0);
    CHECK(ctrl.grip.active != 0u);
    CHECK(ctrl.trac_active == 0u);
    CHECK(ctrl.trac_events_n == events0);
    NEAR(out.safety_out.v_ref_m_s, ctrl.grip.v_m_s, 1e-6f);
    CHECK(out.safety_out.v_ref_m_s > 0.49f);

    /* Host loss releases the limiter, so the TRAC2 governor engages again. */
    in.tick_us += 2000u;
    in.host_valid = 0u;
    in.host_age_ms = 200u;
    balance_ctrl_tick(&ctrl, &in, &out);
    CHECK(ctrl.grip.active == 0u);
    CHECK(ctrl.trac_active != 0u);
    CHECK(out.safety_out.state == SAFETY_STATE_CONTROLLED_STOP);

    /* A governor that was already running (reactive case: it started before
     * the limiter) is not cut off by the limiter becoming active. */
    trac4_setup(&ctrl, &in, 1u);
    balance_ctrl_tick(&ctrl, &in, &out);
    CHECK(ctrl.s10.alarm != 0);
    CHECK(ctrl.grip.active != 0u);
    CHECK(ctrl.trac_active != 0u);
    CHECK(out.safety_out.v_ref_m_s < 0.49f);   /* governed, lead-bounded to est_v */
}

int main(void)
{
    test_default_pass_through();
    test_hint_stretches_brake();
    test_high_hint_and_gentle_commands_unchanged();
    test_mid_command_drop_applies_at_once();
    test_host_loss_bypasses();
    test_hint_validation();
    test_slip_history_estimate_and_decay();
    test_demonstrated_grip_raises_estimate();
    test_ineligible_evidence_ignored();
    test_balance_ctrl_wiring();
    test_governor_does_not_restart_under_limiter();
    printf("test_grip_limit: all passed\n");
    return 0;
}
