/* FW6: S11 frozen-encoder detector binding and reaction (STEP3RC B6).
 *
 * Policy level (s11_packet_policy.c + s11_frozen.c):
 *   P1/P2 single wheel freeze -> S11_DEGRADED, frame valid, partner substitution
 *   P3    single hip freeze   -> S11_DEGRADED (mirror partner stationary: held)
 *   P3b   hip substitution: mirror (default) follows the other leg's same-role hip;
 *         hold option; fallback to hold when the mirror hip is also frozen
 *   P4    double faults (both wheels, both hips of a leg, three axes) -> STUCK latch
 *   P5    one hip per leg / wheel + hip -> DEGRADED (no escalation)
 *   P6    two simultaneous GROSS -> STUCK latch
 *   P7    IMU transient gate: NET/RECT suppressed, GROSS still confirms
 *   P8    freeze while ARMING -> STUCK latch (torque never enabled)
 *   P9    detector disabled (FIL hook) -> no detection
 *   P10   ADVISORY -> flag only, mode FRESH, clears on code change
 *   P11   healthy moving/stationary traffic for 10 s -> no detection
 * Supervisor level (safety_fsm.c):
 *   F1 BALANCING + DEGRADED -> CONTROLLED_STOP (torque on) -> REST; no re-grant
 *   F2 DEGRADED stop with pitch > 15 deg -> FAULT (torque off)
 *   F3 e-stop, drive fault and cannot-balance (tilt) keep torque-off, also while degraded
 *   F4 STUCK (double fault) -> FAULT torque off
 *   F5 STEP_ASSIST + DEGRADED -> CONTROLLED_STOP
 *   F6 ARMED / PRECHARGE + DEGRADED -> FAULT (no torque to cut)
 */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "safety_fsm.h"
#include "s11_packet_policy.h"

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

#define ALL 0x3Fu
#define DT 2.0

/* ---------------------------------------------------------------- policy rig */
typedef struct {
    S11PacketPolicy st;
    S11PacketFrame  f;
    double   ang[6];
    double   rate[6];
    double   age[6];
    uint32_t seq[6];
} Rig;

static void rig_tick(Rig *r)
{
    for (unsigned i = 0u; i < 6u; ++i) {
        r->seq[i] += 1u;
    }
    s11_packets_ex(&r->st, ALL, ALL, ALL, r->age, r->ang, r->rate, DT, r->seq, &r->f);
}

/* Healthy motion: every axis advances by rate*dt (code changes each tick). */
static void rig_move(Rig *r, uint32_t frozen_mask)
{
    for (unsigned i = 0u; i < 6u; ++i) {
        if ((frozen_mask & (1u << i)) == 0u) {
            r->ang[i] += r->rate[i] * 0.001 * DT;
            if ((i % 3u) == 2u) {
                r->ang[i] = s11_wrap_angle(r->ang[i], 12.5);
            }
        }
    }
}

static void rig_arm(Rig *r, double hip_rate, double wheel_rate)
{
    memset(r, 0, sizeof(*r));
    s11_init(&r->st);
    CHECK(s11_arm(&r->st) == 1);
    for (unsigned i = 0u; i < 6u; ++i) {
        r->ang[i] = ((i % 3u) == 2u) ? 0.3 : 0.2;
        r->rate[i] = ((i % 3u) == 2u) ? wheel_rate : hip_rate;
        r->age[i] = 1.0;
    }
    for (unsigned k = 0u; k < 4u; ++k) {
        rig_move(r, 0u);
        rig_tick(r);
    }
    CHECK(r->f.mode == S11_FRESH && r->f.valid == 1u);
}

/* Freeze `mask` with a constant reported rate; healthy axes keep moving.
 * Returns ticks until frame mode != FRESH (or max_ticks + 1). */
static unsigned rig_freeze(Rig *r, uint32_t mask, double frozen_rate, unsigned max_ticks)
{
    for (unsigned i = 0u; i < 6u; ++i) {
        if ((mask & (1u << i)) != 0u) {
            r->rate[i] = frozen_rate;
        }
    }
    for (unsigned k = 1u; k <= max_ticks; ++k) {
        rig_move(r, mask);
        rig_tick(r);
        if (r->f.mode != S11_FRESH) {
            return k;
        }
    }
    return max_ticks + 1u;
}

/* Keep `mask` frozen at `frozen_rate` until all its axes are CONFIRMED or the
 * frame leaves DEGRADED/FRESH; returns ticks used. */
static unsigned rig_until(Rig *r, uint32_t mask, double frozen_rate, unsigned max_ticks)
{
    for (unsigned i = 0u; i < 6u; ++i) {
        if ((mask & (1u << i)) != 0u) {
            r->rate[i] = frozen_rate;
        }
    }
    for (unsigned k = 1u; k <= max_ticks; ++k) {
        rig_move(r, mask);
        rig_tick(r);
        if ((r->f.mode != S11_FRESH && r->f.mode != S11_DEGRADED)
            || (r->f.degraded_mask & mask) == mask) {
            return k;
        }
    }
    return max_ticks + 1u;
}

static void test_p1_p2_single_wheel_partner_substitution(void)
{
    for (unsigned w = 2u; w <= 5u; w += 3u) {
        const unsigned p = s11_partner_wheel(w);
        CHECK(p == (w == 2u ? 5u : 2u));
        Rig r;
        rig_arm(&r, 0.0, 5.0);   /* driving: both wheels at 5 rad/s */
        /* 1a pattern (+/-20 rad/s, GROSS) on one wheel */
        const unsigned k = rig_freeze(&r, 1u << w, 20.0, 10u);
        CHECK(k <= 2u);                                 /* GROSS: first frozen sample */
        CHECK(r.f.mode == S11_DEGRADED);
        CHECK(r.f.valid == 1u);
        CHECK(fabs(r.f.authority - 1.0) < 1e-12);
        CHECK(r.f.degraded_mask == (1u << w));
        CHECK(r.f.rate[w] == r.f.rate[p]);
        const double u_self0 = r.f.unwrapped_angle[w];
        const double u_part0 = r.f.unwrapped_angle[p];
        for (unsigned n = 0u; n < 200u; ++n) {        /* 400 ms, partner keeps moving */
            r.rate[w] = (n % 2u == 0u) ? 20.0 : -20.0;
            rig_move(&r, 1u << w);
            rig_tick(&r);
            CHECK(r.f.mode == S11_DEGRADED && r.f.valid == 1u);
            CHECK(r.f.rate[w] == r.f.rate[p]);
            CHECK(fabs((r.f.unwrapped_angle[w] - u_self0)
                       - (r.f.unwrapped_angle[p] - u_part0)) < 1e-9);
            CHECK(fabs(r.f.angle[w] - s11_wrap_angle(r.f.unwrapped_angle[w], 12.5)) < 1e-9);
        }
        /* partner advanced 2.0 rad in 400 ms; substituted wheel followed it */
        CHECK(fabs((r.f.unwrapped_angle[w] - u_self0) - 2.0) < 1e-6);
        /* latched until reset */
        s11_reset(&r.st);
        CHECK(r.st.frozen_mask == 0u);
    }
}

static void test_p3_single_hip_holds_geometry(void)
{
    for (unsigned h = 0u; h < 6u; ++h) {
        if ((h % 3u) == 2u) {
            continue;
        }
        Rig r;
        rig_arm(&r, 0.0, 0.0);
        const double held = r.ang[h];
        /* uni_0p15: constant 0.15 rad/s on an unchanged code -> NET (<= 26 ms) */
        const unsigned k = rig_freeze(&r, 1u << h, 0.15, 40u);
        CHECK(k <= 13u);
        CHECK(r.f.mode == S11_DEGRADED && r.f.valid == 1u);
        CHECK(r.f.degraded_mask == (1u << h));
        CHECK(r.st.axis[h].frozen.reason == S11FZ_R_NET);
        CHECK(r.f.angle[h] == held && r.f.rate[h] == 0.0);
        /* the reported code later jumps: geometry stays held */
        r.ang[h] = held + 0.3;
        rig_tick(&r);
        CHECK(r.f.mode == S11_DEGRADED && r.f.angle[h] == held && r.f.rate[h] == 0.0);
    }
}

static void test_p3b_hip_mirror_hold_and_fallback(void)
{
    /* default (mirror): faulted LF hip follows the RF hip increment and rate */
    Rig r;
    rig_arm(&r, 0.0, 0.0);
    CHECK(r.st.config.frozen_hip_mirror == 1u);
    CHECK(rig_until(&r, 0x01u, 0.15, 40u) <= 40u);
    const double held = r.f.angle[0];
    const double rf0 = r.f.angle[3];
    r.rate[3] = 0.4;                                /* RF hip moves */
    for (unsigned n = 0u; n < 50u; ++n) {
        rig_move(&r, 0x01u);
        rig_tick(&r);
        CHECK(r.f.mode == S11_DEGRADED);
        CHECK(r.f.rate[0] == r.f.rate[3]);
        CHECK(fabs((r.f.angle[0] - held) - (r.f.angle[3] - rf0)) < 1e-12);
    }
    CHECK(fabs(r.f.angle[0] - held - 0.04) < 1e-9);  /* 0.4 rad/s for 100 ms */

    /* hold option: geometry held with zero rate although RF moves */
    rig_arm(&r, 0.0, 0.0);
    r.st.config.frozen_hip_mirror = 0u;
    CHECK(rig_until(&r, 0x01u, 0.15, 40u) <= 40u);
    const double held2 = r.f.angle[0];
    r.rate[3] = 0.4;
    for (unsigned n = 0u; n < 50u; ++n) {
        rig_move(&r, 0x01u);
        rig_tick(&r);
        CHECK(r.f.angle[0] == held2 && r.f.rate[0] == 0.0);
    }

    /* fallback: LF and RF both frozen (no escalation) -> both hold geometry */
    rig_arm(&r, 0.0, 0.0);
    CHECK(rig_until(&r, 0x09u, 0.15, 40u) <= 40u);
    CHECK(r.f.mode == S11_DEGRADED && r.f.degraded_mask == 0x09u);
    const double h0 = r.f.angle[0], h3 = r.f.angle[3];
    r.rate[1] = 0.4;
    for (unsigned n = 0u; n < 20u; ++n) {
        rig_move(&r, 0x09u);
        rig_tick(&r);
        CHECK(r.f.angle[0] == h0 && r.f.angle[3] == h3);
        CHECK(r.f.rate[0] == 0.0 && r.f.rate[3] == 0.0);
    }
}

static void test_p4_double_faults_escalate(void)
{
    const uint32_t pairs[3] = { 0x24u /* both wheels */, 0x03u /* left hips */,
                                0x18u /* right hips */ };
    for (unsigned c = 0u; c < 3u; ++c) {
        Rig r;
        rig_arm(&r, 0.0, 0.0);
        /* first axis by NET, second axis 10 ms later (not simultaneous) */
        const uint32_t m = pairs[c];
        uint32_t first = 0u;
        for (unsigned i = 0u; i < 6u; ++i) {
            if ((m & (1u << i)) != 0u) { first = 1u << i; break; }
        }
        (void)rig_freeze(&r, first, 0.15, 40u);
        CHECK(r.f.mode == S11_DEGRADED && r.f.degraded_mask == first);
        const unsigned k = rig_freeze(&r, m, 0.15, 1u);
        (void)k;
        unsigned n = 0u;
        while (r.f.mode == S11_DEGRADED && n < 40u) {
            rig_move(&r, m);
            rig_tick(&r);
            ++n;
        }
        CHECK(r.f.mode == S11_STUCK);
        CHECK(r.f.valid == 0u && isnan(r.f.angle[0]));
        CHECK(r.st.latched == S11_STUCK);
        CHECK(s11_arm(&r.st) == 0);                  /* latched until reset */
    }
    /* three axes: wheel L, hip LF, hip RR -> STUCK */
    Rig r;
    rig_arm(&r, 0.0, 0.0);
    CHECK(rig_until(&r, 0x04u, 0.15, 40u) <= 40u);
    CHECK(rig_until(&r, 0x05u, 0.15, 40u) <= 40u);
    CHECK(r.f.mode == S11_DEGRADED && r.f.degraded_mask == 0x05u);
    CHECK(rig_until(&r, 0x15u, 0.15, 40u) <= 40u);
    CHECK(r.f.mode == S11_STUCK && r.f.valid == 0u);
}

static void test_p5_non_escalating_pairs(void)
{
    const uint32_t pairs[3] = { 0x09u /* LF + RF hips */, 0x05u /* LF hip + L wheel */,
                                0x12u /* LR hip + RR hip */ };
    for (unsigned c = 0u; c < 3u; ++c) {
        Rig r;
        rig_arm(&r, 0.0, 0.0);
        CHECK(rig_until(&r, pairs[c], 0.15, 40u) <= 40u);
        for (unsigned n = 0u; n < 40u; ++n) {
            rig_move(&r, pairs[c]);
            rig_tick(&r);
        }
        CHECK(r.f.mode == S11_DEGRADED && r.f.valid == 1u);
        CHECK(r.f.degraded_mask == pairs[c]);
    }
}

static void test_p6_simultaneous_gross(void)
{
    Rig r;
    rig_arm(&r, 0.0, 0.0);
    const unsigned k = rig_freeze(&r, 0x09u, 20.0, 5u);  /* LF + RF hip, 1a */
    CHECK(k == 1u);
    CHECK(r.f.mode == S11_STUCK && r.f.valid == 0u);
}

static void test_p7_imu_gate(void)
{
    Rig r;
    rig_arm(&r, 0.0, 0.0);
    /* impact transient: gyro_y 1.0 rad/s, held 60 ms after it clears */
    s11_frozen_imu_update(&r.st, 0.0, 1.0, 9.81, 1.0);
    CHECK(r.st.frozen_is_gated == 1u);
    r.rate[0] = 0.15;
    for (unsigned n = 0u; n < 25u; ++n) {           /* 50 ms, gate kept active */
        s11_frozen_imu_update(&r.st, 0.0, 1.0, 9.81, 1.0);
        s11_frozen_imu_update(&r.st, 0.0, 1.0, 9.81, 1.0);
        rig_move(&r, 0x01u);
        rig_tick(&r);
        CHECK(r.f.mode == S11_FRESH);               /* NET suppressed while gated */
    }
    /* GROSS stays active while gated (the detector restarts its run on the
     * sample after a gated one, so GROSS fires within two samples) */
    r.rate[0] = 20.0;
    for (unsigned n = 0u; n < 2u; ++n) {
        s11_frozen_imu_update(&r.st, 0.0, 1.0, 9.81, 1.0);
        s11_frozen_imu_update(&r.st, 0.0, 1.0, 9.81, 1.0);
        rig_move(&r, 0x01u);
        rig_tick(&r);
    }
    CHECK(r.st.frozen_is_gated == 1u);
    CHECK(r.f.mode == S11_DEGRADED && r.st.axis[0].frozen.reason == S11FZ_R_GROSS);

    /* the gate releases 60 ms after the last transient sample */
    Rig q;
    rig_arm(&q, 0.0, 0.0);
    s11_frozen_imu_update(&q.st, 0.0, 0.0, 9.81 + 9.0, 1.0);  /* |a| - g > 8 */
    CHECK(q.st.frozen_is_gated == 1u);
    for (unsigned n = 0u; n < 59u; ++n) {
        s11_frozen_imu_update(&q.st, 0.0, 0.0, 9.81, 1.0);
    }
    CHECK(q.st.frozen_is_gated == 1u);
    s11_frozen_imu_update(&q.st, 0.0, 0.0, 9.81, 1.0);
    s11_frozen_imu_update(&q.st, 0.0, 0.0, 9.81, 1.0);
    CHECK(q.st.frozen_is_gated == 0u);
}

static void test_p8_freeze_while_arming(void)
{
    S11PacketPolicy st;
    S11PacketFrame f;
    double ang[6] = { 0.2, 0.2, 0.3, 0.2, 0.2, 0.3 };
    double rate[6] = { 0.0, 0.0, 15.0, 0.0, 0.0, 0.0 };
    double age[6] = { 1, 1, 1, 1, 1, 1 };
    s11_init(&st);
    CHECK(s11_arm(&st) == 1);
    for (unsigned k = 0u; k < 4u; ++k) {
        s11_packets_ex(&st, ALL, ALL, ALL, age, ang, rate, DT, NULL, &f);
    }
    CHECK(f.mode == S11_STUCK && f.valid == 0u);
    CHECK(st.arm_state != S11_STATE_ARMED);
}

static void test_p9_detector_disabled(void)
{
    Rig r;
    rig_arm(&r, 0.0, 0.0);
    r.st.config.frozen_detect_enable = 0u;
    CHECK(rig_freeze(&r, 0x04u, 20.0, 50u) == 51u);
    CHECK(r.f.mode == S11_FRESH && r.f.degraded_mask == 0u);
}

static void test_p10_advisory_flag_only(void)
{
    Rig r;
    rig_arm(&r, 0.0, 0.0);
    /* zero-mean +/-0.3 rad/s in one code: rect8 < 7, |net| < 5, run sum >= 6 */
    unsigned adv = 0u;
    for (unsigned n = 0u; n < 30u; ++n) {
        r.rate[1] = (n % 2u == 0u) ? 0.3 : -0.3;
        rig_tick(&r);
        CHECK(r.f.mode == S11_FRESH && r.f.valid == 1u);
        CHECK(r.f.degraded_mask == 0u);
        if ((r.f.advisory_mask & 0x02u) != 0u) {
            adv = 1u;
        }
    }
    CHECK(adv == 1u);
    /* code change clears ADVISORY */
    r.ang[1] += 0.01;
    rig_tick(&r);
    CHECK((r.f.advisory_mask & 0x02u) == 0u);
}

static void test_p11_healthy_traffic(void)
{
    /* 10 s: hips quantised slow walk at 0.1 rad/s (1 LSB per ~3.8 ms), wheels
     * moving 3 rad/s, plus +/-1 velocity-count noise on stationary codes. */
    Rig r;
    rig_arm(&r, 0.0, 3.0);
    const double lsb = 25.0 / 65536.0;
    double hip_true[6] = { 0.2, 0.2, 0.0, 0.2, 0.2, 0.0 };
    for (unsigned n = 0u; n < 5000u; ++n) {
        for (unsigned i = 0u; i < 6u; ++i) {
            if ((i % 3u) == 2u) {
                r.ang[i] = s11_wrap_angle(r.ang[i] + r.rate[i] * 0.001 * DT, 12.5);
            } else {
                hip_true[i] += 0.1 * 0.001 * DT;
                r.ang[i] = floor(hip_true[i] / lsb) * lsb;
                r.rate[i] = 0.1 + ((n % 2u == 0u) ? 90.0 / 4095.0 : -90.0 / 4095.0);
            }
        }
        rig_tick(&r);
        if (!(r.f.mode == S11_FRESH && r.f.degraded_mask == 0u)) {
            fprintf(stderr, "P11 n=%u mode=%d mask=%x r0=%d r2=%d\n", n, (int)r.f.mode,
                    r.f.degraded_mask, (int)r.st.axis[0].frozen.reason,
                    (int)r.st.axis[2].frozen.reason);
        }
        CHECK(r.f.mode == S11_FRESH && r.f.degraded_mask == 0u);
    }
}

/* ------------------------------------------------------------ supervisor rig */
static SafetyInputs base_inputs(void)
{
    SafetyInputs in;
    memset(&in, 0, sizeof(in));
    in.dt_ms              = 2u;
    in.boot_self_test_ok  = 1u;
    in.dump_continuity_ok = 1u;
    in.v_bus_v            = 0.5f;
    in.v_batt_ocv_v       = 24.0f;
    in.imu_est_valid      = 1u;
    in.ground_left        = 1u;
    in.ground_right       = 1u;
    in.radio_age_ms       = 4u;
    in.host_age_ms        = 20u;
    in.host_valid         = 1u;
    return in;
}

static SafetyInputs live_inputs(enum S11Mode mode)
{
    SafetyInputs in = base_inputs();
    in.k1_closed        = 1u;
    in.k2_closed        = 1u;
    in.precharge_dv_ok  = 1u;
    in.v_bus_v          = 24.0f;
    in.drives_healthy_6 = 1u;
    in.radio_arm_sw     = 1u;
    S11PacketFrame f;
    memset(&f, 0, sizeof(f));
    f.mode = mode;
    f.valid = (mode == S11_FRESH || mode == S11_DEGRADED) ? 1u : 0u;
    f.authority = f.valid != 0u ? 1.0 : 0.0;
    f.degraded_mask = (mode == S11_DEGRADED) ? 0x04u : 0u;
    safety_fsm_bind_s11_frame(&in, &f);
    return in;
}

static void reach_armed(SafetyFsm *fsm, SafetyOutput *out)
{
    safety_fsm_init(fsm);
    SafetyInputs in = base_inputs();
    in.local_reset_req = 1u;
    in.local_arm_req = 1u;
    safety_fsm_step(fsm, &in, out);
    CHECK(out->state == SAFETY_STATE_PRECHARGE);
    in = live_inputs(S11_FRESH);
    safety_fsm_step(fsm, &in, out);
    CHECK(out->state == SAFETY_STATE_ARMED);
}

static void reach_balancing(SafetyFsm *fsm, SafetyOutput *out)
{
    reach_armed(fsm, out);
    SafetyInputs in = live_inputs(S11_FRESH);
    in.stand_handoff_req = 1u;
    safety_fsm_step(fsm, &in, out);
    CHECK(out->state == SAFETY_STATE_BALANCING && out->torque_enable == 1u);
    in = live_inputs(S11_FRESH);
    in.v_target_m_s = 0.3f;
    safety_fsm_step(fsm, &in, out);
    CHECK(fabsf(out->v_ref_m_s - 0.3f) < 1e-6f);
}

static void test_f1_degraded_controlled_stop_to_rest(void)
{
    SafetyFsm fsm;
    SafetyOutput out;
    reach_balancing(&fsm, &out);
    SafetyInputs in = live_inputs(S11_DEGRADED);
    in.v_target_m_s = 0.3f;
    in.v_est_m_s = 0.3f;
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_CONTROLLED_STOP);
    CHECK(out.reason == SAFETY_REASON_S11_DEGRADED);
    CHECK(out.torque_enable == 1u && out.can_disable_cmd == 0u && out.can_abort_tx == 0u);
    CHECK(out.inhibit_jump == 1u && out.inhibit_trim == 1u);
    CHECK(out.host_lost == 1u);              /* S9 brake-position mode */
    /* brake ramp then rest dwell */
    unsigned t = 0u;
    while (out.state == SAFETY_STATE_CONTROLLED_STOP && t < 2000u) {
        in = live_inputs(S11_DEGRADED);
        in.v_target_m_s = 0.3f;             /* host command ignored during the stop */
        in.v_est_m_s = (t < 300u) ? 0.3f * (1.0f - (float)t / 300.0f) : 0.0f;
        safety_fsm_step(&fsm, &in, &out);
        CHECK(out.torque_enable == 1u);
        CHECK(out.v_ref_m_s <= 0.3f + 1e-6f);
        t += 2u;
    }
    CHECK(out.state == SAFETY_STATE_REST && out.torque_enable == 1u);
    CHECK(t <= 1000u);                       /* REST within 1 s */
    /* re-grant refused while the axis stays degraded */
    in = live_inputs(S11_DEGRADED);
    in.local_regrant_req = 1u;
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_REST);
    /* clean disarm from REST still works */
    in = live_inputs(S11_DEGRADED);
    in.radio_arm_sw = 0u;
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_DISARMED);
}

static void test_f2_degraded_pitch_escalates(void)
{
    SafetyFsm fsm;
    SafetyOutput out;
    reach_balancing(&fsm, &out);
    SafetyInputs in = live_inputs(S11_DEGRADED);
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_CONTROLLED_STOP);
    in.pitch_deg = 14.0f;
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_CONTROLLED_STOP);
    in.pitch_deg = 15.5f;
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_FAULT && out.reason == SAFETY_REASON_S11_FAULT);
    CHECK(out.torque_enable == 0u && out.can_disable_cmd == 1u);

    /* same in REST */
    reach_balancing(&fsm, &out);
    in = live_inputs(S11_DEGRADED);
    for (unsigned t = 0u; t < 1000u && out.state != SAFETY_STATE_REST; ++t) {
        safety_fsm_step(&fsm, &in, &out);
    }
    CHECK(out.state == SAFETY_STATE_REST);
    in.pitch_deg = -16.0f;
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_FAULT && out.torque_enable == 0u);

    /* a non-degraded controlled stop does not use the 15 deg escalation */
    reach_balancing(&fsm, &out);
    in = live_inputs(S11_FRESH);
    in.voluntary_stop_req = 1u;
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_CONTROLLED_STOP);
    in.pitch_deg = 16.0f;
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_CONTROLLED_STOP);
}

static void test_f3_torque_off_paths_kept(void)
{
    SafetyFsm fsm;
    SafetyOutput out;
    for (unsigned degraded = 0u; degraded < 2u; ++degraded) {
        const enum S11Mode m = degraded != 0u ? S11_DEGRADED : S11_FRESH;
        /* e-stop */
        reach_balancing(&fsm, &out);
        SafetyInputs in = live_inputs(m);
        in.estop_open = 1u;
        safety_fsm_step(&fsm, &in, &out);
        CHECK(out.state == SAFETY_STATE_ESTOP && out.torque_enable == 0u);
        CHECK(out.can_abort_tx == 1u && out.can_disable_cmd == 1u);
        /* drive fault (status nibble) */
        reach_balancing(&fsm, &out);
        in = live_inputs(m);
        in.drives_healthy_6 = 0u;
        safety_fsm_step(&fsm, &in, &out);
        CHECK(out.state == SAFETY_STATE_FAULT && out.reason == SAFETY_REASON_DRIVE_FAULT);
        CHECK(out.torque_enable == 0u);
        reach_balancing(&fsm, &out);
        in = live_inputs(S11_DRIVE_FAULT);
        safety_fsm_step(&fsm, &in, &out);
        CHECK(out.state == SAFETY_STATE_FAULT && out.torque_enable == 0u);
        /* cannot balance: tilt >= 30 deg for 20 ms */
        reach_balancing(&fsm, &out);
        in = live_inputs(m);
        in.pitch_deg = 31.0f;
        for (unsigned t = 0u; t < 12u && out.state != SAFETY_STATE_FAULT; ++t) {
            safety_fsm_step(&fsm, &in, &out);
        }
        /* degraded: the 15 deg degraded-stop escalation fires before the 30 deg
         * tilt dwell; both are torque-off FAULTs */
        CHECK(out.state == SAFETY_STATE_FAULT);
        CHECK(out.reason == SAFETY_REASON_TILT_LIMIT
              || (degraded != 0u && out.reason == SAFETY_REASON_S11_FAULT));
        CHECK(out.torque_enable == 0u && out.can_disable_cmd == 1u);
    }
}

static void test_f4_double_fault_torque_off(void)
{
    SafetyFsm fsm;
    SafetyOutput out;
    reach_balancing(&fsm, &out);
    SafetyInputs in = live_inputs(S11_STUCK);
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_FAULT && out.reason == SAFETY_REASON_S11_FAULT);
    CHECK(out.torque_enable == 0u && out.can_disable_cmd == 1u);
    /* from a degraded stop as well */
    reach_balancing(&fsm, &out);
    in = live_inputs(S11_DEGRADED);
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_CONTROLLED_STOP);
    in = live_inputs(S11_STUCK);
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_FAULT && out.torque_enable == 0u);
}

static void test_f5_step_assist_degraded(void)
{
    SafetyFsm fsm;
    SafetyOutput out;
    reach_balancing(&fsm, &out);
    SafetyInputs in = live_inputs(S11_FRESH);
    in.v_target_m_s = 0.1f;
    in.step_assist_req = 1u;
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_STEP_ASSIST);
    in = live_inputs(S11_DEGRADED);
    in.v_target_m_s = 0.1f;
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_CONTROLLED_STOP);
    CHECK(out.reason == SAFETY_REASON_S11_DEGRADED && out.torque_enable == 1u);
}

static void test_f6_unarmed_states(void)
{
    SafetyFsm fsm;
    SafetyOutput out;
    reach_armed(&fsm, &out);
    SafetyInputs in = live_inputs(S11_DEGRADED);
    in.stand_handoff_req = 1u;
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_FAULT && out.torque_enable == 0u);

    safety_fsm_init(&fsm);
    in = base_inputs();
    in.local_reset_req = 1u;
    in.local_arm_req = 1u;
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_PRECHARGE);
    in = live_inputs(S11_DEGRADED);
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_FAULT);
}

int main(void)
{
    test_p1_p2_single_wheel_partner_substitution();
    test_p3_single_hip_holds_geometry();
    test_p3b_hip_mirror_hold_and_fallback();
    test_p4_double_faults_escalate();
    test_p5_non_escalating_pairs();
    test_p6_simultaneous_gross();
    test_p7_imu_gate();
    test_p8_freeze_while_arming();
    test_p9_detector_disabled();
    test_p10_advisory_flag_only();
    test_p11_healthy_traffic();
    test_f1_degraded_controlled_stop_to_rest();
    test_f2_degraded_pitch_escalates();
    test_f3_torque_off_paths_kept();
    test_f4_double_fault_torque_off();
    test_f5_step_assist_degraded();
    test_f6_unarmed_states();
    printf("test_s11_frozen_reaction: 17 groups PASSED\n");
    return 0;
}
