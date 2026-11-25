#ifdef NDEBUG
#undef NDEBUG
#endif
#include "s11_packet_policy.h"

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

static const uint32_t ALL_AXES = 0x3Fu;

static void fill6(double dst[S11_AXES], double v)
{
    for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
        dst[i] = v;
    }
}

static void arm_policy_clean(S11PacketPolicy *st, double angle, double rate, double age_ms)
{
    double a[S11_AXES], r[S11_AXES], age[S11_AXES];
    S11PacketFrame frame;

    s11_init(st);
    assert(s11_arm(st) == 1);
    for (uint32_t k = 0u; k < (uint32_t)S11_ARM_GROUPS; ++k) {
        fill6(a, angle + 0.01 * (double)k);
        fill6(r, rate);
        fill6(age, age_ms);
        s11_packets(st, ALL_AXES, ALL_AXES, ALL_AXES, age, a, r, &frame);
        if (k + 1u < (uint32_t)S11_ARM_GROUPS) {
            assert(frame.mode == S11_ARMING);
            assert(frame.valid == 0u && frame.authority == 0.0);
        } else {
            assert(frame.mode == S11_FRESH);
            assert(frame.valid == 1u && frame.authority == 1.0);
        }
    }
}

static void test_arming_miss_ladder_and_latch(void)
{
    S11PacketPolicy st;
    S11PacketFrame frame;
    double a[S11_AXES], r[S11_AXES], age[S11_AXES];

    /* Verify derived age-budget defaults (6.0 ms arrival, 10.0 ms hold cut, 16.0 ms total) */
    const S11Config cfg = s11_default_config();
    assert(fabs(cfg.max_transport_age_ms - 6.0) < 1e-12);
    assert(fabs(cfg.predict_hold_ms - 4.0) < 1e-12);
    assert(fabs(cfg.cut_hold_ms - 10.0) < 1e-12);
    assert(fabs(cfg.max_total_age_ms - 16.0) < 1e-12);
    assert(fabs(cfg.max_predict_ms - 10.0) < 1e-12);

    /* Zero-init starts safely DISARMED */
    memset(&st, 0, sizeof(st));
    fill6(a, 0.3); fill6(r, 1.0); fill6(age, 1.0);
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, a, r, &frame);
    assert(frame.mode == S11_DISARMED && frame.valid == 0u && frame.authority == 0.0);
    assert(isnan(frame.angle[0]));

    /* Arm cleanly at worst-case valid arrival age 5.9 ms (inside 6.0 ms budget, A3) */
    arm_policy_clean(&st, 0.3, 1.0, 5.9);

    /* Drop axis 2 (left wheel): misses 1-2 PREDICT (1.0), misses 3-4 LIMIT (0.75), miss 5 TIMEOUT */
    const uint32_t drop_w = ALL_AXES ^ (1u << 2u);
    for (uint32_t m = 1u; m <= 5u; ++m) {
        fill6(a, 0.32 + 0.002 * (double)m);
        fill6(r, 1.0);
        fill6(age, 5.9);
        s11_packets(&st, drop_w, ALL_AXES, ALL_AXES, age, a, r, &frame);
        if (m <= 2u) {
            assert(frame.mode == S11_PREDICT && frame.authority == 1.0 && frame.valid == 1u);
        } else if (m <= 4u) {
            assert(frame.mode == S11_LIMIT && frame.authority == 0.75 && frame.valid == 1u);
            /* Extrapolates across full LIMIT window (A12) */
            assert(fabs(frame.angle[2] - (0.32 + 0.002 * (double)m)) < 1e-9);
        } else {
            assert(frame.mode == S11_TIMEOUT && frame.authority == 0.0 && frame.valid == 0u);
            assert(frame.max_misses == 5u);
            assert(isnan(frame.angle[2]));
        }
    }

    /* Latch persists through good traffic and bare s11_arm() until explicit s11_reset() */
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, a, r, &frame);
    assert(frame.mode == S11_TIMEOUT && frame.max_misses == 5u && frame.valid == 0u);
    assert(s11_arm(&st) == 0);

    s11_reset(&st);
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, a, r, &frame);
    assert(frame.mode == S11_DISARMED);
}

static void test_invalid_stuck_precedence_and_wrap(void)
{
    S11PacketPolicy st;
    S11PacketFrame frame;
    double a[S11_AXES], r[S11_AXES], age[S11_AXES];

    /* 1. Isolated NaN/invalid degrades to PREDICT, 3 consecutive latch INVALID (A8) */
    arm_policy_clean(&st, 0.2, 0.5, 1.0);
    fill6(a, 0.23); fill6(r, 0.5); fill6(age, 1.0);
    a[0] = NAN;
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, a, r, &frame);
    assert(frame.mode == S11_PREDICT && frame.valid == 1u);
    assert(isfinite(frame.angle[0]));

    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, a, r, &frame);
    assert(frame.mode == S11_PREDICT);

    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, a, r, &frame);
    assert(frame.mode == S11_INVALID && frame.valid == 0u);

    /* 2. Drive health precedence overrides everything immediately (A12) */
    arm_policy_clean(&st, 0.2, 0.5, 1.0);
    fill6(a, 0.25); fill6(r, 0.5); fill6(age, 1.0);
    s11_packets(&st, ALL_AXES, 0u, ALL_AXES ^ 1u, age, a, r, &frame);
    assert(frame.mode == S11_DRIVE_FAULT && frame.valid == 0u);

    /* 3a. Moving frozen angle at 1.0 rad/s triggers STUCK within 10 ms (A4) */
    arm_policy_clean(&st, 0.5, 1.0, 1.0);
    fill6(a, 0.52); /* Same angle as last arming tick */
    fill6(r, 1.0);
    fill6(age, 1.0);
    uint8_t stuck_hit = 0u;
    for (uint32_t t = 0u; t < 6u; ++t) {
        s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, a, r, &frame);
        if (frame.mode == S11_STUCK) {
            stuck_hit = 1u;
            break;
        }
    }
    assert(stuck_hit == 1u);

    /* 3b. Slow frozen angle at 0.1 rad/s (< 0.12 rad/s threshold) with advancing seq[]
     *     accumulates net displacement > 2 LSB and latches S11_STUCK (round-2 fix 1). */
    s11_init(&st);
    assert(s11_arm(&st) == 1);
    fill6(a, 0.2);
    fill6(r, 0.0);
    r[0] = 0.1;
    fill6(age, 1.0);
    uint32_t seq[S11_AXES] = { 0u, 0u, 0u, 0u, 0u, 0u };
    for (uint32_t k = 0u; k < 103u; ++k) {
        for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
            seq[i] += 1u;
        }
        s11_packets_ex(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, a, r, 2.0, seq, &frame);
    }
    /* FW6: single frozen hip is CONFIRMED by NET evidence -> DEGRADED, frame
     * valid, hip geometry held with zero rate. */
    assert(frame.mode == S11_DEGRADED && frame.valid == 1u);
    assert(frame.degraded_mask == 0x01u && frame.rate[0] == 0.0);

    /* 4. Wheel angle wrap and continuous unwrap across +/-12.5 rad (A7) */
    assert(fabs(s11_wrap_angle(12.6, 12.5) - (-12.4)) < 1e-9);
    assert(fabs(s11_unwrap_delta(12.4, -12.4, 12.5) - 0.2) < 1e-9);

    /* 5. Bus voltage rate scaling (A6) */
    arm_policy_clean(&st, 0.1, 0.0, 1.0);
    assert(s11_set_bus_voltage(&st, 18.0) == 0);
    assert(st.config.hip_rate_limit_rad_s < 20.0);

    /* 6. Wrap-safe timer age helpers (A9) */
    assert(fabs(s11_age_ms_u32(10u, 0xFFFFFFF6u, 10.0) - 2.0) < 1e-9);
    assert(fabs(s11_age_ms_u16(5u, 65531u, 5.0) - 2.0) < 1e-9);
}

static void test_isr_mailbox_and_variable_dt(void)
{
    S11IsrMailbox mb;
    S11InputSnapshot snap;
    S11PacketPolicy st;
    S11PacketFrame frame;

    s11_mailbox_init(&mb);
    s11_init(&st);
    assert(s11_arm(&st) == 1);

    /* Partial mailbox startup (A2 round-2 fix 2): 5 axes written, axis 0 unwritten ->
     * status_unknown bit 0 set -> stays S11_ARMING (never latches S11_DRIVE_FAULT). */
    for (uint32_t i = 1u; i < (uint32_t)S11_AXES; ++i) {
        s11_mailbox_write_axis(&mb, i, 1u, 1u, 500u, 0.1, 0.0);
    }
    assert(s11_mailbox_snapshot(&mb, 700u, 1000.0, &snap) == 0u);
    assert((snap.status_unknown & 1u) != 0u);
    s11_packets_snapshot(&st, &snap, 1.0, &frame);
    assert(frame.mode == S11_ARMING && frame.valid == 0u);

    /* Write 3 consecutive 6-axis groups at 1 kHz (dt_ms = 1.0) through mailbox */
    for (uint32_t k = 0u; k < 3u; ++k) {
        const uint32_t stamp = 1000u + k * 1000u;
        for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
            s11_mailbox_write_axis(&mb, i, 1u, 1u, stamp, 0.1 + 0.02 * (double)k, 0.5);
        }
        const uint32_t torn = s11_mailbox_snapshot(&mb, stamp + 200u, 1000.0, &snap);
        assert(torn == 0u);
        assert(snap.status_unknown == 0u);
        assert(snap.fresh == ALL_AXES);
        s11_packets_snapshot(&st, &snap, 1.0, &frame);
    }
    assert(frame.mode == S11_FRESH && frame.valid == 1u);

    /* No new ISR writes -> seq unchanged -> 4 ticks at 1 ms = 4 ms -> still PREDICT (A11) */
    for (uint32_t t = 0u; t < 4u; ++t) {
        const uint32_t torn = s11_mailbox_snapshot(&mb, 4000u + t * 1000u, 1000.0, &snap);
        assert(torn == 0u && snap.fresh == 0u);
        s11_packets_snapshot(&st, &snap, 1.0, &frame);
        assert(frame.mode == S11_PREDICT);
    }

    /* Generation wrap at UINT32_MAX -> 0 preserves freshness & health (round-2 fix 4) */
    atomic_store(&mb.axis[0].gen, UINT32_MAX - 1u);
    for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
        s11_mailbox_write_axis(&mb, i, 1u, 1u, 8000u, 0.25, 0.0);
    }
    assert(s11_mailbox_snapshot(&mb, 8200u, 1000.0, &snap) == 0u);
    assert((snap.fresh & 1u) != 0u && (snap.healthy & 1u) != 0u);
    s11_packets_snapshot(&st, &snap, 1.0, &frame);
    assert(frame.mode == S11_FRESH && frame.valid == 1u);
}

int main(void)
{
    test_arming_miss_ladder_and_latch();
    test_invalid_stuck_precedence_and_wrap();
    test_isr_mailbox_and_variable_dt();
    printf("test_s11_packet_policy: ALL PASSED\n");
    return 0;
}
