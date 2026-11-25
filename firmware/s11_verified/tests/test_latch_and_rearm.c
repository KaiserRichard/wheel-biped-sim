#include "test_common.h"

/* Latch & re-arm test: every fault latch (TIMEOUT, INVALID, DRIVE_FAULT, STUCK)
 * must stay permanently latched across valid traffic and bare s11_arm() calls,
 * clear only on explicit s11_reset(), return to DISARMED, and require s11_arm()
 * plus N consecutive fresh valid 6-axis groups to regain authority == 1.0. */
static void verify_latch_and_rearm(S11PacketPolicy *st, enum S11Mode expected_latch)
{
    S11PacketFrame frame;
    double angle[S11_AXES] = {0.1, -0.1, 0.2, 0.1, -0.1, -0.2};
    double rate[S11_AXES]  = {0.0,  0.0, 0.0, 0.0,  0.0,  0.0};
    double age[S11_AXES];
    fill_arr(age, 1.0);

    /* Good traffic alone must NOT clear the latch. */
    for (unsigned k = 0; k < 5u; ++k) {
        s11_packets(st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
        assert(frame.mode == expected_latch);
        assert(near_eq(frame.authority, 0.0, 1e-12));
        assert(frame.valid == 0);
        for (unsigned i = 0; i < S11_AXES; ++i)
            assert(isnan(frame.angle[i]) && isnan(frame.rate[i]));
    }

    /* Bare s11_arm() while latched must be rejected (returns 0). */
    assert(s11_arm(st) == 0);
    s11_packets(st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == expected_latch);
    assert(frame.valid == 0);

    /* Explicit operator s11_reset() clears latch -> DISARMED (still authority 0). */
    s11_reset(st);
    s11_packets(st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_DISARMED);
    assert(near_eq(frame.authority, 0.0, 1e-12));
    assert(frame.valid == 0);

    /* Explicit s11_arm() enters ARMING -> needs 3 consecutive full valid groups. */
    assert(s11_arm(st) == 1);
    s11_packets(st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_ARMING && frame.valid == 0);
    /* Interrupt arming with a partial tick -> resets arm counter to 0! */
    s11_packets(st, 0x3Eu, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_ARMING && frame.valid == 0);

    /* Now 3 consecutive good ticks -> armed! */
    s11_packets(st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_ARMING && frame.valid == 0);
    s11_packets(st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_ARMING && frame.valid == 0);
    s11_packets(st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_FRESH);
    assert(near_eq(frame.authority, 1.0, 1e-12));
    assert(frame.valid == 1);
}

int main(void)
{
    S11PacketPolicy st;
    S11PacketFrame frame;
    double angle[S11_AXES] = {0.2, -0.2, 0.3, 0.2, -0.2, -0.3};
    double rate[S11_AXES]  = {0.0,  0.0, 0.0, 0.0,  0.0,  0.0};
    double age[S11_AXES];
    fill_arr(age, 1.0);

    /* 1. TIMEOUT latch -> re-arm */
    arm_policy(&st, 1.0, angle, rate, &frame);
    for (unsigned k = 0; k < 5u; ++k)
        s11_packets(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_TIMEOUT);
    verify_latch_and_rearm(&st, S11_TIMEOUT);

    /* 2. INVALID latch -> re-arm */
    for (unsigned k = 0; k < 3u; ++k)
        s11_packets(&st, ALL_AXES, 0x3Eu, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_INVALID);
    verify_latch_and_rearm(&st, S11_INVALID);

    /* 3. DRIVE_FAULT latch -> re-arm */
    s11_packets(&st, ALL_AXES, ALL_AXES, 0x3Eu, age, angle, rate, &frame);
    assert(frame.mode == S11_DRIVE_FAULT);
    verify_latch_and_rearm(&st, S11_DRIVE_FAULT);

    /* 4. FW6: a single frozen axis is DEGRADED (valid frame, latched until
     *    s11_reset); a double fault (both hips of one leg) latches STUCK. */
    double one_rate[S11_AXES] = {2.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    arm_policy(&st, 1.0, angle, one_rate, &frame);
    for (unsigned k = 0; k < 3u; ++k)
        s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, one_rate, &frame);
    assert(frame.mode == S11_DEGRADED && frame.valid == 1 && frame.degraded_mask == 0x01u);
    for (unsigned k = 0; k < 5u; ++k)
        s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_DEGRADED);           /* latched across good traffic */
    s11_reset(&st);

    /* 5. STUCK latch (double fault) -> re-arm */
    double mov_rate[S11_AXES] = {2.0, 2.0, 0.0, 0.0, 0.0, 0.0};
    arm_policy(&st, 1.0, angle, mov_rate, &frame);
    for (unsigned k = 0; k < 3u; ++k)
        s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, mov_rate, &frame);
    assert(frame.mode == S11_STUCK);
    verify_latch_and_rearm(&st, S11_STUCK);

    printf("PASS: test_latch_and_rearm\n");
    return 0;
}
