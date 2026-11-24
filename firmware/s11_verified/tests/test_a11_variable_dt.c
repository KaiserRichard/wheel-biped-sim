#include "test_common.h"

/* REVIEW A11: Calling at rates other than 500 Hz (e.g., 1 kHz dt=1 ms bridge
 * or 250 Hz dt=4 ms) must compute ages, hold modes, and extrapolation from
 * actual dt_ms rather than hard-coded 2 ms. Invalid dt_ms must fail safe. */
int main(void)
{
    S11PacketPolicy st;
    S11PacketFrame frame;
    double angle[S11_AXES] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double rate[S11_AXES]  = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double age[S11_AXES];
    fill_arr(age, 0.5);

#ifdef S11_TEST_ORIG
    /* orig hard-codes 2 ms/tick; running 4 missed ticks at 1 kHz (4 ms elapsed)
     * wrongly thinks 8 ms / 4 misses elapsed and latches S11_TIMEOUT! */
    arm_policy(&st, 0.5, angle, rate, &frame);
    for (unsigned k = 0; k < 4u; ++k)
        s11_packets(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_PREDICT);
    assert(near_eq(frame.angle[0], 0.004, 1e-9));
#else
    /* 1 kHz (dt = 1.0 ms): 4 missed ticks = 4 ms -> PREDICT, +0.004 rad. */
    s11_init(&st);
    assert(s11_arm(&st) == 1);
    for (unsigned k = 0; k < 3u; ++k) {
        double a[S11_AXES] = {0.0 - (2.0 - (double)k) * 0.001, 0, 0, 0, 0, 0};
        s11_packets_ex(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, a, rate, 1.0, NULL, &frame);
    }
    assert(frame.mode == S11_FRESH && near_eq(frame.angle[0], 0.0, 1e-12));

    for (unsigned k = 1; k <= 4u; ++k) {
        s11_packets_ex(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, 1.0, NULL, &frame);
        assert(frame.mode == S11_PREDICT);
        assert(near_eq(frame.angle[0], 0.001 * (double)k, 1e-9));
    }
    /* Ticks 5..9 (5..9 ms) -> LIMIT (authority 0.75). */
    for (unsigned k = 5; k <= 9u; ++k) {
        s11_packets_ex(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, 1.0, NULL, &frame);
        assert(frame.mode == S11_LIMIT);
        assert(near_eq(frame.authority, 0.75, 1e-12));
    }
    /* Tick 10 (10 ms) -> TIMEOUT latch. */
    s11_packets_ex(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, 1.0, NULL, &frame);
    assert(frame.mode == S11_TIMEOUT);
    assert(near_eq(frame.authority, 0.0, 1e-12));

    /* 250 Hz (dt = 4.0 ms): 1 miss = 4 ms (PREDICT), 2 = 8 ms (LIMIT), 3 = 12 ms (TIMEOUT). */
    s11_reset(&st);
    assert(s11_arm(&st) == 1);
    for (unsigned k = 0; k < 3u; ++k) {
        double a[S11_AXES] = {0.0 - (2.0 - (double)k) * 0.004, 0, 0, 0, 0, 0};
        s11_packets_ex(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, a, rate, 4.0, NULL, &frame);
    }
    s11_packets_ex(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, 4.0, NULL, &frame);
    assert(frame.mode == S11_PREDICT && near_eq(frame.angle[0], 0.004, 1e-9));
    s11_packets_ex(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, 4.0, NULL, &frame);
    assert(frame.mode == S11_LIMIT && near_eq(frame.angle[0], 0.008, 1e-9));
    s11_packets_ex(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, 4.0, NULL, &frame);
    assert(frame.mode == S11_TIMEOUT && frame.valid == 0);

    /* Invalid dt_ms (0, negative, NAN, INF) must fail safe. */
    const double bad_dts[4] = {0.0, -2.0, NAN, INFINITY};
    for (unsigned j = 0; j < 4u; ++j) {
        arm_policy(&st, 1.0, angle, rate, &frame);
        s11_packets_ex(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate,
                       bad_dts[j], NULL, &frame);
        assert(frame.mode == S11_INVALID);
        assert(frame.valid == 0);
        assert(near_eq(frame.authority, 0.0, 1e-12));
    }
#endif

    printf("PASS: test_a11_variable_dt\n");
    return 0;
}
