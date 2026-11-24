#include "test_common.h"

/* REVIEW A1: A latched call must not return a frame of zeros (angle=rate=0,
 * max_misses=0). It must output NAN, preserve max_misses, set valid=0, and
 * store mode as enum S11Mode. */
int main(void)
{
    S11PacketPolicy st;
    S11PacketFrame frame;
    double angle[S11_AXES] = {0.5, -0.4, 1.2, 0.3, -0.6, -1.1};
    double rate[S11_AXES]  = {0.0,  0.0, 0.0, 0.0,  0.0,  0.0};
    double age[S11_AXES];
    fill_arr(age, 1.0);

    arm_policy(&st, 1.0, angle, rate, &frame);

    /* Drive into TIMEOUT latch via consecutive missed packets. */
    for (unsigned k = 0; k < 6u; ++k) {
        s11_packets(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
        if (frame.mode == S11_TIMEOUT)
            break;
    }
    assert(frame.mode == S11_TIMEOUT);
    assert(near_eq(frame.authority, 0.0, 1e-12));
    const unsigned latched_misses = frame.max_misses;
    assert(latched_misses >= 4u);

    /* Call again while already latched (orig returned all zeros here). */
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_TIMEOUT);
    assert(near_eq(frame.authority, 0.0, 1e-12));
    assert(frame.max_misses == latched_misses);
    for (unsigned i = 0; i < S11_AXES; ++i) {
        assert(isnan(frame.angle[i]));
        assert(isnan(frame.rate[i]));
    }
#ifndef S11_TEST_ORIG
    assert(frame.valid == 0);
    enum S11Mode m = frame.mode;
    assert(m == S11_TIMEOUT);
    for (unsigned i = 0; i < S11_AXES; ++i)
        assert(isnan(frame.unwrapped_angle[i]));
#endif

    printf("PASS: test_a1_latched_frame\n");
    return 0;
}
