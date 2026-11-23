#include "test_common.h"

/* REVIEW A2: State machine must have DISARMED/ARMING states, must not latch
 * TIMEOUT on a partial first tick, must require N consecutive fresh valid
 * 6-axis groups before authority > 0, and must support explicit s11_reset(). */
int main(void)
{
    S11PacketPolicy st;
    S11PacketFrame frame;
    double angle[S11_AXES] = {0.2, 0.2, 0.2, 0.2, 0.2, 0.2};
    double rate[S11_AXES]  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double age[S11_AXES];
    fill_arr(age, 1.0);

#ifdef S11_TEST_ORIG
    memset(&st, 0, sizeof st);
#else
    /* Zero-initialized policy starts DISARMED with authority == 0. */
    memset(&st, 0, sizeof st);
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_DISARMED);
    assert(near_eq(frame.authority, 0.0, 1e-12));
    assert(frame.valid == 0);

    assert(s11_arm(&st) == 1);
#endif

    /* Tick 1: partial 5-of-6 axes fresh (axis 0 not yet arrived).
     * orig immediately latches permanent S11_TIMEOUT here! */
    s11_packets(&st, 0x3Eu, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(near_eq(frame.authority, 0.0, 1e-12));
#ifndef S11_TEST_ORIG
    assert(frame.mode == S11_ARMING);
    assert(frame.valid == 0);
#endif

    /* Now supply 3 consecutive full 6-axis fresh, valid, healthy groups. */
    for (unsigned k = 0; k < 3u; ++k) {
        s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
#ifndef S11_TEST_ORIG
        if (k < 2u) {
            assert(frame.mode == S11_ARMING);
            assert(near_eq(frame.authority, 0.0, 1e-12));
            assert(frame.valid == 0);
        }
#endif
    }
    /* Must now be armed with full authority (orig stays permanently latched TIMEOUT). */
    assert(frame.mode == S11_FRESH);
    assert(near_eq(frame.authority, 1.0, 1e-12));
#ifndef S11_TEST_ORIG
    assert(frame.valid == 1);

    /* Explicit reset returns to DISARMED cleanly. */
    s11_reset(&st);
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_DISARMED);
    assert(near_eq(frame.authority, 0.0, 1e-12));
    assert(frame.valid == 0);
#endif

    printf("PASS: test_a2_arming_state_machine\n");
    return 0;
}
