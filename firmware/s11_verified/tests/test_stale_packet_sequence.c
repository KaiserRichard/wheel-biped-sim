#include "test_common.h"

/* Stale packet test: verifies rejection of (1) duplicate acquisition time
 * age_ms == stored + dt, (2) older/out-of-order acquisition time age_ms > stored + dt,
 * (3) excessive transport age (> 12 ms), and (4) non-advancing sequence counter. */
int main(void)
{
    S11PacketPolicy st;
    S11PacketFrame frame;
    double angle[S11_AXES] = {0.8, 0.2, 0.3, -0.2, -0.3, 0.1};
    double rate[S11_AXES]  = {0.0, 0.0, 0.0,  0.0,  0.0, 0.0};
    double age[S11_AXES];
    fill_arr(age, 2.0);

    /* Case 1: Duplicate acquisition time (stored=2.0 + dt=2.0 => 4.0 ms). */
    arm_policy(&st, 2.0, angle, rate, &frame);
    double stale_angle[S11_AXES];
    double stale_age[S11_AXES];
    memcpy(stale_angle, angle, sizeof stale_angle);
    memcpy(stale_age, age, sizeof stale_age);
    stale_angle[0] = 0.1;
    stale_age[0] = 4.0; /* exact same acquisition instant as previous tick */
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, stale_age, stale_angle, rate, &frame);
    assert(frame.mode == S11_PREDICT);
    assert(near_eq(frame.angle[0], 0.8, 1e-12));

    /* Case 2: Strictly older acquisition time (age = 6.5 > 4.0 + 2.0). */
    stale_age[0] = 6.5;
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, stale_age, stale_angle, rate, &frame);
    assert(frame.mode == S11_PREDICT);
    assert(near_eq(frame.angle[0], 0.8, 1e-12));

    /* Case 3: Excessive transport age (> 12 ms budget) rejected on arrival. */
    arm_policy(&st, 1.0, angle, rate, &frame);
    memcpy(stale_age, age, sizeof stale_age);
    stale_age[1] = 15.0;
    stale_angle[1] = -1.0;
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, stale_age, stale_angle, rate, &frame);
    assert(frame.mode != S11_FRESH);
    assert(near_eq(frame.angle[1], 0.2, 1e-12));

#ifndef S11_TEST_ORIG
    /* Case 4: Non-advancing seq[] on a single axis treated as stale miss. */
    uint32_t seq[S11_AXES] = {1, 1, 1, 1, 1, 1};
    s11_init(&st);
    assert(s11_arm(&st) == 1);
    for (unsigned k = 0; k < 3u; ++k) {
        for (unsigned i = 0; i < S11_AXES; ++i)
            seq[i] += 1u;
        s11_packets_ex(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate,
                       2.0, seq, &frame);
    }
    assert(frame.mode == S11_FRESH);

    /* Keep seq[3] constant while claiming fresh=ALL_AXES and different angle. */
    for (unsigned i = 0; i < S11_AXES; ++i)
        if (i != 3u)
            seq[i] += 1u;
    stale_angle[3] = 0.99;
    s11_packets_ex(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, stale_angle, rate,
                   2.0, seq, &frame);
    assert(frame.mode == S11_PREDICT);
    assert(near_eq(frame.angle[3], -0.2, 1e-12));
#endif

    printf("PASS: test_stale_packet_sequence\n");
    return 0;
}
