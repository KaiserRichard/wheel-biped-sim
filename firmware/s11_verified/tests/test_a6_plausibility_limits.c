#include "test_common.h"

/* REVIEW A6: Saturated codes (+/-12.5) must be rejected (>= not >), hip joint
 * limits tighter than full code range must be enforced, and rate limits must
 * track ~1.2x no-load speed at the bus voltage (25.08 / 40.20 rad/s at 24 V). */
int main(void)
{
    S11PacketPolicy st;
    S11PacketFrame frame;
    double angle[S11_AXES] = {0.2, -0.2, 0.5, 0.2, -0.2, -0.5};
    double rate[S11_AXES]  = {0.0,  0.0, 0.0, 0.0,  0.0,  0.0};
    double age[S11_AXES];
    fill_arr(age, 1.0);

    /* Case 1: Saturated hip angle +12.5 rad must be rejected (orig accepts it). */
    arm_policy(&st, 1.0, angle, rate, &frame);
    double bad_angle[S11_AXES];
    memcpy(bad_angle, angle, sizeof bad_angle);
    bad_angle[0] = 12.5;
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, bad_angle, rate, &frame);
    assert(frame.mode != S11_FRESH);
    assert(near_eq(frame.angle[0], 0.2, 1e-12));

    /* Case 2: Hip rate 28.0 rad/s (> 1.2 * 20.9 = 25.08, < orig's 30.0) rejected. */
    arm_policy(&st, 1.0, angle, rate, &frame);
    double bad_rate[S11_AXES];
    memcpy(bad_rate, rate, sizeof bad_rate);
    bad_rate[0] = 28.0;
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, bad_rate, &frame);
    assert(frame.mode != S11_FRESH);

    /* Case 3: Wheel rate 42.0 rad/s (> 1.2 * 33.5 = 40.20, < orig's 45.0) rejected. */
    arm_policy(&st, 1.0, angle, rate, &frame);
    memcpy(bad_rate, rate, sizeof bad_rate);
    bad_rate[2] = 42.0;
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, bad_rate, &frame);
    assert(frame.mode != S11_FRESH);

#ifndef S11_TEST_ORIG
    /* Case 4: Hip mechanical limit (default 3.5 rad) rejects 5.0 rad, and
     * voltage scaling at 18 V scales hip limit to 18/24 * 25.08 = 18.81 rad/s. */
    arm_policy(&st, 1.0, angle, rate, &frame);
    memcpy(bad_angle, angle, sizeof bad_angle);
    bad_angle[1] = 5.0;
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, bad_angle, rate, &frame);
    assert(frame.mode == S11_PREDICT);

    assert(s11_set_bus_voltage(&st, 18.0) == 0);
    assert(near_eq(st.config.hip_rate_limit_rad_s, 18.81, 1e-9));
    assert(near_eq(st.config.wheel_rate_limit_rad_s, 30.15, 1e-9));
    memcpy(bad_rate, rate, sizeof bad_rate);
    bad_rate[0] = 20.0; /* valid at 24 V, invalid at 18 V */
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, bad_rate, &frame);
    assert(frame.mode != S11_FRESH);
#endif

    printf("PASS: test_a6_plausibility_limits\n");
    return 0;
}
