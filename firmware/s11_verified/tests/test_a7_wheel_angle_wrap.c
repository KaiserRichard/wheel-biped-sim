#include "test_common.h"

/* REVIEW A7: Wheel angle must be bounds-checked, predicted wheel angle must
 * wrap into [-12.5, +12.5), and unwrapped wheel angle must be continuous across
 * the wrap without a ~25 rad jump. */
int main(void)
{
    S11PacketPolicy st;
    S11PacketFrame frame;
    double angle[S11_AXES] = {0.1, -0.1, 12.48, 0.1, -0.1, 0.0};
    double rate[S11_AXES]  = {0.0,  0.0, 20.0,  0.0,  0.0, 0.0};
    double age[S11_AXES];
    fill_arr(age, 1.0);

    /* Case 1: Out-of-range wheel angle (50.0 rad) must be rejected (orig accepts). */
    arm_policy(&st, 1.0, angle, rate, &frame);
    double bad_angle[S11_AXES];
    memcpy(bad_angle, angle, sizeof bad_angle);
    bad_angle[2] = 50.0;
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, bad_angle, rate, &frame);
    assert(frame.mode != S11_FRESH);

    /* Case 2: Wheel at +12.48 rad, rate +20.0 rad/s. On 1 missed tick (2 ms),
     * raw extrapolation is 12.48 + 0.04 = 12.52 rad -> must wrap to -12.48 rad
     * (orig outputs 12.52 rad unwrapped outside [-12.5, +12.5)). */
    arm_policy(&st, 1.0, angle, rate, &frame);
    s11_packets(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_PREDICT);
    assert(frame.angle[2] >= -12.5 && frame.angle[2] < 12.5);
    assert(near_eq(frame.angle[2], -12.48, 1e-9));

#ifndef S11_TEST_ORIG
    /* Case 3: Unwrapped wheel angle stays continuous across +12.48 -> -12.44 wrap. */
    assert(near_eq(frame.unwrapped_angle[2], 12.52, 1e-9));

    double next_angle[S11_AXES] = {0.1, -0.1, -12.44, 0.1, -0.1, 0.0};
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, next_angle, rate, &frame);
    assert(frame.mode == S11_FRESH);
    assert(near_eq(frame.angle[2], -12.44, 1e-9));
    /* Delta from +12.48 to -12.44 modulo 25.0 is +0.08 -> unwrapped = 12.56 rad */
    assert(near_eq(frame.unwrapped_angle[2], 12.56, 1e-9));
    assert(near_eq(s11_unwrap_delta(12.45, -12.45, 12.5), 0.10, 1e-9));
#endif

    printf("PASS: test_a7_wheel_angle_wrap\n");
    return 0;
}
