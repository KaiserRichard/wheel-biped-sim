#include "test_common.h"

/* NaN / Inf safety test: NaN, +Inf, and -Inf in angle, rate, or age_ms must
 * never enter stored state; an isolated NaN/Inf packet degrades via PREDICT
 * and recovers on valid traffic; 3 consecutive NaN/Inf packets latch
 * S11_INVALID and output NAN (never 0.0). */
int main(void)
{
    S11PacketPolicy st;
    S11PacketFrame frame;
    double angle[S11_AXES] = {0.4, -0.4, 0.6, 0.2, -0.2, -0.6};
    double rate[S11_AXES]  = {0.0,  0.0, 0.0, 0.0,  0.0,  0.0};
    double age[S11_AXES];
    fill_arr(age, 1.0);

    const double nonfinite[3] = {NAN, INFINITY, -INFINITY};

    /* Isolated NaN/+Inf/-Inf in angle, rate, and age_ms must NOT corrupt state
     * and must NOT cause a single-packet torque-off latch (orig latches on 1). */
    for (unsigned k = 0; k < 3u; ++k) {
        arm_policy(&st, 1.0, angle, rate, &frame);
        double bad_a[S11_AXES], bad_r[S11_AXES], bad_t[S11_AXES];
        memcpy(bad_a, angle, sizeof bad_a);
        memcpy(bad_r, rate, sizeof bad_r);
        memcpy(bad_t, age, sizeof bad_t);

        bad_a[0] = nonfinite[k];
        s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, bad_a, rate, &frame);
        assert(frame.mode == S11_PREDICT);
        assert(near_eq(frame.angle[0], 0.4, 1e-12));
        s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
        assert(frame.mode == S11_FRESH);

        bad_r[1] = nonfinite[k];
        s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, bad_r, &frame);
        assert(frame.mode == S11_PREDICT);
        assert(near_eq(frame.rate[1], 0.0, 1e-12));
        s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
        assert(frame.mode == S11_FRESH);

        bad_t[2] = nonfinite[k];
        s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, bad_t, angle, rate, &frame);
        assert(frame.mode == S11_PREDICT);
        assert(near_eq(frame.angle[2], 0.6, 1e-12));
    }

    /* 3 consecutive NaN packets latch S11_INVALID; subsequent call emits NAN. */
    arm_policy(&st, 1.0, angle, rate, &frame);
    double bad_a[S11_AXES];
    memcpy(bad_a, angle, sizeof bad_a);
    bad_a[0] = NAN;
    for (unsigned k = 0; k < 3u; ++k)
        s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, bad_a, rate, &frame);
    assert(frame.mode == S11_INVALID);
    assert(near_eq(frame.authority, 0.0, 1e-12));

    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_INVALID);
    for (unsigned i = 0; i < S11_AXES; ++i)
        assert(isnan(frame.angle[i]) && isnan(frame.rate[i]));

    printf("PASS: test_nan_and_inf\n");
    return 0;
}
