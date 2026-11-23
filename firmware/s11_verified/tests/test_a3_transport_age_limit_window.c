#include "test_common.h"

/* REVIEW A3: Separate arrival age from miss/hold count using the logged age
 * budget (6.0 ms arrival ceiling = 3.0 ms phase + 1.7 ms CAN + 1.0 ms bridge +
 * 0.3 ms jitter; 10.0 ms cut_hold_ms; 16.0 ms max_total_age_ms) so worst-case
 * valid arrival ages still get PREDICT (2 ticks) + LIMIT (2 ticks) before
 * TIMEOUT. */
int main(void)
{
    S11PacketPolicy st;
    S11PacketFrame frame;
    double angle[S11_AXES] = {0.1, 0.2, 0.3, -0.1, -0.2, -0.3};
    double rate[S11_AXES]  = {0.0, 0.0, 0.0,  0.0,  0.0,  0.0};
    double age[S11_AXES];
    fill_arr(age, 5.9); /* Just under derived 6.0 ms arrival budget (zero holds) */

    arm_policy(&st, 5.9, angle, rate, &frame);

    /* Miss 1 (2 ms hold, total 7.9 ms): must be PREDICT (authority 1.0). */
    s11_packets(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_PREDICT);
    assert(near_eq(frame.authority, 1.0, 1e-12));

    /* Miss 2 (4 ms hold, total 9.9 ms): still PREDICT (authority 1.0). */
    s11_packets(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_PREDICT);
    assert(near_eq(frame.authority, 1.0, 1e-12));

    /* Miss 3 (6 ms hold, total 11.9 ms): LIMIT (authority 0.75).
     * orig has age > 10.0 ms here and skips LIMIT -> latches TIMEOUT! */
    s11_packets(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_LIMIT);
    assert(near_eq(frame.authority, 0.75, 1e-12));

    /* Miss 4 (8 ms hold, total 13.9 ms): still LIMIT (usable 2-tick / 4 ms LIMIT window). */
    s11_packets(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_LIMIT);
    assert(near_eq(frame.authority, 0.75, 1e-12));

    /* Miss 5 (10 ms hold, total 15.9 ms): TIMEOUT latch (authority 0.0). */
    s11_packets(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_TIMEOUT);
    assert(near_eq(frame.authority, 0.0, 1e-12));

    printf("PASS: test_a3_transport_age_limit_window\n");
    return 0;
}
