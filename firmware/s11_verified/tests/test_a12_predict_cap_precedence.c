#include "test_common.h"

/* REVIEW A12: Prediction cap must be consistent with the LIMIT window (6 ms
 * stale LIMIT tick extrapolates by 6 ms, not capped at 4 ms), and healthy bit
 * precedence over valid/fresh must be enforced. */
int main(void)
{
    S11PacketPolicy st;
    S11PacketFrame frame;
    double angle[S11_AXES] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double rate[S11_AXES]  = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double age[S11_AXES];
    fill_arr(age, 0.0);

    /* Case 1: 3 missed ticks at 2 ms = 6 ms stale (S11_LIMIT).
     * Prediction must extrapolate by 6 ms -> 0.006 rad (orig caps at 0.004 rad). */
    arm_policy(&st, 0.0, angle, rate, &frame);
    s11_packets(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_PREDICT && near_eq(frame.angle[0], 0.002, 1e-9));
    s11_packets(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_PREDICT && near_eq(frame.angle[0], 0.004, 1e-9));
    s11_packets(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_LIMIT);
    assert(near_eq(frame.authority, 0.75, 1e-12));
    assert(near_eq(frame.angle[0], 0.006, 1e-9));

    /* Case 2: Precedence: healthy==0 wins over valid==0 and over fresh==0. */
    arm_policy(&st, 0.0, angle, rate, &frame);
    s11_packets(&st, ALL_AXES, 0x3Eu, 0x3Eu, age, angle, rate, &frame);
    assert(frame.mode == S11_DRIVE_FAULT);

    arm_policy(&st, 0.0, angle, rate, &frame);
    s11_packets(&st, 0u, ALL_AXES, 0x3Eu, age, angle, rate, &frame);
    assert(frame.mode == S11_DRIVE_FAULT);

    printf("PASS: test_a12_predict_cap_precedence\n");
    return 0;
}
