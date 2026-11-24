#include "test_common.h"

/* REVIEW A5: Stale or out-of-order replies whose acquisition time is earlier
 * than the stored sample (age_ms[i] > axis->age_ms + dt) must be rejected so
 * time cannot go backwards. */
int main(void)
{
    S11PacketPolicy st;
    S11PacketFrame frame;
    double angle[S11_AXES] = {1.0, 0.5, 0.2, -0.5, -0.3, 0.4};
    double rate[S11_AXES]  = {0.0, 0.0, 0.0,  0.0,  0.0, 0.0};
    double age[S11_AXES];
    fill_arr(age, 1.0);

    arm_policy(&st, 1.0, angle, rate, &frame);
    assert(near_eq(frame.angle[0], 1.0, 1e-12));

    /* Next tick (dt = 2 ms): stored sample is now 3.0 ms old.
     * Axis 0 delivers an out-of-order reply with age_ms = 5.0 ms (acquired
     * 2 ms earlier than stored sample) carrying old angle 0.2 rad. */
    angle[0] = 0.2;
    age[0] = 5.0;
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);

    /* Stale reply must be rejected: angle must remain 1.0 (not jump back to 0.2),
     * and axis 0 must count as a miss (PREDICT). */
    assert(near_eq(frame.angle[0], 1.0, 1e-12));
    assert(frame.mode == S11_PREDICT);
    assert(frame.max_misses == 1u);
    assert(near_eq(frame.authority, 1.0, 1e-12));

    printf("PASS: test_a5_stale_out_of_order\n");
    return 0;
}
