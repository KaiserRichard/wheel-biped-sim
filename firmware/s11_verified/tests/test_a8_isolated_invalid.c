#include "test_common.h"

/* REVIEW A8: An isolated invalid packet must NOT latch torque-off; it must
 * degrade like a miss (PREDICT) and recover when valid traffic resumes.
 * N consecutive invalid packets must latch S11_INVALID, while a drive fault
 * (healthy bit clear) must still latch S11_DRIVE_FAULT immediately. */
int main(void)
{
    S11PacketPolicy st;
    S11PacketFrame frame;
    double angle[S11_AXES] = {0.3, -0.3, 0.5, 0.2, -0.2, -0.5};
    double rate[S11_AXES]  = {0.0,  0.0, 0.0, 0.0,  0.0,  0.0};
    double age[S11_AXES];
    fill_arr(age, 1.0);

    arm_policy(&st, 1.0, angle, rate, &frame);

    /* Inject 1 isolated invalid packet on axis 0 (orig latches S11_INVALID!). */
    s11_packets(&st, ALL_AXES, 0x3Eu, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_PREDICT);
    assert(near_eq(frame.authority, 1.0, 1e-12));
    assert(near_eq(frame.angle[0], 0.3, 1e-12));

    /* Next tick valid resumes: must recover to S11_FRESH with full authority. */
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_FRESH);
    assert(near_eq(frame.authority, 1.0, 1e-12));

    /* Inject 3 consecutive invalid packets on axis 0: must latch on the 3rd. */
    s11_packets(&st, ALL_AXES, 0x3Eu, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_PREDICT);
    s11_packets(&st, ALL_AXES, 0x3Eu, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_PREDICT);
    s11_packets(&st, ALL_AXES, 0x3Eu, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_INVALID);
    assert(near_eq(frame.authority, 0.0, 1e-12));

    /* Drive health fault (healthy bit clear) must still latch immediately on tick 1. */
    arm_policy(&st, 1.0, angle, rate, &frame);
    s11_packets(&st, ALL_AXES, ALL_AXES, 0x3Eu, age, angle, rate, &frame);
    assert(frame.mode == S11_DRIVE_FAULT);
    assert(near_eq(frame.authority, 0.0, 1e-12));

    printf("PASS: test_a8_isolated_invalid\n");
    return 0;
}
