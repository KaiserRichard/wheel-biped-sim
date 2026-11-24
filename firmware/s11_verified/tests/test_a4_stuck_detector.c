#include "test_common.h"

/* REVIEW A4: Stuck detector must catch frozen angle while moving within the
 * 10 ms budget (including when |rate|*dt > 2 LSB even if rate jitters), and
 * must detect frozen stationary replies via sequence counter freshness. */
int main(void)
{
    S11PacketPolicy st;
    S11PacketFrame frame;
    double angle[S11_AXES] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    double rate[S11_AXES]  = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double age[S11_AXES];
    fill_arr(age, 1.0);

    /* Case 1: Axis 0 reports rate ~1.0 rad/s (with slight jitter) while angle
     * stays frozen at 0.5 rad. Must latch S11_STUCK within <= 10 ms (5 ticks).
     * orig needs 11 identical packets (20 ms) and fails forever if rate jitters. */
    arm_policy(&st, 1.0, angle, rate, &frame);
    int detected = 0;
    for (unsigned k = 0; k < 5u; ++k) {
        rate[0] = (k % 2u == 0u) ? 1.0 : 1.02;
        s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
        if (frame.mode == S11_STUCK) {
            detected = 1;
            break;
        }
    }
    assert(detected == 1);
    assert(near_eq(frame.authority, 0.0, 1e-12));

#ifndef S11_TEST_ORIG
    /* Case 2: Stationary joint (rate == 0.0) with frozen mailbox sequence
     * counter (seq[0] stops advancing). Must be treated as miss and latch
     * TIMEOUT instead of being accepted forever. */
    double zero_rate[S11_AXES] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    uint32_t seq[S11_AXES] = {10u, 10u, 10u, 10u, 10u, 10u};
    s11_init(&st);
    assert(s11_arm(&st) == 1);
    for (unsigned k = 0; k < 3u; ++k) {
        for (unsigned i = 0; i < S11_AXES; ++i)
            seq[i] += 1u;
        s11_packets_ex(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, zero_rate,
                       2.0, seq, &frame);
    }
    assert(frame.mode == S11_FRESH && frame.valid == 1);

    /* Freeze seq[0] while other axes keep advancing; fresh bit still set. */
    for (unsigned k = 0; k < 5u; ++k) {
        for (unsigned i = 1; i < S11_AXES; ++i)
            seq[i] += 1u;
        s11_packets_ex(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, zero_rate,
                       2.0, seq, &frame);
    }
    assert(frame.mode == S11_TIMEOUT);
    assert(frame.valid == 0);

    /* Case 3: Truly stationary joint with advancing seq[] must stay S11_FRESH. */
    s11_reset(&st);
    assert(s11_arm(&st) == 1);
    for (unsigned k = 0; k < 20u; ++k) {
        for (unsigned i = 0; i < S11_AXES; ++i)
            seq[i] += 1u;
        s11_packets_ex(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, zero_rate,
                       2.0, seq, &frame);
    }
    assert(frame.mode == S11_FRESH);
    assert(near_eq(frame.authority, 1.0, 1e-12));
    assert(frame.valid == 1);

    /* Case 4: Slow frozen angle at 0.1 rad/s (< stuck_rate_thresh 0.12) with
     * advancing seq[] must accumulate displacement and latch S11_STUCK once
     * net displacement exceeds 2 * LSB. */
    double slow_rate[S11_AXES] = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0};
    s11_reset(&st);
    assert(s11_arm(&st) == 1);
    int slow_stuck = 0;
    for (unsigned k = 0; k < 103u; ++k) {
        for (unsigned i = 0; i < S11_AXES; ++i)
            seq[i] += 1u;
        s11_packets_ex(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, slow_rate,
                       2.0, seq, &frame);
        if (frame.mode == S11_STUCK) {
            slow_stuck = 1;
            break;
        }
    }
    assert(slow_stuck == 1);
    assert(near_eq(frame.authority, 0.0, 1e-12));

    /* Case 5: Negative control - slow moving joint at 0.1 rad/s whose quantized
     * angle updates by 1 LSB every 4 ms must stay S11_FRESH. */
    double moving_ang[S11_AXES] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    const double lsb = st.config.angle_lsb_rad;
    s11_reset(&st);
    assert(s11_arm(&st) == 1);
    for (unsigned k = 0; k < 50u; ++k) {
        for (unsigned i = 0; i < S11_AXES; ++i)
            seq[i] += 1u;
        if (k % 2u == 1u)
            moving_ang[0] += lsb;
        s11_packets_ex(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, moving_ang,
                       slow_rate, 2.0, seq, &frame);
        if (k >= 2u) {
            assert(frame.mode == S11_FRESH);
            assert(frame.valid == 1);
        }
    }

    /* Case 6: Negative control - stationary joint with zero-mean rate jitter
     * (+/-0.005 rad/s) and unchanged angle must stay S11_FRESH. */
    double noise_rate[S11_AXES] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    s11_reset(&st);
    assert(s11_arm(&st) == 1);
    for (unsigned k = 0; k < 50u; ++k) {
        for (unsigned i = 0; i < S11_AXES; ++i)
            seq[i] += 1u;
        noise_rate[0] = (k % 2u == 0u) ? 0.005 : -0.005;
        s11_packets_ex(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, noise_rate,
                       2.0, seq, &frame);
        if (k >= 2u) {
            assert(frame.mode == S11_FRESH);
            assert(frame.valid == 1);
        }
    }
#endif

    printf("PASS: test_a4_stuck_detector\n");
    return 0;
}
