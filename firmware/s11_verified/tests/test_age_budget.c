#include "test_common.h"

/* Verify logged age budget thresholds (A3, B9) and mandatory NDEBUG-immune
 * test assertions. */
static int side_effect_helper(int *counter)
{
    *counter += 1;
    return 1;
}

int main(void)
{
    /* 1. Assert side effects must never compile away (even in Release builds). */
    int side_effects = 0;
    assert(side_effect_helper(&side_effects) == 1);
    S11_CHECK(side_effect_helper(&side_effects) == 1);
    if (side_effects != 2) {
        fprintf(stderr, "FATAL: assertions compiled away!\n");
        return 1;
    }

    /* 2. Verify ledger-derived default config values:
     *    3.000 ms (phase) + 1.700 ms (CAN 81%) + 1.000 ms (bridge) + 0.300 ms (jitter) = 6.0 ms
     *    max_total_age_ms = max_transport_age_ms (6.0) + cut_hold_ms (10.0) = 16.0 ms. */
    const S11Config cfg = s11_default_config();
    assert(near_eq(cfg.max_transport_age_ms, 6.0, 1e-12));
    assert(near_eq(cfg.predict_hold_ms, 4.0, 1e-12));
    assert(near_eq(cfg.cut_hold_ms, 10.0, 1e-12));
    assert(near_eq(cfg.max_total_age_ms, 16.0, 1e-12));
    assert(near_eq(cfg.max_predict_ms, 10.0, 1e-12));

    S11PacketPolicy st;
    S11PacketFrame frame;
    double angle[S11_AXES] = {0.2, 0.2, 0.2, -0.2, -0.2, -0.2};
    double rate[S11_AXES]  = {0.0, 0.0, 0.0,  0.0,  0.0,  0.0};
    double age[S11_AXES];

    /* 3. S11 matched fresh arrival max (2.997048 ms) + 3 misses (6 ms hold ->
     *    8.997048 ms total) must enter LIMIT with authority 0.75. */
    arm_policy(&st, 2.997048, angle, rate, &frame);
    fill_arr(age, 2.997048);
    s11_packets(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_PREDICT);
    s11_packets(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_PREDICT);
    s11_packets(&st, 0u, ALL_AXES, ALL_AXES, age, angle, rate, &frame);
    assert(frame.mode == S11_LIMIT);
    assert(near_eq(frame.authority, 0.75, 1e-12));

    /* 4. Arrival above 6.0 ms ledger ceiling (e.g. 6.1 ms) must be rejected on
     *    arrival without overwriting stored state. */
    arm_policy(&st, 2.0, angle, rate, &frame);
    double bad_age[S11_AXES];
    double bad_angle[S11_AXES];
    fill_arr(bad_age, 2.0);
    memcpy(bad_angle, angle, sizeof bad_angle);
    bad_age[0] = 6.1;
    bad_angle[0] = 0.9;
    s11_packets(&st, ALL_AXES, ALL_AXES, ALL_AXES, bad_age, bad_angle, rate, &frame);
    assert(frame.mode == S11_PREDICT);
    assert(near_eq(frame.angle[0], 0.2, 1e-12));

    printf("PASS: test_age_budget\n");
    return 0;
}
