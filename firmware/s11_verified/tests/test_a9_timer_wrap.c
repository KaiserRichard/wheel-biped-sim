#include "test_common.h"

/* REVIEW A9 + timer-wrap: Verify wrap-safe modular unsigned timestamp
 * subtraction for 32-bit us timers (71.6 min wrap) and 16-bit timers
 * (65.5 ms wrap), and run s11_packets_ex across both wraps with zero
 * nuisance latches. */
int main(void)
{
    /* 32-bit microsecond timer wrap (ticks_per_ms = 1000.0) */
    const uint32_t stamp32 = 0xFFFFF830u; /* 2000 us before wrap */
    const uint32_t now32   = 0x000007D0u; /* 2000 us after wrap */
    const double age32 = s11_age_ms_u32(now32, stamp32, 1000.0);
    assert(near_eq(age32, 4.0, 1e-12));

    /* Exact boundary UINT32_MAX -> 0 */
    assert(near_eq(s11_age_ms_u32(0u, 0xFFFFFFFFu, 1000.0), 0.001, 1e-12));
    assert(near_eq(s11_age_ms_u32(100u, 100u, 1000.0), 0.0, 1e-12));
    assert(isnan(s11_age_ms_u32(100u, 50u, 0.0)));
    assert(isnan(s11_age_ms_u32(100u, 50u, -1.0)));

    /* 16-bit timer wrap (e.g., 1 us/tick -> 65.536 ms period) */
    const uint16_t stamp16 = 65400u; /* 136 ticks before 65536 */
    const uint16_t now16   = 100u;   /* 100 ticks after wrap -> delta = 236 */
    const double age16 = s11_age_ms_u16(now16, stamp16, 1000.0);
    assert(near_eq(age16, 0.236, 1e-12));
    assert(near_eq(s11_age_ms_u16(0u, 65535u, 1000.0), 0.001, 1e-12));
    assert(isnan(s11_age_ms_u16(10u, 5u, 0.0)));

    /* Run s11_packets_ex for 20 ticks spanning both 32-bit and 16-bit wraps. */
    S11PacketPolicy st;
    S11PacketFrame frame;
    double angle[S11_AXES] = {0.1, 0.2, 0.3, -0.1, -0.2, -0.3};
    double rate[S11_AXES]  = {0.0, 0.0, 0.0,  0.0,  0.0,  0.0};
    double age[S11_AXES];
    uint32_t seq[S11_AXES] = {0, 0, 0, 0, 0, 0};

    s11_init(&st);
    assert(s11_arm(&st) == 1);

    uint32_t now_us = 0xFFFFD000u; /* ~12.288 ms before 32-bit wrap */
    uint16_t now_16 = 60000u;      /* ~5.536 ms before 16-bit wrap */
    for (unsigned tick = 0; tick < 20u; ++tick) {
        now_us += 2000u;
        now_16 = (uint16_t)(now_16 + 2000u);
        const uint32_t rx_us = (uint32_t)(now_us - 1500u);
        const uint16_t rx_16 = (uint16_t)(now_16 - 1500u);
        for (unsigned i = 0; i < S11_AXES; ++i) {
            age[i] = (i < 3u)
                   ? s11_age_ms_u32(now_us, rx_us, 1000.0)
                   : s11_age_ms_u16(now_16, rx_16, 1000.0);
            assert(near_eq(age[i], 1.5, 1e-9));
            seq[i] += 1u;
        }
        s11_packets_ex(&st, ALL_AXES, ALL_AXES, ALL_AXES, age, angle, rate,
                       2.0, seq, &frame);
        if (tick >= 2u) {
            assert(frame.mode == S11_FRESH);
            assert(near_eq(frame.authority, 1.0, 1e-12));
            assert(frame.valid == 1);
        }
    }

    printf("PASS: test_a9_timer_wrap\n");
    return 0;
}
