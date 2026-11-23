#include "test_common.h"
#include <stdint.h>

/* Independent acceptance checks. FAIL means a review finding, not a harness error. */
static unsigned failures;
static void check(const char *name, int good, int actual, int expected)
{
    printf("%s,%s,%d,%d\n", name, good ? "PASS" : "FAIL", actual, expected);
    if (!good) ++failures;
}

int main(void)
{
    S11PacketPolicy st;
    S11PacketFrame f;
    double a[6] = {.2,.2,.2,.2,.2,.2};
    double r[6] = {.1,0,0,0,0,0};
    double age[6] = {1,1,1,1,1,1};
    uint32_t seq[6] = {0};
    printf("check,result,actual,expected\n");
    s11_init(&st);
    s11_arm(&st);
    /* Frozen angle at 0.1 rad/s accumulates 0.02 rad over 200 ms:
       more than 52 default LSBs, with genuine advancing RX sequences. */
    for (unsigned k = 0; k < 103u; ++k) {
        for (unsigned i = 0; i < 6u; ++i) ++seq[i];
        s11_packets_ex(&st, ALL_AXES, ALL_AXES, ALL_AXES, age,a,r,2.,seq,&f);
    }
    check("A4_slow_frozen_angle", f.mode == S11_STUCK, f.mode, S11_STUCK);

    S11IsrMailbox mb;
    S11InputSnapshot snap;
    s11_init(&st); s11_arm(&st); s11_mailbox_init(&mb);
    for (unsigned i = 1; i < 6u; ++i)
        s11_mailbox_write_axis(&mb,i,1,1,9000u,.2,0.);
    s11_mailbox_snapshot(&mb,10000u,1000.,&snap);
    s11_packets_snapshot(&st,&snap,2.,&f);
    check("A2_partial_mailbox_startup", f.mode == S11_ARMING, f.mode, S11_ARMING);

    s11_mailbox_init(&mb);
    for (unsigned i = 0; i < 6u; ++i)
        s11_mailbox_write_axis(&mb,i,1,1,9000u,.2,0.);
    s11_mailbox_snapshot(&mb,10000u,1000.,&snap);
    atomic_store(&mb.axis[0].gen, UINT32_MAX-1u);
    s11_mailbox_write_axis(&mb,0u,1,1,11000u,.3,0.);
    s11_mailbox_snapshot(&mb,12000u,1000.,&snap);
    check("A10_generation_wrap_fresh", (snap.fresh & 1u) != 0u,
          (int)(snap.fresh & 1u), 1);
    check("A10_generation_wrap_healthy", (snap.healthy & 1u) != 0u,
          (int)(snap.healthy & 1u), 1);

    /* Temporal helpers: a non-zero wrap result; no zero-only fixture. */
    check("A9_u16_modular", near_eq(s11_age_ms_u16(100u,65400u,1000.),.236,1e-12),
          (int)lround(1000.*s11_age_ms_u16(100u,65400u,1000.)),236);
    check("A9_u32_modular", near_eq(s11_age_ms_u32(2000u,UINT32_MAX-1999u,1000.),4.,1e-12),
          (int)lround(1000.*s11_age_ms_u32(2000u,UINT32_MAX-1999u,1000.)),4000);

    /* Actual fault status still wins during arming, independent of freshness. */
    s11_init(&st); s11_arm(&st);
    s11_packets(&st,0u,0u,ALL_AXES^1u,age,a,r,&f);
    check("A12_explicit_unhealthy", f.mode == S11_DRIVE_FAULT,f.mode,S11_DRIVE_FAULT);
    return failures ? 1 : 0;
}
