#include "test_common.h"

/* REVIEW A10 & A2: Ownership-safe triple-buffered ISR mailbox snapshot must
 * auto-track sequence freshness, distinguish never-replied axes from explicit
 * unhealthy status during startup, and handle gen/msg_seq wraps at UINT32_MAX
 * without losing initialization, freshness, or health. */
int main(void)
{
    S11IsrMailbox mb;
    S11InputSnapshot snap;
    S11PacketPolicy st;
    S11PacketFrame frame;

    /* 1. Partial mailbox startup (A2): 5 axes reply healthy, axis 0 never wrote.
     * Must stay S11_ARMING (not latch S11_DRIVE_FAULT), then arm once axis 0
     * delivers 3 consecutive valid healthy groups. */
    s11_mailbox_init(&mb);
    s11_init(&st);
    assert(s11_arm(&st) == 1);

    uint32_t now_us = 100000u;
    for (unsigned i = 1; i < S11_AXES; ++i)
        s11_mailbox_write_axis(&mb, i, 1, 1, now_us - 1000u, 0.25 + 0.01 * (double)i, 0.0);

    unsigned torn = s11_mailbox_snapshot(&mb, now_us, 1000.0, &snap);
    assert(torn == 0u);
    assert((snap.status_unknown & 1u) != 0u);
    assert((snap.status_unknown & 0x3Eu) == 0u);
    s11_packets_snapshot(&st, &snap, 2.0, &frame);
    assert(frame.mode == S11_ARMING);
    assert(frame.valid == 0);

    /* Explicit unhealthy status on axis 0 via mailbox must still latch DRIVE_FAULT. */
    S11PacketPolicy st_fault;
    S11IsrMailbox mb_fault;
    s11_init(&st_fault);
    assert(s11_arm(&st_fault) == 1);
    s11_mailbox_init(&mb_fault);
    s11_mailbox_write_axis(&mb_fault, 0u, 1, 0, now_us - 1000u, 0.2, 0.0);
    for (unsigned i = 1; i < S11_AXES; ++i)
        s11_mailbox_write_axis(&mb_fault, i, 1, 1, now_us - 1000u, 0.2, 0.0);
    torn = s11_mailbox_snapshot(&mb_fault, now_us, 1000.0, &snap);
    assert(torn == 0u);
    assert((snap.status_unknown & 1u) == 0u);
    assert((snap.healthy & 1u) == 0u);
    s11_packets_snapshot(&st_fault, &snap, 2.0, &frame);
    assert(frame.mode == S11_DRIVE_FAULT);

    /* 2. Complete 3 clean 6-axis ISR writes -> arm cleanly via s11_packets_snapshot. */
    for (unsigned k = 0; k < 3u; ++k) {
        now_us += 2000u;
        for (unsigned i = 0; i < S11_AXES; ++i) {
            s11_mailbox_write_axis(&mb, i, 1, 1, now_us - 1000u,
                                   0.25 + 0.01 * (double)i, 0.0);
        }
        torn = s11_mailbox_snapshot(&mb, now_us, 1000.0, &snap);
        assert(torn == 0u);
        assert(snap.status_unknown == 0u);
        assert(snap.fresh == ALL_AXES);
        assert(snap.valid == ALL_AXES);
        assert(snap.healthy == ALL_AXES);
        s11_packets_snapshot(&st, &snap, 2.0, &frame);
    }
    assert(frame.mode == S11_FRESH);
    assert(frame.valid == 1);
    assert(near_eq(frame.angle[0], 0.25, 1e-12));

    /* 3. Snapshot again with NO new ISR write: fresh must be 0 -> PREDICT. */
    now_us += 2000u;
    torn = s11_mailbox_snapshot(&mb, now_us, 1000.0, &snap);
    assert(torn == 0u);
    assert(snap.fresh == 0u);
    s11_packets_snapshot(&st, &snap, 2.0, &frame);
    assert(frame.mode == S11_PREDICT);
    assert(frame.max_misses == 1u);

    /* 4. Generation counter AND msg_seq wrap across UINT32_MAX -> 0 -> 2. */
    atomic_store(&mb.axis[0].gen, UINT32_MAX - 1u);
    mb.axis[0].msg_seq = UINT32_MAX - 1u;
    for (unsigned step = 0; step < 3u; ++step) {
        now_us += 2000u;
        for (unsigned i = 0; i < S11_AXES; ++i) {
            s11_mailbox_write_axis(&mb, i, 1, 1, now_us - 1000u,
                                   0.30 + 0.01 * (double)step + 0.01 * (double)i,
                                   0.0);
        }
        torn = s11_mailbox_snapshot(&mb, now_us, 1000.0, &snap);
        assert(torn == 0u);
        assert(snap.status_unknown == 0u);
        assert((snap.fresh & 1u) != 0u);
        assert((snap.healthy & 1u) != 0u);
        assert((snap.valid & 1u) != 0u);
        s11_packets_snapshot(&st, &snap, 2.0, &frame);
        assert(frame.mode == S11_FRESH);
        assert(frame.valid == 1);
        assert(near_eq(frame.angle[0], 0.30 + 0.01 * (double)step, 1e-12));
    }
    assert(atomic_load(&mb.axis[0].gen) == 4u);
    assert(mb.axis[0].msg_seq == 1u);

    printf("PASS: test_a10_isr_snapshot\n");
    return 0;
}
