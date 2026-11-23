#define _POSIX_C_SOURCE 200809L
#include "test_common.h"

#include <pthread.h>

/* REVIEW A10: Real concurrent writer/reader stress test interrupting reads.
 * Writer continuously alternates two distinct consistent payloads on axis 0
 * while reader snapshots concurrently and verifies no mixed/torn payloads,
 * valid/healthy preservation, and monotonic msg_seq across wraps. */

typedef struct {
    S11IsrMailbox *mb;
    _Atomic int done;
} WriterArgs;

static void *writer_thread(void *arg)
{
    WriterArgs *wa = (WriterArgs *)arg;
    uint32_t stamp = 100000u;
    const unsigned iters = 500000u;

    for (unsigned k = 0; k < iters; ++k) {
        stamp += 1u;
        const int even = (k & 1u) == 0u;
        const double ang = even ? 1.25 : -2.50;
        const double vel = even ? 12.5 : -25.0;
        s11_mailbox_write_axis(wa->mb, 0u, 1, 1, stamp, ang, vel);
    }
    atomic_store_explicit(&wa->done, 1, memory_order_release);
    return NULL;
}

int main(void)
{
    S11IsrMailbox mb;
    S11InputSnapshot snap;
    s11_mailbox_init(&mb);

    assert(atomic_is_lock_free(&mb.axis[0].shared));
    assert(atomic_is_lock_free(&mb.axis[0].gen));

    for (unsigned i = 0; i < S11_AXES; ++i)
        s11_mailbox_write_axis(&mb, i, 1, 1, 100000u, 0.0, 0.0);

    WriterArgs wa;
    wa.mb = &mb;
    atomic_init(&wa.done, 0);

    pthread_t th;
    assert(pthread_create(&th, NULL, writer_thread, &wa) == 0);

    unsigned reads = 0u;
    uint32_t prev_seq = 0u;
    do {
        const unsigned torn = s11_mailbox_snapshot(&mb, 350000u, 1000.0, &snap);
        assert(torn == 0u);
        assert((snap.status_unknown & 1u) == 0u);
        assert((snap.valid & 1u) != 0u);
        assert((snap.healthy & 1u) != 0u);

        const double a = snap.angle[0];
        const double r = snap.rate[0];
        const int pair0 = near_eq(a, 0.0, 1e-12) && near_eq(r, 0.0, 1e-12);
        const int pair1 = near_eq(a, 1.25, 1e-12) && near_eq(r, 12.5, 1e-12);
        const int pair2 = near_eq(a, -2.50, 1e-12) && near_eq(r, -25.0, 1e-12);
        /* Must NEVER observe mixed (1.25, -25.0) or (-2.50, 12.5). */
        assert(pair0 || pair1 || pair2);

        const uint32_t s = snap.seq[0];
        assert((uint32_t)(s - prev_seq) <= 500005u);
        prev_seq = s;
        ++reads;
    } while (!atomic_load_explicit(&wa.done, memory_order_acquire));

    assert(pthread_join(th, NULL) == 0);

    /* Final snapshot after writer completion must reflect last written state. */
    const unsigned torn = s11_mailbox_snapshot(&mb, 350000u, 1000.0, &snap);
    assert(torn == 0u);
    assert(snap.seq[0] == 500001u);
    assert(reads > 0u);

    printf("PASS: test_a10_concurrent_mailbox (reads=%u)\n", reads);
    return 0;
}
