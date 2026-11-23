#ifndef S11_TEST_COMMON_H
#define S11_TEST_COMMON_H

#include "s11_packet_policy.h"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Mandatory runtime check that cannot compile away even if NDEBUG is defined. */
#define S11_CHECK(expr)                                                   \
    do {                                                                  \
        if (!(expr)) {                                                    \
            fprintf(stderr, "%s:%d: %s: Assertion `%s' failed.\n",        \
                    __FILE__, __LINE__, __func__, #expr);                 \
            abort();                                                      \
        }                                                                 \
    } while (0)

#undef assert
#define assert(expr) S11_CHECK(expr)

#define ALL_AXES 0x3Fu

static inline void fill_arr(double dst[S11_AXES], double val)
{
    for (unsigned i = 0; i < S11_AXES; ++i)
        dst[i] = val;
}

static inline int near_eq(double a, double b, double tol)
{
    return isfinite(a) && isfinite(b) && fabs(a - b) <= tol;
}

/* Bring policy to armed S11_FRESH state ending on exact target angle/rate/age. */
static inline void arm_policy(S11PacketPolicy *st, double age_val,
                              const double target_angle[S11_AXES],
                              const double rate[S11_AXES],
                              S11PacketFrame *frame)
{
    double age[S11_AXES];
    double ang[S11_AXES];
    fill_arr(age, age_val);

#ifdef S11_TEST_ORIG
    memset(st, 0, sizeof *st);
    const unsigned groups = 3u;
#else
    s11_init(st);
    assert(s11_arm(st) == 1);
    const unsigned groups = st->config.arm_groups;
#endif

    for (unsigned k = 0; k < groups; ++k) {
        const double ticks_before = (double)(groups - 1u - k);
        for (unsigned i = 0; i < S11_AXES; ++i)
            ang[i] = target_angle[i] - ticks_before * 0.002 * rate[i];
        s11_packets(st, ALL_AXES, ALL_AXES, ALL_AXES, age, ang, rate, frame);
    }
    assert(frame->mode == S11_FRESH);
    assert(near_eq(frame->authority, 1.0, 1e-12));
#ifndef S11_TEST_ORIG
    assert(frame->valid == 1);
#endif
}

#endif
