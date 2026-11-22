#ifndef S8_STOP_POSITION_H
#define S8_STOP_POSITION_H

#include <math.h>
#include <stdbool.h>

/* Call once per enabled tick. host_lost is the local supervisor latch. */
static inline float s8_stop_position(float *error_m, bool *was_host_lost,
                                     bool host_lost, float speed_error_m_s,
                                     float period_s, float position_limit_m)
{
    if (!host_lost || !*was_host_lost)
        *error_m = 0.0f;
    else {
        const float next = *error_m + period_s * speed_error_m_s;
        *error_m = fminf(position_limit_m, fmaxf(-position_limit_m, next));
    }
    *was_host_lost = host_lost;
    return *error_m;
}

#endif
