/* Ported from S11_packet_tolerance/firmware_copy/src/s9_balance_point.h. */
#ifndef S9_BALANCE_POINT_H
#define S9_BALANCE_POINT_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#define S9_BLEND_TIME_S         0.75f
#define S9_LEAN_PER_PITCH       0.08f
#define S9_TRIM_RAD_PER_M       0.34f
#define S9_TRIM_TIME_S          1.10f
#define S9_TRIM_MAX_RATE_RAD_S  0.045f
#define S9_TRIM_LIMIT_RAD       0.105f
#define S9_QUIET_TICKS          125u

typedef struct {
    float position_m;
    float blend_m;
    bool  host_lost;
} s9_brake_t;

typedef struct {
    float    pitch_rad;
    uint32_t quiet_ticks;
} s9_trim_t;

static inline float s9_clamp(float value, float limit)
{
    return fminf(limit, fmaxf(-limit, value));
}

/* The position term presented to the 500 Hz balance loop is continuous at latch. */
static inline float s9_brake_position(s9_brake_t *state, bool host_lost,
                                      float position_m, float speed_error_m_s,
                                      float period_s, float bound_m)
{
    if (!host_lost) {
        state->position_m = position_m;
        state->blend_m = 0.0f;
    } else if (!state->host_lost) {
        state->blend_m = state->position_m;
    } else {
        const float previous_blend_m = state->blend_m;
        state->blend_m *= 1.0f - period_s / S9_BLEND_TIME_S;
        const float next = state->position_m + speed_error_m_s * period_s
                           + state->blend_m - previous_blend_m;
        state->position_m = s9_clamp(next, bound_m);
    }
    state->host_lost = host_lost;
    return state->position_m;
}

/* The integrated odometry error is the steady position correction to unload. */
static inline void s9_update_trim(s9_trim_t *state, bool quiet,
                                  float position_m, float period_s)
{
    if (!quiet) {
        state->quiet_ticks = 0u;
        return;
    }
    if (state->quiet_ticks < S9_QUIET_TICKS) {
        ++state->quiet_ticks;
        return;
    }
    const float target = s9_clamp(-S9_TRIM_RAD_PER_M * position_m, S9_TRIM_LIMIT_RAD);
    const float rate = s9_clamp((target - state->pitch_rad) / S9_TRIM_TIME_S,
                                S9_TRIM_MAX_RATE_RAD_S);
    state->pitch_rad = s9_clamp(state->pitch_rad + period_s * rate, S9_TRIM_LIMIT_RAD);
}

#endif /* S9_BALANCE_POINT_H */
