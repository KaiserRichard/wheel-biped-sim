/* Two total standing efforts, shared by the left and right H5 kernels.
 * Ported from S11_packet_tolerance/s5_predictor.h. */
#ifndef S5_PREDICTOR_H
#define S5_PREDICTOR_H

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include "s5_gain_table.h"

enum { S5_EFFORTS = 2, S5_HISTORY = 3, S5_OBS = 6 };

typedef struct {
    float command[S5_HISTORY][S5_EFFORTS]; /* newest first */
    float mass_kg;
    float length_ref_m;
} s5_predictor_t;

static inline float s5_weight(const float *axis, size_t count, float value, size_t *lower)
{
    *lower = 0u;
    while (*lower + 2u < count && axis[*lower + 1u] < value) {
        ++(*lower);
    }
    const float lo = axis[*lower];
    const float hi = axis[*lower + 1u];
    return fminf(1.0f, fmaxf(0.0f, (value - lo) / (hi - lo)));
}

static inline float s5_mix(float a, float b, float weight)
{
    return a + weight * (b - a);
}

static inline float s5_bilinear(float a, float b, float c, float d, float wm, float wl)
{
    return s5_mix(s5_mix(a, b, wl), s5_mix(c, d, wl), wm);
}

static inline void s5_trim(const s5_predictor_t *state, float out[3])
{
    size_t m = 0u;
    size_t l = 0u;
    const float wm = s5_weight(s5_masses, (size_t)S5_MASS_COUNT, state->mass_kg, &m);
    const float wl = s5_weight(s5_lengths, (size_t)S5_LENGTH_COUNT, state->length_ref_m, &l);
    for (size_t j = 0u; j < 3u; ++j) {
        out[j] = s5_bilinear(s5_standing_trim[m][l][j], s5_standing_trim[m][l + 1u][j],
                             s5_standing_trim[m + 1u][l][j], s5_standing_trim[m + 1u][l + 1u][j],
                             wm, wl);
    }
}

static inline void s5_feedback(const s5_predictor_t *state, float length, const float obs[S5_OBS],
                               float effort[S5_EFFORTS])
{
    size_t m = 0u;
    size_t l = 0u;
    const float wm = s5_weight(s5_masses, (size_t)S5_MASS_COUNT, state->mass_kg, &m);
    const float wl = s5_weight(s5_lengths, (size_t)S5_LENGTH_COUNT, length, &l);
    for (size_t i = 0u; i < (size_t)S5_EFFORTS; ++i) {
        effort[i] = 0.0f;
        for (size_t j = 0u; j < (size_t)S5_COEFFICIENTS; ++j) {
            const float gain = s5_bilinear(s5_gain[i][m][l][j], s5_gain[i][m][l + 1u][j],
                                           s5_gain[i][m + 1u][l][j], s5_gain[i][m + 1u][l + 1u][j],
                                           wm, wl);
            if (j < (size_t)S5_OBS) {
                effort[i] += gain * obs[j];
            } else {
                const size_t k = j - (size_t)S5_OBS;
                effort[i] += gain * state->command[k / (size_t)S5_EFFORTS][k % (size_t)S5_EFFORTS];
            }
        }
    }
}

static inline void s5_commit(s5_predictor_t *state, const float effort[S5_EFFORTS], bool standing)
{
    if (!standing) {
        memset(state->command, 0, sizeof(state->command));
        return;
    }
    for (size_t k = (size_t)S5_HISTORY - 1u; k > 0u; --k) {
        for (size_t j = 0u; j < (size_t)S5_EFFORTS; ++j) {
            state->command[k][j] = state->command[k - 1u][j];
        }
    }
    for (size_t j = 0u; j < (size_t)S5_EFFORTS; ++j) {
        state->command[0][j] = effort[j];
    }
}

#endif /* S5_PREDICTOR_H */
