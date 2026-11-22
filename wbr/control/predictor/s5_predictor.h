/* Two total standing efforts, shared by the left and right H5 kernels. */
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
    *lower = 0;
    while (*lower + 2 < count && axis[*lower + 1] < value)
        ++*lower;
    const float lo = axis[*lower];
    const float hi = axis[*lower + 1];
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
    size_t m, l;
    const float wm = s5_weight(s5_masses, S5_MASS_COUNT, state->mass_kg, &m);
    const float wl = s5_weight(s5_lengths, S5_LENGTH_COUNT, state->length_ref_m, &l);
    for (size_t j = 0; j < 3; ++j)
        out[j] = s5_bilinear(s5_standing_trim[m][l][j], s5_standing_trim[m][l + 1][j],
                             s5_standing_trim[m + 1][l][j], s5_standing_trim[m + 1][l + 1][j], wm, wl);
}

static inline void s5_feedback(const s5_predictor_t *state, float length, const float obs[S5_OBS],
                               float effort[S5_EFFORTS])
{
    size_t m, l;
    const float wm = s5_weight(s5_masses, S5_MASS_COUNT, state->mass_kg, &m);
    const float wl = s5_weight(s5_lengths, S5_LENGTH_COUNT, length, &l);
    for (size_t i = 0; i < S5_EFFORTS; ++i) {
        effort[i] = 0.0f;
        for (size_t j = 0; j < S5_COEFFICIENTS; ++j) {
            const float gain = s5_bilinear(s5_gain[i][m][l][j], s5_gain[i][m][l + 1][j],
                                           s5_gain[i][m + 1][l][j], s5_gain[i][m + 1][l + 1][j],
                                           wm, wl);
            if (j < S5_OBS) {
                effort[i] += gain * obs[j];
            } else {
                const size_t k = j - S5_OBS;
                effort[i] += gain * state->command[k / S5_EFFORTS][k % S5_EFFORTS];
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
    for (size_t k = S5_HISTORY - 1; k > 0; --k)
        for (size_t j = 0; j < S5_EFFORTS; ++j)
            state->command[k][j] = state->command[k - 1][j];
    for (size_t j = 0; j < S5_EFFORTS; ++j)
        state->command[0][j] = effort[j];
}

#endif
