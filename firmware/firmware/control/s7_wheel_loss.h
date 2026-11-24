/* S7 standing reference feedforward and the experimental rolling estimator.
 * Ported from S11_packet_tolerance/firmware_copy/src/s7_wheel_loss.h. */
#ifndef S7_WHEEL_LOSS_H
#define S7_WHEEL_LOSS_H

#include "s5_predictor.h"
#include "s7_design_table.h"

#include <stdint.h>

enum s7_mode {
    S7_BASELINE = 0,
    S7_LOSS,
    S7_MID,
    S7_ADAPT,
    S7_V1,
    S7_TRACKING,
    S7_TRACKING_NO_ROLL,
    S7_TRACKING_ADAPT
};

typedef struct {
    float speed_m_s;
    float accel_m_s2;
    float jerk_m_s3;
    float wheel_rate_rad_s;
    float yaw_rad_s;
} s7_reference_t;

typedef struct {
    float    damping;
    float    rate;
    float    rate_accel;
    float    torque;
    float    previous_request;
    uint32_t quiet_ticks;
} s7_loss_t;

static inline void s7_update_loss(s7_loss_t *state, const s7_reference_t *ref,
                                  float rolling_torque, float wheel_accel_gain,
                                  float lean_rate, bool allow_adapt)
{
    /* State-variable filter (tau = 80 ms at 500 Hz) on wheel rate and mean wheel torque.
     * rate_deriv = d/dt(LPF(omega)) is phase-aligned with LPF(omega) and LPF(torque), so
     * subtracting 0.5 * wheel_accel_gain * R * rate_deriv isolates viscous + rolling loss
     * during both steady rolling and smooth acceleration ramps. */
    const float radius = 0.0603f;
    const float rate_delta = 0.025f * (ref->wheel_rate_rad_s - state->rate);
    const float rate_deriv = rate_delta / 0.002f;
    state->rate += rate_delta;
    state->rate_accel += 0.025f * (rate_deriv - state->rate_accel);
    state->torque += 0.025f * (state->previous_request - state->torque);
    const bool quiet = allow_adapt
        && fabsf(ref->yaw_rad_s) < 0.08f
        && fabsf(lean_rate) < 0.18f
        && fabsf(state->rate) > 1.8f
        && fabsf(state->rate_accel) < 4.5f
        && fabsf(rate_deriv) < 5.0f
        && fabsf(state->previous_request) < 0.8f;
    if (!quiet) {
        state->quiet_ticks = 0u;
        return;
    }
    if (state->quiet_ticks < 30u) {
        ++state->quiet_ticks;
        return;
    }
    const float friction = rolling_torque * tanhf(state->rate / 0.12f);
    const float accel_torque = 0.5f * wheel_accel_gain * radius * rate_deriv;
    const float estimate = fminf(0.05f, fmaxf(0.001f,
        (state->torque - friction - accel_torque) / state->rate));
    state->damping += 0.015f * (estimate - state->damping);
}

static inline void s7_efforts_ex(const s5_predictor_t *predictor, s7_loss_t *loss,
                                 enum s7_mode mode, bool enable_v1_adapt,
                                 float length, float obs[S5_OBS], const s7_reference_t *ref,
                                 bool standing, bool inhibit_loss,
                                 float effort[S5_EFFORTS], float feedback_out[S5_EFFORTS])
{
    if (mode == S7_BASELINE || !standing) {
        s5_feedback(predictor, length, obs, effort);
        if (feedback_out != NULL) {
            feedback_out[0] = effort[0];
            feedback_out[1] = effort[1];
        }
        return;
    }

    const float radius = 0.0603f;
    const float crr = (mode == S7_TRACKING_NO_ROLL) ? 0.0f : 0.005f;
    const float rolling_torque = 0.5f * crr * predictor->mass_kg * 9.81f * radius;

    size_t m = 0u;
    size_t l = 0u;
    const float wm = s5_weight(s5_masses, (size_t)S5_MASS_COUNT, predictor->mass_kg, &m);
    const float wl = s5_weight(s5_lengths, (size_t)S5_LENGTH_COUNT, length, &l);
    const bool tracking = (mode >= S7_TRACKING);
    const bool use_v1_ff = (mode == S7_V1 || tracking);
    const bool use_learned_damping = (mode == S7_ADAPT || mode == S7_TRACKING_ADAPT
                                      || (mode == S7_V1 && enable_v1_adapt));

    float coefficients[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    if (use_v1_ff || use_learned_damping) {
        for (size_t j = 0u; j < 4u; ++j) {
            coefficients[j] = s5_bilinear(s7_accel[m][l][j], s7_accel[m][l + 1u][j],
                                          s7_accel[m + 1u][l][j], s7_accel[m + 1u][l + 1u][j],
                                          wm, wl);
        }
    }

    if (use_learned_damping) {
        s7_update_loss(loss, ref, rolling_torque, coefficients[2], obs[3],
                       standing && !inhibit_loss);
    }
    const float damping = use_learned_damping ? loss->damping : 0.0255f;
    const float rate_ref = ref->speed_m_s / radius;
    float wheel_ff = 2.0f * (damping * rate_ref + rolling_torque * tanhf(rate_ref / 0.12f));
    float leg_ff = 0.0f;

    if (use_v1_ff) {
        obs[2] -= coefficients[0] * ref->accel_m_s2;
        obs[3] -= coefficients[0] * ref->jerk_m_s3;
        obs[4] -= coefficients[1] * ref->accel_m_s2;
        obs[5] -= coefficients[1] * ref->jerk_m_s3;
        wheel_ff += coefficients[2] * ref->accel_m_s2;
        leg_ff += coefficients[3] * ref->accel_m_s2;
    }

    if (mode == S7_LOSS) {
        s5_feedback(predictor, length, obs, effort);
    } else {
        const float (*gain)[3][3][12] = tracking ? s7_gain_tracking : s7_gain_mid;
        for (size_t i = 0u; i < (size_t)S5_EFFORTS; ++i) {
            effort[i] = 0.0f;
            for (size_t j = 0u; j < (size_t)S5_COEFFICIENTS; ++j) {
                const float k = s5_bilinear(gain[i][m][l][j], gain[i][m][l + 1u][j],
                                            gain[i][m + 1u][l][j], gain[i][m + 1u][l + 1u][j],
                                            wm, wl);
                if (j < (size_t)S5_OBS) {
                    effort[i] += k * obs[j];
                } else {
                    const size_t h = j - (size_t)S5_OBS;
                    effort[i] += k * predictor->command[h / (size_t)S5_EFFORTS][h % (size_t)S5_EFFORTS];
                }
            }
        }
    }
    if (feedback_out != NULL) {
        feedback_out[0] = effort[0];
        feedback_out[1] = effort[1];
    }
    effort[0] += wheel_ff;
    effort[1] += leg_ff;
}

static inline void s7_efforts(const s5_predictor_t *predictor, s7_loss_t *loss, enum s7_mode mode,
                              float length, float obs[S5_OBS], const s7_reference_t *ref,
                              bool standing, float effort[S5_EFFORTS])
{
    s7_efforts_ex(predictor, loss, mode, true, length, obs, ref, standing, false, effort, NULL);
}

#endif /* S7_WHEEL_LOSS_H */
