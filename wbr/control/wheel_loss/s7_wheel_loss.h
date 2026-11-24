#ifndef S7_WHEEL_LOSS_H
#define S7_WHEEL_LOSS_H

#include <math.h>

typedef struct {
    float radius_m;
    float viscous_nms_per_rad;
    float normal_load_n;
    float rolling_coefficient;
    float smoothing_rad_s;
} s7_wheel_loss_t;

/* Aggregate torque for two identical wheels in straight rolling. The caller
 * provides positive radius and smoothing and nonnegative loss parameters.
 * Evaluate before actuator limiting and only in the intended rolling mode. */
static inline float s7_wheel_loss_feedforward(const s7_wheel_loss_t *loss, float speed_m_s)
{
    const float rate = speed_m_s / loss->radius_m;
    const float rolling_nm = loss->rolling_coefficient * loss->normal_load_n * loss->radius_m;
    return 2.0f * (loss->viscous_nms_per_rad * rate
                   + rolling_nm * tanhf(rate / loss->smoothing_rad_s));
}

#endif
