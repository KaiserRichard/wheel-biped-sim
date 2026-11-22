#include <assert.h>
#include <math.h>

#include "s7_wheel_loss.h"

int main(void)
{
    s7_wheel_loss_t loss = {
        .radius_m = 0.125f, .viscous_nms_per_rad = 0.25f,
        .normal_load_n = 8.0f, .rolling_coefficient = 0.125f,
        .smoothing_rad_s = 2.0f,
    };
    const float speed = 0.25f;
    const float rolling_only = 0.19039854f; /* 2 * 0.125 * tanh(1) */
    assert(s7_wheel_loss_feedforward(&loss, 0.0f) == 0.0f);
    const float forward = s7_wheel_loss_feedforward(&loss, speed);
    assert(fabsf(forward - (1.0f + rolling_only)) < 1e-6f);
    assert(fabsf(s7_wheel_loss_feedforward(&loss, -speed) + forward) < 1e-6f);
    assert(s7_wheel_loss_feedforward(&loss, speed / 2.0f) < forward);

    loss.viscous_nms_per_rad = 0.0f;
    assert(fabsf(s7_wheel_loss_feedforward(&loss, speed) - rolling_only) < 1e-6f);
    loss.normal_load_n = 0.0f;
    assert(s7_wheel_loss_feedforward(&loss, speed) == 0.0f);
    return 0;
}
