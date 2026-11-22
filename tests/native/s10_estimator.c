#include <assert.h>
#include <math.h>

#include "s10_estimator.h"

static const double gravity_force[3] = {0.0, 0.0, 9.80665};
static const double zero[3] = {0.0, 0.0, 0.0};
static const S10Encoder stationary = {0};
static const S10Encoder rolling = {.spin = {1.5, 1.5}};
static const S10Encoder leg_moving = {.leg_rate = {0.15, 0.0, 0.0, 0.0}};

static void check_startup_and_packet_order(void)
{
    S10Estimator estimator, free_fall;
    S10Frame frame;
    const double known_gyro[3] = {0.001953125, -0.0078125, 0.00390625};
    s10_init(&estimator, 0.1, zero);
    s10_init(&free_fall, 0.1, zero);
    for (int tick = 0; tick < S10_STARTUP; ++tick) {
        assert(!s10_ready(&estimator));
        s10_imu(&estimator, tick, known_gyro, gravity_force);
        s10_imu(&estimator, tick, zero, zero);
        s10_imu(&free_fall, tick, zero, zero);
    }
    assert(!s10_ready(&estimator));
    s10_encoder(&estimator, 100, 102, &stationary);
    s10_encoder(&free_fall, 100, 102, &stationary);
    assert(s10_ready(&estimator) && !s10_ready(&free_fall));
    s10_encoder(&estimator, 100, 105, &rolling);
    s10_encoder(&estimator, 103, 101, &rolling);
    assert(estimator.enc_tick == 100 && estimator.encoder.spin[0] == 0.0);

    s10_frame(&estimator, &frame);
    for (int axis = 0; axis < 3; ++axis)
        assert(fabs(frame.rate[axis]) < 1e-14);
    assert(frame.pitch == 0.0 && frame.roll == 0.0 && frame.speed == 0.0);
}

static void check_drift_gates_and_gap(void)
{
    S10Estimator still, moving, leg, fixed;
    int saw_still = 0, saw_integral = 0;
    s10_init(&still, 0.1, zero);
    s10_init(&moving, 0.1, zero);
    s10_init(&leg, 0.1, zero);
    s10_init(&fixed, 0.1, zero);
    fixed.ki = fixed.wheel_limit = 0.0;

    for (int tick = 0; tick < 5000; ++tick) {
        const double gyro[3] = {0.0, 0.0072 * fmax(0, tick - 1000) * 0.001, 0.0};
        s10_imu(&still, tick, gyro, gravity_force);
        s10_imu(&moving, tick, gyro, gravity_force);
        s10_imu(&leg, tick, gyro, gravity_force);
        s10_imu(&fixed, tick, gyro, gravity_force);
        if (tick % 2 == 0) {
            if (tick < 3000 || tick >= 3040)
                s10_encoder(&still, tick, tick, &stationary);
            s10_encoder(&moving, tick, tick, &rolling);
            s10_encoder(&leg, tick, tick, &leg_moving);
            s10_encoder(&fixed, tick, tick, &stationary);
        }

        assert(!moving.still_update && !leg.still_update);
        if (tick > 3008 && tick <= 3240)
            assert(!still.still_update);
        saw_still |= still.still_update;
        saw_integral |= moving.integral_update;
    }
    assert(saw_still && saw_integral);
    assert(moving.bias[1] > 0.0 && moving.bias[1] <= 0.08);
    assert(still.bias[1] > 0.0 && still.bias[1] <= 0.08);
    assert(fabs(still.rate[1]) < fabs(fixed.rate[1]));
}

static void check_signed_bias_bound(void)
{
    for (int sign = -1; sign <= 1; sign += 2) {
        S10Estimator estimator;
        const double gyro[3] = {0.0, sign * 0.08, 0.0};
        s10_init(&estimator, 0.1, zero);
        estimator.wheel_limit = 0.0;
        for (int tick = 0; tick < S10_STARTUP; ++tick)
            s10_imu(&estimator, tick, zero, gravity_force);
        s10_encoder(&estimator, 100, 100, &stationary);
        estimator.pitch = sign * 0.006;
        estimator.bias[1] = gyro[1];
        s10_imu(&estimator, 101, gyro, gravity_force);
        assert(estimator.integral_update && estimator.bias[1] == gyro[1]);
    }
}

int main(void)
{
    check_startup_and_packet_order();
    check_drift_gates_and_gap();
    check_signed_bias_bound();
    return 0;
}
