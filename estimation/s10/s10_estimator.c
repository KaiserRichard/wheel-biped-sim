#include "s10_estimator.h"

#include <limits.h>
#include <math.h>
#include <string.h>

static const double gravity = 9.80665;
static const int no_tick = INT_MIN;

static double blend(double h, double tau)
{
    return -expm1(-h / tau);
}

static double pitch(const double f[3])
{
    return atan2(-f[0], hypot(f[1], f[2]));
}

static void cross(const double a[3], const double b[3], double c[3])
{
    c[0] = a[1]*b[2] - a[2]*b[1];
    c[1] = a[2]*b[0] - a[0]*b[2];
    c[2] = a[0]*b[1] - a[1]*b[0];
}

void s10_init(S10Estimator *s, double radius, const double lever[3])
{
    memset(s, 0, sizeof *s);
    s->radius = radius;
    memcpy(s->lever, lever, sizeof s->lever);
    s->ki = 1.;
    s->wheel_limit = .12;
    s->odometry_weight = 1.;
    s->imu_tick = s->enc_tick = s->last_release = no_tick;
}

int s10_ready(const S10Estimator *s)
{
    return s->n == S10_STARTUP && !s->startup_failed && s->enc_tick != no_tick;
}

void s10_imu(S10Estimator *s, int tick, const double gyro[3], const double force[3])
{
    if (tick <= s->imu_tick || s->startup_failed)
        return;
    const double h = s->imu_tick == no_tick ? .001 : (tick - s->imu_tick)*.001;
    s->imu_tick = tick;
    if (s->n < S10_STARTUP) {
        for (int axis = 0; axis < 3; ++axis) {
            s->gyro_sum[axis] += gyro[axis];
            s->force_sum[axis] += force[axis];
            s->previous_gyro[axis] = gyro[axis];
        }
        if (++s->n == S10_STARTUP) {
            const double *f = s->force_sum;
            if (hypot(f[0], hypot(f[1], f[2])) <= .65*gravity*S10_STARTUP) {
                s->startup_failed = 1;
                return;
            }
            s->pitch = pitch(f);
            s->roll = atan2(f[1], f[2]);
            for (int axis = 0; axis < 3; ++axis) {
                s->bias[axis] = s->gyro_sum[axis]/S10_STARTUP;
                s->rate[axis] = gyro[axis] - s->bias[axis];
            }
        }
        return;
    }

    const double ka = blend(h, .03);
    for (int axis = 0; axis < 3; ++axis) {
        s->alpha[axis] += ka*((gyro[axis] - s->previous_gyro[axis])/h - s->alpha[axis]);
        s->previous_gyro[axis] = gyro[axis];
        s->rate[axis] = gyro[axis] - s->bias[axis];
    }
    double tangential[3], cross_rate[3], centripetal[3], f[3];
    cross(s->alpha, s->lever, tangential);
    cross(s->rate, s->lever, cross_rate);
    cross(s->rate, cross_rate, centripetal);
    for (int axis = 0; axis < 3; ++axis)
        f[axis] = force[axis] - tangential[axis] - centripetal[axis];

    const double norm = hypot(f[0], hypot(f[1], f[2]));
    s->low = norm < .30*gravity ? s->low + h : 0.;
    if (s->low >= .004)
        s->air = 1;
    else if (norm > .65*gravity)
        s->air = 0;

    s->pitch += h*s->rate[1];
    s->roll += h*s->rate[0];
    s->yaw += h*s->rate[2];
    const double ax = cos(s->pitch)*f[0] + sin(s->pitch)*f[2];
    s->speed += h*ax;
    s->fast += h*ax;

    const int accel_ok = !s->air && !s->alarm && fabs(norm/gravity - 1.) < .10
                         && fabs(ax) < .40 && fabs(s->gate) < .35;
    const double residual = pitch(f) - s->pitch;
    if (accel_ok) {
        const double kp = blend(h, 1.);
        s->pitch += kp*residual;
        s->roll += kp*(atan2(f[1], f[2]) - s->roll);
    }

    const int fresh = h <= .008 && s->enc_tick != no_tick && tick >= s->enc_tick
                      && tick - s->enc_tick <= 8;
    s->still_update = fresh && accel_ok && fabs(ax) < .08 && fabs(s->gate) < .12
                      && s->still_ms >= 200 && fabs(s->rate[0]) < .015
                      && fabs(s->rate[1]) < .015 && fabs(s->rate[2]) < .015;
    const double bias_residual = residual + s->odometry_weight*s->gate/gravity;
    s->integral_update = fresh && accel_ok && !s->still_update && s->ki > 0.
                         && fabs(bias_residual) < .014;
    double delta = 0.;
    if (s->still_update)
        delta = blend(h, 1.)*s->rate[1];
    else if (s->integral_update)
        delta = -h*s->ki*bias_residual;
    if (s->still_update || s->integral_update) {
        s->bias[1] = fmax(-.08, fmin(.08, s->bias[1] + delta));
        s->rate[1] = gyro[1] - s->bias[1];
    }
}

static void wheel_observations(const S10Estimator *s, double speed[2], double relative[2])
{
    const double c = cos(s->pitch), p = sin(s->pitch);
    for (int side = 0; side < 2; ++side) {
        double rotational[3];
        cross(s->rate, s->encoder.position[side], rotational);
        relative[side] = c*(rotational[0] + s->encoder.relative[side][0])
                         + p*(rotational[2] + s->encoder.relative[side][2]);
        speed[side] = s->radius*(s->rate[1] + s->encoder.spin[side]) - relative[side];
    }
}

void s10_encoder(S10Estimator *s, int tick, int release, const S10Encoder *encoder)
{
    if (tick <= s->enc_tick || release < s->last_release)
        return;
    const int interval = s->enc_tick == no_tick ? 0 : tick - s->enc_tick;
    s->enc_tick = tick;
    s->encoder = *encoder;
    int quiet = s->wheel_limit > 0.;
    for (int side = 0; side < 2; ++side)
        quiet = quiet && fabs(encoder->spin[side]) < s->wheel_limit;
    for (int joint = 0; joint < 4; ++joint)
        quiet = quiet && fabs(encoder->leg_rate[joint]) < .11;
    /* Require a complete observed dwell, even when quiet resumes after a gap. */
    s->still_ms = quiet && interval <= 8 ? s->still_ms + interval : 0;
    if (s->still_ms > 200)
        s->still_ms = 200;
    if (!s10_ready(s))
        return;

    double speed[2], relative[2];
    wheel_observations(s, speed, relative);
    const double mean = .5*(speed[0] + speed[1]);
    const double h = s->last_release == no_tick ? .001 : fmax(.001, (release - s->last_release)*.001);
    if (s->last_release == no_tick || s->air || h > .018) {
        s->fast = s->last_speed = mean;
        s->gate = 0.;
        s->alarm = 0;
    }
    s->gate += blend(h, .045)*((mean - s->last_speed)/h - s->gate);
    s->last_speed = mean;
    s->last_release = release;
    const double score = fmax(fabs(mean - s->fast)/.095, fabs(speed[1] - speed[0])/.14);
    s->fast += blend(h, .085)*(mean - s->fast);
    s->alarm = !s->air && fmax(fabs(speed[0]), fabs(speed[1])) > .15 && score >= 1.;
    if (!s->air && !s->alarm)
        s->speed += blend(h, .08)*(mean - s->speed);
}

void s10_frame(const S10Estimator *s, S10Frame *frame)
{
    double speed[2], relative[2];
    wheel_observations(s, speed, relative);
    frame->pitch = s->pitch;
    frame->roll = asin(fmax(-1., fmin(1., cos(s->pitch)*sin(s->roll))));
    frame->yaw = s->yaw;
    memcpy(frame->rate, s->rate, sizeof frame->rate);
    frame->speed = s->speed + .5*(relative[0] + relative[1]);
    frame->air = s->air;
    frame->still_update = s->still_update;
    frame->integral_update = s->integral_update;
}
