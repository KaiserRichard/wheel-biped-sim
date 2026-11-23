/*
 * Standalone C11 Complementary Estimator (S10)
 *
 * Ported from source-reference
 * with exact double-precision constants and pure-C five-bar encoder decode.
 */

#include "s10_estimator.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

static const double s10_gravity = 9.80665;
static const int32_t s10_no_tick = INT32_MIN;

static double s10_blend(double h, double tau)
{
    return -expm1(-h / tau);
}

static double s10_accel_pitch(const double f[3])
{
    return atan2(-f[0], hypot(f[1], f[2]));
}

static void s10_cross(const double a[3], const double b[3], double c[3])
{
    c[0] = a[1] * b[2] - a[2] * b[1];
    c[1] = a[2] * b[0] - a[0] * b[2];
    c[2] = a[0] * b[1] - a[1] * b[0];
}

S10FiveBarGeom s10_default_fivebar_geom(void)
{
    /* Exact H3 CAD geometry from params_cad.json and sim_model/draw_001.xml. */
    S10FiveBarGeom g;
    g.upper_m = 0.07249993968718672;
    g.lower_m = 0.14000000823631453;
    g.half_hip_m = 0.5 * 0.07900003157632506;
    g.wheel_y_m = 0.5 * 0.3134999252050108;

    g.hip_centre_xz[0][0] = 0.5 * (0.039500099590052 + (-0.039499931986263));
    g.hip_centre_xz[0][1] = 0.5 * (0.0364998750588717 + 0.0364999149166151);
    g.hip_centre_xz[1][0] = 0.5 * (0.0395000000000003 + (-0.0394999999999986));
    g.hip_centre_xz[1][1] = 0.5 * (0.0364999999999997 + 0.0364999999999985);

    g.knee_offset_xz[0][0] = 0.0613996669346546;
    g.knee_offset_xz[0][1] = 0.0385528488565916;
    g.knee_offset_xz[1][0] = 0.0613998538757104;
    g.knee_offset_xz[1][1] = 0.0385526645517722;

    g.front_angle_offset[0] = atan2(g.knee_offset_xz[0][0], -g.knee_offset_xz[0][1]);
    g.front_angle_offset[1] = atan2(g.knee_offset_xz[1][0], -g.knee_offset_xz[1][1]);
    return g;
}

uint8_t s10_decode_fivebar(const S10FiveBarGeom *geom,
                           const double angle[6],
                           const double rate[6],
                           S10Encoder *out_enc)
{
    if (geom == NULL || angle == NULL || rate == NULL || out_enc == NULL) {
        return 0u;
    }
    memset(out_enc, 0, sizeof(*out_enc));

    for (uint32_t i = 0u; i < 2u; ++i) {
        const uint32_t j = 3u * i;
        const double front = angle[j];
        const double rear = angle[j + 1u];
        const double fdot = rate[j];
        const double rdot = rate[j + 1u];

        const double sf = sin(front);
        const double cf = cos(front);
        const double sr = sin(rear);
        const double cr = cos(rear);
        const double u = geom->upper_m;

        const double fx = geom->half_hip_m + u * sf;
        const double fz = -u * cf;
        const double rx = -geom->half_hip_m - u * sr;
        const double rz = -u * cr;
        const double dx = rx - fx;
        const double dz = rz - fz;
        const double distance_sq = dx * dx + dz * dz;
        if (distance_sq <= 1e-12) {
            return 0u;
        }
        const double rad = (geom->lower_m * geom->lower_m) / distance_sq - 0.25;
        if (rad < 0.0) {
            return 0u;
        }
        double offset = sqrt(rad);
        if (dx < 0.0) {
            offset = -offset;
        }

        const double x = 0.5 * (fx + rx) + offset * dz;
        const double z = 0.5 * (fz + rz) - offset * dx;
        const double a = x - fx;
        const double b = z - fz;
        const double c = x - rx;
        const double d = z - rz;
        const double p = u * fdot * (a * cf + b * sf);
        const double r = u * rdot * (-c * cr + d * sr);
        const double det = a * d - b * c;
        if (fabs(det) <= 1e-12) {
            return 0u;
        }
        const double vx = (d * p - b * r) / det;
        const double vz = (a * r - c * p) / det;

        const double cx = geom->hip_centre_xz[i][0];
        const double cz = geom->hip_centre_xz[i][1];
        const double y = (i == 0u) ? geom->wheel_y_m : -geom->wheel_y_m;

        out_enc->position[i][0] = cx + x;
        out_enc->position[i][1] = y;
        out_enc->position[i][2] = cz + z;

        out_enc->relative[i][0] = vx;
        out_enc->relative[i][1] = 0.0;
        out_enc->relative[i][2] = vz;

        const double k_ang = geom->front_angle_offset[i] - front;
        const double co = cos(k_ang);
        const double si = sin(k_ang);
        const double kx0 = geom->knee_offset_xz[i][0];
        const double kz0 = geom->knee_offset_xz[i][1];
        const double kx = co * kx0 + si * kz0;
        const double kz = -si * kx0 + co * kz0;
        const double lx = x - geom->half_hip_m - kx;
        const double lz = z - kz;
        const double l_sq = lx * lx + lz * lz;
        if (l_sq <= 1e-12) {
            return 0u;
        }
        const double lower_rate = (lz * (vx + fdot * kz) - lx * (vz - fdot * kx)) / l_sq;
        out_enc->spin[i] = rate[j + 2u] + lower_rate;
        out_enc->leg_rate[2u * i] = rdot;
        out_enc->leg_rate[2u * i + 1u] = -fdot;
    }

    return 1u;
}

void s10_init(S10Estimator *s, double radius, const double lever[3])
{
    if (s == NULL || lever == NULL) {
        return;
    }
    memset(s, 0, sizeof(*s));
    s->radius = radius;
    memcpy(s->lever, lever, sizeof(s->lever));
    s->ki = 1.0;
    s->wheel_limit = 0.12;
    s->odometry_weight = 1.0;
    s->imu_tick = s10_no_tick;
    s->enc_tick = s10_no_tick;
    s->last_release = s10_no_tick;
}

int32_t s10_ready(const S10Estimator *s)
{
    if (s == NULL) {
        return 0;
    }
    return (s->n == S10_STARTUP && s->startup_failed == 0 && s->enc_tick != s10_no_tick) ? 1 : 0;
}

void s10_imu(S10Estimator *s, int32_t tick, const double gyro[3], const double force[3])
{
    if (s == NULL || gyro == NULL || force == NULL) {
        return;
    }
    if (tick <= s->imu_tick || s->startup_failed != 0) {
        return;
    }
    const double h = (s->imu_tick == s10_no_tick) ? 0.001 : ((double)(tick - s->imu_tick) * 0.001);
    s->imu_tick = tick;
    if (s->n < S10_STARTUP) {
        for (int32_t axis = 0; axis < 3; ++axis) {
            s->gyro_sum[axis] += gyro[axis];
            s->force_sum[axis] += force[axis];
            s->previous_gyro[axis] = gyro[axis];
        }
        if (++s->n == S10_STARTUP) {
            const double *fs = s->force_sum;
            if (hypot(fs[0], hypot(fs[1], fs[2])) <= 0.65 * s10_gravity * (double)S10_STARTUP) {
                s->startup_failed = 1;
                return;
            }
            s->pitch = s10_accel_pitch(fs);
            s->roll = atan2(fs[1], fs[2]);
            for (int32_t axis = 0; axis < 3; ++axis) {
                s->bias[axis] = s->gyro_sum[axis] / (double)S10_STARTUP;
                s->rate[axis] = gyro[axis] - s->bias[axis];
            }
        }
        return;
    }

    const double ka = s10_blend(h, 0.03);
    for (int32_t axis = 0; axis < 3; ++axis) {
        s->alpha[axis] += ka * ((gyro[axis] - s->previous_gyro[axis]) / h - s->alpha[axis]);
        s->previous_gyro[axis] = gyro[axis];
        s->rate[axis] = gyro[axis] - s->bias[axis];
    }
    double tangential[3];
    double cross_rate[3];
    double centripetal[3];
    double f[3];
    s10_cross(s->alpha, s->lever, tangential);
    s10_cross(s->rate, s->lever, cross_rate);
    s10_cross(s->rate, cross_rate, centripetal);
    for (int32_t axis = 0; axis < 3; ++axis) {
        f[axis] = force[axis] - tangential[axis] - centripetal[axis];
    }

    const double norm = hypot(f[0], hypot(f[1], f[2]));
    s->low = (norm < 0.30 * s10_gravity) ? (s->low + h) : 0.0;
    if (s->low >= 0.004) {
        s->air = 1;
    } else if (norm > 0.65 * s10_gravity) {
        s->air = 0;
    }

    s->pitch += h * s->rate[1];
    s->roll += h * s->rate[0];
    s->yaw += h * s->rate[2];
    const double ax = cos(s->pitch) * f[0] + sin(s->pitch) * f[2];
    s->speed += h * ax;
    s->fast += h * ax;

    const int32_t accel_ok = (s->air == 0 && s->alarm == 0 && fabs(norm / s10_gravity - 1.0) < 0.10
                              && fabs(ax) < 0.40 && fabs(s->gate) < 0.35) ? 1 : 0;
    const double residual = s10_accel_pitch(f) - s->pitch;
    const int32_t fresh = (h <= 0.008 && s->enc_tick != s10_no_tick && tick >= s->enc_tick
                           && (tick - s->enc_tick) <= ((s->robust != 0) ? 3 : 8)
                           && (s->robust == 0 || s->encoder_interval <= 2)) ? 1 : 0;
    const int32_t wheel_trusted = (fresh != 0 && s->air == 0
                                   && fabs(norm / s10_gravity - 1.0) < 0.10
                                   && fabs(s->gate) < 0.55
                                   && fabs(s->common_error) < 0.065
                                   && fabs(s->wheel_difference) < 0.08) ? 1 : 0;
    const double compensated = residual + atan2(s->gate, s10_gravity);
    if (s->robust != 0 && wheel_trusted != 0) {
        s->pitch += s10_blend(h, 0.55) * fmax(-0.055, fmin(0.055, compensated));
    } else if (accel_ok != 0 && s->robust == 0) {
        s->pitch += s10_blend(h, 1.0) * residual;
    }
    if (accel_ok != 0) {
        const double kp = s10_blend(h, 1.0);
        s->roll += kp * (atan2(f[1], f[2]) - s->roll);
    }

    s->still_update = (fresh != 0 && accel_ok != 0 && fabs(ax) < 0.08 && fabs(s->gate) < 0.12
                       && s->still_ms >= 200 && fabs(s->rate[0]) < 0.015
                       && fabs(s->rate[1]) < 0.015 && fabs(s->rate[2]) < 0.015) ? 1 : 0;
    const double bias_residual = residual + s->odometry_weight * s->gate / s10_gravity;
    if (s->robust != 0) {
        s->integral_update = (wheel_trusted != 0 && s->still_update == 0 && s->ki > 0.0) ? 1 : 0;
    } else {
        s->integral_update = (fresh != 0 && accel_ok != 0 && s->still_update == 0 && s->ki > 0.0
                              && fabs(bias_residual) < 0.014) ? 1 : 0;
    }
    double delta = 0.0;
    if (s->still_update != 0) {
        delta = s10_blend(h, 1.0) * s->rate[1];
    } else if (s->integral_update != 0) {
        if (s->robust != 0) {
            delta = -1.6 * h * s->ki * compensated;
        } else {
            delta = -h * s->ki * bias_residual;
        }
    }
    if (s->robust != 0) {
        delta = fmax(-0.018 * h, fmin(0.018 * h, delta));
    }
    if (s->still_update != 0 || s->integral_update != 0) {
        s->bias[1] = fmax(-0.08, fmin(0.08, s->bias[1] + delta));
        s->rate[1] = gyro[1] - s->bias[1];
    }
}

static void s10_wheel_observations(const S10Estimator *s, double speed[2], double relative[2])
{
    const double c = cos(s->pitch);
    const double p = sin(s->pitch);
    for (int32_t side = 0; side < 2; ++side) {
        double rotational[3];
        s10_cross(s->rate, s->encoder.position[side], rotational);
        relative[side] = c * (rotational[0] + s->encoder.relative[side][0])
                         + p * (rotational[2] + s->encoder.relative[side][2]);
        speed[side] = s->radius * (s->rate[1] + s->encoder.spin[side]) - relative[side];
    }
}

void s10_encoder(S10Estimator *s, int32_t tick, int32_t release, const S10Encoder *encoder)
{
    if (s == NULL || encoder == NULL) {
        return;
    }
    if (tick <= s->enc_tick || release < s->last_release) {
        return;
    }
    const int32_t interval = (s->enc_tick == s10_no_tick) ? 0 : (tick - s->enc_tick);
    s->encoder_interval = interval;
    s->enc_tick = tick;
    s->encoder = *encoder;

    int32_t quiet = (s->wheel_limit > 0.0) ? 1 : 0;
    for (int32_t side = 0; side < 2; ++side) {
        if (fabs(encoder->spin[side]) >= s->wheel_limit) {
            quiet = 0;
        }
    }
    for (int32_t joint = 0; joint < 4; ++joint) {
        if (fabs(encoder->leg_rate[joint]) >= 0.11) {
            quiet = 0;
        }
    }
    /* Require a complete observed dwell, even when quiet resumes after a gap. */
    s->still_ms = (quiet != 0 && interval <= ((s->robust != 0) ? 2 : 8))
                ? (s->still_ms + interval) : 0;
    if (s->still_ms > 200) {
        s->still_ms = 200;
    }
    if (s10_ready(s) == 0) {
        return;
    }

    double speed[2];
    double relative[2];
    s10_wheel_observations(s, speed, relative);
    const double mean = 0.5 * (speed[0] + speed[1]);
    const double h = (s->last_release == s10_no_tick)
                   ? 0.001
                   : fmax(0.001, (double)(release - s->last_release) * 0.001);
    if (s->last_release == s10_no_tick || s->air != 0 || h > 0.018) {
        s->fast = mean;
        s->last_speed = mean;
        s->gate = 0.0;
        s->alarm = 0;
    }
    s->gate += s10_blend(h, 0.045) * ((mean - s->last_speed) / h - s->gate);
    s->last_speed = mean;
    s->last_release = release;
    s->common_error = mean - s->fast;
    s->wheel_difference = speed[1] - speed[0];
    const double score = fmax(fabs(s->common_error) / 0.095, fabs(s->wheel_difference) / 0.14);
    s->fast += s10_blend(h, 0.085) * s->common_error;
    s->alarm = (s->air == 0 && fmax(fabs(speed[0]), fabs(speed[1])) > 0.15 && score >= 1.0) ? 1 : 0;
    if (s->robust != 0 && s->air == 0 && fabs(s->wheel_difference) < 0.14) {
        /* Two agreeing wheels can recapture a drifting inertial speed.
         * Bound the correction when common-mode slip remains plausible. */
        const double error = fmax(-0.10, fmin(0.10, mean - s->speed));
        s->speed += s10_blend(h, 0.04) * error;
    } else if (s->air == 0 && s->alarm == 0) {
        s->speed += s10_blend(h, 0.08) * (mean - s->speed);
    }
}

void s10_frame(const S10Estimator *s, S10Frame *frame)
{
    if (s == NULL || frame == NULL) {
        return;
    }
    double speed[2];
    double relative[2];
    s10_wheel_observations(s, speed, relative);
    frame->pitch = s->pitch;
    frame->roll = asin(fmax(-1.0, fmin(1.0, cos(s->pitch) * sin(s->roll))));
    frame->yaw = s->yaw;
    memcpy(frame->rate, s->rate, sizeof(frame->rate));
    frame->speed = s->speed + 0.5 * (relative[0] + relative[1]);
    frame->air = s->air;
    frame->still_update = s->still_update;
    frame->integral_update = s->integral_update;
}
