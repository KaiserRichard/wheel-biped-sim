/*
 * Standalone C11 Complementary Estimator (S10)
 *
 * Ported from source-reference
 * with pure-C five-bar encoder kinematics helper matching g3_packets.py.
 *
 * Pure C11, zero HAL/RTOS dependencies, zero dynamic memory.
 */

#ifndef S10_ESTIMATOR_H
#define S10_ESTIMATOR_H

#include <stdint.h>

enum { S10_STARTUP = 101 };

typedef struct {
    /* Index 0 is left and index 1 is right. spin includes the rotation of
     * the distal link relative to the torso. leg_rate holds four joint rates. */
    double position[2][3];
    double relative[2][3];
    double spin[2];
    double leg_rate[4];
} S10Encoder;

typedef struct {
    double  pitch;
    double  roll;
    double  yaw;
    double  rate[3];
    double  speed;
    int32_t air;
    int32_t still_update;
    int32_t integral_update;
} S10Frame;

typedef struct {
    double     radius;
    double     lever[3];
    double     ki;
    double     wheel_limit;
    double     odometry_weight;
    double     pitch;
    double     roll;
    double     yaw;
    double     rate[3];
    double     alpha[3];
    double     previous_gyro[3];
    double     bias[3];
    double     gyro_sum[3];
    double     force_sum[3];
    double     speed;
    double     fast;
    double     gate;
    double     last_speed;
    double     low;
    S10Encoder encoder;
    int32_t    n;
    int32_t    imu_tick;
    int32_t    enc_tick;
    int32_t    last_release;
    int32_t    still_ms;
    int32_t    air;
    int32_t    alarm;
    int32_t    still_update;
    int32_t    integral_update;
    int32_t    startup_failed;
    int32_t    robust;
    int32_t    encoder_interval;
    double     common_error;
    double     wheel_difference;
} S10Estimator;

/* Five-bar geometry constants for decoding raw 6-axis encoder angles/rates into S10Encoder. */
typedef struct {
    double upper_m;
    double lower_m;
    double half_hip_m;
    double wheel_y_m;
    double hip_centre_xz[2][2]; /* [side][0=x, 1=z] */
    double knee_offset_xz[2][2];/* [side][0=kx, 1=kz] of front_lower in front_upper frame */
    double front_angle_offset[2]; /* [side] atan2(+kx, -kz) */
} S10FiveBarGeom;

S10FiveBarGeom s10_default_fivebar_geom(void);

/* Decode genuine fresh 6-axis encoder angles & rates [L_f, L_r, L_w, R_f, R_r, R_w]
 * into S10Encoder body-frame wheel kinematics. Returns 1 on success, 0 on invalid geometry. */
uint8_t s10_decode_fivebar(const S10FiveBarGeom *geom,
                           const double angle[6],
                           const double rate[6],
                           S10Encoder *out_enc);

void    s10_init(S10Estimator *s, double radius, const double lever[3]);
void    s10_imu(S10Estimator *s, int32_t tick, const double gyro[3], const double force[3]);
void    s10_encoder(S10Estimator *s, int32_t tick, int32_t release, const S10Encoder *encoder);
int32_t s10_ready(const S10Estimator *s);
void    s10_frame(const S10Estimator *s, S10Frame *frame);

#endif /* S10_ESTIMATOR_H */
