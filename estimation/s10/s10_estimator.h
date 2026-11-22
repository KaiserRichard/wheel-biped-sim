#ifndef S10_ESTIMATOR_H
#define S10_ESTIMATOR_H

/* Standalone C11 complementary estimator. All lengths, angles and rates are
 * SI. The caller rotates IMU data into body axes and decodes the five-bar
 * geometry. Pass acquisition ticks in milliseconds, after sorting each
 * stream and discarding duplicates. release is the encoder consumption tick.
 * The caller owns the state and should call init before accepting samples. */
enum { S10_STARTUP = 101 };

typedef struct {
    /* Index 0 is left and index 1 is right. spin includes the rotation of
     * the distal link relative to the torso. leg_rate holds four joint rates. */
    double position[2][3], relative[2][3], spin[2], leg_rate[4];
} S10Encoder;

typedef struct {
    double pitch, roll, yaw, rate[3], speed;
    int air, still_update, integral_update;
} S10Frame;

typedef struct {
    double radius, lever[3], ki, wheel_limit, odometry_weight;
    double pitch, roll, yaw, rate[3], alpha[3], previous_gyro[3], bias[3];
    double gyro_sum[3], force_sum[3], speed, fast, gate, last_speed, low;
    S10Encoder encoder;
    int n, imu_tick, enc_tick, last_release, still_ms;
    int air, alarm, still_update, integral_update, startup_failed;
} S10Estimator;

/* Default parameters use s10_init. ki, wheel_limit and odometry_weight can
 * be set before the first packet. Zero disables the corresponding term. */
void s10_init(S10Estimator *s, double radius, const double lever[3]);
void s10_imu(S10Estimator *s, int tick, const double gyro[3], const double force[3]);
void s10_encoder(S10Estimator *s, int tick, int release, const S10Encoder *encoder);
int s10_ready(const S10Estimator *s);
void s10_frame(const S10Estimator *s, S10Frame *frame);

#endif
