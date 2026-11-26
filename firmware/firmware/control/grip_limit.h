/*
 * TRAC3: floor-grip estimate and friction-aware host-command limiter.
 *
 * A wheeled inverted pendulum cannot decelerate its centre of mass faster than
 * mu*g, and TRAC2 showed that the controller commits the braking lean before
 * any slip evidence exists. So the limit has to be known BEFORE the brake.
 *
 * Grip estimate (an upper-bound-style "usable friction" value, 1.0 = no low
 * grip known):
 *   - S10 slip-alarm history: a slip alarm (grounded, torque on, no S11
 *     frozen/advisory mask) lowers the estimate to the recent no-slip wheel
 *     acceleration utilisation |a_wheel|/g, clamped to [mu_min, mu_slip_cap].
 *   - Wheel speed vs IMU sensor: the S10 alarm itself is the wheel/inertial
 *     speed mismatch; while there is NO alarm the filtered wheel acceleration
 *     (S10 gate, which tracks the inertial acceleration while the wheels grip)
 *     is demonstrated grip and can raise a low estimate back up.
 *   - Optional host floor hint: the estimate used is min(slip estimate, hint).
 *   - Decay: after hold_s without a slip alarm the slip estimate relaxes back
 *     towards 1.0 at recover_per_s.
 *
 * Limiter: while the used estimate is at or below mu_limit_on, the host speed /
 * accel / jerk reference is replaced by a jerk-limited tracker whose
 * acceleration magnitude never exceeds k_margin * mu * g. A commanded brake is
 * therefore stretched, not cut. With no low grip known the host reference is
 * passed through bit-for-bit.
 *
 * All constants are simulation development values, not hardware constants.
 * Pure C11, no dynamic memory.
 */
#ifndef WBR_CONTROL_GRIP_LIMIT_H
#define WBR_CONTROL_GRIP_LIMIT_H

#include <stdint.h>

typedef struct {
    float k_margin;        /* fraction of mu*g allowed as reference accel        */
    float mu_limit_on;     /* limiter eligible while mu used <= this (TRAC4)      */
    float mu_min;          /* lowest slip-derived estimate                        */
    float mu_slip_cap;     /* highest slip-derived estimate                       */
    float hold_s;          /* slip memory before decay starts                     */
    float recover_per_s;   /* decay rate of the slip estimate back to 1.0         */
    float util_tau_s;      /* low-pass of |wheel accel|/g utilisation             */
    float util_peak_decay; /* decay of the recent-peak utilisation (1/s)          */
    float demo_s;          /* demonstrated-grip dwell before raising the estimate */
    float jerk_time_s;     /* jerk limit = accel limit / jerk_time_s              */
    float kp_1_s;          /* tracker speed-error gain                            */
} GripLimitConfig;

typedef struct {
    GripLimitConfig cfg;
    /* estimate */
    float    mu_slip;       /* slip-history estimate (1.0 = none)          */
    float    mu_hint;       /* last valid host hint (0 = none)             */
    float    mu_used;       /* min(mu_slip, hint)                           */
    float    hold_left_s;
    float    util_lpf;
    float    util_peak;
    float    demo_t_s;
    uint8_t  prev_slip;
    /* limiter */
    uint8_t  active;        /* 1 while the tracker output is in use        */
    float    v_m_s;
    float    a_m_s2;
    /* diagnostics */
    uint32_t slip_events_n;
    uint32_t limit_ticks_n;
} GripLimit;

GripLimitConfig grip_limit_default_config(void);
void grip_limit_init(GripLimit *g, const GripLimitConfig *cfg);

/* Host floor hint, mu in (0, 1). Any other value (0, negative, >= 1, NaN)
 * clears the hint. Call only with a valid host message; a lost host keeps
 * the last hint. */
void grip_limit_set_hint(GripLimit *g, float mu_hint);

/* One control tick of evidence. eligible: grounded, torque on, balancing /
 * stop / rest, no S11 frozen/advisory mask. slip_alarm: S10 alarm.
 * wheel_accel_m_s2: S10 filtered wheel-mean acceleration. */
void grip_limit_observe(GripLimit *g, float dt_s, uint8_t eligible,
                        uint8_t slip_alarm, float wheel_accel_m_s2);

/* Limit one host reference sample. enable = 0 (e.g. host lost) forces
 * pass-through and resynchronises the tracker. Returns 1 if the output
 * differs from the input (tracker in use). */
uint8_t grip_limit_ref(GripLimit *g, float dt_s, uint8_t enable,
                       float v_cmd, float a_cmd, float j_cmd,
                       float *v_out, float *a_out, float *j_out);

/* Reference acceleration limit currently applied (0 when not limiting). */
float grip_limit_accel_max(const GripLimit *g);

#endif /* WBR_CONTROL_GRIP_LIMIT_H */
