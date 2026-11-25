/* STEP3RC part B: frozen-encoder detector, bound per axis into s11_packet_policy.c
 * since FW6 (replaces s11_update_stuck). One instance per axis plus one shared IMU
 * transient gate.
 *
 * Evidence: inside a run of unchanged position code, d_k = v_eff,k * dt / LSB where
 * v_eff is the reported rate minus one velocity count (90/4095 rad/s) toward zero.
 * Thresholds come from the measured healthy envelope (110 FIL runs, 976k ticks):
 *   ungated single sample |d| max 6.84 LSB; gated (IMU quiet) rectified sum max 4.69,
 *   gated |signed sum| max 3.23 LSB over any window up to 64 ms.
 *
 *   GROSS    |d| >= 10 LSB in one sample (ungated)            -> CONFIRMED
 *   RECT     sum|d| over the last 8 ms >= 7 LSB (gated)         -> CONFIRMED
 *   NET      |sum d| over the run, capped at 64 ms, >= 5 LSB   -> CONFIRMED
 *   ADVISORY sum|d| over the run, capped at 64 ms, >= 6 LSB    -> ADVISORY (log/flag only)
 *
 * CONFIRMED latches until s11fz_reset(). ADVISORY clears on a code change or gate.
 */
#ifndef S11_FROZEN_H
#define S11_FROZEN_H

#include <stdint.h>

#define S11FZ_RING 64u

typedef enum {
    S11FZ_OK = 0,
    S11FZ_ADVISORY = 1,
    S11FZ_CONFIRMED = 2
} S11FzState;

typedef enum {
    S11FZ_R_NONE = 0,
    S11FZ_R_GROSS = 1,
    S11FZ_R_RECT = 2,
    S11FZ_R_NET = 3,
    S11FZ_R_ADVISORY = 4
} S11FzReason;

typedef struct {
    double lsb_rad;         /* axis position LSB: hips 25/65536, wheels 24/65535 */
    double rate_deadband;   /* 90/4095 rad/s */
    double gross_lsb;       /* 10 */
    double rect_lsb;        /* 7 */
    double rect_window_ms;  /* 8 */
    double net_lsb;         /* 5 */
    double advisory_lsb;    /* 6 */
    double run_cap_ms;      /* 64 */
} S11FzConfig;

typedef struct {
    double gyro_rad_s;      /* 0.7: |gyro_x| or |gyro_y| above -> transient */
    double acc_dev_m_s2;    /* 8.0: ||a| - g| above -> transient */
    double hold_ms;         /* 60 */
    double left_ms;
} S11FzGate;

typedef struct {
    uint8_t     seen;
    uint8_t     restart;    /* next same-code sample starts a new run (after a gate) */
    double      last_angle;
    double      d[S11FZ_RING];
    double      dt[S11FZ_RING];
    uint32_t    n;          /* valid entries, newest at head-1 */
    uint32_t    head;
    S11FzState  state;
    S11FzReason reason;
    double      peak_ratio[4]; /* diagnostics: max metric/threshold for GROSS,RECT,NET,ADVISORY */
} S11FzAxis;

S11FzConfig s11fz_default_config(int wheel);
void        s11fz_gate_init(S11FzGate *g);
/* Returns 1 while the chassis shows an impact/fall transient (evidence is reset). */
int         s11fz_gate_update(S11FzGate *g, double gyro_x, double gyro_y,
                              double acc_norm, double dt_ms);
void        s11fz_reset(S11FzAxis *a);
S11FzState  s11fz_step(const S11FzConfig *c, S11FzAxis *a, double angle, double rate,
                       double dt_ms, int gated);

#endif
