/*
 * FW10 bench bring-up modes (portable, host-tested).
 *
 * Three compile-time images share one firmware line:
 *   BRINGUP_MODE_RUN      - FW9 behaviour, no extra gating (stage 6+).
 *   BRINGUP_MODE_BENCH    - stages 0-2 of the bench bring-up plan. Torque is
 *                           impossible: arm/reset/handoff requests are dropped
 *                           before the safety FSM, the PD11 permit heartbeat
 *                           is held LOW, and the CAN TX choke point only lets
 *                           the 0xFF..FD drive-disable frame through.
 *   BRINGUP_MODE_MOTOR_ID - stage 3. Same as BENCH, plus a single-axis,
 *                           pure-torque (kp = kd = 0) MIT pulse of at most
 *                           BRINGUP_MOTOR_ID_TORQUE_CAP_NM for at most
 *                           BRINGUP_MOTOR_ID_MAX_PULSE_MS, requested over USB.
 *                           Every pulse is gated behind the wired e-stop:
 *                           PB12 and PD7 LOW, no latched trip, no pending EXTI.
 *
 * Pure C11, zero HAL dependencies, zero dynamic memory.
 * Simulation/design only: no part of this module has run on hardware.
 */

#ifndef WBR_BRINGUP_MODE_H
#define WBR_BRINGUP_MODE_H

#include "control/balance_ctrl.h"
#include "damiao_mit.h"

#include <stdint.h>

typedef enum {
    BRINGUP_MODE_RUN      = 0,
    BRINGUP_MODE_BENCH    = 1,
    BRINGUP_MODE_MOTOR_ID = 2
} BringupMode;

/* Stage-3 limits (firmware safety review, section 5 item 2: 0.20 Nm cap; bench plan
 * 3.1c: 200 ms pulse). Requests outside these limits are rejected, not clamped. */
#define BRINGUP_MOTOR_ID_TORQUE_CAP_NM   0.20f
#define BRINGUP_MOTOR_ID_MAX_PULSE_MS    200u
#define BRINGUP_MOTOR_ID_COOLDOWN_MS     500u
#define BRINGUP_MOTOR_ID_REQ_MAX_AGE_US  100000u
/* Encoder poll: one 0xFF..FD frame per 500 Hz tick, round-robin over 6 axes. */
#define BRINGUP_MAX_FRAMES               8u
#define BRINGUP_NO_AXIS                  0xFFu

typedef enum {
    BRINGUP_ST_IDLE     = 0,
    BRINGUP_ST_ENABLE   = 1,
    BRINGUP_ST_PULSE    = 2,
    BRINGUP_ST_DISABLE  = 3,
    BRINGUP_ST_COOLDOWN = 4
} BringupState;

typedef enum {
    BRINGUP_REJ_NONE      = 0,
    BRINGUP_REJ_MODE      = 1, /* not a MOTOR_ID image */
    BRINGUP_REJ_ESTOP     = 2, /* wired stop open, latched or pending */
    BRINGUP_REJ_FSM       = 3, /* balance FSM not DISARMED */
    BRINGUP_REJ_AXIS      = 4,
    BRINGUP_REJ_TORQUE    = 5, /* non-finite or above cap */
    BRINGUP_REJ_DURATION  = 6,
    BRINGUP_REJ_STALE     = 7, /* request older than 100 ms */
    BRINGUP_REJ_BUSY      = 8, /* pulse or cool-down in progress */
    BRINGUP_REJ_ABORTED   = 9  /* gate dropped during a pulse */
} BringupReject;

typedef struct {
    uint8_t  valid;       /* a request has been received at least once */
    uint32_t seq;         /* USB link header sequence number */
    uint32_t rx_us;       /* receive time (target_micros) */
    uint8_t  axis;        /* 0..5 */
    float    torque_nm;
    uint16_t duration_ms;
} BringupMotorIdRequest;

typedef struct {
    uint32_t now_us;
    uint8_t  estop_pin_open;   /* PB12 or PD7 HIGH */
    uint8_t  estop_latched;
    uint8_t  estop_pending;    /* EXTI pending bit set for PB12 or PD7 */
    uint8_t  fsm_disarmed;     /* balance safety FSM state == DISARMED */
    BringupMotorIdRequest req;
} BringupTickInput;

typedef struct {
    uint8_t  bus;              /* 0..2 (FDCAN1..3) */
    uint16_t std_id;
    uint8_t  dlc;
    uint8_t  data[DAMIAO_CAN_DLC];
} BringupFrame;

typedef struct {
    uint8_t      n_frames;
    BringupFrame frames[BRINGUP_MAX_FRAMES];
    uint8_t      permit;        /* PD11 heartbeat allowed this tick */
    uint8_t      state;         /* BringupState */
    uint8_t      active_axis;   /* BRINGUP_NO_AXIS when idle */
    uint8_t      last_reject;   /* BringupReject of the latest request */
    float        torque_nm;     /* commanded pulse torque (0 when idle) */
} BringupTickOutput;

typedef struct {
    BringupMode mode;
    uint8_t     state;
    uint8_t     axis;
    float       torque_nm;
    uint16_t    duration_ms;
    uint32_t    phase_us;
    uint8_t     have_seq;
    uint32_t    last_seq;
    uint8_t     poll_axis;
    uint8_t     last_reject;
    /* Authorisation read by the CAN TX choke point (bringup_tx_allowed). It is
     * only ever an axis while a MOTOR_ID pulse is in ENABLE/PULSE. */
    volatile uint8_t tx_auth_axis;
    uint32_t    pulses_done;
    uint32_t    requests_rejected;
    uint32_t    aborts;
    uint32_t    bench_violations;
} BringupCtx;

void bringup_init(BringupCtx *ctx, BringupMode mode);

/* Neutralise every operator request that could advance the safety FSM towards
 * a torque state. No-op in RUN. Called on BalanceTickInput before the tick. */
void bringup_gate_tick_input(const BringupCtx *ctx, BalanceTickInput *in);

/* Post-tick check. Returns 1 if the balance controller asked for torque or a
 * drive enable in BENCH/MOTOR_ID (must never happen); the caller then trips the
 * latched e-stop. Also zeroes torque_cmd[] in that case. */
uint8_t bringup_check_tick_output(BringupCtx *ctx, BalanceTickOutput *out);

/* One 500 Hz step: encoder poll frames and the MOTOR_ID pulse state machine. */
void bringup_step(BringupCtx *ctx, const BringupTickInput *in, BringupTickOutput *out);

/* PD11 heartbeat permission. RUN: the FSM permit. BENCH: never. MOTOR_ID: only
 * while the wired stop chain is healthy (both inputs LOW, nothing latched). */
uint8_t bringup_permit(const BringupCtx *ctx, uint8_t fsm_permit, const BringupTickInput *in);

/* Final CAN TX gate, called for every frame at the hardware choke point.
 * RUN: 1. BENCH: only an exact 0xFF x7 0xFD frame to a rev4 command ID on its
 * own bus. MOTOR_ID: disable frames as BENCH, plus enable / MIT frames to the
 * single authorised axis, MIT only with kp = kd = 0 and |tau| <= cap. */
uint8_t bringup_tx_allowed(const BringupCtx *ctx, uint8_t bus_idx, uint16_t std_id,
                           uint8_t dlc, const uint8_t data[DAMIAO_CAN_DLC]);

/* Helpers shared with tests. */
uint8_t bringup_frame_is_disable(uint8_t dlc, const uint8_t data[DAMIAO_CAN_DLC]);
uint8_t bringup_frame_is_enable(uint8_t dlc, const uint8_t data[DAMIAO_CAN_DLC]);
uint8_t bringup_axis_for_cmd_id(uint8_t bus_idx, uint16_t std_id); /* BRINGUP_NO_AXIS if none */

#endif /* WBR_BRINGUP_MODE_H */
