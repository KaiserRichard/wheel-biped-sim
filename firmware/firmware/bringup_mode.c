/*
 * FW10 bench bring-up modes. See bringup_mode.h.
 * Simulation/design only: host-tested, never run on hardware.
 */

#include "bringup_mode.h"
#include "can_sched.h"

#include <math.h>
#include <string.h>

static uint8_t gate_estop_healthy(const BringupTickInput *in)
{
    return (in->estop_pin_open == 0u && in->estop_latched == 0u
            && in->estop_pending == 0u) ? 1u : 0u;
}

uint8_t bringup_frame_is_disable(uint8_t dlc, const uint8_t data[DAMIAO_CAN_DLC])
{
    if (data == NULL || dlc != DAMIAO_CAN_DLC || data[7] != DAMIAO_CMD_DISABLE_BYTE) {
        return 0u;
    }
    for (uint8_t i = 0u; i < 7u; ++i) {
        if (data[i] != 0xFFu) {
            return 0u;
        }
    }
    return 1u;
}

uint8_t bringup_frame_is_enable(uint8_t dlc, const uint8_t data[DAMIAO_CAN_DLC])
{
    if (data == NULL || dlc != DAMIAO_CAN_DLC || data[7] != DAMIAO_CMD_ENABLE_BYTE) {
        return 0u;
    }
    for (uint8_t i = 0u; i < 7u; ++i) {
        if (data[i] != 0xFFu) {
            return 0u;
        }
    }
    return 1u;
}

uint8_t bringup_axis_for_cmd_id(uint8_t bus_idx, uint16_t std_id)
{
    for (uint8_t axis = 0u; axis < DAMIAO_NUM_AXES; ++axis) {
        DamiaoAxisConfig cfg;
        uint8_t bus = 0u;
        if (damiao_get_axis_config(axis, &cfg) != DAMIAO_OK
            || can_sched_bus_for_axis(axis, &bus) != CAN_SCHED_OK) {
            continue;
        }
        if (cfg.cmd_can_id == std_id && bus == bus_idx) {
            return axis;
        }
    }
    return BRINGUP_NO_AXIS;
}

void bringup_init(BringupCtx *ctx, BringupMode mode)
{
    if (ctx == NULL) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    ctx->mode = (mode == BRINGUP_MODE_RUN || mode == BRINGUP_MODE_MOTOR_ID)
              ? mode : BRINGUP_MODE_BENCH; /* unknown value -> most restrictive */
    ctx->state = BRINGUP_ST_IDLE;
    ctx->axis = BRINGUP_NO_AXIS;
    ctx->tx_auth_axis = BRINGUP_NO_AXIS;
}

void bringup_gate_tick_input(const BringupCtx *ctx, BalanceTickInput *in)
{
    if (ctx == NULL || in == NULL || ctx->mode == BRINGUP_MODE_RUN) {
        return;
    }
    /* No path out of DISARMED: no arm, reset, latch ack, re-grant, handoff,
     * jump or step request reaches the FSM, and the motion targets are zero. */
    in->local_arm_req      = 0u;
    in->local_reset_req    = 0u;
    in->hw_reset_ack       = 0u;
    in->local_regrant_req  = 0u;
    in->stand_handoff_req  = 0u;
    in->jump_req           = 0u;
    in->radio_arm_sw       = 0u;
    in->step_declaration   = (uint8_t)BALANCE_STEP_DECL_NONE;
    in->v_target_m_s       = 0.0f;
    in->yaw_target_rad_s   = 0.0f;
    in->accel_ref_m_s2     = 0.0f;
    in->jerk_ref_m_s3      = 0.0f;
}

uint8_t bringup_check_tick_output(BringupCtx *ctx, BalanceTickOutput *out)
{
    if (ctx == NULL || out == NULL || ctx->mode == BRINGUP_MODE_RUN) {
        return 0u;
    }
    uint8_t violation = (out->safety_out.torque_enable != 0u
                         || out->safety_out.can_enable_cmd != 0u) ? 1u : 0u;
    for (uint32_t i = 0u; i < 6u; ++i) {
        if (out->torque_cmd[i] != 0.0f) {
            violation = 1u;
        }
    }
    if (violation != 0u) {
        for (uint32_t i = 0u; i < 6u; ++i) {
            out->torque_cmd[i] = 0.0f;
        }
        if (ctx->bench_violations < UINT32_MAX) {
            ctx->bench_violations++;
        }
    }
    return violation;
}

uint8_t bringup_permit(const BringupCtx *ctx, uint8_t fsm_permit, const BringupTickInput *in)
{
    if (ctx == NULL) {
        return 0u;
    }
    switch (ctx->mode) {
    case BRINGUP_MODE_RUN:
        return fsm_permit != 0u ? 1u : 0u;
    case BRINGUP_MODE_MOTOR_ID:
        return (in != NULL && gate_estop_healthy(in) != 0u) ? 1u : 0u;
    case BRINGUP_MODE_BENCH:
    default:
        return 0u;
    }
}

static void push_frame(BringupTickOutput *out, uint8_t axis, uint8_t kind, float torque_nm)
{
    if (out->n_frames >= BRINGUP_MAX_FRAMES || axis >= DAMIAO_NUM_AXES) {
        return;
    }
    DamiaoAxisConfig cfg;
    uint8_t bus = 0u;
    if (damiao_get_axis_config(axis, &cfg) != DAMIAO_OK
        || can_sched_bus_for_axis(axis, &bus) != CAN_SCHED_OK) {
        return;
    }
    BringupFrame *f = &out->frames[out->n_frames];
    memset(f, 0, sizeof(*f));
    f->bus = bus;
    f->std_id = cfg.cmd_can_id;
    f->dlc = DAMIAO_CAN_DLC;
    if (kind == 0u) {
        damiao_pack_disable(f->data);
    } else if (kind == 1u) {
        damiao_pack_enable(f->data);
    } else {
        DamiaoMitCmd cmd;
        memset(&cmd, 0, sizeof(cmd));
        cmd.torque = torque_nm; /* pos/vel/kp/kd = 0: pure feed-forward torque */
        if (damiao_pack_mit_cmd_clamped(&cfg.limits, &cmd, f->data) != DAMIAO_OK) {
            return;
        }
    }
    out->n_frames++;
}

static void reject(BringupCtx *ctx, uint8_t why)
{
    ctx->last_reject = why;
    if (ctx->requests_rejected < UINT32_MAX) {
        ctx->requests_rejected++;
    }
}

static void enter_cooldown(BringupCtx *ctx, uint32_t now_us)
{
    ctx->state = BRINGUP_ST_COOLDOWN;
    ctx->phase_us = now_us;
    ctx->tx_auth_axis = BRINGUP_NO_AXIS;
    ctx->torque_nm = 0.0f;
}

static void abort_pulse(BringupCtx *ctx, BringupTickOutput *out, uint32_t now_us)
{
    ctx->tx_auth_axis = BRINGUP_NO_AXIS;
    for (uint8_t a = 0u; a < DAMIAO_NUM_AXES; ++a) {
        push_frame(out, a, 0u, 0.0f);
    }
    if (ctx->aborts < UINT32_MAX) {
        ctx->aborts++;
    }
    ctx->last_reject = BRINGUP_REJ_ABORTED;
    enter_cooldown(ctx, now_us);
    ctx->axis = BRINGUP_NO_AXIS;
}

/* Validate a fresh MOTOR_ID request. Returns BRINGUP_REJ_NONE if acceptable. */
static uint8_t validate_request(const BringupCtx *ctx, const BringupTickInput *in)
{
    const BringupMotorIdRequest *r = &in->req;
    if (ctx->mode != BRINGUP_MODE_MOTOR_ID) {
        return BRINGUP_REJ_MODE;
    }
    if ((uint32_t)(in->now_us - r->rx_us) > BRINGUP_MOTOR_ID_REQ_MAX_AGE_US) {
        return BRINGUP_REJ_STALE;
    }
    if (gate_estop_healthy(in) == 0u) {
        return BRINGUP_REJ_ESTOP;
    }
    if (in->fsm_disarmed == 0u) {
        return BRINGUP_REJ_FSM;
    }
    if (r->axis >= DAMIAO_NUM_AXES) {
        return BRINGUP_REJ_AXIS;
    }
    if (!isfinite(r->torque_nm) || fabsf(r->torque_nm) > BRINGUP_MOTOR_ID_TORQUE_CAP_NM) {
        return BRINGUP_REJ_TORQUE;
    }
    if (r->duration_ms == 0u || r->duration_ms > BRINGUP_MOTOR_ID_MAX_PULSE_MS) {
        return BRINGUP_REJ_DURATION;
    }
    if (ctx->state != BRINGUP_ST_IDLE) {
        return BRINGUP_REJ_BUSY;
    }
    return BRINGUP_REJ_NONE;
}

void bringup_step(BringupCtx *ctx, const BringupTickInput *in, BringupTickOutput *out)
{
    if (out != NULL) {
        memset(out, 0, sizeof(*out));
        out->active_axis = BRINGUP_NO_AXIS;
    }
    if (ctx == NULL || in == NULL || out == NULL) {
        return;
    }

    if (ctx->mode == BRINGUP_MODE_RUN) {
        ctx->tx_auth_axis = BRINGUP_NO_AXIS;
        out->state = ctx->state;
        return;
    }

    const uint8_t gate_ok = (ctx->mode == BRINGUP_MODE_MOTOR_ID
                             && gate_estop_healthy(in) != 0u
                             && in->fsm_disarmed != 0u) ? 1u : 0u;

    /* A new sequence number is consumed exactly once. */
    uint8_t new_req = 0u;
    if (in->req.valid != 0u && (ctx->have_seq == 0u || in->req.seq != ctx->last_seq)) {
        ctx->have_seq = 1u;
        ctx->last_seq = in->req.seq;
        new_req = 1u;
    }

    switch (ctx->state) {
    case BRINGUP_ST_ENABLE:
    case BRINGUP_ST_PULSE:
        if (gate_ok == 0u) {
            abort_pulse(ctx, out, in->now_us);
            break;
        }
        if (new_req != 0u) {
            /* Any new request during a pulse is an operator stop. */
            ctx->tx_auth_axis = BRINGUP_NO_AXIS;
            push_frame(out, ctx->axis, 0u, 0.0f);
            ctx->state = BRINGUP_ST_DISABLE;
            reject(ctx, BRINGUP_REJ_BUSY);
            break;
        }
        if (ctx->state == BRINGUP_ST_ENABLE) {
            ctx->state = BRINGUP_ST_PULSE;
            ctx->phase_us = in->now_us;
            push_frame(out, ctx->axis, 2u, ctx->torque_nm);
        } else if ((uint32_t)(in->now_us - ctx->phase_us)
                   >= (uint32_t)ctx->duration_ms * 1000u) {
            ctx->tx_auth_axis = BRINGUP_NO_AXIS;
            push_frame(out, ctx->axis, 0u, 0.0f);
            ctx->state = BRINGUP_ST_DISABLE;
            if (ctx->pulses_done < UINT32_MAX) {
                ctx->pulses_done++;
            }
        } else {
            push_frame(out, ctx->axis, 2u, ctx->torque_nm);
        }
        break;

    case BRINGUP_ST_DISABLE:
        /* Second disable one tick later (mirrors the +1 ms repeat policy). */
        ctx->tx_auth_axis = BRINGUP_NO_AXIS;
        push_frame(out, ctx->axis, 0u, 0.0f);
        enter_cooldown(ctx, in->now_us);
        break;

    case BRINGUP_ST_COOLDOWN:
        ctx->tx_auth_axis = BRINGUP_NO_AXIS;
        if (new_req != 0u) {
            reject(ctx, BRINGUP_REJ_BUSY);
        }
        if ((uint32_t)(in->now_us - ctx->phase_us) >= BRINGUP_MOTOR_ID_COOLDOWN_MS * 1000u) {
            ctx->state = BRINGUP_ST_IDLE;
            ctx->axis = BRINGUP_NO_AXIS;
        }
        break;

    case BRINGUP_ST_IDLE:
    default:
        ctx->state = BRINGUP_ST_IDLE;
        ctx->tx_auth_axis = BRINGUP_NO_AXIS;
        if (new_req != 0u) {
            const uint8_t why = validate_request(ctx, in);
            if (why != BRINGUP_REJ_NONE) {
                reject(ctx, why);
            } else {
                ctx->last_reject = BRINGUP_REJ_NONE;
                ctx->axis = in->req.axis;
                ctx->torque_nm = in->req.torque_nm;
                ctx->duration_ms = in->req.duration_ms;
                ctx->state = BRINGUP_ST_ENABLE;
                ctx->phase_us = in->now_us;
                ctx->tx_auth_axis = ctx->axis;
                push_frame(out, ctx->axis, 1u, 0.0f);
            }
        }
        break;
    }

    /* Encoder poll with the drive-disable frame (the only frame BENCH can send).
     * ASSUMED: the drive answers 0xFF..FD with a feedback frame (bench 3.1a). */
    if (ctx->state == BRINGUP_ST_IDLE || ctx->state == BRINGUP_ST_COOLDOWN) {
        push_frame(out, ctx->poll_axis, 0u, 0.0f);
        ctx->poll_axis = (uint8_t)((ctx->poll_axis + 1u) % DAMIAO_NUM_AXES);
    }

    out->state = ctx->state;
    out->active_axis = (ctx->state == BRINGUP_ST_ENABLE || ctx->state == BRINGUP_ST_PULSE)
                     ? ctx->axis : BRINGUP_NO_AXIS;
    out->torque_nm = (ctx->state == BRINGUP_ST_PULSE) ? ctx->torque_nm : 0.0f;
    out->last_reject = ctx->last_reject;
    out->permit = bringup_permit(ctx, 0u, in);
}

uint8_t bringup_tx_allowed(const BringupCtx *ctx, uint8_t bus_idx, uint16_t std_id,
                           uint8_t dlc, const uint8_t data[DAMIAO_CAN_DLC])
{
    if (ctx == NULL || data == NULL) {
        return 0u;
    }
    if (ctx->mode == BRINGUP_MODE_RUN) {
        return 1u;
    }
    const uint8_t axis = bringup_axis_for_cmd_id(bus_idx, std_id);
    if (axis == BRINGUP_NO_AXIS) {
        return 0u;
    }
    if (bringup_frame_is_disable(dlc, data) != 0u) {
        return 1u;
    }
    if (ctx->mode != BRINGUP_MODE_MOTOR_ID) {
        return 0u; /* BENCH: nothing but the disable frame */
    }
    const uint8_t auth = ctx->tx_auth_axis;
    if (auth == BRINGUP_NO_AXIS || auth != axis) {
        return 0u;
    }
    if (bringup_frame_is_enable(dlc, data) != 0u) {
        return 1u;
    }
    if (dlc != DAMIAO_CAN_DLC || (data[0] == 0xFFu && data[1] == 0xFFu && data[2] == 0xFFu
                                  && data[3] == 0xFFu && data[4] == 0xFFu && data[5] == 0xFFu
                                  && data[6] == 0xFFu)) {
        return 0u; /* other special commands (zero position 0xFE, clear 0xFB, ...) */
    }
    DamiaoAxisConfig cfg;
    DamiaoMitCmd cmd;
    if (damiao_get_axis_config(axis, &cfg) != DAMIAO_OK
        || damiao_unpack_mit_cmd(&cfg.limits, data, &cmd) != DAMIAO_OK) {
        return 0u;
    }
    const float lsb = (cfg.limits.t_max - cfg.limits.t_min) / 4095.0f;
    if (cmd.kp != 0.0f || cmd.kd != 0.0f || !isfinite(cmd.torque)
        || fabsf(cmd.torque) > BRINGUP_MOTOR_ID_TORQUE_CAP_NM + lsb) {
        return 0u;
    }
    return 1u;
}
