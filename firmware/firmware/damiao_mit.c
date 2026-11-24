/*
 * DAMIAO DM4310 / DM6215 Pure-C MIT CAN Frame Codec
 *
 * Adapted from DM-jump (User/Devices/DM_Motor/dm4310_drv.c):
 *   Copyright (c) 2024-2026 Trần Nguyên Bình
 *   Project: DM-jump (DM-jump), MIT License
 *   Reference: dmBots/wheel-legged (https://github.com/dmBots/wheel-legged)
 *   See firmware/DM_JUMP_LICENSE.txt for the full MIT license notice.
 */

#include "damiao_mit.h"

#include <math.h>
#include <stddef.h>

static uint8_t limits_valid(const DamiaoLimits *lim)
{
    if (lim == NULL) {
        return 0u;
    }
    if (!isfinite(lim->p_min) || !isfinite(lim->p_max) || !(lim->p_max > lim->p_min)) {
        return 0u;
    }
    if (!isfinite(lim->v_min) || !isfinite(lim->v_max) || !(lim->v_max > lim->v_min)) {
        return 0u;
    }
    if (!isfinite(lim->kp_min) || !isfinite(lim->kp_max) || !(lim->kp_max > lim->kp_min)) {
        return 0u;
    }
    if (!isfinite(lim->kd_min) || !isfinite(lim->kd_max) || !(lim->kd_max > lim->kd_min)) {
        return 0u;
    }
    if (!isfinite(lim->t_min) || !isfinite(lim->t_max) || !(lim->t_max > lim->t_min)) {
        return 0u;
    }
    if (!isfinite(lim->cmd_torque_clamp) || lim->cmd_torque_clamp < 0.0f) {
        return 0u;
    }
    return 1u;
}

static float clamp_f32(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

DamiaoLimits damiao_limits_dm4310(void)
{
    DamiaoLimits lim;
    lim.p_min  = -12.5f;
    lim.p_max  =  12.5f;
    lim.v_min  = -30.0f;
    lim.v_max  =  30.0f;
    lim.kp_min =   0.0f;
    lim.kp_max = 500.0f;
    lim.kd_min =   0.0f;
    lim.kd_max =   5.0f;
    lim.t_min  = -10.0f;
    lim.t_max  =  10.0f;
    lim.cmd_torque_clamp = 7.0f;
    return lim;
}

DamiaoLimits damiao_limits_dm6215(void)
{
    DamiaoLimits lim;
    lim.p_min  = -12.0f;
    lim.p_max  =  12.0f;
    lim.v_min  = -45.0f;
    lim.v_max  =  45.0f;
    lim.kp_min =   0.0f;
    lim.kp_max = 500.0f;
    lim.kd_min =   0.0f;
    lim.kd_max =   5.0f;
    lim.t_min  = -18.0f;
    lim.t_max  =  18.0f;
    lim.cmd_torque_clamp = 2.0f;
    return lim;
}

DamiaoStatus damiao_get_axis_config(uint8_t axis, DamiaoAxisConfig *out_cfg)
{
    if (out_cfg == NULL) {
        return DAMIAO_ERR_NULL_PTR;
    }
    if (axis >= DAMIAO_NUM_AXES) {
        return DAMIAO_ERR_INVALID_AXIS;
    }

    /* Rev4 J5 = right, J6 = left, J7 = spare (one-based can_bus). */
    static const uint8_t AXIS_BUS[DAMIAO_NUM_AXES] = { 2u, 2u, 2u, 1u, 1u, 1u };
    static const uint16_t AXIS_CMD_ID[DAMIAO_NUM_AXES] = {
        0x001u, 0x002u, 0x003u, 0x004u, 0x005u, 0x006u
    };
    static const uint16_t AXIS_REPLY_ID[DAMIAO_NUM_AXES] = {
        0x011u, 0x012u, 0x013u, 0x014u, 0x015u, 0x016u
    };
    static const DamiaoMotorType AXIS_TYPE[DAMIAO_NUM_AXES] = {
        DAMIAO_MOTOR_DM4310, DAMIAO_MOTOR_DM4310, DAMIAO_MOTOR_DM6215,
        DAMIAO_MOTOR_DM4310, DAMIAO_MOTOR_DM4310, DAMIAO_MOTOR_DM6215
    };

    out_cfg->axis_index   = axis;
    out_cfg->can_bus      = AXIS_BUS[axis];
    out_cfg->cmd_can_id   = AXIS_CMD_ID[axis];
    out_cfg->reply_can_id = AXIS_REPLY_ID[axis];
    out_cfg->motor_type   = AXIS_TYPE[axis];
    out_cfg->limits       = (AXIS_TYPE[axis] == DAMIAO_MOTOR_DM6215)
                          ? damiao_limits_dm6215()
                          : damiao_limits_dm4310();
    return DAMIAO_OK;
}

uint16_t damiao_float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    if (bits == 0u || bits > 16u || !isfinite(x) || !isfinite(x_min)
        || !isfinite(x_max) || !(x_max > x_min)) {
        return 0u;
    }
    const uint32_t max_code = (1u << bits) - 1u;
    const float clamped = clamp_f32(x, x_min, x_max);
    const float span = x_max - x_min;
    const float scaled = ((clamped - x_min) * (float)max_code) / span;
    const float rounded = floorf(scaled + 0.5f);
    if (rounded <= 0.0f) {
        return 0u;
    }
    if (rounded >= (float)max_code) {
        return (uint16_t)max_code;
    }
    return (uint16_t)rounded;
}

float damiao_uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    if (bits == 0u || bits > 16u || !isfinite(x_min) || !isfinite(x_max)
        || !(x_max > x_min)) {
        return NAN;
    }
    const uint32_t max_code = (1u << bits) - 1u;
    const uint32_t masked = (uint32_t)x_int & max_code;
    const float span = x_max - x_min;
    return (((float)masked * span) / (float)max_code) + x_min;
}

DamiaoStatus damiao_pack_mit_cmd(const DamiaoLimits *lim,
                                 const DamiaoMitCmd *cmd,
                                 uint8_t out_frame[DAMIAO_CAN_DLC])
{
    if (lim == NULL || cmd == NULL || out_frame == NULL) {
        return DAMIAO_ERR_NULL_PTR;
    }
    if (limits_valid(lim) == 0u) {
        return DAMIAO_ERR_INVALID_LIMITS;
    }
    if (!isfinite(cmd->pos) || !isfinite(cmd->vel) || !isfinite(cmd->kp)
        || !isfinite(cmd->kd) || !isfinite(cmd->torque)) {
        return DAMIAO_ERR_NONFINITE;
    }

    const uint16_t p_int  = damiao_float_to_uint(cmd->pos,    lim->p_min,  lim->p_max,  16u);
    const uint16_t v_int  = damiao_float_to_uint(cmd->vel,    lim->v_min,  lim->v_max,  12u);
    const uint16_t kp_int = damiao_float_to_uint(cmd->kp,     lim->kp_min, lim->kp_max, 12u);
    const uint16_t kd_int = damiao_float_to_uint(cmd->kd,     lim->kd_min, lim->kd_max, 12u);
    const uint16_t t_int  = damiao_float_to_uint(cmd->torque, lim->t_min,  lim->t_max,  12u);

    out_frame[0] = (uint8_t)((p_int >> 8u) & 0xFFu);
    out_frame[1] = (uint8_t)(p_int & 0xFFu);
    out_frame[2] = (uint8_t)((v_int >> 4u) & 0xFFu);
    out_frame[3] = (uint8_t)(((v_int & 0x0Fu) << 4u) | ((kp_int >> 8u) & 0x0Fu));
    out_frame[4] = (uint8_t)(kp_int & 0xFFu);
    out_frame[5] = (uint8_t)((kd_int >> 4u) & 0xFFu);
    out_frame[6] = (uint8_t)(((kd_int & 0x0Fu) << 4u) | ((t_int >> 8u) & 0x0Fu));
    out_frame[7] = (uint8_t)(t_int & 0xFFu);

    return DAMIAO_OK;
}

DamiaoStatus damiao_pack_mit_cmd_clamped(const DamiaoLimits *lim,
                                         const DamiaoMitCmd *cmd,
                                         uint8_t out_frame[DAMIAO_CAN_DLC])
{
    if (lim == NULL || cmd == NULL || out_frame == NULL) {
        return DAMIAO_ERR_NULL_PTR;
    }
    if (limits_valid(lim) == 0u) {
        return DAMIAO_ERR_INVALID_LIMITS;
    }
    if (!isfinite(cmd->torque)) {
        return DAMIAO_ERR_NONFINITE;
    }

    DamiaoMitCmd clamped = *cmd;
    clamped.torque = clamp_f32(cmd->torque, -lim->cmd_torque_clamp, lim->cmd_torque_clamp);
    return damiao_pack_mit_cmd(lim, &clamped, out_frame);
}

DamiaoStatus damiao_unpack_mit_cmd(const DamiaoLimits *lim,
                                   const uint8_t in_frame[DAMIAO_CAN_DLC],
                                   DamiaoMitCmd *out_cmd)
{
    if (lim == NULL || in_frame == NULL || out_cmd == NULL) {
        return DAMIAO_ERR_NULL_PTR;
    }
    if (limits_valid(lim) == 0u) {
        return DAMIAO_ERR_INVALID_LIMITS;
    }

    const uint16_t p_int  = (uint16_t)(((uint16_t)in_frame[0] << 8u) | (uint16_t)in_frame[1]);
    const uint16_t v_int  = (uint16_t)(((uint16_t)in_frame[2] << 4u) | ((uint16_t)in_frame[3] >> 4u));
    const uint16_t kp_int = (uint16_t)((((uint16_t)in_frame[3] & 0x0Fu) << 8u) | (uint16_t)in_frame[4]);
    const uint16_t kd_int = (uint16_t)(((uint16_t)in_frame[5] << 4u) | ((uint16_t)in_frame[6] >> 4u));
    const uint16_t t_int  = (uint16_t)((((uint16_t)in_frame[6] & 0x0Fu) << 8u) | (uint16_t)in_frame[7]);

    out_cmd->pos    = damiao_uint_to_float(p_int,  lim->p_min,  lim->p_max,  16u);
    out_cmd->vel    = damiao_uint_to_float(v_int,  lim->v_min,  lim->v_max,  12u);
    out_cmd->kp     = damiao_uint_to_float(kp_int, lim->kp_min, lim->kp_max, 12u);
    out_cmd->kd     = damiao_uint_to_float(kd_int, lim->kd_min, lim->kd_max, 12u);
    out_cmd->torque = damiao_uint_to_float(t_int,  lim->t_min,  lim->t_max,  12u);

    return DAMIAO_OK;
}

DamiaoStatus damiao_pack_feedback(const DamiaoLimits *lim,
                                  const DamiaoMitFeedback *fb,
                                  uint8_t out_frame[DAMIAO_CAN_DLC])
{
    if (lim == NULL || fb == NULL || out_frame == NULL) {
        return DAMIAO_ERR_NULL_PTR;
    }
    if (limits_valid(lim) == 0u) {
        return DAMIAO_ERR_INVALID_LIMITS;
    }
    if (!isfinite(fb->pos) || !isfinite(fb->vel) || !isfinite(fb->torque)
        || !isfinite(fb->t_mos_c) || !isfinite(fb->t_coil_c)) {
        return DAMIAO_ERR_NONFINITE;
    }

    const uint16_t p_int = damiao_float_to_uint(fb->pos,    lim->p_min, lim->p_max, 16u);
    const uint16_t v_int = damiao_float_to_uint(fb->vel,    lim->v_min, lim->v_max, 12u);
    const uint16_t t_int = damiao_float_to_uint(fb->torque, lim->t_min, lim->t_max, 12u);

    const float t_mos_clamped  = clamp_f32(fb->t_mos_c,  0.0f, 255.0f);
    const float t_coil_clamped = clamp_f32(fb->t_coil_c, 0.0f, 255.0f);

    out_frame[0] = (uint8_t)(((fb->status_nibble & 0x0Fu) << 4u) | (fb->motor_id & 0x0Fu));
    out_frame[1] = (uint8_t)((p_int >> 8u) & 0xFFu);
    out_frame[2] = (uint8_t)(p_int & 0xFFu);
    out_frame[3] = (uint8_t)((v_int >> 4u) & 0xFFu);
    out_frame[4] = (uint8_t)(((v_int & 0x0Fu) << 4u) | ((t_int >> 8u) & 0x0Fu));
    out_frame[5] = (uint8_t)(t_int & 0xFFu);
    out_frame[6] = (uint8_t)floorf(t_mos_clamped + 0.5f);
    out_frame[7] = (uint8_t)floorf(t_coil_clamped + 0.5f);

    return DAMIAO_OK;
}

DamiaoStatus damiao_unpack_feedback(const DamiaoLimits *lim,
                                    const uint8_t in_frame[DAMIAO_CAN_DLC],
                                    uint8_t dlc,
                                    DamiaoMitFeedback *out_fb)
{
    if (lim == NULL || in_frame == NULL || out_fb == NULL) {
        return DAMIAO_ERR_NULL_PTR;
    }
    if (limits_valid(lim) == 0u) {
        return DAMIAO_ERR_INVALID_LIMITS;
    }
    if (dlc != DAMIAO_CAN_DLC) {
        return DAMIAO_ERR_BAD_DLC;
    }

    out_fb->motor_id      = (uint8_t)(in_frame[0] & 0x0Fu);
    out_fb->status_nibble = (uint8_t)((in_frame[0] >> 4u) & 0x0Fu);
    out_fb->p_raw         = (uint16_t)(((uint16_t)in_frame[1] << 8u) | (uint16_t)in_frame[2]);
    out_fb->v_raw         = (uint16_t)(((uint16_t)in_frame[3] << 4u) | ((uint16_t)in_frame[4] >> 4u));
    out_fb->t_raw         = (uint16_t)((((uint16_t)in_frame[4] & 0x0Fu) << 8u) | (uint16_t)in_frame[5]);

    out_fb->pos           = damiao_uint_to_float(out_fb->p_raw, lim->p_min, lim->p_max, 16u);
    out_fb->vel           = damiao_uint_to_float(out_fb->v_raw, lim->v_min, lim->v_max, 12u);
    out_fb->torque        = damiao_uint_to_float(out_fb->t_raw, lim->t_min, lim->t_max, 12u);
    out_fb->t_mos_c       = (float)in_frame[6];
    out_fb->t_coil_c      = (float)in_frame[7];

    return DAMIAO_OK;
}

void damiao_pack_enable(uint8_t out_frame[DAMIAO_CAN_DLC])
{
    if (out_frame == NULL) {
        return;
    }
    for (uint8_t i = 0u; i < 7u; ++i) {
        out_frame[i] = 0xFFu;
    }
    out_frame[7] = DAMIAO_CMD_ENABLE_BYTE;
}

void damiao_pack_disable(uint8_t out_frame[DAMIAO_CAN_DLC])
{
    if (out_frame == NULL) {
        return;
    }
    for (uint8_t i = 0u; i < 7u; ++i) {
        out_frame[i] = 0xFFu;
    }
    out_frame[7] = DAMIAO_CMD_DISABLE_BYTE;
}

uint8_t damiao_status_is_healthy(uint8_t status_nibble, uint8_t expected_enabled_code)
{
    const uint8_t code = (uint8_t)(status_nibble & 0x0Fu);
    if (code == DAMIAO_DRIVE_STATUS_COMM_LOSS) {
        return 0u;
    }
    return (code == (uint8_t)(expected_enabled_code & 0x0Fu)) ? 1u : 0u;
}
