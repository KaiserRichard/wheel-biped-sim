#ifdef NDEBUG
#undef NDEBUG
#endif
#include "damiao_mit.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define S11_CHECK(expr)                                                   \
    do {                                                                  \
        if (!(expr)) {                                                    \
            fprintf(stderr, "%s:%d: %s: Assertion `%s' failed.\n",        \
                    __FILE__, __LINE__, __func__, #expr);                 \
            abort();                                                      \
        }                                                                 \
    } while (0)

#undef assert
#define assert(expr) S11_CHECK(expr)

static int near_f32(float a, float b, float tol)
{
    return (fabsf(a - b) <= tol) ? 1 : 0;
}

static void assert_cmd_round_trip(const DamiaoLimits *lim, const DamiaoMitCmd *cmd)
{
    uint8_t frame[DAMIAO_CAN_DLC];
    DamiaoMitCmd out;

    assert(damiao_pack_mit_cmd(lim, cmd, frame) == DAMIAO_OK);
    assert(damiao_unpack_mit_cmd(lim, frame, &out) == DAMIAO_OK);

    /* Half-LSB tolerance for each quantized channel */
    const float tol_p  = 0.51f * (lim->p_max  - lim->p_min)  / 65535.0f;
    const float tol_v  = 0.51f * (lim->v_max  - lim->v_min)  / 4095.0f;
    const float tol_kp = 0.51f * (lim->kp_max - lim->kp_min) / 4095.0f;
    const float tol_kd = 0.51f * (lim->kd_max - lim->kd_min) / 4095.0f;
    const float tol_t  = 0.51f * (lim->t_max  - lim->t_min)  / 4095.0f;

    assert(near_f32(out.pos,    cmd->pos,    tol_p));
    assert(near_f32(out.vel,    cmd->vel,    tol_v));
    assert(near_f32(out.kp,     cmd->kp,     tol_kp));
    assert(near_f32(out.kd,     cmd->kd,     tol_kd));
    assert(near_f32(out.torque, cmd->torque, tol_t));
}

static void test_mit_cmd_limits(void)
{
    const DamiaoLimits hip = damiao_limits_dm4310();
    const DamiaoLimits whl = damiao_limits_dm6215();
    uint8_t frame[DAMIAO_CAN_DLC];
    DamiaoMitCmd out;

    /* 1. All minimums -> exact all-zero raw bytes */
    const DamiaoMitCmd hip_min = { hip.p_min, hip.v_min, hip.kp_min, hip.kd_min, hip.t_min };
    assert(damiao_pack_mit_cmd(&hip, &hip_min, frame) == DAMIAO_OK);
    for (uint8_t i = 0u; i < DAMIAO_CAN_DLC; ++i) {
        assert(frame[i] == 0x00u);
    }
    assert(damiao_unpack_mit_cmd(&hip, frame, &out) == DAMIAO_OK);
    assert(near_f32(out.pos, hip.p_min, 1e-6f));
    assert(near_f32(out.vel, hip.v_min, 1e-6f));
    assert(near_f32(out.kp, hip.kp_min, 1e-6f));
    assert(near_f32(out.kd, hip.kd_min, 1e-6f));
    assert(near_f32(out.torque, hip.t_min, 1e-6f));

    /* 2. All maximums -> exact all-0xFF raw bytes */
    const DamiaoMitCmd hip_max = { hip.p_max, hip.v_max, hip.kp_max, hip.kd_max, hip.t_max };
    assert(damiao_pack_mit_cmd(&hip, &hip_max, frame) == DAMIAO_OK);
    for (uint8_t i = 0u; i < DAMIAO_CAN_DLC; ++i) {
        assert(frame[i] == 0xFFu);
    }
    assert(damiao_unpack_mit_cmd(&hip, frame, &out) == DAMIAO_OK);
    assert(near_f32(out.pos, hip.p_max, 1e-6f));
    assert(near_f32(out.vel, hip.v_max, 1e-6f));
    assert(near_f32(out.kp, hip.kp_max, 1e-6f));
    assert(near_f32(out.kd, hip.kd_max, 1e-6f));
    assert(near_f32(out.torque, hip.t_max, 1e-6f));

    /* 3. DM6215 wheel limits (min, max, zero, mixed) */
    const DamiaoMitCmd whl_min = { whl.p_min, whl.v_min, whl.kp_min, whl.kd_min, whl.t_min };
    const DamiaoMitCmd whl_max = { whl.p_max, whl.v_max, whl.kp_max, whl.kd_max, whl.t_max };
    const DamiaoMitCmd whl_zero = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    const DamiaoMitCmd whl_mix = { -3.14159f, 22.5f, 125.0f, 1.75f, -1.85f };
    assert_cmd_round_trip(&whl, &whl_min);
    assert_cmd_round_trip(&whl, &whl_max);
    assert_cmd_round_trip(&whl, &whl_zero);
    assert_cmd_round_trip(&whl, &whl_mix);

    /* 4. DM4310 hip mixed values */
    const DamiaoMitCmd hip_mix = { 1.234f, -14.2f, 45.0f, 2.3f, 6.5f };
    assert_cmd_round_trip(&hip, &hip_mix);

    /* 5. Out-of-range inputs saturate cleanly without wraparound */
    const DamiaoMitCmd over = { 999.0f, -999.0f, 999.0f, -50.0f, 500.0f };
    assert(damiao_pack_mit_cmd(&hip, &over, frame) == DAMIAO_OK);
    assert(damiao_unpack_mit_cmd(&hip, frame, &out) == DAMIAO_OK);
    assert(near_f32(out.pos, hip.p_max, 1e-5f));
    assert(near_f32(out.vel, hip.v_min, 1e-5f));
    assert(near_f32(out.kp, hip.kp_max, 1e-5f));
    assert(near_f32(out.kd, hip.kd_min, 1e-5f));
    assert(near_f32(out.torque, hip.t_max, 1e-5f));

    /* 6. Firmware torque clamp (+/-7 Nm hip, +/-2 Nm wheel) */
    const DamiaoMitCmd big_torq = { 0.0f, 0.0f, 0.0f, 0.0f, 9.5f };
    assert(damiao_pack_mit_cmd_clamped(&hip, &big_torq, frame) == DAMIAO_OK);
    assert(damiao_unpack_mit_cmd(&hip, frame, &out) == DAMIAO_OK);
    assert(near_f32(out.torque, 7.0f, 0.01f));

    const DamiaoMitCmd neg_torq = { 0.0f, 0.0f, 0.0f, 0.0f, -12.0f };
    assert(damiao_pack_mit_cmd_clamped(&whl, &neg_torq, frame) == DAMIAO_OK);
    assert(damiao_unpack_mit_cmd(&whl, frame, &out) == DAMIAO_OK);
    assert(near_f32(out.torque, -2.0f, 0.01f));

    /* 7. Non-finite rejection */
    const DamiaoMitCmd bad_nan = { NAN, 0.0f, 0.0f, 0.0f, 0.0f };
    const DamiaoMitCmd bad_inf = { 0.0f, INFINITY, 0.0f, 0.0f, 0.0f };
    assert(damiao_pack_mit_cmd(&hip, &bad_nan, frame) == DAMIAO_ERR_NONFINITE);
    assert(damiao_pack_mit_cmd(&hip, &bad_inf, frame) == DAMIAO_ERR_NONFINITE);
}

static void test_feedback_pack_unpack(void)
{
    const DamiaoLimits hip = damiao_limits_dm4310();
    const DamiaoLimits whl = damiao_limits_dm6215();
    uint8_t frame[DAMIAO_CAN_DLC];
    DamiaoMitFeedback out;

    /* DM4310 feedback at negative limits */
    DamiaoMitFeedback fb_in = { 0 };
    fb_in.motor_id      = 0x4u;
    fb_in.status_nibble = DAMIAO_DRIVE_STATUS_ENABLED_DEFAULT;
    fb_in.pos           = hip.p_min;
    fb_in.vel           = hip.v_min;
    fb_in.torque        = hip.t_min;
    fb_in.t_mos_c       = 42.0f;
    fb_in.t_coil_c      = 55.0f;

    assert(damiao_pack_feedback(&hip, &fb_in, frame) == DAMIAO_OK);
    assert(frame[0] == 0x14u);
    assert(frame[1] == 0x00u && frame[2] == 0x00u);
    assert(frame[6] == 42u && frame[7] == 55u);

    assert(damiao_unpack_feedback(&hip, frame, DAMIAO_CAN_DLC, &out) == DAMIAO_OK);
    assert(out.motor_id == 0x4u);
    assert(out.status_nibble == 0x1u);
    assert(out.p_raw == 0u && out.v_raw == 0u && out.t_raw == 0u);
    assert(near_f32(out.pos, hip.p_min, 1e-6f));
    assert(near_f32(out.vel, hip.v_min, 1e-6f));
    assert(near_f32(out.torque, hip.t_min, 1e-6f));
    assert(near_f32(out.t_mos_c, 42.0f, 1e-6f));
    assert(near_f32(out.t_coil_c, 55.0f, 1e-6f));

    /* DM6215 feedback at positive limits + status 0xD comms loss */
    fb_in.motor_id      = 0x6u;
    fb_in.status_nibble = DAMIAO_DRIVE_STATUS_COMM_LOSS;
    fb_in.pos           = whl.p_max;
    fb_in.vel           = whl.v_max;
    fb_in.torque        = whl.t_max;
    fb_in.t_mos_c       = 80.0f;
    fb_in.t_coil_c      = 95.0f;

    assert(damiao_pack_feedback(&whl, &fb_in, frame) == DAMIAO_OK);
    assert(frame[0] == 0xD6u);
    assert(damiao_unpack_feedback(&whl, frame, DAMIAO_CAN_DLC, &out) == DAMIAO_OK);
    assert(out.motor_id == 0x6u);
    assert(out.status_nibble == 0xDu);
    assert(out.p_raw == 0xFFFFu && out.v_raw == 0x0FFFu && out.t_raw == 0x0FFFu);
    assert(near_f32(out.pos, whl.p_max, 1e-6f));
    assert(near_f32(out.vel, whl.v_max, 1e-6f));
    assert(near_f32(out.torque, whl.t_max, 1e-6f));

    /* Status health check */
    assert(damiao_status_is_healthy(0x1u, 0x1u) == 1u);
    assert(damiao_status_is_healthy(0x0u, 0x1u) == 0u);
    assert(damiao_status_is_healthy(0xDu, 0x1u) == 0u);
    assert(damiao_status_is_healthy(0xDu, 0xDu) == 0u); /* 0xD comm loss never healthy */

    /* Bad DLC rejected */
    assert(damiao_unpack_feedback(&whl, frame, 7u, &out) == DAMIAO_ERR_BAD_DLC);
}

static void test_enable_disable_and_axis_table(void)
{
    uint8_t en[DAMIAO_CAN_DLC];
    uint8_t dis[DAMIAO_CAN_DLC];
    damiao_pack_enable(en);
    damiao_pack_disable(dis);

    for (uint8_t i = 0u; i < 7u; ++i) {
        assert(en[i] == 0xFFu);
        assert(dis[i] == 0xFFu);
    }
    assert(en[7] == 0xFCu);
    assert(dis[7] == 0xFDu);

    /* Verify unified 6-axis bus & ID mapping from FIRMWARE_DESIGN_v2.md Section 4.2 */
    const uint8_t exp_bus[6] = { 2u, 2u, 2u, 1u, 1u, 1u };
    const uint16_t exp_cmd[6] = { 0x001u, 0x002u, 0x003u, 0x004u, 0x005u, 0x006u };
    const uint16_t exp_rep[6] = { 0x011u, 0x012u, 0x013u, 0x014u, 0x015u, 0x016u };
    for (uint8_t a = 0u; a < DAMIAO_NUM_AXES; ++a) {
        DamiaoAxisConfig cfg;
        assert(damiao_get_axis_config(a, &cfg) == DAMIAO_OK);
        assert(cfg.axis_index == a);
        assert(cfg.can_bus == exp_bus[a]);
        assert(cfg.cmd_can_id == exp_cmd[a]);
        assert(cfg.reply_can_id == exp_rep[a]);
        if (a == 2u || a == 5u) {
            assert(cfg.motor_type == DAMIAO_MOTOR_DM6215);
            assert(near_f32(cfg.limits.cmd_torque_clamp, 2.0f, 1e-6f));
        } else {
            assert(cfg.motor_type == DAMIAO_MOTOR_DM4310);
            assert(near_f32(cfg.limits.cmd_torque_clamp, 7.0f, 1e-6f));
        }
    }
    DamiaoAxisConfig bad_cfg;
    assert(damiao_get_axis_config(6u, &bad_cfg) == DAMIAO_ERR_INVALID_AXIS);
}

int main(void)
{
    test_mit_cmd_limits();
    test_feedback_pack_unpack();
    test_enable_disable_and_axis_table();
    printf("test_damiao_mit: ALL PASSED\n");
    return 0;
}
