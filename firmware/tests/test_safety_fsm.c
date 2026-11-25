#ifdef NDEBUG
#undef NDEBUG
#endif
#include "safety_fsm.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

static SafetyInputs make_disarmed_clean_inputs(void)
{
    SafetyInputs in;
    memset(&in, 0, sizeof(in));
    in.dt_ms              = 2u;
    in.boot_self_test_ok  = 1u;
    in.dump_continuity_ok = 1u;
    in.v_bus_v            = 0.5f;   /* Discharged < 5 V */
    in.v_batt_ocv_v       = 24.0f;  /* Healthy 6S battery OCV > 20.5 V */
    in.imu_est_valid      = 1u;
    in.ground_left        = 1u;
    in.ground_right       = 1u;
    in.radio_age_ms       = 4u;
    in.host_age_ms        = 20u;
    in.host_valid         = 1u;
    return in;
}

static void reach_armed(SafetyFsm *fsm, SafetyOutput *out)
{
    SafetyInputs in = make_disarmed_clean_inputs();
    in.local_reset_req = 1u;
    in.local_arm_req   = 1u;
    safety_fsm_step(fsm, &in, out);
    assert(out->state == SAFETY_STATE_PRECHARGE);
    assert(out->permit_out == 1u);
    assert(out->can_enable_cmd == 0u); /* Must NOT enable CAN before K1 closed */

    /* K2 + Kpre closed, K1 still open -> still in PRECHARGE, no CAN enable */
    in.local_reset_req = 0u;
    in.local_arm_req   = 0u;
    in.k2_closed       = 1u;
    in.kpre_closed     = 1u;
    in.v_bus_v         = 18.0f;
    safety_fsm_step(fsm, &in, out);
    assert(out->state == SAFETY_STATE_PRECHARGE);
    assert(out->can_enable_cmd == 0u);

    /* K1 closed, Kpre open, dV <= 1 V -> sends CAN 0xFC enable; waits for 6 healthy drives */
    in.kpre_closed     = 0u;
    in.k1_closed       = 1u;
    in.precharge_dv_ok = 1u;
    in.v_bus_v         = 24.0f;
    in.drives_healthy_6 = 0u;
    safety_fsm_step(fsm, &in, out);
    assert(out->state == SAFETY_STATE_PRECHARGE);
    assert(out->can_enable_cmd == 1u);

    /* All 6 drives report healthy -> ARMED */
    in.drives_healthy_6 = 1u;
    in.radio_arm_sw     = 1u;
    safety_fsm_step(fsm, &in, out);
    assert(out->state == SAFETY_STATE_ARMED);
    assert(out->reason == SAFETY_REASON_PRECHARGE_OK);
    assert(out->torque_enable == 0u);
    assert(out->permit_out == 1u);
}

static void reach_balancing(SafetyFsm *fsm, SafetyOutput *out)
{
    reach_armed(fsm, out);
    SafetyInputs in = make_disarmed_clean_inputs();
    in.k1_closed         = 1u;
    in.k2_closed         = 1u;
    in.v_bus_v           = 24.0f;
    in.drives_healthy_6  = 1u;
    in.radio_arm_sw      = 1u;
    in.stand_handoff_req = 1u;
    in.pitch_deg         = 2.0f;
    safety_fsm_step(fsm, &in, out);
    assert(out->state == SAFETY_STATE_BALANCING);
    assert(out->torque_enable == 1u);
}

static void test_disarmed_faults_and_estop(void)
{
    SafetyFsm fsm;
    SafetyOutput out;

    /* 1. DISARMED -> FAULT on each boot/pre-arm fault condition */
    for (uint32_t c = 0u; c < 5u; ++c) {
        safety_fsm_init(&fsm);
        SafetyInputs in = make_disarmed_clean_inputs();
        if (c == 0u) {
            in.boot_self_test_ok = 0u;
        } else if (c == 1u) {
            in.bor3_fault = 1u;
        } else if (c == 2u) {
            in.can_bus_off = 1u;
        } else if (c == 3u) {
            in.clamp_fault = 1u;
        } else {
            in.v_bus_v = 5.5f; /* Link > 5 V while disarmed */
        }
        safety_fsm_step(&fsm, &in, &out);
        assert(out.state == SAFETY_STATE_FAULT);
        assert(out.permit_out == 0u);
        assert(out.can_disable_cmd == 1u);
    }

    /* 2. Missing dump continuity blocks DISARMED -> PRECHARGE */
    safety_fsm_init(&fsm);
    SafetyInputs in = make_disarmed_clean_inputs();
    in.dump_continuity_ok = 0u;
    in.local_reset_req = 1u;
    in.local_arm_req   = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_DISARMED);

    /* 3. DISARMED -> ESTOP */
    in = make_disarmed_clean_inputs();
    in.estop_open = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_ESTOP);
    assert(out.can_abort_tx == 1u && out.can_disable_cmd == 1u);
}

static void test_precharge_transitions(void)
{
    SafetyFsm fsm;
    SafetyOutput out;

    /* PRECHARGE -> FAULT on > 600 ms timeout (no auto-retry) */
    safety_fsm_init(&fsm);
    SafetyInputs in = make_disarmed_clean_inputs();
    in.local_reset_req = 1u;
    in.local_arm_req   = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_PRECHARGE);

    in.local_reset_req = 0u;
    in.local_arm_req   = 0u;
    in.k2_closed       = 1u;
    in.kpre_closed     = 1u;
    in.dt_ms           = 100u;
    for (uint32_t i = 0u; i < 6u; ++i) {
        safety_fsm_step(&fsm, &in, &out);
        assert(out.state == SAFETY_STATE_PRECHARGE);
    }
    safety_fsm_step(&fsm, &in, &out); /* 700 ms > 600 ms */
    assert(out.state == SAFETY_STATE_FAULT);
    assert(out.reason == SAFETY_REASON_PRECHARGE_TIMEOUT);

    /* PRECHARGE -> FAULT on mirror mismatch */
    safety_fsm_init(&fsm);
    in = make_disarmed_clean_inputs();
    in.local_reset_req = 1u;
    in.local_arm_req   = 1u;
    safety_fsm_step(&fsm, &in, &out);
    in.mirror_mismatch = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT);
    assert(out.reason == SAFETY_REASON_MIRROR_MISMATCH);

    /* PRECHARGE -> FAULT on drive enable fail */
    safety_fsm_init(&fsm);
    in = make_disarmed_clean_inputs();
    in.local_reset_req = 1u;
    in.local_arm_req   = 1u;
    safety_fsm_step(&fsm, &in, &out);
    in.drive_enable_fail = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT);
    assert(out.reason == SAFETY_REASON_DRIVE_ENABLE_FAIL);

    /* PRECHARGE -> ESTOP */
    safety_fsm_init(&fsm);
    in = make_disarmed_clean_inputs();
    in.local_reset_req = 1u;
    in.local_arm_req   = 1u;
    safety_fsm_step(&fsm, &in, &out);
    in.estop_open = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_ESTOP);
}

static void test_armed_transitions(void)
{
    SafetyFsm fsm;
    SafetyOutput out;

    /* ARMED -> DISARMED (clean disarm on radio arm SW LOW) */
    safety_fsm_init(&fsm);
    reach_armed(&fsm, &out);
    SafetyInputs in = make_disarmed_clean_inputs();
    in.k1_closed        = 1u;
    in.k2_closed        = 1u;
    in.v_bus_v          = 24.0f;
    in.drives_healthy_6 = 1u;
    in.radio_arm_sw     = 0u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_DISARMED);
    assert(out.reason == SAFETY_REASON_CLEAN_DISARM);
    assert(out.can_disable_cmd == 1u);
    assert(out.permit_out == 0u);

    /* ARMED -> BALANCING blocked if pitch > 5 deg or airborne, allowed when ok */
    safety_fsm_init(&fsm);
    reach_armed(&fsm, &out);
    in.radio_arm_sw      = 1u;
    in.stand_handoff_req = 1u;
    in.pitch_deg         = 6.0f;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_ARMED);

    in.pitch_deg   = 3.0f;
    in.ground_left = 0u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_ARMED);

    in.ground_left = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_BALANCING);

    /* ARMED -> FAULT (S11 fault) & ARMED -> ESTOP */
    safety_fsm_init(&fsm);
    reach_armed(&fsm, &out);
    in.stand_handoff_req = 0u;
    in.s11_fault         = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT);
    assert(out.reason == SAFETY_REASON_S11_FAULT);

    safety_fsm_init(&fsm);
    reach_armed(&fsm, &out);
    in.s11_fault  = 0u;
    in.estop_open = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_ESTOP);
}

static void test_balancing_stop_rest_cycle_and_radio_policies(void)
{
    SafetyFsm fsm;
    SafetyOutput out;

    /* 1. Default RADIO_LOSS_BRAKE_BALANCE (policy c): host loss -> CONTROLLED_STOP -> REST -> BALANCING -> REST -> DISARMED */
    safety_fsm_init(&fsm);
    assert(fsm.config.radio_loss_policy == SAFETY_RADIO_LOSS_BRAKE_BALANCE);
    reach_balancing(&fsm, &out);

    SafetyInputs in = make_disarmed_clean_inputs();
    in.k1_closed        = 1u;
    in.k2_closed        = 1u;
    in.v_bus_v          = 24.0f;
    in.drives_healthy_6 = 1u;
    in.radio_arm_sw     = 1u;
    in.v_target_m_s     = 1.2f;
    in.yaw_target_rad_s = 0.6f;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_BALANCING);
    assert(fabsf(out.v_ref_m_s - 1.2f) < 1e-5f);

    /* Host timeout > 150 ms -> BALANCING -> CONTROLLED_STOP */
    in.host_age_ms = 152u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
    assert(out.reason == SAFETY_REASON_HOST_LOSS);
    assert(out.torque_enable == 1u);
    assert(out.host_lost == 1u && out.host_revoke == 1u);

    /* Run 600 ms brake ramp (in 100 ms steps for concise check) */
    in.dt_ms = 100u;
    for (uint32_t i = 0u; i < 6u; ++i) {
        safety_fsm_step(&fsm, &in, &out);
        assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
    }
    assert(fabsf(out.v_ref_m_s) < 1e-6f);
    assert(fabsf(out.yaw_ref_rad_s) < 1e-6f);

    /* Disturbed velocity prevents REST confirmation */
    in.v_est_m_s = 0.10f;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP);

    /* Quiet rest dwell for 250 ms -> CONTROLLED_STOP -> REST */
    in.v_est_m_s = 0.005f;
    in.dt_ms     = 50u;
    for (uint32_t i = 0u; i < 4u; ++i) {
        safety_fsm_step(&fsm, &in, &out);
        assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
    }
    safety_fsm_step(&fsm, &in, &out); /* 250 ms dwell reached */
    assert(out.state == SAFETY_STATE_REST);
    assert(out.rest_confirmed == 1u);
    assert(out.torque_enable == 1u);

    /* REST -> BALANCING requires explicit local re-grant + fresh host */
    in.host_age_ms       = 10u;
    in.host_valid        = 1u;
    in.local_regrant_req = 0u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_REST);

    in.local_regrant_req = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_BALANCING);
    assert(out.host_lost == 0u);

    /* Voluntary stop -> CONTROLLED_STOP -> REST -> clean DISARMED (arm SW LOW at REST) */
    in.local_regrant_req  = 0u;
    in.voluntary_stop_req = 1u;
    in.dt_ms              = 2u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
    assert(out.reason == SAFETY_REASON_VOLUNTARY_STOP);

    in.voluntary_stop_req = 0u;
    in.dt_ms              = 600u;
    safety_fsm_step(&fsm, &in, &out); /* Brake complete */
    in.dt_ms              = 250u;
    safety_fsm_step(&fsm, &in, &out); /* Rest dwell complete */
    assert(out.state == SAFETY_STATE_REST);

    in.radio_arm_sw = 0u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_DISARMED);
    assert(out.reason == SAFETY_REASON_CLEAN_DISARM);

    /* 2. Optional RADIO_LOSS_TORQUE_OFF: RF loss > 150 ms -> immediate FAULT from BALANCING, STOP, REST */
    SafetyConfig off_cfg = safety_config_with_radio_policy(RADIO_LOSS_TORQUE_OFF);
    safety_fsm_init_with_config(&fsm, &off_cfg);
    reach_balancing(&fsm, &out);
    in = make_disarmed_clean_inputs();
    in.k1_closed = 1u; in.k2_closed = 1u; in.v_bus_v = 24.0f;
    in.drives_healthy_6 = 1u; in.radio_arm_sw = 1u;
    in.radio_age_ms = 152u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT);
    assert(out.reason == SAFETY_REASON_RADIO_LOSS_FAULT);

    /* 3. Optional RADIO_LOSS_BRAKE_BALANCE (policy c): RF loss > 150 ms ->
     *    CONTROLLED_STOP -> REST (keep balancing in place), even if receiver
     *    zeros radio_arm_sw on RF loss, while deliberate radio e-stop switch,
     *    lab tilt limit (12 deg), PD7 ESTOP, and drive/CAN/power faults still
     *    override immediately! */
    SafetyConfig opt_cfg = safety_config_with_radio_policy(RADIO_LOSS_BRAKE_BALANCE);
    assert(opt_cfg.radio_loss_policy == RADIO_LOSS_BRAKE_BALANCE);
    safety_fsm_init_with_config(&fsm, &opt_cfg);
    reach_balancing(&fsm, &out);

    in = make_disarmed_clean_inputs();
    in.k1_closed = 1u; in.k2_closed = 1u; in.v_bus_v = 24.0f;
    in.drives_healthy_6 = 1u; in.radio_arm_sw = 0u; /* failsafe zeroed during RF loss */
    in.radio_age_ms = 200u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
    assert(out.reason == SAFETY_REASON_RADIO_LOSS_BRAKE);
    assert(out.radio_lost == 1u);
    assert(out.torque_enable == 1u);
    assert(out.host_lost == 1u);

    in.dt_ms = 100u;
    for (uint32_t i = 0u; i < 6u; ++i) {
        safety_fsm_step(&fsm, &in, &out);
        assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
    }
    in.dt_ms = 150u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_REST);
    assert(out.torque_enable == 1u);
    assert(out.radio_lost == 1u);
    assert(out.rest_confirmed == 1u);

    /* Stays upright in REST while radio remains lost (does NOT disarm or fault) */
    in.dt_ms = 2u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_REST);
    assert(out.torque_enable == 1u);

    /* When radio returns fresh with arm SW HIGH + local re-grant -> back to BALANCING */
    in.radio_age_ms = 4u;
    in.radio_arm_sw = 1u;
    in.boot_self_test_ok = 1u;
    in.dump_continuity_ok = 1u;
    in.k1_closed = 1u;
    in.k2_closed = 1u;
    in.precharge_dv_ok = 1u;
    in.drives_healthy_6 = 1u;
    in.imu_est_valid = 1u;
    in.v_bus_v = 24.0f;
    in.local_regrant_req = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_BALANCING);
    assert(out.reason == SAFETY_REASON_LOCAL_REGRANT);

    /* Radio loss again -> CONTROLLED_STOP -> REST -> radio returns with arm SW LOW -> clean DISARMED */
    in.local_regrant_req = 0u;
    in.radio_age_ms = 200u;
    in.radio_arm_sw = 0u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
    in.dt_ms = 600u; safety_fsm_step(&fsm, &in, &out);
    in.dt_ms = 250u; safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_REST);

    in.radio_age_ms = 4u; /* Fresh radio with arm SW LOW -> clean disarm */
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_DISARMED);
    assert(out.reason == SAFETY_REASON_CLEAN_DISARM);

    /* Lab-mode 12 deg tilt limit & radio e-stop override REST under RADIO_LOSS_BRAKE_BALANCE */
    opt_cfg = safety_config_with_radio_policy(RADIO_LOSS_BRAKE_BALANCE);
    opt_cfg.tilt_limit_deg = 12.0f;
    safety_fsm_init_with_config(&fsm, &opt_cfg);
    reach_balancing(&fsm, &out);
    in = make_disarmed_clean_inputs();
    in.k1_closed = 1u; in.k2_closed = 1u; in.v_bus_v = 24.0f;
    in.drives_healthy_6 = 1u; in.radio_arm_sw = 1u; in.radio_age_ms = 200u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
    in.pitch_deg = 13.0f;
    in.dt_ms = 20u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT);
    assert(out.reason == SAFETY_REASON_TILT_LIMIT);

    /* Radio e-stop switch overrides REST even in BRAKE_BALANCE policy */
    opt_cfg = safety_config_with_radio_policy(RADIO_LOSS_BRAKE_BALANCE);
    safety_fsm_init_with_config(&fsm, &opt_cfg);
    reach_balancing(&fsm, &out);
    in = make_disarmed_clean_inputs();
    in.k1_closed = 1u; in.k2_closed = 1u; in.v_bus_v = 24.0f;
    in.drives_healthy_6 = 1u; in.radio_arm_sw = 1u; in.radio_age_ms = 200u;
    safety_fsm_step(&fsm, &in, &out);
    in.dt_ms = 600u; safety_fsm_step(&fsm, &in, &out);
    in.dt_ms = 250u; safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_REST);
    in.radio_estop_sw = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT);
    assert(out.reason == SAFETY_REASON_RADIO_ESTOP_SW);
}

static void test_fault_overrides_power_tilt_and_reset_guards(void)
{
    SafetyFsm fsm;
    SafetyOutput out;
    SafetyInputs in;

    /* 1. Mild loaded sag (18.0 V) keeps BALANCING ON, inhibits jump & signals set-down;
     *    Hard UV (< 15 V) latches FAULT */
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    in = make_disarmed_clean_inputs();
    in.k1_closed = 1u; in.k2_closed = 1u; in.drives_healthy_6 = 1u; in.radio_arm_sw = 1u;
    in.v_bus_v = 18.0f;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_BALANCING);
    assert(out.torque_enable == 1u);
    assert(out.inhibit_jump == 1u);
    assert(out.mild_sag_setdown == 1u);

    /* Transient regen spike tapers regen_scale without faulting */
    in.v_bus_v = 26.3f;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_BALANCING);
    assert(fabsf(out.regen_scale - 0.5f) < 1e-4f);

    /* Persistent OV (> 28 V) past 20 ms deglitch latches FAULT */
    in.v_bus_v = 28.5f;
    in.dt_ms   = 10u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_BALANCING); /* 10 ms < 20 ms */
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT);
    assert(out.reason == SAFETY_REASON_PERSISTENT_OV);

    /* 2. Tilt limit dwell (30 deg for 20 ms) + arm SW LOW during BALANCING -> FAULT */
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    in = make_disarmed_clean_inputs();
    in.k1_closed = 1u; in.k2_closed = 1u; in.v_bus_v = 24.0f;
    in.drives_healthy_6 = 1u; in.radio_arm_sw = 1u;
    in.pitch_deg = 31.0f;
    in.dt_ms     = 10u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_BALANCING);
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT);
    assert(out.reason == SAFETY_REASON_TILT_LIMIT);

    /* 3. BALANCING -> ESTOP, CONTROLLED_STOP -> FAULT, CONTROLLED_STOP -> ESTOP,
     *    REST -> FAULT, REST -> ESTOP, hard UV, and immediate drive OV */
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    in = make_disarmed_clean_inputs();
    in.k1_closed = 1u; in.k2_closed = 1u; in.v_bus_v = 24.0f;
    in.drives_healthy_6 = 1u; in.radio_arm_sw = 1u;
    in.estop_open = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_ESTOP);

    /* Hard UV (< 15 V) -> BALANCING -> FAULT */
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    in = make_disarmed_clean_inputs();
    in.k1_closed = 1u; in.k2_closed = 1u; in.v_bus_v = 14.5f;
    in.drives_healthy_6 = 1u; in.radio_arm_sw = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT);
    assert(out.reason == SAFETY_REASON_HARD_UV);

    /* Immediate drive OV (> 32 V) -> BALANCING -> FAULT on single tick */
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    in = make_disarmed_clean_inputs();
    in.k1_closed = 1u; in.k2_closed = 1u; in.v_bus_v = 32.5f;
    in.drives_healthy_6 = 1u; in.radio_arm_sw = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT);
    assert(out.reason == SAFETY_REASON_PERSISTENT_OV);

    /* CONTROLLED_STOP -> FAULT (optional TORQUE_OFF policy, RF loss > 150 ms) */
    {
        SafetyConfig off_cfg2 = safety_config_with_radio_policy(RADIO_LOSS_TORQUE_OFF);
        safety_fsm_init_with_config(&fsm, &off_cfg2);
    }
    reach_balancing(&fsm, &out);
    in = make_disarmed_clean_inputs();
    in.k1_closed = 1u; in.k2_closed = 1u; in.v_bus_v = 24.0f;
    in.drives_healthy_6 = 1u; in.radio_arm_sw = 1u;
    in.voluntary_stop_req = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
    in.radio_age_ms = 160u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT);
    assert(out.reason == SAFETY_REASON_RADIO_LOSS_FAULT);

    /* Default policy c: the same RF loss during CONTROLLED_STOP must NOT fault (keep balancing) */
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    in = make_disarmed_clean_inputs();
    in.k1_closed = 1u; in.k2_closed = 1u; in.v_bus_v = 24.0f;
    in.drives_healthy_6 = 1u; in.radio_arm_sw = 1u;
    in.voluntary_stop_req = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
    in.radio_age_ms = 160u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state != SAFETY_STATE_FAULT);
    assert(out.torque_enable != 0u);

    /* CONTROLLED_STOP -> ESTOP */
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    in = make_disarmed_clean_inputs();
    in.k1_closed = 1u; in.k2_closed = 1u; in.v_bus_v = 24.0f;
    in.drives_healthy_6 = 1u; in.radio_arm_sw = 1u;
    in.voluntary_stop_req = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
    in.estop_open = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_ESTOP);

    /* REST -> FAULT (optional TORQUE_OFF policy, RF loss > 150 ms) */
    {
        SafetyConfig off_cfg3 = safety_config_with_radio_policy(RADIO_LOSS_TORQUE_OFF);
        safety_fsm_init_with_config(&fsm, &off_cfg3);
    }
    reach_balancing(&fsm, &out);
    in = make_disarmed_clean_inputs();
    in.k1_closed = 1u; in.k2_closed = 1u; in.v_bus_v = 24.0f;
    in.drives_healthy_6 = 1u; in.radio_arm_sw = 1u;
    in.voluntary_stop_req = 1u;
    safety_fsm_step(&fsm, &in, &out);
    in.voluntary_stop_req = 0u;
    in.dt_ms = 600u; safety_fsm_step(&fsm, &in, &out);
    in.dt_ms = 250u; safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_REST);
    in.radio_age_ms = 160u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT);
    assert(out.reason == SAFETY_REASON_RADIO_LOSS_FAULT);

    /* REST -> ESTOP */
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    in = make_disarmed_clean_inputs();
    in.k1_closed = 1u; in.k2_closed = 1u; in.v_bus_v = 24.0f;
    in.drives_healthy_6 = 1u; in.radio_arm_sw = 1u;
    in.voluntary_stop_req = 1u;
    safety_fsm_step(&fsm, &in, &out);
    in.voluntary_stop_req = 0u;
    in.dt_ms = 600u; safety_fsm_step(&fsm, &in, &out);
    in.dt_ms = 250u; safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_REST);
    in.estop_open = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_ESTOP);

    /* 4. FAULT -> ESTOP, FAULT -> DISARMED guards, and ESTOP -> DISARMED guards */
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    in = make_disarmed_clean_inputs();
    in.k1_closed = 1u; in.k2_closed = 1u; in.v_bus_v = 24.0f;
    in.drives_healthy_6 = 1u; in.radio_arm_sw = 0u;
    /* FW8 F-04: arm SW LOW alone now brakes (CONTROLLED_STOP); reach FAULT via
     * the radio e-stop switch instead, then release it for the reset guards. */
    in.radio_estop_sw = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT);
    assert(out.reason == SAFETY_REASON_RADIO_ESTOP_SW);
    in.radio_estop_sw = 0u;

    /* Manual reset blocked while K1 closed or link >= 5 V or arm SW HIGH */
    in.local_reset_req = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT); /* K1/K2 still closed, v_bus 24 V */

    in.k1_closed = 0u; in.k2_closed = 0u; in.v_bus_v = 6.0f;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT); /* Link still >= 5 V */

    in.v_bus_v = 2.0f; in.radio_arm_sw = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT); /* Arm SW still HIGH */

    in.radio_arm_sw = 0u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_DISARMED); /* All guards satisfied -> DISARMED */

    /* FAULT -> ESTOP -> DISARMED requires hw_reset_ack + local_reset_req */
    in = make_disarmed_clean_inputs();
    in.boot_self_test_ok = 0u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT);
    in.estop_open = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_ESTOP);

    /* Release estop, clean conditions, missing hw_reset_ack -> stays ESTOP */
    in = make_disarmed_clean_inputs();
    in.local_reset_req = 1u;
    in.hw_reset_ack    = 0u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_ESTOP);

    in.hw_reset_ack = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_DISARMED);
}

static void test_s11_state_wiring_in_safety_fsm(void)
{
    SafetyFsm fsm;
    SafetyOutput out;
    SafetyInputs in;
    S11PacketFrame frame;

    /* 1. S11_ARMING holds PRECHARGE until S11 reaches S11_FRESH */
    safety_fsm_init(&fsm);
    in = make_disarmed_clean_inputs();
    in.local_reset_req = 1u;
    in.local_arm_req   = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_PRECHARGE);

    in.local_reset_req  = 0u;
    in.local_arm_req    = 0u;
    in.k1_closed        = 1u;
    in.k2_closed        = 1u;
    in.precharge_dv_ok  = 1u;
    in.v_bus_v          = 24.0f;
    in.drives_healthy_6 = 1u;
    in.radio_arm_sw     = 1u;

    memset(&frame, 0, sizeof(frame));
    frame.mode = S11_ARMING;
    frame.valid = 0u;
    frame.authority = 0.0;
    safety_fsm_step_with_s11(&fsm, &in, &frame, &out);
    assert(out.state == SAFETY_STATE_PRECHARGE);
    assert(out.can_enable_cmd == 1u);
    assert(out.torque_enable == 0u);

    /* Once S11 completes 3 groups and outputs S11_FRESH -> enters ARMED */
    frame.mode = S11_FRESH;
    frame.valid = 1u;
    frame.authority = 1.0;
    safety_fsm_step_with_s11(&fsm, &in, &frame, &out);
    assert(out.state == SAFETY_STATE_ARMED);
    assert(out.reason == SAFETY_REASON_PRECHARGE_OK);

    /* Stand handoff into BALANCING with S11_FRESH */
    in.stand_handoff_req = 1u;
    in.v_target_m_s      = 0.40f;
    in.yaw_target_rad_s  = 0.20f;
    safety_fsm_step_with_s11(&fsm, &in, &frame, &out);
    assert(out.state == SAFETY_STATE_BALANCING);
    assert(out.torque_enable == 1u);

    in.stand_handoff_req = 0u;
    safety_fsm_step_with_s11(&fsm, &in, &frame, &out);
    assert(out.state == SAFETY_STATE_BALANCING);
    assert(fabsf(out.v_ref_m_s - 0.40f) < 1e-6f);
    assert(out.inhibit_trim == 0u && out.inhibit_jump == 0u);

    /* 2. Stale S11_PREDICT (1..2 misses): maintains full authority (1.0) */
    frame.mode = S11_PREDICT;
    frame.valid = 1u;
    frame.authority = 1.0;
    safety_fsm_step_with_s11(&fsm, &in, &frame, &out);
    assert(out.state == SAFETY_STATE_BALANCING);
    assert(out.torque_enable == 1u);
    assert(fabsf(out.v_ref_m_s - 0.40f) < 1e-6f);

    /* 3. Stale S11_LIMIT (3..4 misses): scales authority to 75% and inhibits trim & jump */
    frame.mode = S11_LIMIT;
    frame.valid = 1u;
    frame.authority = 0.75;
    safety_fsm_step_with_s11(&fsm, &in, &frame, &out);
    assert(out.state == SAFETY_STATE_BALANCING);
    assert(out.torque_enable == 1u);
    assert(fabsf(out.v_ref_m_s - 0.30f) < 1e-6f);
    assert(fabsf(out.yaw_ref_rad_s - 0.15f) < 1e-6f);
    assert(out.inhibit_trim == 1u && out.inhibit_jump == 1u);

    /* 4. Stale cutoff S11_TIMEOUT, frozen axis S11_STUCK, and S11_INVALID -> immediate latched FAULT */
    const enum S11Mode pkt_faults[3] = { S11_TIMEOUT, S11_STUCK, S11_INVALID };
    for (uint32_t k = 0u; k < 3u; ++k) {
        safety_fsm_init(&fsm);
        reach_balancing(&fsm, &out);
        frame.mode = pkt_faults[k];
        frame.valid = 0u;
        frame.authority = 0.0;
        safety_fsm_step_with_s11(&fsm, &in, &frame, &out);
        assert(out.state == SAFETY_STATE_FAULT);
        assert(out.reason == SAFETY_REASON_S11_FAULT);
        assert(out.torque_enable == 0u);
        assert(out.permit_out == 0u);
        assert(out.can_disable_cmd == 1u && out.can_abort_tx == 1u);
        assert(out.host_revoke == 1u);
    }

    /* 5. S11_DRIVE_FAULT -> immediate latched FAULT with SAFETY_REASON_DRIVE_FAULT */
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    frame.mode = S11_DRIVE_FAULT;
    frame.valid = 0u;
    frame.authority = 0.0;
    safety_fsm_step_with_s11(&fsm, &in, &frame, &out);
    assert(out.state == SAFETY_STATE_FAULT);
    assert(out.reason == SAFETY_REASON_DRIVE_FAULT);
    assert(out.torque_enable == 0u);
    assert(out.permit_out == 0u);
    assert(out.can_disable_cmd == 1u && out.can_abort_tx == 1u);
}

static void test_step_assist_supervision(void)
{
    SafetyFsm fsm;
    SafetyOutput out;
    SafetyInputs in = make_disarmed_clean_inputs();

    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    in.radio_arm_sw = 1u;
    in.v_bus_v = 24.0f;
    in.drives_healthy_6 = 1u;
    in.host_valid = 1u;
    in.host_age_ms = 0u;
    in.v_target_m_s = 0.12f;

    /* Missing legs/assist is represented by no qualified request. */
    in.step_assist_req = 0u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_BALANCING);

    /* Qualified entry keeps torque enabled in the supervised step state. */
    in.step_assist_req = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_STEP_ASSIST);
    assert(out.torque_enable == 1u);

    /* Failure/timeout is a controlled stop, preserving the balance torque path. */
    in.step_assist_failed = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
    assert(out.torque_enable == 1u);

    /* A fresh attempt can complete and hand back to ordinary balancing. */
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    in.step_assist_failed = 0u;
    in.step_assist_req = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_STEP_ASSIST);
    in.step_assist_req = 0u;
    in.step_assist_done = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_BALANCING);
}

/* FW6REVIEW: the torque-off paths must also win from STEP_ASSIST (the run-up
 * charge is the highest-energy state the robot has). */
static void reach_step_assist(SafetyFsm *fsm, SafetyOutput *out, SafetyInputs *in)
{
    safety_fsm_init(fsm);
    reach_balancing(fsm, out);
    *in = make_disarmed_clean_inputs();
    in->k1_closed = 1u;
    in->k2_closed = 1u;
    in->radio_arm_sw = 1u;
    in->v_bus_v = 24.0f;
    in->drives_healthy_6 = 1u;
    in->v_target_m_s = 0.12f;
    in->step_assist_req = 1u;
    safety_fsm_step(fsm, in, out);
    assert(out->state == SAFETY_STATE_STEP_ASSIST && out->torque_enable == 1u);
    in->step_assist_req = 0u;
}

static void test_step_assist_torque_off_paths(void)
{
    SafetyFsm fsm;
    SafetyOutput out;
    SafetyInputs in;

    /* wired / safety-radio e-stop */
    reach_step_assist(&fsm, &out, &in);
    in.estop_open = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_ESTOP && out.torque_enable == 0u);
    assert(out.can_disable_cmd == 1u && out.can_abort_tx == 1u);

    /* radio e-stop switch */
    reach_step_assist(&fsm, &out, &in);
    in.radio_estop_sw = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT && out.torque_enable == 0u);

    /* drive fault (status) and S11 drive-fault mode */
    reach_step_assist(&fsm, &out, &in);
    in.drives_healthy_6 = 0u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT && out.reason == SAFETY_REASON_DRIVE_FAULT);
    assert(out.torque_enable == 0u && out.can_disable_cmd == 1u);
    reach_step_assist(&fsm, &out, &in);
    in.s11_mode = S11_DRIVE_FAULT;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT && out.torque_enable == 0u);

    /* cannot balance: tilt >= 30 deg held 20 ms */
    reach_step_assist(&fsm, &out, &in);
    in.pitch_deg = 31.0f;
    for (unsigned t = 0u; t < 12u && out.state != SAFETY_STATE_FAULT; ++t) {
        safety_fsm_step(&fsm, &in, &out);
    }
    assert(out.state == SAFETY_STATE_FAULT && out.reason == SAFETY_REASON_TILT_LIMIT);
    assert(out.torque_enable == 0u);

    /* arm switch dropped with fresh radio: FW8 F-04 brakes instead of cutting
     * torque (same path as radio loss over the curb); e-stop above still cuts. */
    reach_step_assist(&fsm, &out, &in);
    in.radio_arm_sw = 0u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP && out.torque_enable == 1u);
    assert(out.reason == SAFETY_REASON_ARM_SW_DROPPED);

    /* S11 double fault (latched STUCK) */
    reach_step_assist(&fsm, &out, &in);
    in.s11_mode = S11_STUCK;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_FAULT && out.torque_enable == 0u);
}

/* FW6FIX D3: a run-up abort has its own reason; an operator stop from
 * STEP_ASSIST keeps VOLUNTARY_STOP. */
static void test_fw6fix_step_abort_reason(void)
{
    SafetyFsm fsm;
    SafetyOutput out;
    SafetyInputs in;

    reach_step_assist(&fsm, &out, &in);
    in.step_assist_failed = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
    assert(out.reason == SAFETY_REASON_STEP_ASSIST_ABORT);
    assert(out.reason != SAFETY_REASON_VOLUNTARY_STOP);
    assert(out.torque_enable == 1u && out.can_disable_cmd == 0u);

    reach_step_assist(&fsm, &out, &in);
    in.voluntary_stop_req = 1u;
    in.step_assist_failed = 1u;   /* operator request takes precedence */
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
    assert(out.reason == SAFETY_REASON_VOLUNTARY_STOP);

    /* A failure flag outside STEP_ASSIST is ignored (no spurious abort reason). */
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    in = make_disarmed_clean_inputs();
    in.k1_closed = 1u;
    in.k2_closed = 1u;
    in.radio_arm_sw = 1u;
    in.v_bus_v = 24.0f;
    in.drives_healthy_6 = 1u;
    in.step_assist_failed = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_BALANCING);
}

/* FW6FIX D2: CONTROLLED_STOP is bounded. A robot that never settles (pitch held
 * at 7.5 deg > 5 deg REST limit) goes to REST after stop_timeout_ms with torque
 * kept, the alarm latched and REST not confirmed; torque-off escalations still
 * apply afterwards. A normal stop is unaffected. */
static SafetyInputs stop_inputs(void)
{
    SafetyInputs in = make_disarmed_clean_inputs();
    in.k1_closed = 1u;
    in.k2_closed = 1u;
    in.radio_arm_sw = 1u;
    in.v_bus_v = 24.0f;
    in.drives_healthy_6 = 1u;
    return in;
}

static void test_fw6fix_stop_timeout(void)
{
    SafetyFsm fsm;
    SafetyOutput out;
    const SafetyConfig def = safety_default_config();
    assert(def.stop_timeout_ms == 20000u);

    /* stuck: never meets the REST condition */
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    SafetyInputs in = stop_inputs();
    in.v_target_m_s = 0.3f;
    safety_fsm_step(&fsm, &in, &out);
    in.host_valid = 0u;                     /* host loss -> controlled stop */
    in.pitch_deg = -7.5f;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
    assert(out.reason == SAFETY_REASON_HOST_LOSS);
    uint32_t ms = 2u;
    while (out.state == SAFETY_STATE_CONTROLLED_STOP && ms < 30000u) {
        assert(out.stop_timeout_alarm == 0u);
        assert(out.torque_enable == 1u);
        safety_fsm_step(&fsm, &in, &out);
        ms += 2u;
    }
    assert(ms >= 20000u && ms <= 20004u);    /* bounded at 20 s from stop entry */
    assert(out.state == SAFETY_STATE_REST);
    assert(out.reason == SAFETY_REASON_STOP_TIMEOUT);
    assert(out.stop_timeout_alarm == 1u);
    assert(out.rest_confirmed == 0u);
    assert(out.torque_enable == 1u && out.can_disable_cmd == 0u);  /* torque kept */
    assert(out.v_ref_m_s == 0.0f && out.host_lost == 1u && out.host_revoke == 1u);
    for (unsigned k = 0u; k < 100u; ++k) {    /* alarm stays latched in REST */
        safety_fsm_step(&fsm, &in, &out);
        assert(out.state == SAFETY_STATE_REST && out.stop_timeout_alarm == 1u);
    }
    /* torque-off escalation still applies from the timed-out REST */
    in.pitch_deg = 31.0f;
    for (unsigned t = 0u; t < 12u && out.state != SAFETY_STATE_FAULT; ++t) {
        safety_fsm_step(&fsm, &in, &out);
    }
    assert(out.state == SAFETY_STATE_FAULT && out.reason == SAFETY_REASON_TILT_LIMIT);
    assert(out.torque_enable == 0u && out.stop_timeout_alarm == 0u);

    /* clean disarm from the timed-out REST clears the alarm */
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    in = stop_inputs();
    in.voluntary_stop_req = 1u;
    in.pitch_deg = 7.5f;
    for (uint32_t t = 0u; t <= 20000u; t += 2u) {
        safety_fsm_step(&fsm, &in, &out);
    }
    assert(out.state == SAFETY_STATE_REST && out.stop_timeout_alarm == 1u);
    in.radio_arm_sw = 0u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_DISARMED && out.stop_timeout_alarm == 0u);

    /* local re-grant from the timed-out REST clears the alarm */
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    in = stop_inputs();
    in.voluntary_stop_req = 1u;
    in.pitch_deg = 7.5f;
    for (uint32_t t = 0u; t <= 20000u; t += 2u) {
        safety_fsm_step(&fsm, &in, &out);
    }
    assert(out.stop_timeout_alarm == 1u);
    in.voluntary_stop_req = 0u;
    in.local_regrant_req = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_BALANCING && out.stop_timeout_alarm == 0u);

    /* a stop that settles reaches a confirmed REST without the alarm */
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    in = stop_inputs();
    in.voluntary_stop_req = 1u;
    in.pitch_deg = 1.0f;
    for (uint32_t t = 0u; t < 2000u && out.state != SAFETY_STATE_REST; t += 2u) {
        safety_fsm_step(&fsm, &in, &out);
    }
    assert(out.state == SAFETY_STATE_REST && out.reason == SAFETY_REASON_REST_CONFIRMED);
    assert(out.rest_confirmed == 1u && out.stop_timeout_alarm == 0u);

    /* stop_timeout_ms = 0 disables the bound (configs not built from the default) */
    SafetyConfig c = safety_default_config();
    c.stop_timeout_ms = 0u;
    safety_fsm_init_with_config(&fsm, &c);
    reach_balancing(&fsm, &out);
    in = stop_inputs();
    in.voluntary_stop_req = 1u;
    in.pitch_deg = 7.5f;
    for (uint32_t t = 0u; t < 30000u; t += 2u) {
        safety_fsm_step(&fsm, &in, &out);
    }
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP && out.stop_timeout_alarm == 0u);
}

/* FW7 open point A: a pure single-axis S11_DEGRADED substitution no longer freezes
 * the S9 trim once the brake ramp is over (CONTROLLED_STOP and REST). The trim stays
 * inhibited during the ramp, when the frame is not claimed degraded-only (stale
 * packets / CAN error-passive), in BALANCING, and with the config flag off. */
static int fw7_trim_open_after_ramp(uint8_t cfg_flag, uint8_t degraded_only,
                                    enum S11Mode mode, uint8_t *ramp_inhibit)
{
    SafetyFsm fsm;
    SafetyOutput out;
    SafetyConfig cfg = safety_default_config();
    cfg.degraded_settle_trim = cfg_flag;
    safety_fsm_init_with_config(&fsm, &cfg);
    reach_balancing(&fsm, &out);
    SafetyInputs in = stop_inputs();
    in.s11_mode = mode;
    in.s11_degraded = (mode == S11_DEGRADED) ? 1u : 0u;
    in.s11_limited = 1u;
    in.s11_degraded_only = degraded_only;
    in.pitch_deg = -5.6f;                    /* held just past the REST limit */
    if (mode != S11_DEGRADED) {
        in.voluntary_stop_req = 1u;           /* LIMIT alone does not stop */
    }
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
    *ramp_inhibit = out.inhibit_trim;
    for (uint32_t t = 0u; t < 1000u; t += 2u) {
        safety_fsm_step(&fsm, &in, &out);
        if (t < 500u) {
            assert(out.inhibit_trim == 1u);   /* 600 ms brake ramp: always inhibited */
        }
    }
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP && out.torque_enable == 1u);
    const int open_stop = (out.inhibit_trim == 0u);
    /* settle (trim learned) -> REST confirmed; trim gate identical in REST */
    in.pitch_deg = -1.0f;
    for (uint32_t t = 0u; t < 400u; t += 2u) {
        safety_fsm_step(&fsm, &in, &out);
    }
    assert(out.state == SAFETY_STATE_REST && out.rest_confirmed == 1u);
    assert(out.torque_enable == 1u);
    assert((out.inhibit_trim == 0u) == open_stop);
    /* torque-off escalation is unchanged: tilt > 30 deg from REST -> FAULT */
    in.pitch_deg = 31.0f;
    for (unsigned k = 0u; k < 12u && out.state != SAFETY_STATE_FAULT; ++k) {
        safety_fsm_step(&fsm, &in, &out);
    }
    assert(out.state == SAFETY_STATE_FAULT && out.torque_enable == 0u && out.inhibit_trim == 1u);
    return open_stop;
}

static void test_fw7_degraded_settle_trim(void)
{
    uint8_t ramp = 0u;
    assert(safety_default_config().degraded_settle_trim == 1u);
    assert(fw7_trim_open_after_ramp(1u, 1u, S11_DEGRADED, &ramp) == 1 && ramp == 1u);
    assert(fw7_trim_open_after_ramp(1u, 0u, S11_DEGRADED, &ramp) == 0 && ramp == 1u);
    assert(fw7_trim_open_after_ramp(0u, 1u, S11_DEGRADED, &ramp) == 0 && ramp == 1u);
    /* a degraded_only claim without a DEGRADED frame (e.g. LIMIT) is ignored */
    assert(fw7_trim_open_after_ramp(1u, 1u, S11_LIMIT, &ramp) == 0 && ramp == 1u);
    /* BALANCING with a DEGRADED frame never reaches here without a stop; trim stays
     * inhibited on the entry tick (the stop is entered on the same tick) */
    SafetyFsm fsm;
    SafetyOutput out;
    safety_fsm_init(&fsm);
    reach_balancing(&fsm, &out);
    SafetyInputs in = stop_inputs();
    in.s11_mode = S11_DEGRADED;
    in.s11_degraded = 1u;
    in.s11_limited = 1u;
    in.s11_degraded_only = 1u;
    safety_fsm_step(&fsm, &in, &out);
    assert(out.state == SAFETY_STATE_CONTROLLED_STOP && out.reason == SAFETY_REASON_S11_DEGRADED);
    assert(out.inhibit_trim == 1u);
    printf("test_fw7_degraded_settle_trim: passed\n");
}

int main(void)
{
    test_disarmed_faults_and_estop();
    test_precharge_transitions();
    test_armed_transitions();
    test_balancing_stop_rest_cycle_and_radio_policies();
    test_fault_overrides_power_tilt_and_reset_guards();
    test_s11_state_wiring_in_safety_fsm();
    test_step_assist_supervision();
    test_step_assist_torque_off_paths();
    test_fw6fix_step_abort_reason();
    test_fw6fix_stop_timeout();
    test_fw7_degraded_settle_trim();
    printf("test_safety_fsm: ALL PASSED\n");
    return 0;
}
