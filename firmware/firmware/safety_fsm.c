#include "safety_fsm.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static uint8_t is_active_state(SafetyState s)
{
    return (s == SAFETY_STATE_ARMED
         || s == SAFETY_STATE_BALANCING
         || s == SAFETY_STATE_CONTROLLED_STOP
         || s == SAFETY_STATE_REST) ? 1u : 0u;
}

static uint8_t is_torque_state(SafetyState s)
{
    return (s == SAFETY_STATE_BALANCING
         || s == SAFETY_STATE_CONTROLLED_STOP
         || s == SAFETY_STATE_REST) ? 1u : 0u;
}

static uint8_t s11_mode_is_packet_fault(enum S11Mode m)
{
    return (m == S11_TIMEOUT || m == S11_STUCK || m == S11_INVALID) ? 1u : 0u;
}

static uint8_t s11_mode_is_any_fault(enum S11Mode m)
{
    return (m == S11_TIMEOUT || m == S11_STUCK
         || m == S11_INVALID || m == S11_DRIVE_FAULT) ? 1u : 0u;
}

static uint8_t s11_mode_is_unarmed(enum S11Mode m)
{
    return (m == S11_ARMING || m == S11_DISARMED) ? 1u : 0u;
}

static uint8_t effective_s11_limited(const SafetyInputs *in)
{
    if (in->s11_limited != 0u || in->s11_mode == S11_LIMIT) {
        return 1u;
    }
    if (in->s11_frame_bound != 0u && in->s11_authority < 1.0f) {
        return 1u;
    }
    return 0u;
}

static float effective_s11_authority(const SafetyInputs *in)
{
    if (in->s11_frame_bound != 0u) {
        if (!isfinite(in->s11_authority) || in->s11_authority <= 0.0f) {
            return 0.0f;
        }
        return (in->s11_authority > 1.0f) ? 1.0f : in->s11_authority;
    }
    if (in->s11_mode == S11_LIMIT) {
        return 0.75f;
    }
    if (s11_mode_is_any_fault(in->s11_mode) != 0u || s11_mode_is_unarmed(in->s11_mode) != 0u) {
        return 0.0f;
    }
    return 1.0f;
}

static float compute_regen_scale(const SafetyConfig *cfg, float v_bus_v)
{
    if (!isfinite(v_bus_v) || v_bus_v <= cfg->regen_taper_start_v) {
        return 1.0f;
    }
    if (v_bus_v >= cfg->regen_taper_full_v) {
        return 0.0f;
    }
    const float span = cfg->regen_taper_full_v - cfg->regen_taper_start_v;
    if (span <= 0.0f) {
        return 0.0f;
    }
    return 1.0f - ((v_bus_v - cfg->regen_taper_start_v) / span);
}

static void reset_timers(SafetyFsm *fsm)
{
    fsm->precharge_elapsed_ms = 0u;
    fsm->brake_elapsed_ms = 0u;
    fsm->rest_dwell_ms = 0u;
    fsm->tilt_dwell_ms = 0u;
    fsm->ov_dwell_ms = 0u;
    fsm->v_ref_m_s = 0.0f;
    fsm->yaw_ref_rad_s = 0.0f;
    fsm->brake_start_v_m_s = 0.0f;
    fsm->brake_start_yaw_rad_s = 0.0f;
    fsm->host_lost_latched = 0u;
}

static void enter_fault(SafetyFsm *fsm, SafetyReason reason)
{
    fsm->state = SAFETY_STATE_FAULT;
    fsm->reason = reason;
    reset_timers(fsm);
}

static void enter_estop(SafetyFsm *fsm)
{
    fsm->state = SAFETY_STATE_ESTOP;
    fsm->reason = SAFETY_REASON_ESTOP_HW;
    reset_timers(fsm);
}

static void enter_controlled_stop(SafetyFsm *fsm, SafetyReason reason)
{
    fsm->state = SAFETY_STATE_CONTROLLED_STOP;
    fsm->reason = reason;
    fsm->brake_start_v_m_s = fsm->v_ref_m_s;
    fsm->brake_start_yaw_rad_s = fsm->yaw_ref_rad_s;
    fsm->brake_elapsed_ms = 0u;
    fsm->rest_dwell_ms = 0u;
    fsm->host_lost_latched = 1u;
}

void safety_fsm_bind_s11_frame(SafetyInputs *in, const S11PacketFrame *s11_frame)
{
    if (in == NULL) {
        return;
    }
    if (s11_frame == NULL) {
        in->s11_mode        = S11_INVALID;
        in->s11_fault       = 1u;
        in->s11_limited     = 1u;
        in->s11_frame_bound = 1u;
        in->s11_authority   = 0.0f;
        return;
    }

    in->s11_mode        = s11_frame->mode;
    in->s11_frame_bound = 1u;
    in->s11_authority   = (s11_frame->valid != 0 && isfinite(s11_frame->authority))
                        ? (float)s11_frame->authority : 0.0f;
    in->s11_fault       = s11_mode_is_any_fault(s11_frame->mode);
    in->s11_limited     = (s11_frame->mode == S11_LIMIT
                           || (s11_frame->valid != 0 && s11_frame->authority < 1.0)) ? 1u : 0u;
    if (s11_frame->mode == S11_DRIVE_FAULT) {
        in->drives_healthy_6 = 0u;
    }
}

static SafetyReason check_active_faults(SafetyFsm *fsm, const SafetyInputs *in)
{
    const SafetyConfig *cfg = &fsm->config;

    if (in->radio_estop_sw != 0u) {
        return SAFETY_REASON_RADIO_ESTOP_SW;
    }
    if (in->can_bus_off != 0u) {
        return SAFETY_REASON_CAN_BUS_OFF;
    }
    if (in->s11_mode == S11_DRIVE_FAULT) {
        return SAFETY_REASON_DRIVE_FAULT;
    }
    if (in->s11_fault != 0u || s11_mode_is_packet_fault(in->s11_mode) != 0u) {
        return SAFETY_REASON_S11_FAULT;
    }
    if (in->drives_healthy_6 == 0u) {
        return SAFETY_REASON_DRIVE_FAULT;
    }
    if (in->clamp_fault != 0u) {
        return SAFETY_REASON_CLAMP_FAULT;
    }
    if (in->wdg_fault != 0u) {
        return SAFETY_REASON_WDG_FAULT;
    }
    if (in->bor3_fault != 0u) {
        return SAFETY_REASON_BOR3;
    }
    if (in->hard_uv_fault != 0u || !isfinite(in->v_bus_v) || in->v_bus_v < cfg->hard_uv_v) {
        return SAFETY_REASON_HARD_UV;
    }
    if (in->v_bus_v > cfg->immediate_ov_v) {
        return SAFETY_REASON_PERSISTENT_OV;
    }
    if (in->v_bus_v > cfg->persistent_ov_v) {
        fsm->ov_dwell_ms += in->dt_ms;
        if (fsm->ov_dwell_ms >= cfg->ov_deglitch_ms) {
            return SAFETY_REASON_PERSISTENT_OV;
        }
    } else {
        fsm->ov_dwell_ms = 0u;
    }

    if (in->imu_est_valid == 0u
        || !isfinite(in->pitch_deg)
        || !isfinite(in->roll_deg)
        || !isfinite(in->pitch_rate_deg_s)
        || !isfinite(in->v_est_m_s)
        || !isfinite(in->v_target_m_s)
        || !isfinite(in->yaw_target_rad_s)) {
        return SAFETY_REASON_IMU_EST_INVALID;
    }

    if (fabsf(in->pitch_deg) >= cfg->tilt_limit_deg
        || fabsf(in->roll_deg) >= cfg->tilt_limit_deg) {
        fsm->tilt_dwell_ms += in->dt_ms;
        if (fsm->tilt_dwell_ms >= cfg->tilt_dwell_ms) {
            return SAFETY_REASON_TILT_LIMIT;
        }
    } else {
        fsm->tilt_dwell_ms = 0u;
    }

    const uint8_t radio_lost = (in->radio_age_ms > cfg->radio_timeout_ms) ? 1u : 0u;
    if (radio_lost != 0u && cfg->radio_loss_policy == RADIO_LOSS_TORQUE_OFF) {
        return SAFETY_REASON_RADIO_LOSS_FAULT;
    }

    return SAFETY_REASON_NONE;
}

static void populate_output(const SafetyFsm *fsm, const SafetyInputs *in,
                            uint8_t can_enable_cmd, uint8_t clean_disarm_pulse,
                            SafetyOutput *out)
{
    memset(out, 0, sizeof(*out));
    out->state = fsm->state;
    out->reason = fsm->reason;
    out->radio_lost = (in->radio_age_ms > fsm->config.radio_timeout_ms) ? 1u : 0u;
    out->regen_scale = 1.0f;

    if (fsm->state == SAFETY_STATE_FAULT || fsm->state == SAFETY_STATE_ESTOP) {
        out->permit_out = 0u;
        out->torque_enable = 0u;
        out->can_disable_cmd = 1u;
        out->can_abort_tx = 1u;
        out->host_revoke = 1u;
        out->inhibit_trim = 1u;
        out->inhibit_jump = 1u;
        return;
    }

    if (fsm->state == SAFETY_STATE_DISARMED) {
        out->permit_out = 0u;
        out->torque_enable = 0u;
        out->can_disable_cmd = clean_disarm_pulse;
        out->host_revoke = 1u;
        out->inhibit_trim = 1u;
        out->inhibit_jump = 1u;
        return;
    }

    if (fsm->state == SAFETY_STATE_PRECHARGE) {
        out->permit_out = 1u;
        out->torque_enable = 0u;
        out->can_enable_cmd = can_enable_cmd;
        out->host_revoke = 1u;
        out->inhibit_trim = 1u;
        out->inhibit_jump = 1u;
        return;
    }

    /* Active states: ARMED, BALANCING, CONTROLLED_STOP, REST */
    const uint8_t s11_unarmed = s11_mode_is_unarmed(in->s11_mode);
    const uint8_t s11_lim     = effective_s11_limited(in);

    out->permit_out = 1u;
    out->torque_enable = (is_torque_state(fsm->state) != 0u && s11_unarmed == 0u) ? 1u : 0u;
    out->host_lost = fsm->host_lost_latched;
    out->host_revoke = (fsm->state == SAFETY_STATE_CONTROLLED_STOP
                     || fsm->state == SAFETY_STATE_REST) ? 1u : 0u;
    out->rest_confirmed = (fsm->state == SAFETY_STATE_REST) ? 1u : 0u;
    out->v_ref_m_s = (out->torque_enable != 0u) ? fsm->v_ref_m_s : 0.0f;
    out->yaw_ref_rad_s = (out->torque_enable != 0u) ? fsm->yaw_ref_rad_s : 0.0f;
    out->regen_scale = compute_regen_scale(&fsm->config, in->v_bus_v);

    const uint8_t mild_sag = (isfinite(in->v_bus_v)
                           && in->v_bus_v < fsm->config.mild_sag_v
                           && in->v_bus_v >= fsm->config.hard_uv_v) ? 1u : 0u;
    out->mild_sag_setdown = mild_sag;

    out->inhibit_jump = (fsm->state != SAFETY_STATE_BALANCING
                      || mild_sag != 0u
                      || s11_lim != 0u
                      || s11_unarmed != 0u) ? 1u : 0u;

    const uint8_t active_brake_ramp =
        (fsm->state == SAFETY_STATE_CONTROLLED_STOP
         && fsm->brake_elapsed_ms < fsm->config.brake_duration_ms) ? 1u : 0u;

    out->inhibit_trim = (fsm->state == SAFETY_STATE_ARMED
                      || active_brake_ramp != 0u
                      || s11_lim != 0u
                      || s11_unarmed != 0u
                      || in->ground_left == 0u
                      || in->ground_right == 0u) ? 1u : 0u;
}

SafetyConfig safety_default_config(void)
{
    SafetyConfig c;
    c.radio_loss_policy         = (uint8_t)RADIO_LOSS_POLICY;
    c.precharge_timeout_ms      = 600u;
    c.radio_timeout_ms          = 150u;
    c.host_timeout_ms           = 150u;
    c.brake_duration_ms         = 600u;
    c.rest_dwell_ms             = 250u;
    c.tilt_dwell_ms             = 20u;
    c.ov_deglitch_ms            = 20u;
    c.link_discharged_max_v     = 5.0f;
    c.arm_vbus_min_v            = 20.5f;
    c.mild_sag_v                = 18.5f;
    c.hard_uv_v                 = 15.0f;
    c.regen_taper_start_v       = 25.8f;
    c.regen_taper_full_v        = 26.8f;
    c.persistent_ov_v           = 28.0f;
    c.immediate_ov_v            = 32.0f;
    c.stand_pitch_max_deg       = 5.0f;
    c.tilt_limit_deg            = 30.0f;
    c.rest_pitch_max_deg        = 5.0f;
    c.rest_vel_max_m_s          = 0.02f;
    c.rest_pitch_rate_max_deg_s = 2.0f;
    return c;
}

SafetyConfig safety_config_with_radio_policy(uint8_t radio_loss_policy)
{
    SafetyConfig c = safety_default_config();
    c.radio_loss_policy = (radio_loss_policy == RADIO_LOSS_BRAKE_BALANCE)
                        ? RADIO_LOSS_BRAKE_BALANCE
                        : RADIO_LOSS_TORQUE_OFF;
    return c;
}

void safety_fsm_init(SafetyFsm *fsm)
{
    if (fsm == NULL) {
        return;
    }
    const SafetyConfig cfg = safety_default_config();
    safety_fsm_init_with_config(fsm, &cfg);
}

void safety_fsm_init_with_config(SafetyFsm *fsm, const SafetyConfig *cfg)
{
    if (fsm == NULL) {
        return;
    }
    memset(fsm, 0, sizeof(*fsm));
    fsm->config = (cfg != NULL) ? *cfg : safety_default_config();
    fsm->state = SAFETY_STATE_DISARMED;
    fsm->reason = SAFETY_REASON_POWER_ON;
}

void safety_fsm_step(SafetyFsm *fsm, const SafetyInputs *in, SafetyOutput *out)
{
    if (fsm == NULL || in == NULL || out == NULL) {
        return;
    }

    uint8_t can_enable_cmd = 0u;
    uint8_t clean_disarm_pulse = 0u;

    /* Highest priority from ANY state: independent wired NC or S13 safety-radio stop (PD7). */
    if (in->estop_open != 0u) {
        enter_estop(fsm);
        populate_output(fsm, in, 0u, 0u, out);
        return;
    }

    const SafetyConfig *cfg = &fsm->config;
    const uint8_t radio_lost = (in->radio_age_ms > cfg->radio_timeout_ms) ? 1u : 0u;

    /* Check active-state faults first when in ARMED, BALANCING, CONTROLLED_STOP, REST. */
    if (is_active_state(fsm->state) != 0u) {
        const SafetyReason fault = check_active_faults(fsm, in);
        if (fault != SAFETY_REASON_NONE) {
            enter_fault(fsm, fault);
            populate_output(fsm, in, 0u, 0u, out);
            return;
        }
    }

    switch (fsm->state) {
    case SAFETY_STATE_DISARMED:
        if (in->boot_self_test_ok == 0u) {
            enter_fault(fsm, SAFETY_REASON_BOOT_SELF_TEST);
        } else if (in->bor3_fault != 0u) {
            enter_fault(fsm, SAFETY_REASON_BOR3);
        } else if (in->can_bus_off != 0u) {
            enter_fault(fsm, SAFETY_REASON_CAN_BUS_OFF);
        } else if (in->clamp_fault != 0u) {
            enter_fault(fsm, SAFETY_REASON_CLAMP_FAULT);
        } else if (!isfinite(in->v_bus_v) || in->v_bus_v >= cfg->link_discharged_max_v) {
            enter_fault(fsm, SAFETY_REASON_LINK_NOT_DISCHARGED);
        } else {
            /* Pre-checks green + local reset ack + separate local arm -> PRECHARGE. */
            const uint8_t prechecks_ok =
                (in->dump_continuity_ok != 0u
                 && in->k1_closed == 0u
                 && in->k2_closed == 0u
                 && in->kpre_closed == 0u
                 && in->imu_est_valid != 0u
                 && radio_lost == 0u
                 && in->radio_estop_sw == 0u
                 && isfinite(in->v_batt_ocv_v)
                 && in->v_batt_ocv_v >= cfg->arm_vbus_min_v) ? 1u : 0u;

            if (prechecks_ok != 0u
                && in->local_reset_req != 0u
                && in->local_arm_req != 0u) {
                reset_timers(fsm);
                fsm->state = SAFETY_STATE_PRECHARGE;
                fsm->reason = SAFETY_REASON_PRECHARGE_START;
            }
        }
        break;

    case SAFETY_STATE_PRECHARGE:
        fsm->precharge_elapsed_ms += in->dt_ms;
        if (in->mirror_mismatch != 0u) {
            enter_fault(fsm, SAFETY_REASON_MIRROR_MISMATCH);
        } else if (in->drive_enable_fail != 0u) {
            enter_fault(fsm, SAFETY_REASON_DRIVE_ENABLE_FAIL);
        } else if (in->can_bus_off != 0u) {
            enter_fault(fsm, SAFETY_REASON_CAN_BUS_OFF);
        } else if (in->clamp_fault != 0u) {
            enter_fault(fsm, SAFETY_REASON_CLAMP_FAULT);
        } else if (in->wdg_fault != 0u) {
            enter_fault(fsm, SAFETY_REASON_WDG_FAULT);
        } else if (in->s11_mode == S11_DRIVE_FAULT) {
            enter_fault(fsm, SAFETY_REASON_DRIVE_FAULT);
        } else if (in->s11_fault != 0u || s11_mode_is_packet_fault(in->s11_mode) != 0u) {
            enter_fault(fsm, SAFETY_REASON_S11_FAULT);
        } else if (in->hard_uv_fault != 0u) {
            enter_fault(fsm, SAFETY_REASON_HARD_UV);
        } else if (in->radio_estop_sw != 0u) {
            enter_fault(fsm, SAFETY_REASON_RADIO_ESTOP_SW);
        } else {
            const uint8_t contactors_ready =
                (in->k2_closed != 0u
                 && in->k1_closed != 0u
                 && in->kpre_closed == 0u
                 && in->precharge_dv_ok != 0u) ? 1u : 0u;

            if (contactors_ready != 0u) {
                /* Send CAN 0xFC enable ONLY after K1 mirror confirms closed. */
                can_enable_cmd = 1u;
                const uint8_t s11_armed_ready = (s11_mode_is_unarmed(in->s11_mode) == 0u) ? 1u : 0u;
                if (in->drives_healthy_6 != 0u && s11_armed_ready != 0u) {
                    fsm->state = SAFETY_STATE_ARMED;
                    fsm->reason = SAFETY_REASON_PRECHARGE_OK;
                    fsm->precharge_elapsed_ms = 0u;
                } else if (fsm->precharge_elapsed_ms > cfg->precharge_timeout_ms) {
                    enter_fault(fsm, SAFETY_REASON_PRECHARGE_TIMEOUT);
                }
            } else if (fsm->precharge_elapsed_ms > cfg->precharge_timeout_ms) {
                enter_fault(fsm, SAFETY_REASON_PRECHARGE_TIMEOUT);
            }
        }
        break;

    case SAFETY_STATE_ARMED:
        if (in->radio_arm_sw == 0u) {
            reset_timers(fsm);
            fsm->state = SAFETY_STATE_DISARMED;
            fsm->reason = SAFETY_REASON_CLEAN_DISARM;
            clean_disarm_pulse = 1u;
        } else if (in->stand_handoff_req != 0u
                   && s11_mode_is_unarmed(in->s11_mode) == 0u
                   && in->ground_left != 0u
                   && in->ground_right != 0u
                   && fabsf(in->pitch_deg) <= cfg->stand_pitch_max_deg
                   && radio_lost == 0u) {
            fsm->state = SAFETY_STATE_BALANCING;
            fsm->reason = SAFETY_REASON_STAND_HANDOFF;
            fsm->v_ref_m_s = 0.0f;
            fsm->yaw_ref_rad_s = 0.0f;
            fsm->host_lost_latched = 0u;
        }
        break;

    case SAFETY_STATE_BALANCING:
        if (radio_lost == 0u && in->radio_arm_sw == 0u) {
            /* Qualified arm switch LOW while actively balancing -> latched FAULT (immediate disable). */
            enter_fault(fsm, SAFETY_REASON_ARM_SW_DROPPED);
        } else {
            const uint8_t host_lost_now =
                (in->host_valid == 0u || in->host_age_ms > cfg->host_timeout_ms) ? 1u : 0u;
            const uint8_t radio_brake_now =
                (radio_lost != 0u
                 && cfg->radio_loss_policy == RADIO_LOSS_BRAKE_BALANCE) ? 1u : 0u;

            if (radio_brake_now != 0u) {
                enter_controlled_stop(fsm, SAFETY_REASON_RADIO_LOSS_BRAKE);
            } else if (host_lost_now != 0u) {
                enter_controlled_stop(fsm, SAFETY_REASON_HOST_LOSS);
            } else if (in->voluntary_stop_req != 0u) {
                enter_controlled_stop(fsm, SAFETY_REASON_VOLUNTARY_STOP);
            } else {
                const float auth = effective_s11_authority(in);
                fsm->v_ref_m_s = auth * in->v_target_m_s;
                fsm->yaw_ref_rad_s = auth * in->yaw_target_rad_s;
            }
        }
        break;

    case SAFETY_STATE_CONTROLLED_STOP:
        if (radio_lost == 0u && in->radio_arm_sw == 0u) {
            enter_fault(fsm, SAFETY_REASON_ARM_SW_DROPPED);
        } else {
            fsm->brake_elapsed_ms += in->dt_ms;
            if (fsm->brake_elapsed_ms < cfg->brake_duration_ms && cfg->brake_duration_ms > 0u) {
                const float frac = 1.0f - ((float)fsm->brake_elapsed_ms / (float)cfg->brake_duration_ms);
                fsm->v_ref_m_s = fsm->brake_start_v_m_s * frac;
                fsm->yaw_ref_rad_s = fsm->brake_start_yaw_rad_s * frac;
                fsm->rest_dwell_ms = 0u;
            } else {
                fsm->v_ref_m_s = 0.0f;
                fsm->yaw_ref_rad_s = 0.0f;
                const uint8_t resting =
                    (in->ground_left != 0u
                     && in->ground_right != 0u
                     && fabsf(in->pitch_deg) < cfg->rest_pitch_max_deg
                     && fabsf(in->v_est_m_s) < cfg->rest_vel_max_m_s
                     && fabsf(in->pitch_rate_deg_s) < cfg->rest_pitch_rate_max_deg_s) ? 1u : 0u;
                if (resting != 0u) {
                    fsm->rest_dwell_ms += in->dt_ms;
                    if (fsm->rest_dwell_ms >= cfg->rest_dwell_ms) {
                        fsm->state = SAFETY_STATE_REST;
                        fsm->reason = SAFETY_REASON_REST_CONFIRMED;
                    }
                } else {
                    fsm->rest_dwell_ms = 0u;
                }
            }
        }
        break;

    case SAFETY_STATE_REST:
        fsm->v_ref_m_s = 0.0f;
        fsm->yaw_ref_rad_s = 0.0f;
        if (radio_lost == 0u && in->radio_arm_sw == 0u) {
            /* Clean disarm when radio is fresh and arm switch is LOW at REST. */
            reset_timers(fsm);
            fsm->state = SAFETY_STATE_DISARMED;
            fsm->reason = SAFETY_REASON_CLEAN_DISARM;
            clean_disarm_pulse = 1u;
        } else if (in->local_regrant_req != 0u
                   && s11_mode_is_unarmed(in->s11_mode) == 0u
                   && radio_lost == 0u
                   && in->host_valid != 0u
                   && in->host_age_ms <= cfg->host_timeout_ms) {
            fsm->state = SAFETY_STATE_BALANCING;
            fsm->reason = SAFETY_REASON_LOCAL_REGRANT;
            fsm->host_lost_latched = 0u;
            fsm->brake_elapsed_ms = 0u;
            fsm->rest_dwell_ms = 0u;
        }
        break;

    case SAFETY_STATE_FAULT:
        if (in->local_reset_req != 0u
            && in->radio_arm_sw == 0u
            && in->k1_closed == 0u
            && in->k2_closed == 0u
            && isfinite(in->v_bus_v)
            && in->v_bus_v < cfg->link_discharged_max_v
            && in->boot_self_test_ok != 0u
            && in->bor3_fault == 0u
            && in->can_bus_off == 0u
            && in->clamp_fault == 0u
            && in->s11_fault == 0u
            && s11_mode_is_any_fault(in->s11_mode) == 0u
            && in->wdg_fault == 0u
            && in->hard_uv_fault == 0u
            && in->radio_estop_sw == 0u) {
            reset_timers(fsm);
            fsm->state = SAFETY_STATE_DISARMED;
            fsm->reason = SAFETY_REASON_MANUAL_RESET;
        }
        break;

    case SAFETY_STATE_ESTOP:
        if (in->hw_reset_ack != 0u
            && in->local_reset_req != 0u
            && in->radio_arm_sw == 0u
            && in->k1_closed == 0u
            && in->k2_closed == 0u
            && isfinite(in->v_bus_v)
            && in->v_bus_v < cfg->link_discharged_max_v
            && in->boot_self_test_ok != 0u
            && in->bor3_fault == 0u
            && in->can_bus_off == 0u
            && in->clamp_fault == 0u
            && in->radio_estop_sw == 0u) {
            reset_timers(fsm);
            fsm->state = SAFETY_STATE_DISARMED;
            fsm->reason = SAFETY_REASON_MANUAL_RESET;
        }
        break;
    }

    populate_output(fsm, in, can_enable_cmd, clean_disarm_pulse, out);
}

void safety_fsm_step_with_s11(SafetyFsm *fsm, const SafetyInputs *in,
                              const S11PacketFrame *s11_frame, SafetyOutput *out)
{
    if (fsm == NULL || in == NULL || out == NULL) {
        return;
    }
    SafetyInputs bound = *in;
    safety_fsm_bind_s11_frame(&bound, s11_frame);
    safety_fsm_step(fsm, &bound, out);
}
