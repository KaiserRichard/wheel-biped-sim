/*
 * 3-Bus x 2-Motor CAN TX Scheduler, RX Dispatcher, Bus-Off & Timeout Monitor
 */

#include "can_sched.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* Section 4.2 topology:
 *   Bus 0 (FDCAN1): axis 0 (L-front, 0x001/0x011), axis 3 (R-front, 0x004/0x014)
 *   Bus 1 (FDCAN2): axis 1 (L-rear,  0x002/0x012), axis 4 (R-rear,  0x005/0x015)
 *   Bus 2 (FDCAN3): axis 2 (L-wheel, 0x003/0x013), axis 5 (R-wheel, 0x006/0x016) */
static const uint8_t s_bus_axes[CAN_SCHED_NUM_BUSES][CAN_SCHED_MOTORS_PER_BUS] = {
    { 0u, 3u },
    { 1u, 4u },
    { 2u, 5u }
};

static uint8_t clamp_u8_u16(uint16_t v)
{
    return (v > 255u) ? 255u : (uint8_t)v;
}

static void abort_bus_tx(CanBusState *b)
{
    if (b->tx_count > 0u) {
        b->abort_count = (uint16_t)(b->abort_count + (uint16_t)b->tx_count);
    }
    memset(b->tx_queue, 0, sizeof(b->tx_queue));
    b->tx_count = 0u;
}

static CanSchedStatus enqueue_on_bus(CanBusState *b, const CanFrame *frame,
                                     uint8_t axis, uint8_t is_emergency,
                                     uint8_t replace_periodic, uint32_t now_us)
{
    if (b->bus_off_latched != 0u || b->bus_off_hw != 0u) {
        b->tx_drop_count++;
        return CAN_SCHED_ERR_BUS_OFF;
    }

    /* AutoRetransmission=DISABLE periodic behavior: overwrite any stale unsent
     * periodic command for the same axis instead of queuing stale torque. */
    if (replace_periodic != 0u && is_emergency == 0u) {
        for (uint8_t i = 0u; i < CAN_SCHED_TX_QUEUE_CAP; ++i) {
            if (b->tx_queue[i].used != 0u
                && b->tx_queue[i].is_emergency == 0u
                && b->tx_queue[i].axis == axis) {
                b->tx_queue[i].frame = *frame;
                b->tx_queue[i].enqueue_us = now_us;
                return CAN_SCHED_OK;
            }
        }
    }

    for (uint8_t i = 0u; i < CAN_SCHED_TX_QUEUE_CAP; ++i) {
        if (b->tx_queue[i].used == 0u) {
            b->tx_queue[i].frame = *frame;
            b->tx_queue[i].enqueue_us = now_us;
            b->tx_queue[i].axis = axis;
            b->tx_queue[i].is_emergency = is_emergency;
            b->tx_queue[i].used = 1u;
            b->tx_count++;
            return CAN_SCHED_OK;
        }
    }

    b->tx_drop_count++;
    return CAN_SCHED_ERR_QUEUE_FULL;
}

static void enqueue_disable_burst(CanSched *sched, uint32_t now_us)
{
    uint8_t payload[DAMIAO_CAN_DLC];
    damiao_pack_disable(payload);

    for (uint8_t bus_idx = 0u; bus_idx < CAN_SCHED_NUM_BUSES; ++bus_idx) {
        CanBusState *b = &sched->bus[bus_idx];
        if (b->bus_off_latched != 0u || b->bus_off_hw != 0u) {
            continue;
        }
        for (uint8_t slot = 0u; slot < CAN_SCHED_MOTORS_PER_BUS; ++slot) {
            const uint8_t axis = s_bus_axes[bus_idx][slot];
            DamiaoAxisConfig cfg;
            if (damiao_get_axis_config(axis, &cfg) != DAMIAO_OK) {
                continue;
            }
            CanFrame f;
            memset(&f, 0, sizeof(f));
            f.std_id = cfg.cmd_can_id;
            f.dlc = DAMIAO_CAN_DLC;
            memcpy(f.data, payload, DAMIAO_CAN_DLC);
            (void)enqueue_on_bus(b, &f, axis, 1u, 0u, now_us);
        }
    }
}

CanSchedConfig can_sched_default_config(void)
{
    CanSchedConfig c;
    c.expected_enabled_status = DAMIAO_DRIVE_STATUS_ENABLED_DEFAULT;
    c.supervisor_timeout_us   = CAN_SCHED_SUPERVISOR_TIMEOUT_US;
    c.drive_timeout_us        = CAN_SCHED_DRIVE_TIMEOUT_US;
    c.disable_repeat_us       = CAN_SCHED_DISABLE_REPEAT_US;
    return c;
}

void can_sched_init(CanSched *sched, const CanSchedConfig *cfg)
{
    if (sched == NULL) {
        return;
    }
    memset(sched, 0, sizeof(*sched));
    sched->cfg = (cfg != NULL) ? *cfg : can_sched_default_config();
    s11_mailbox_init(&sched->s11_mb);
}

CanSchedStatus can_sched_bus_for_axis(uint8_t axis, uint8_t *out_bus_idx)
{
    if (out_bus_idx == NULL) {
        return CAN_SCHED_ERR_NULL_PTR;
    }
    DamiaoAxisConfig cfg;
    if (damiao_get_axis_config(axis, &cfg) != DAMIAO_OK
        || cfg.can_bus == 0u || cfg.can_bus > CAN_SCHED_NUM_BUSES) {
        return CAN_SCHED_ERR_INVALID_AXIS;
    }
    *out_bus_idx = (uint8_t)(cfg.can_bus - 1u);
    return CAN_SCHED_OK;
}

CanSchedStatus can_sched_axes_for_bus(uint8_t bus_idx,
                                      uint8_t out_axes[CAN_SCHED_MOTORS_PER_BUS])
{
    if (out_axes == NULL) {
        return CAN_SCHED_ERR_NULL_PTR;
    }
    if (bus_idx >= CAN_SCHED_NUM_BUSES) {
        return CAN_SCHED_ERR_INVALID_BUS;
    }
    out_axes[0] = s_bus_axes[bus_idx][0];
    out_axes[1] = s_bus_axes[bus_idx][1];
    return CAN_SCHED_OK;
}

CanSchedStatus can_sched_enqueue_mit_cmds(CanSched *sched,
                                          const DamiaoMitCmd cmds[CAN_SCHED_NUM_AXES],
                                          uint32_t now_us)
{
    if (sched == NULL || cmds == NULL) {
        return CAN_SCHED_ERR_NULL_PTR;
    }
    if (sched->any_bus_off_latched != 0u) {
        return CAN_SCHED_ERR_BUS_OFF;
    }

    sched->disable_repeat_armed = 0u;

    /* Enqueue 2 frames per bus across FDCAN1, FDCAN2, FDCAN3 back-to-back. */
    CanSchedStatus result = CAN_SCHED_OK;
    for (uint8_t bus_idx = 0u; bus_idx < CAN_SCHED_NUM_BUSES; ++bus_idx) {
        for (uint8_t slot = 0u; slot < CAN_SCHED_MOTORS_PER_BUS; ++slot) {
            const uint8_t axis = s_bus_axes[bus_idx][slot];
            DamiaoAxisConfig cfg;
            if (damiao_get_axis_config(axis, &cfg) != DAMIAO_OK) {
                return CAN_SCHED_ERR_INVALID_AXIS;
            }
            CanFrame f;
            memset(&f, 0, sizeof(f));
            f.std_id = cfg.cmd_can_id;
            f.dlc = DAMIAO_CAN_DLC;
            if (damiao_pack_mit_cmd_clamped(&cfg.limits, &cmds[axis], f.data) != DAMIAO_OK) {
                return CAN_SCHED_ERR_CODEC;
            }
            const CanSchedStatus st = enqueue_on_bus(&sched->bus[bus_idx], &f, axis, 0u, 1u, now_us);
            if (st != CAN_SCHED_OK && result == CAN_SCHED_OK) {
                result = st;
            }
        }
    }
    return result;
}

CanSchedStatus can_sched_enqueue_torques(CanSched *sched,
                                         const float torque_nm[CAN_SCHED_NUM_AXES],
                                         uint32_t now_us)
{
    if (sched == NULL || torque_nm == NULL) {
        return CAN_SCHED_ERR_NULL_PTR;
    }
    DamiaoMitCmd cmds[CAN_SCHED_NUM_AXES];
    memset(cmds, 0, sizeof(cmds));
    for (uint8_t i = 0u; i < CAN_SCHED_NUM_AXES; ++i) {
        cmds[i].torque = torque_nm[i];
    }
    return can_sched_enqueue_mit_cmds(sched, cmds, now_us);
}

CanSchedStatus can_sched_enqueue_enable_all(CanSched *sched, uint32_t now_us)
{
    if (sched == NULL) {
        return CAN_SCHED_ERR_NULL_PTR;
    }
    if (sched->any_bus_off_latched != 0u) {
        return CAN_SCHED_ERR_BUS_OFF;
    }

    sched->disable_repeat_armed = 0u;

    uint8_t payload[DAMIAO_CAN_DLC];
    damiao_pack_enable(payload);

    CanSchedStatus result = CAN_SCHED_OK;
    for (uint8_t bus_idx = 0u; bus_idx < CAN_SCHED_NUM_BUSES; ++bus_idx) {
        for (uint8_t slot = 0u; slot < CAN_SCHED_MOTORS_PER_BUS; ++slot) {
            const uint8_t axis = s_bus_axes[bus_idx][slot];
            DamiaoAxisConfig cfg;
            if (damiao_get_axis_config(axis, &cfg) != DAMIAO_OK) {
                return CAN_SCHED_ERR_INVALID_AXIS;
            }
            CanFrame f;
            memset(&f, 0, sizeof(f));
            f.std_id = cfg.cmd_can_id;
            f.dlc = DAMIAO_CAN_DLC;
            memcpy(f.data, payload, DAMIAO_CAN_DLC);
            const CanSchedStatus st = enqueue_on_bus(&sched->bus[bus_idx], &f, axis, 0u, 1u, now_us);
            if (st != CAN_SCHED_OK && result == CAN_SCHED_OK) {
                result = st;
            }
        }
    }
    return result;
}

CanSchedStatus can_sched_emergency_disable(CanSched *sched, uint32_t now_us)
{
    if (sched == NULL) {
        return CAN_SCHED_ERR_NULL_PTR;
    }

    /* 1. Abort all pending ordinary TX buffers on all 3 buses. */
    for (uint8_t bus_idx = 0u; bus_idx < CAN_SCHED_NUM_BUSES; ++bus_idx) {
        abort_bus_tx(&sched->bus[bus_idx]);
    }

    /* 2. Enqueue immediate 0xFF..FD disable frames on all healthy buses. */
    enqueue_disable_burst(sched, now_us);

    /* 3. Arm +1 ms repeat burst per FIRMWARE_DESIGN_v2.md Section 4.3. */
    sched->disable_repeat_armed = 1u;
    sched->disable_first_us = now_us;
    return CAN_SCHED_OK;
}

void can_sched_service_tx(CanSched *sched, uint32_t now_us)
{
    if (sched == NULL || sched->disable_repeat_armed == 0u) {
        return;
    }
    const uint32_t elapsed_us = (uint32_t)(now_us - sched->disable_first_us);
    if (elapsed_us >= sched->cfg.disable_repeat_us) {
        sched->disable_repeat_armed = 0u;
        enqueue_disable_burst(sched, now_us);
    }
}

CanSchedStatus can_sched_pop_tx(CanSched *sched, uint8_t bus_idx, CanFrame *out_frame)
{
    if (sched == NULL || out_frame == NULL) {
        return CAN_SCHED_ERR_NULL_PTR;
    }
    if (bus_idx >= CAN_SCHED_NUM_BUSES) {
        return CAN_SCHED_ERR_INVALID_BUS;
    }

    CanBusState *b = &sched->bus[bus_idx];
    if (b->tx_count == 0u) {
        return CAN_SCHED_ERR_QUEUE_EMPTY;
    }

    /* Priority TX Queue (FDCAN_TX_QUEUE_OPERATION): lowest standard CAN ID wins
     * arbitration; ties break by emergency flag then FIFO order. */
    uint8_t best = CAN_SCHED_TX_QUEUE_CAP;
    for (uint8_t i = 0u; i < CAN_SCHED_TX_QUEUE_CAP; ++i) {
        if (b->tx_queue[i].used == 0u) {
            continue;
        }
        if (best == CAN_SCHED_TX_QUEUE_CAP) {
            best = i;
            continue;
        }
        const CanTxSlot *cur = &b->tx_queue[i];
        const CanTxSlot *b_slot = &b->tx_queue[best];
        if (cur->frame.std_id < b_slot->frame.std_id) {
            best = i;
        } else if (cur->frame.std_id == b_slot->frame.std_id) {
            if (cur->is_emergency > b_slot->is_emergency) {
                best = i;
            } else if (cur->is_emergency == b_slot->is_emergency
                       && (int32_t)(cur->enqueue_us - b_slot->enqueue_us) < 0) {
                best = i;
            }
        }
    }

    if (best >= CAN_SCHED_TX_QUEUE_CAP) {
        b->tx_count = 0u;
        return CAN_SCHED_ERR_QUEUE_EMPTY;
    }

    *out_frame = b->tx_queue[best].frame;
    b->tx_queue[best].used = 0u;
    b->tx_count--;
    return CAN_SCHED_OK;
}

uint8_t can_sched_tx_pending(const CanSched *sched, uint8_t bus_idx)
{
    if (sched == NULL || bus_idx >= CAN_SCHED_NUM_BUSES) {
        return 0u;
    }
    return sched->bus[bus_idx].tx_count;
}

CanSchedStatus can_sched_rx_frame(CanSched *sched, uint8_t bus_idx,
                                  const CanFrame *frame, uint32_t rx_us)
{
    if (sched == NULL || frame == NULL) {
        return CAN_SCHED_ERR_NULL_PTR;
    }
    if (bus_idx >= CAN_SCHED_NUM_BUSES) {
        return CAN_SCHED_ERR_INVALID_BUS;
    }

    CanBusState *b = &sched->bus[bus_idx];

    /* Hardware global filter rejects extended and remote frames (Section 4.3). */
    if (frame->is_extended != 0u || frame->is_rtr != 0u) {
        b->rx_rejected_count++;
        return CAN_SCHED_ERR_REJECTED_FRAME;
    }

    /* Strict per-bus RX filter: only accept the 2 expected reply IDs for this bus. */
    uint8_t matched_axis = CAN_SCHED_NUM_AXES;
    DamiaoAxisConfig matched_cfg;
    memset(&matched_cfg, 0, sizeof(matched_cfg));
    for (uint8_t slot = 0u; slot < CAN_SCHED_MOTORS_PER_BUS; ++slot) {
        const uint8_t axis = s_bus_axes[bus_idx][slot];
        DamiaoAxisConfig cfg;
        if (damiao_get_axis_config(axis, &cfg) == DAMIAO_OK
            && frame->std_id == cfg.reply_can_id) {
            matched_axis = axis;
            matched_cfg = cfg;
            break;
        }
    }

    if (matched_axis >= CAN_SCHED_NUM_AXES) {
        b->rx_rejected_count++;
        return CAN_SCHED_ERR_REJECTED_FRAME;
    }

    CanAxisRxSlot *rx = &sched->axis_rx[matched_axis];
    DamiaoMitFeedback fb;
    if (damiao_unpack_feedback(&matched_cfg.limits, frame->data, frame->dlc, &fb) != DAMIAO_OK) {
        b->rx_drop_count++;
        rx->rx_us = rx_us;
        rx->seq++;
        rx->seen = 1u;
        rx->valid = 0u;
        s11_mailbox_write_axis(&sched->s11_mb, (uint32_t)matched_axis,
                               0u, rx->healthy, rx_us, NAN, NAN);
        return CAN_SCHED_ERR_CODEC;
    }

    rx->fb = fb;
    rx->rx_us = rx_us;
    rx->seq++;
    rx->seen = 1u;
    rx->valid = (isfinite(fb.pos) && isfinite(fb.vel) && isfinite(fb.torque)) ? 1u : 0u;
    rx->healthy = damiao_status_is_healthy(fb.status_nibble, sched->cfg.expected_enabled_status);

    /* Publish decoded feedback into ownership-safe triple-buffered S11 ISR mailbox. */
    s11_mailbox_write_axis(&sched->s11_mb, (uint32_t)matched_axis,
                           rx->valid, rx->healthy,
                           rx_us, (double)fb.pos, (double)fb.vel);
    return CAN_SCHED_OK;
}

CanSchedStatus can_sched_snapshot_rx(CanSched *sched, uint32_t now_us,
                                     CanSchedRxSnapshot *out_snap)
{
    if (sched == NULL || out_snap == NULL) {
        return CAN_SCHED_ERR_NULL_PTR;
    }
    memset(out_snap, 0, sizeof(*out_snap));

    /* Tear-free snapshot via verified S11 triple-buffered mailbox & s11_age_ms_u32 (1000 ticks/ms). */
    (void)s11_mailbox_snapshot(&sched->s11_mb, now_us, 1000.0, &out_snap->s11_snap);
    out_snap->fresh_mask          = (uint32_t)out_snap->s11_snap.fresh;
    out_snap->valid_mask          = (uint32_t)out_snap->s11_snap.valid;
    out_snap->healthy_mask        = (uint32_t)out_snap->s11_snap.healthy;
    out_snap->status_unknown_mask = (uint32_t)out_snap->s11_snap.status_unknown;

    const double supervisor_timeout_ms = (double)sched->cfg.supervisor_timeout_us * 0.001;
    const double drive_timeout_ms      = (double)sched->cfg.drive_timeout_us * 0.001;

    uint8_t healthy_count = 0u;
    for (uint8_t i = 0u; i < CAN_SCHED_NUM_AXES; ++i) {
        const uint32_t bit = 1u << i;
        const CanAxisRxSlot *rx = &sched->axis_rx[i];

        out_snap->age_ms[i] = out_snap->s11_snap.age_ms[i];
        out_snap->angle[i]  = out_snap->s11_snap.angle[i];
        out_snap->rate[i]   = out_snap->s11_snap.rate[i];
        out_snap->seq[i]    = out_snap->s11_snap.seq[i];

        if ((out_snap->status_unknown_mask & bit) != 0u || rx->seen == 0u) {
            out_snap->torque_fb[i] = 0.0f;
            out_snap->supervisor_timeout_mask |= bit;
            out_snap->drive_timeout_mask |= bit;
            continue;
        }

        out_snap->torque_fb[i]   = rx->fb.torque;
        out_snap->motor_state[i] = rx->fb.status_nibble;
        out_snap->t_mos_c[i]     = (uint8_t)rx->fb.t_mos_c;
        out_snap->t_coil_c[i]    = (uint8_t)rx->fb.t_coil_c;

        if (!isfinite(out_snap->age_ms[i]) || out_snap->age_ms[i] > supervisor_timeout_ms) {
            out_snap->supervisor_timeout_mask |= bit;
        }
        if (!isfinite(out_snap->age_ms[i]) || out_snap->age_ms[i] >= drive_timeout_ms) {
            out_snap->drive_timeout_mask |= bit;
        }

        sched->last_consumed_seq[i] = out_snap->seq[i];
        sched->last_consumed_mask |= (uint8_t)(1u << i);

        if ((out_snap->valid_mask & bit) != 0u
            && (out_snap->healthy_mask & bit) != 0u
            && (out_snap->supervisor_timeout_mask & bit) == 0u) {
            healthy_count++;
        }
    }

    out_snap->drives_healthy_6 = (healthy_count == CAN_SCHED_NUM_AXES
                                  && sched->any_bus_off_latched == 0u) ? 1u : 0u;
    return CAN_SCHED_OK;
}

CanSchedStatus can_sched_step_s11_500hz(CanSched *sched, S11PacketPolicy *s11,
                                        uint32_t now_us, double dt_ms,
                                        CanSchedRxSnapshot *out_snap,
                                        S11PacketFrame *out_frame)
{
    if (sched == NULL || s11 == NULL || out_snap == NULL || out_frame == NULL) {
        return CAN_SCHED_ERR_NULL_PTR;
    }
    const CanSchedStatus st = can_sched_snapshot_rx(sched, now_us, out_snap);
    if (st != CAN_SCHED_OK) {
        return st;
    }
    s11_packets_snapshot(s11, &out_snap->s11_snap, dt_ms, out_frame);
    return CAN_SCHED_OK;
}

CanSchedStatus can_sched_update_bus_status(CanSched *sched, uint8_t bus_idx,
                                           uint16_t tec, uint16_t rec,
                                           uint8_t hw_bus_off,
                                           uint8_t hw_err_passive,
                                           uint8_t hw_err_warning,
                                           uint32_t now_us)
{
    if (sched == NULL) {
        return CAN_SCHED_ERR_NULL_PTR;
    }
    if (bus_idx >= CAN_SCHED_NUM_BUSES) {
        return CAN_SCHED_ERR_INVALID_BUS;
    }

    CanBusState *b = &sched->bus[bus_idx];
    b->tec = tec;
    b->rec = rec;
    b->error_warning = (hw_err_warning != 0u || tec >= 96u || rec >= 96u) ? 1u : 0u;
    b->error_passive = (hw_err_passive != 0u
                        || tec >= CAN_SCHED_ERR_PASSIVE_THRESH
                        || rec >= CAN_SCHED_ERR_PASSIVE_THRESH) ? 1u : 0u;

    const uint8_t is_bus_off = (hw_bus_off != 0u || tec >= CAN_SCHED_BUS_OFF_THRESH) ? 1u : 0u;
    b->bus_off_hw = is_bus_off;

    if (is_bus_off != 0u) {
        const uint8_t newly_latched = (b->bus_off_latched == 0u) ? 1u : 0u;
        b->bus_off_latched = 1u;
        b->recovery_requested = 0u;
        sched->any_bus_off_latched = 1u;
        if (newly_latched != 0u) {
            b->bus_off_count++;
            /* Abort TX on all buses and disable all remaining healthy buses (Section 4.3). */
            (void)can_sched_emergency_disable(sched, now_us);
        }
    }
    return CAN_SCHED_OK;
}

CanSchedStatus can_sched_request_bus_recovery(CanSched *sched, uint8_t is_disarmed)
{
    if (sched == NULL) {
        return CAN_SCHED_ERR_NULL_PTR;
    }
    /* Never auto-recover bus-off while armed (FIRMWARE_DESIGN_v2.md Section 4.3). */
    if (is_disarmed == 0u) {
        return CAN_SCHED_ERR_NOT_DISARMED;
    }
    for (uint8_t bus_idx = 0u; bus_idx < CAN_SCHED_NUM_BUSES; ++bus_idx) {
        abort_bus_tx(&sched->bus[bus_idx]);
        if (sched->bus[bus_idx].bus_off_latched != 0u) {
            sched->bus[bus_idx].recovery_requested = 1u;
        }
    }
    sched->disable_repeat_armed = 0u;
    return CAN_SCHED_OK;
}

CanSchedStatus can_sched_confirm_bus_recovery(CanSched *sched, uint8_t bus_idx,
                                              uint8_t hw_bus_off,
                                              uint8_t all_6_motors_answered)
{
    if (sched == NULL) {
        return CAN_SCHED_ERR_NULL_PTR;
    }
    if (bus_idx >= CAN_SCHED_NUM_BUSES) {
        return CAN_SCHED_ERR_INVALID_BUS;
    }

    CanBusState *b = &sched->bus[bus_idx];
    b->bus_off_hw = (hw_bus_off != 0u) ? 1u : 0u;
    if (b->bus_off_latched != 0u) {
        if (b->recovery_requested == 0u || hw_bus_off != 0u || all_6_motors_answered == 0u) {
            return CAN_SCHED_ERR_RECOVERY_INCOMPLETE;
        }
        b->bus_off_latched = 0u;
        b->recovery_requested = 0u;
        b->error_passive = 0u;
        b->error_warning = 0u;
        b->tec = 0u;
        b->rec = 0u;
    }

    uint8_t any_latched = 0u;
    for (uint8_t i = 0u; i < CAN_SCHED_NUM_BUSES; ++i) {
        if (sched->bus[i].bus_off_latched != 0u) {
            any_latched = 1u;
        }
    }
    sched->any_bus_off_latched = any_latched;
    return CAN_SCHED_OK;
}

void can_sched_get_health(const CanSched *sched, CanSchedHealth *out_health)
{
    if (sched == NULL || out_health == NULL) {
        return;
    }
    memset(out_health, 0, sizeof(*out_health));
    out_health->any_bus_off = sched->any_bus_off_latched;

    for (uint8_t i = 0u; i < CAN_SCHED_NUM_BUSES; ++i) {
        const CanBusState *b = &sched->bus[i];
        const uint8_t bit = (uint8_t)(1u << i);
        if (b->bus_off_latched != 0u || b->bus_off_hw != 0u) {
            out_health->any_bus_off = 1u;
            out_health->bus_off_mask |= bit;
        }
        if (b->error_passive != 0u) {
            out_health->any_error_passive = 1u;
            out_health->error_passive_mask |= bit;
            out_health->inhibit_jump = 1u;
        }
        if (b->error_warning != 0u) {
            out_health->any_error_warning = 1u;
        }
        out_health->tec[i] = clamp_u8_u16(b->tec);
        out_health->rec[i] = clamp_u8_u16(b->rec);
        out_health->bus_off_cnt[i] = b->bus_off_count;
        out_health->rx_drop_cnt[i] = b->rx_drop_count;
    }
}
