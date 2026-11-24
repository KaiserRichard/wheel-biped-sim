#ifdef NDEBUG
#undef NDEBUG
#endif
#include "can_sched.h"

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

static int near_f(float a, float b, float tol)
{
    return (fabsf(a - b) <= tol) ? 1 : 0;
}

static CanFrame make_fb_frame(uint8_t axis, uint8_t status_nibble,
                              float pos, float vel, float torque)
{
    DamiaoAxisConfig cfg;
    assert(damiao_get_axis_config(axis, &cfg) == DAMIAO_OK);

    DamiaoMitFeedback fb;
    memset(&fb, 0, sizeof(fb));
    fb.motor_id = (uint8_t)(axis + 1u);
    fb.status_nibble = status_nibble;
    fb.pos = pos;
    fb.vel = vel;
    fb.torque = torque;
    fb.t_mos_c = 41.0f;
    fb.t_coil_c = 52.0f;

    CanFrame f;
    memset(&f, 0, sizeof(f));
    f.std_id = cfg.reply_can_id;
    f.dlc = DAMIAO_CAN_DLC;
    assert(damiao_pack_feedback(&cfg.limits, &fb, f.data) == DAMIAO_OK);
    return f;
}

static void test_topology_and_tx_priority(void)
{
    CanSched sched;
    can_sched_init(&sched, NULL);

    /* FROM FILE: rev4 J5 right / J6 left / J7 spare. */
    const uint8_t exp_bus[6] = { 1u, 1u, 1u, 0u, 0u, 0u };
    for (uint8_t a = 0u; a < 6u; ++a) {
        uint8_t bus = 99u;
        assert(can_sched_bus_for_axis(a, &bus) == CAN_SCHED_OK);
        assert(bus == exp_bus[a]);
    }
    uint8_t axes[CAN_SCHED_MOTORS_PER_BUS];
    assert(can_sched_axes_for_bus(0u, axes) == CAN_SCHED_OK && axes[0] == 3u && axes[1] == 4u && axes[2] == 5u);
    assert(can_sched_axes_for_bus(1u, axes) == CAN_SCHED_OK && axes[0] == 0u && axes[1] == 1u && axes[2] == 2u);
    assert(can_sched_axes_for_bus(2u, axes) == CAN_SCHED_OK && axes[0] == CAN_SCHED_UNUSED_AXIS);
    assert(can_sched_axes_for_bus(3u, axes) == CAN_SCHED_ERR_INVALID_BUS);

    /* Enqueue 6 torques (including exceeding firmware clamps +/-7 Nm hip, +/-2 Nm wheel) */
    const float torq[6] = { 9.5f, -3.2f, 5.0f, -8.5f, 1.5f, -4.5f };
    assert(can_sched_enqueue_torques(&sched, torq, 1000u) == CAN_SCHED_OK);

    /* Three frames on each active bus; no traffic on the spare. */
    const uint16_t exp_ids[3][3] = {
        { 0x004u, 0x005u, 0x006u },
        { 0x001u, 0x002u, 0x003u },
        { 0u, 0u, 0u }
    };
    const float exp_clamped[3][3] = {
        { -7.0f, 1.5f, -2.0f },
        { 7.0f, -3.2f, 2.0f },
        { 0.0f, 0.0f, 0.0f }
    };
    for (uint8_t b = 0u; b < CAN_SCHED_NUM_BUSES; ++b) {
        assert(can_sched_tx_pending(&sched, b) == can_sched_axis_count_for_bus(b));
        assert(can_sched_axes_for_bus(b, axes) == CAN_SCHED_OK);
        for (uint8_t slot = 0u; slot < can_sched_axis_count_for_bus(b); ++slot) {
            CanFrame out;
            assert(can_sched_pop_tx(&sched, b, &out) == CAN_SCHED_OK);
            assert(out.std_id == exp_ids[b][slot]);
            assert(out.dlc == DAMIAO_CAN_DLC);

            DamiaoAxisConfig acfg;
            DamiaoMitCmd decoded;
            assert(damiao_get_axis_config(axes[slot], &acfg) == DAMIAO_OK);
            assert(damiao_unpack_mit_cmd(&acfg.limits, out.data, &decoded) == DAMIAO_OK);
            assert(near_f(decoded.torque, exp_clamped[b][slot], 0.015f));
        }
        assert(can_sched_tx_pending(&sched, b) == 0u);
    }

    /* Periodic replacement (AutoRetransmission=DISABLE): enqueuing twice without pop
     * updates the existing three frames per active bus rather than growing stale queue backlog. */
    const float t1[6] = { 1.0f, 1.0f, 0.5f, 1.0f, 1.0f, 0.5f };
    const float t2[6] = { -2.0f, -2.0f, -1.0f, -2.0f, -2.0f, -1.0f };
    assert(can_sched_enqueue_torques(&sched, t1, 2000u) == CAN_SCHED_OK);
    assert(can_sched_enqueue_torques(&sched, t2, 4000u) == CAN_SCHED_OK);
    for (uint8_t b = 0u; b < CAN_SCHED_NUM_BUSES; ++b) {
        assert(can_sched_tx_pending(&sched, b) == can_sched_axis_count_for_bus(b));
        if (can_sched_axis_count_for_bus(b) == 0u) { continue; }
        CanFrame out;
        assert(can_sched_pop_tx(&sched, b, &out) == CAN_SCHED_OK);
        DamiaoAxisConfig acfg;
        DamiaoMitCmd decoded;
        assert(can_sched_axes_for_bus(b, axes) == CAN_SCHED_OK);
        assert(damiao_get_axis_config(axes[0], &acfg) == DAMIAO_OK);
        assert(damiao_unpack_mit_cmd(&acfg.limits, out.data, &decoded) == DAMIAO_OK);
        assert(near_f(decoded.torque, -2.0f, 0.015f));
    }
}

static void test_rx_filter_dispatch_and_timeouts(void)
{
    CanSched sched;
    can_sched_init(&sched, NULL);

    /* Wrong bus reply ID rejected (0x011 belongs on bus 1, not bus 0) */
    CanFrame f0 = make_fb_frame(0u, 0x1u, 0.5f, 1.2f, 0.3f);
    assert(can_sched_rx_frame(&sched, 0u, &f0, 1000u) == CAN_SCHED_ERR_REJECTED_FRAME);
    assert(sched.bus[0].rx_rejected_count == 1u);

    /* Extended and RTR frames rejected */
    CanFrame ext = f0;
    ext.is_extended = 1u;
    assert(can_sched_rx_frame(&sched, 1u, &ext, 1000u) == CAN_SCHED_ERR_REJECTED_FRAME);
    CanFrame rtr = f0;
    rtr.is_rtr = 1u;
    assert(can_sched_rx_frame(&sched, 1u, &rtr, 1000u) == CAN_SCHED_ERR_REJECTED_FRAME);

    /* Bad DLC rejected and marks axis invalid */
    CanFrame bad_dlc = f0;
    bad_dlc.dlc = 7u;
    assert(can_sched_rx_frame(&sched, 1u, &bad_dlc, 1000u) == CAN_SCHED_ERR_CODEC);
    assert(sched.bus[1].rx_drop_count == 1u);

    /* Feed all 6 valid healthy replies near UINT32_MAX to verify wrap-safe age */
    const uint32_t t_rx = 0xFFFFFF00u;
    for (uint8_t a = 0u; a < 6u; ++a) {
        uint8_t bus = 0u;
        assert(can_sched_bus_for_axis(a, &bus) == CAN_SCHED_OK);
        CanFrame f = make_fb_frame(a, 0x1u, 0.1f * (float)a, -0.2f * (float)a, 0.05f);
        assert(can_sched_rx_frame(&sched, bus, &f, t_rx) == CAN_SCHED_OK);
    }

    CanSchedRxSnapshot snap;
    /* 1.5 ms later across wraparound (0xFFFFFF00 + 1500 = 0x000004DC) */
    assert(can_sched_snapshot_rx(&sched, (uint32_t)(t_rx + 1500u), &snap) == CAN_SCHED_OK);
    assert(snap.fresh_mask == 0x3Fu);
    assert(snap.valid_mask == 0x3Fu);
    assert(snap.healthy_mask == 0x3Fu);
    assert(snap.supervisor_timeout_mask == 0u);
    assert(snap.drive_timeout_mask == 0u);
    assert(snap.drives_healthy_6 == 1u);
    for (uint8_t a = 0u; a < 6u; ++a) {
        assert(fabs(snap.age_ms[a] - 1.5) < 1e-6);
        assert(snap.t_mos_c[a] == 41u && snap.t_coil_c[a] == 52u);
    }

    /* Second snapshot without new RX -> fresh_mask is 0 */
    assert(can_sched_snapshot_rx(&sched, (uint32_t)(t_rx + 2500u), &snap) == CAN_SCHED_OK);
    assert(snap.fresh_mask == 0u);
    assert(snap.drives_healthy_6 == 1u);

    /* 12 ms later -> supervisor timeout (>10 ms), drive timeout not yet (>=20 ms) */
    assert(can_sched_snapshot_rx(&sched, (uint32_t)(t_rx + 12000u), &snap) == CAN_SCHED_OK);
    assert(snap.supervisor_timeout_mask == 0x3Fu);
    assert(snap.drive_timeout_mask == 0u);
    assert(snap.drives_healthy_6 == 0u);

    /* 21 ms later -> both supervisor and drive timeouts active */
    assert(can_sched_snapshot_rx(&sched, (uint32_t)(t_rx + 21000u), &snap) == CAN_SCHED_OK);
    assert(snap.supervisor_timeout_mask == 0x3Fu);
    assert(snap.drive_timeout_mask == 0x3Fu);

    /* 0xD comm-loss status nibble clears healthy bit for axis 2 */
    CanFrame comm_loss = make_fb_frame(2u, DAMIAO_DRIVE_STATUS_COMM_LOSS, 0.0f, 0.0f, 0.0f);
    assert(can_sched_rx_frame(&sched, 1u, &comm_loss, (uint32_t)(t_rx + 22000u)) == CAN_SCHED_OK);
    assert(can_sched_snapshot_rx(&sched, (uint32_t)(t_rx + 22500u), &snap) == CAN_SCHED_OK);
    assert((snap.healthy_mask & (1u << 2u)) == 0u);
    assert(snap.motor_state[2] == DAMIAO_DRIVE_STATUS_COMM_LOSS);
    assert(snap.drives_healthy_6 == 0u);
}

static void test_emergency_disable_and_bus_off_recovery(void)
{
    CanSched sched;
    can_sched_init(&sched, NULL);

    /* Enqueue ordinary torque frames, then trigger emergency disable */
    const float torq[6] = { 2.0f, -2.0f, 1.0f, 2.0f, -2.0f, 1.0f };
    assert(can_sched_enqueue_torques(&sched, torq, 10000u) == CAN_SCHED_OK);
    assert(can_sched_emergency_disable(&sched, 10200u) == CAN_SCHED_OK);

    /* All 3 buses aborted ordinary TX and queued immediate 0xFF..FD disable frames */
    for (uint8_t b = 0u; b < CAN_SCHED_NUM_BUSES; ++b) {
        assert(sched.bus[b].abort_count == can_sched_axis_count_for_bus(b));
        assert(can_sched_tx_pending(&sched, b) == can_sched_axis_count_for_bus(b));
        for (uint8_t k = 0u; k < can_sched_axis_count_for_bus(b); ++k) {
            CanFrame f;
            assert(can_sched_pop_tx(&sched, b, &f) == CAN_SCHED_OK);
            assert(f.data[0] == 0xFFu && f.data[7] == DAMIAO_CMD_DISABLE_BYTE);
        }
    }

    /* Before +1 ms: service_tx does not emit repeat yet */
    can_sched_service_tx(&sched, 11199u);
    assert(can_sched_tx_pending(&sched, 0u) == 0u);

    /* At +1 ms, repeat exactly once for all six motors. */
    can_sched_service_tx(&sched, 11200u);
    for (uint8_t b = 0u; b < CAN_SCHED_NUM_BUSES; ++b) {
        assert(can_sched_tx_pending(&sched, b) == can_sched_axis_count_for_bus(b));
        for (uint8_t k = 0u; k < can_sched_axis_count_for_bus(b); ++k) {
            CanFrame f;
            assert(can_sched_pop_tx(&sched, b, &f) == CAN_SCHED_OK);
            assert(f.data[7] == DAMIAO_CMD_DISABLE_BYTE);
        }
    }

    /* ErrorPassive on bus 1 -> warning + inhibit_jump = 1, not bus-off */
    CanSchedHealth h;
    assert(can_sched_update_bus_status(&sched, 1u, 130u, 10u, 0u, 1u, 1u, 20000u) == CAN_SCHED_OK);
    can_sched_get_health(&sched, &h);
    assert(h.any_error_passive == 1u && h.inhibit_jump == 1u && h.any_bus_off == 0u);

    /* BusOff on spare bus 2 (conservative whole-system fault) -> latches bus_off, aborts TX, disables buses 0 & 1 */
    assert(can_sched_enqueue_torques(&sched, torq, 21000u) == CAN_SCHED_OK);
    assert(can_sched_update_bus_status(&sched, 2u, 256u, 0u, 1u, 1u, 1u, 21100u) == CAN_SCHED_OK);
    can_sched_get_health(&sched, &h);
    assert(h.any_bus_off == 1u && h.bus_off_cnt[2] == 1u);
    assert(can_sched_tx_pending(&sched, 2u) == 0u); /* Bus 2 is bus-off */
    assert(can_sched_tx_pending(&sched, 0u) == 3u); /* Right leg disable */
    assert(can_sched_tx_pending(&sched, 1u) == 3u); /* Left leg disable */
    for (uint8_t b = 0u; b < 2u; ++b) {
        for (uint8_t k = 0u; k < can_sched_axis_count_for_bus(b); ++k) {
            CanFrame f;
            assert(can_sched_pop_tx(&sched, b, &f) == CAN_SCHED_OK);
            assert(f.data[0] == 0xFFu && f.data[7] == DAMIAO_CMD_DISABLE_BYTE);
        }
    }

    /* Normal torque/enable enqueue refused while bus-off latched */
    assert(can_sched_enqueue_torques(&sched, torq, 22000u) == CAN_SCHED_ERR_BUS_OFF);
    assert(can_sched_enqueue_enable_all(&sched, 22000u) == CAN_SCHED_ERR_BUS_OFF);

    /* Bus-off NEVER auto-recovers even if hw_bus_off drops back to 0 */
    assert(can_sched_update_bus_status(&sched, 2u, 0u, 0u, 0u, 0u, 0u, 23000u) == CAN_SCHED_OK);
    can_sched_get_health(&sched, &h);
    assert(h.any_bus_off == 1u);

    /* Recovery refused while armed (is_disarmed == 0) */
    assert(can_sched_request_bus_recovery(&sched, 0u) == CAN_SCHED_ERR_NOT_DISARMED);

    /* Recovery allowed ONLY in DISARMED and requires hw_bus_off == 0 + all 6 motors answered */
    assert(can_sched_request_bus_recovery(&sched, 1u) == CAN_SCHED_OK);
    assert(can_sched_confirm_bus_recovery(&sched, 2u, 1u, 1u) == CAN_SCHED_ERR_RECOVERY_INCOMPLETE);
    assert(can_sched_confirm_bus_recovery(&sched, 2u, 0u, 0u) == CAN_SCHED_ERR_RECOVERY_INCOMPLETE);
    assert(can_sched_confirm_bus_recovery(&sched, 2u, 0u, 1u) == CAN_SCHED_OK);
    can_sched_get_health(&sched, &h);
    assert(h.any_bus_off == 0u);

    /* Enable frames accepted again after clean recovery */
    assert(can_sched_enqueue_enable_all(&sched, 30000u) == CAN_SCHED_OK);
    for (uint8_t b = 0u; b < CAN_SCHED_NUM_BUSES; ++b) {
        CanFrame f;
        if (b == 2u) {
            assert(can_sched_pop_tx(&sched, b, &f) == CAN_SCHED_ERR_QUEUE_EMPTY);
        } else {
            assert(can_sched_pop_tx(&sched, b, &f) == CAN_SCHED_OK);
            assert(f.data[0] == 0xFFu && f.data[7] == DAMIAO_CMD_ENABLE_BYTE);
        }
    }
}

static void test_s11_500hz_integration(void)
{
    CanSched sched;
    S11PacketPolicy s11;
    CanSchedRxSnapshot snap;
    S11PacketFrame frame;

    can_sched_init(&sched, NULL);
    s11_init(&s11);
    assert(s11_arm(&s11) == 1);

    /* Partial startup: only axes 1..5 reply -> status_unknown_mask bit 0 set -> S11_ARMING */
    uint32_t now_us = 10000u;
    for (uint8_t a = 1u; a < 6u; ++a) {
        uint8_t bus = 0u;
        assert(can_sched_bus_for_axis(a, &bus) == CAN_SCHED_OK);
        CanFrame f = make_fb_frame(a, 0x1u, 0.2f, 0.0f, 0.0f);
        assert(can_sched_rx_frame(&sched, bus, &f, now_us - 500u) == CAN_SCHED_OK);
    }
    assert(can_sched_step_s11_500hz(&sched, &s11, now_us, 2.0, &snap, &frame) == CAN_SCHED_OK);
    assert((snap.status_unknown_mask & 1u) != 0u);
    assert(frame.mode == S11_ARMING && frame.valid == 0u);

    /* 3 full 6-axis groups -> transitions S11_ARMING -> S11_FRESH */
    for (uint32_t g = 0u; g < 3u; ++g) {
        now_us += 2000u;
        for (uint8_t a = 0u; a < 6u; ++a) {
            uint8_t bus = 0u;
            assert(can_sched_bus_for_axis(a, &bus) == CAN_SCHED_OK);
            CanFrame f = make_fb_frame(a, 0x1u, 0.2f + 0.01f * (float)g, 0.0f, 0.0f);
            assert(can_sched_rx_frame(&sched, bus, &f, now_us - 500u) == CAN_SCHED_OK);
        }
        assert(can_sched_step_s11_500hz(&sched, &s11, now_us, 2.0, &snap, &frame) == CAN_SCHED_OK);
    }
    assert(snap.status_unknown_mask == 0u);
    assert(frame.mode == S11_FRESH && frame.valid == 1u);

    /* 5 missed 500 Hz ticks -> PREDICT (1..2), LIMIT (3..4), TIMEOUT (5) */
    for (uint32_t m = 1u; m <= 5u; ++m) {
        now_us += 2000u;
        assert(can_sched_step_s11_500hz(&sched, &s11, now_us, 2.0, &snap, &frame) == CAN_SCHED_OK);
        if (m <= 2u) {
            assert(frame.mode == S11_PREDICT && frame.valid == 1u);
        } else if (m <= 4u) {
            assert(frame.mode == S11_LIMIT && frame.valid == 1u && fabs(frame.authority - 0.75) < 1e-9);
        } else {
            assert(frame.mode == S11_TIMEOUT && frame.valid == 0u);
        }
    }
}

int main(void)
{
    test_topology_and_tx_priority();
    test_rx_filter_dispatch_and_timeouts();
    test_emergency_disable_and_bus_off_recovery();
    test_s11_500hz_integration();
    printf("test_can_sched: ALL PASSED\n");
    return 0;
}
