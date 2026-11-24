#ifdef NDEBUG
#undef NDEBUG
#endif
#include "log_packer.h"

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

static void test_telem_fast_pack_unpack(void)
{
    LogTelemFast in;
    memset(&in, 0, sizeof(in));
    in.fsm_state      = 3u;
    in.s11_mode       = 1u;
    in.s11_max_misses = 2u;
    in.flags          = 0xA5u;
    in.fault_mask     = 0x12345678u;
    in.euler_rad[0]   = 0.05f;
    in.euler_rad[1]   = -0.02f;
    in.euler_rad[2]   = 1.57f;
    in.gyro_rad_s[0]  = 0.1f;
    in.gyro_rad_s[1]  = -0.2f;
    in.gyro_rad_s[2]  = 0.3f;
    in.v_est_m_s      = 0.75f;
    in.x_est_m        = -1.25f;
    in.leg_length_m[0] = 0.14f;
    in.leg_length_m[1] = 0.145f;
    in.leg_theta_rad[0] = 0.03f;
    in.leg_theta_rad[1] = -0.04f;
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        in.motor_pos_rad[i]    = 0.1f * (float)(i + 1u);
        in.motor_vel_rad_s[i]  = -1.5f * (float)(i + 1u);
        in.motor_tor_cmd_nm[i] = 0.5f * (float)i;
        in.motor_tor_fb_nm[i]  = 0.45f * (float)i;
        in.enc_age_us[i]       = (uint16_t)(400u + 50u * i);
    }
    in.s9_trim_rad       = -0.012f;
    in.gyro_bias_y_rad_s = 0.0015f;
    in.v_bus_mv          = 24150u;
    in.i_bus_ma          = -3250; /* Negative regen current */

    uint8_t buf[LOG_TELEM_FAST_SIZE];
    size_t len = 0u;
    assert(log_pack_telem_fast(&in, buf, sizeof(buf) - 1u, &len) == LOG_ERR_BUFFER_TOO_SMALL);
    assert(log_pack_telem_fast(&in, buf, sizeof(buf), &len) == LOG_OK);
    assert(len == LOG_TELEM_FAST_SIZE);
    assert(len == 176u);

    /* Verify exact little-endian wire encoding of fault_mask at bytes [4..7] */
    assert(buf[4] == 0x78u && buf[5] == 0x56u && buf[6] == 0x34u && buf[7] == 0x12u);

    LogTelemFast out;
    memset(&out, 0, sizeof(out));
    assert(log_unpack_telem_fast(buf, len - 1u, &out) == LOG_ERR_BAD_LENGTH);
    assert(log_unpack_telem_fast(buf, len, &out) == LOG_OK);
    assert(memcmp(&in, &out, sizeof(in)) == 0);
}

static void test_telem_health_and_bench_raw(void)
{
    LogTelemHealth h_in;
    memset(&h_in, 0, sizeof(h_in));
    for (uint32_t i = 0u; i < LOG_NUM_CAN_BUSES; ++i) {
        h_in.can_tec[i]        = (uint8_t)(10u + i);
        h_in.can_rec[i]        = (uint8_t)(20u + i);
        h_in.can_busoff_cnt[i] = (uint16_t)(i);
        h_in.can_rx_drop[i]    = (uint16_t)(100u + i);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        h_in.motor_state[i] = 1u;
        h_in.t_mos_c[i]     = (uint8_t)(35u + i);
        h_in.t_coil_c[i]    = (uint8_t)(45u + i);
    }
    h_in.rc_rssi_dbm   = -68;
    h_in.rc_lq_pct     = 99u;
    h_in.rc_age_ms     = 4u;
    h_in.rc_switches   = 0x0005u;
    h_in.power_io_bits = 0x01F3u;
    h_in.reset_cause   = 0x04u;
    h_in.cpu_load_pct  = 17u;
    for (uint32_t i = 0u; i < LOG_NUM_RTOS_TASKS; ++i) {
        h_in.min_stack_words[i] = (uint16_t)(200u + 10u * i);
    }

    uint8_t h_buf[LOG_TELEM_HEALTH_SIZE];
    size_t h_len = 0u;
    assert(log_pack_telem_health(&h_in, h_buf, sizeof(h_buf), &h_len) == LOG_OK);
    assert(h_len == LOG_TELEM_HEALTH_SIZE);

    LogTelemHealth h_out;
    memset(&h_out, 0, sizeof(h_out));
    assert(log_unpack_telem_health(h_buf, h_len, &h_out) == LOG_OK);
    assert(h_out.rc_rssi_dbm == -68);
    assert(memcmp(&h_in, &h_out, sizeof(h_in)) == 0);

    LogBenchRaw b_in;
    memset(&b_in, 0, sizeof(b_in));
    b_in.imu_ts_us = 9876543210ULL;
    b_in.gyro_rad_s[0] = 0.01f; b_in.gyro_rad_s[1] = -0.02f; b_in.gyro_rad_s[2] = 0.03f;
    b_in.accel_m_s2[0] = 0.1f;  b_in.accel_m_s2[1] = -0.2f;  b_in.accel_m_s2[2] = 9.81f;
    b_in.imu_temp_c = 38.5f;
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        b_in.motor_pos_rad[i]   = -0.5f + 0.2f * (float)i;
        b_in.motor_vel_rad_s[i] = 2.0f * (float)i;
        b_in.motor_rx_us[i]     = 500000u + 100u * i;
    }
    b_in.fresh_mask     = 0x3Fu;
    b_in.pitch_base_rad = 0.011f;
    b_in.v_base_m_s     = 0.42f;
    b_in.pitch_s10_rad  = 0.010f;
    b_in.v_s10_m_s      = 0.43f;

    uint8_t b_buf[LOG_BENCH_RAW_SIZE];
    size_t b_len = 0u;
    assert(log_pack_bench_raw(&b_in, b_buf, sizeof(b_buf), &b_len) == LOG_OK);
    assert(b_len == LOG_BENCH_RAW_SIZE);

    LogBenchRaw b_out;
    memset(&b_out, 0, sizeof(b_out));
    assert(log_unpack_bench_raw(b_buf, b_len, &b_out) == LOG_OK);
    assert(memcmp(&b_in, &b_out, sizeof(b_in)) == 0);
}

static void test_flight_ring_and_usb_framing_integration(void)
{
    /* Static circular flight recorder: capacity 5, post-fault 2 ticks */
    LogFlightRecord storage[5];
    LogFlightRing ring;
    assert(log_ring_init(&ring, storage, 5u, 2u) == LOG_OK);

    /* Push 7 pre-fault records -> oldest should be seq 2..6 */
    for (uint32_t s = 0u; s < 7u; ++s) {
        LogFlightRecord rec;
        memset(&rec, 0, sizeof(rec));
        rec.seq = s;
        rec.timestamp_us = (uint64_t)s * 2000ULL;
        rec.fast.v_bus_mv = (uint16_t)(24000u - 10u * s);
        assert(log_ring_push(&ring, &rec) == LOG_OK);
    }
    assert(log_ring_count(&ring) == 5u);
    assert(log_ring_is_frozen(&ring) == 0u);

    /* Trigger fault -> captures 2 post-fault ticks (seq 7, 8) then freezes */
    log_ring_trigger_fault(&ring);
    for (uint32_t s = 7u; s < 12u; ++s) {
        LogFlightRecord rec;
        memset(&rec, 0, sizeof(rec));
        rec.seq = s;
        rec.timestamp_us = (uint64_t)s * 2000ULL;
        rec.fast.fsm_state = 6u; /* FAULT */
        assert(log_ring_push(&ring, &rec) == LOG_OK);
    }
    assert(log_ring_is_frozen(&ring) == 1u);
    assert(log_ring_count(&ring) == 5u);

    /* Stored records must be seq 4, 5, 6 (pre-fault) and 7, 8 (post-fault); 9..11 ignored */
    for (uint32_t idx = 0u; idx < 5u; ++idx) {
        LogFlightRecord got;
        assert(log_ring_get(&ring, idx, &got) == LOG_OK);
        assert(got.seq == 4u + idx);

        uint8_t packed[LOG_FLIGHT_RECORD_SIZE];
        size_t plen = 0u;
        assert(log_pack_flight_record(&got, packed, sizeof(packed), &plen) == LOG_OK);
        assert(plen == LOG_FLIGHT_RECORD_SIZE);
        LogFlightRecord unpacked;
        assert(log_unpack_flight_record(packed, plen, &unpacked) == LOG_OK);
        assert(unpacked.seq == got.seq && unpacked.timestamp_us == got.timestamp_us);
    }
    LogFlightRecord oob;
    assert(log_ring_get(&ring, 5u, &oob) == LOG_ERR_OUT_OF_RANGE);

    /* End-to-end: frame TELEM_FAST over USB COBS + CRC-32C -> streaming RX -> unpack */
    LogTelemFast tf;
    memset(&tf, 0, sizeof(tf));
    tf.fsm_state = 3u;
    tf.v_bus_mv  = 23800u;
    tf.i_bus_ma  = 4500;
    tf.euler_rad[0] = 0.123f;

    uint8_t wire[USB_LINK_MAX_WIRE_FRAME];
    size_t wire_len = 0u;
    assert(log_frame_telem_fast(777u, 12345678ULL, USB_LINK_FLAG_BENCH_MODE,
                                &tf, wire, sizeof(wire), &wire_len) == LOG_OK);

    UsbLinkRxParser rx;
    usb_link_rx_init(&rx);
    UsbLinkHeader rx_hdr;
    uint8_t rx_payload[USB_LINK_MAX_PAYLOAD];
    UsbLinkStatus st = USB_LINK_NEED_MORE;
    for (size_t i = 0u; i < wire_len; ++i) {
        st = usb_link_rx_feed(&rx, wire[i], &rx_hdr, rx_payload, sizeof(rx_payload));
    }
    assert(st == USB_LINK_OK);
    assert(rx_hdr.msg_type == (uint8_t)USB_MSG_TELEM_FAST);
    assert(rx_hdr.seq == 777u);
    assert(rx_hdr.timestamp_us == 12345678ULL);
    assert(rx_hdr.payload_len == LOG_TELEM_FAST_SIZE);

    LogTelemFast tf_rx;
    assert(log_unpack_telem_fast(rx_payload, rx_hdr.payload_len, &tf_rx) == LOG_OK);
    assert(memcmp(&tf, &tf_rx, sizeof(tf)) == 0);
}

int main(void)
{
    test_telem_fast_pack_unpack();
    test_telem_health_and_bench_raw();
    test_flight_ring_and_usb_framing_integration();
    printf("test_log_packer: ALL PASSED\n");
    return 0;
}
