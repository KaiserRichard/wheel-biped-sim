#include "log_packer.h"

#include <string.h>

static void wr_u8(uint8_t *buf, size_t *pos, uint8_t v)
{
    buf[*pos] = v;
    *pos += 1u;
}

static void wr_u16_le(uint8_t *buf, size_t *pos, uint16_t v)
{
    buf[*pos]      = (uint8_t)(v & 0xFFu);
    buf[*pos + 1u] = (uint8_t)((v >> 8u) & 0xFFu);
    *pos += 2u;
}

static void wr_i16_le(uint8_t *buf, size_t *pos, int16_t v)
{
    wr_u16_le(buf, pos, (uint16_t)v);
}

static void wr_u32_le(uint8_t *buf, size_t *pos, uint32_t v)
{
    buf[*pos]      = (uint8_t)(v & 0xFFu);
    buf[*pos + 1u] = (uint8_t)((v >> 8u) & 0xFFu);
    buf[*pos + 2u] = (uint8_t)((v >> 16u) & 0xFFu);
    buf[*pos + 3u] = (uint8_t)((v >> 24u) & 0xFFu);
    *pos += 4u;
}

static void wr_u64_le(uint8_t *buf, size_t *pos, uint64_t v)
{
    for (uint32_t i = 0u; i < 8u; ++i) {
        buf[*pos + i] = (uint8_t)((v >> (i * 8u)) & 0xFFu);
    }
    *pos += 8u;
}

static void wr_f32_le(uint8_t *buf, size_t *pos, float v)
{
    uint32_t bits = 0u;
    memcpy(&bits, &v, sizeof(bits));
    wr_u32_le(buf, pos, bits);
}

static uint8_t rd_u8(const uint8_t *buf, size_t *pos)
{
    const uint8_t v = buf[*pos];
    *pos += 1u;
    return v;
}

static uint16_t rd_u16_le(const uint8_t *buf, size_t *pos)
{
    const uint16_t v = (uint16_t)((uint16_t)buf[*pos] | ((uint16_t)buf[*pos + 1u] << 8u));
    *pos += 2u;
    return v;
}

static int16_t rd_i16_le(const uint8_t *buf, size_t *pos)
{
    return (int16_t)rd_u16_le(buf, pos);
}

static uint32_t rd_u32_le(const uint8_t *buf, size_t *pos)
{
    const uint32_t v = (uint32_t)buf[*pos]
                     | ((uint32_t)buf[*pos + 1u] << 8u)
                     | ((uint32_t)buf[*pos + 2u] << 16u)
                     | ((uint32_t)buf[*pos + 3u] << 24u);
    *pos += 4u;
    return v;
}

static uint64_t rd_u64_le(const uint8_t *buf, size_t *pos)
{
    uint64_t v = 0u;
    for (uint32_t i = 0u; i < 8u; ++i) {
        v |= ((uint64_t)buf[*pos + i] << (i * 8u));
    }
    *pos += 8u;
    return v;
}

static float rd_f32_le(const uint8_t *buf, size_t *pos)
{
    const uint32_t bits = rd_u32_le(buf, pos);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

LogStatus log_pack_telem_fast(const LogTelemFast *rec, uint8_t *dst,
                              size_t dst_cap, size_t *out_len)
{
    if (rec == NULL || dst == NULL || out_len == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    *out_len = 0u;
    if (dst_cap < LOG_TELEM_FAST_SIZE) {
        return LOG_ERR_BUFFER_TOO_SMALL;
    }

    size_t pos = 0u;
    wr_u8(dst, &pos, rec->fsm_state);
    wr_u8(dst, &pos, rec->s11_mode);
    wr_u8(dst, &pos, rec->s11_max_misses);
    wr_u8(dst, &pos, rec->flags);
    wr_u32_le(dst, &pos, rec->fault_mask);

    for (uint32_t i = 0u; i < 3u; ++i) {
        wr_f32_le(dst, &pos, rec->euler_rad[i]);
    }
    for (uint32_t i = 0u; i < 3u; ++i) {
        wr_f32_le(dst, &pos, rec->gyro_rad_s[i]);
    }
    wr_f32_le(dst, &pos, rec->v_est_m_s);
    wr_f32_le(dst, &pos, rec->x_est_m);
    for (uint32_t i = 0u; i < 2u; ++i) {
        wr_f32_le(dst, &pos, rec->leg_length_m[i]);
    }
    for (uint32_t i = 0u; i < 2u; ++i) {
        wr_f32_le(dst, &pos, rec->leg_theta_rad[i]);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        wr_f32_le(dst, &pos, rec->motor_pos_rad[i]);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        wr_f32_le(dst, &pos, rec->motor_vel_rad_s[i]);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        wr_f32_le(dst, &pos, rec->motor_tor_cmd_nm[i]);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        wr_f32_le(dst, &pos, rec->motor_tor_fb_nm[i]);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        wr_u16_le(dst, &pos, rec->enc_age_us[i]);
    }
    wr_f32_le(dst, &pos, rec->s9_trim_rad);
    wr_f32_le(dst, &pos, rec->gyro_bias_y_rad_s);
    wr_u16_le(dst, &pos, rec->v_bus_mv);
    wr_i16_le(dst, &pos, rec->i_bus_ma);

    *out_len = pos;
    return LOG_OK;
}

LogStatus log_unpack_telem_fast(const uint8_t *src, size_t src_len, LogTelemFast *out_rec)
{
    if (src == NULL || out_rec == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    if (src_len != LOG_TELEM_FAST_SIZE) {
        return LOG_ERR_BAD_LENGTH;
    }

    size_t pos = 0u;
    out_rec->fsm_state      = rd_u8(src, &pos);
    out_rec->s11_mode       = rd_u8(src, &pos);
    out_rec->s11_max_misses = rd_u8(src, &pos);
    out_rec->flags          = rd_u8(src, &pos);
    out_rec->fault_mask     = rd_u32_le(src, &pos);

    for (uint32_t i = 0u; i < 3u; ++i) {
        out_rec->euler_rad[i] = rd_f32_le(src, &pos);
    }
    for (uint32_t i = 0u; i < 3u; ++i) {
        out_rec->gyro_rad_s[i] = rd_f32_le(src, &pos);
    }
    out_rec->v_est_m_s = rd_f32_le(src, &pos);
    out_rec->x_est_m   = rd_f32_le(src, &pos);
    for (uint32_t i = 0u; i < 2u; ++i) {
        out_rec->leg_length_m[i] = rd_f32_le(src, &pos);
    }
    for (uint32_t i = 0u; i < 2u; ++i) {
        out_rec->leg_theta_rad[i] = rd_f32_le(src, &pos);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        out_rec->motor_pos_rad[i] = rd_f32_le(src, &pos);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        out_rec->motor_vel_rad_s[i] = rd_f32_le(src, &pos);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        out_rec->motor_tor_cmd_nm[i] = rd_f32_le(src, &pos);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        out_rec->motor_tor_fb_nm[i] = rd_f32_le(src, &pos);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        out_rec->enc_age_us[i] = rd_u16_le(src, &pos);
    }
    out_rec->s9_trim_rad       = rd_f32_le(src, &pos);
    out_rec->gyro_bias_y_rad_s = rd_f32_le(src, &pos);
    out_rec->v_bus_mv          = rd_u16_le(src, &pos);
    out_rec->i_bus_ma          = rd_i16_le(src, &pos);

    return LOG_OK;
}

LogStatus log_pack_telem_health(const LogTelemHealth *rec, uint8_t *dst,
                                size_t dst_cap, size_t *out_len)
{
    if (rec == NULL || dst == NULL || out_len == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    *out_len = 0u;
    if (dst_cap < LOG_TELEM_HEALTH_SIZE) {
        return LOG_ERR_BUFFER_TOO_SMALL;
    }

    size_t pos = 0u;
    for (uint32_t i = 0u; i < LOG_NUM_CAN_BUSES; ++i) {
        wr_u8(dst, &pos, rec->can_tec[i]);
    }
    for (uint32_t i = 0u; i < LOG_NUM_CAN_BUSES; ++i) {
        wr_u8(dst, &pos, rec->can_rec[i]);
    }
    for (uint32_t i = 0u; i < LOG_NUM_CAN_BUSES; ++i) {
        wr_u16_le(dst, &pos, rec->can_busoff_cnt[i]);
    }
    for (uint32_t i = 0u; i < LOG_NUM_CAN_BUSES; ++i) {
        wr_u16_le(dst, &pos, rec->can_rx_drop[i]);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        wr_u8(dst, &pos, rec->motor_state[i]);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        wr_u8(dst, &pos, rec->t_mos_c[i]);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        wr_u8(dst, &pos, rec->t_coil_c[i]);
    }
    wr_u8(dst, &pos, (uint8_t)rec->rc_rssi_dbm);
    wr_u8(dst, &pos, rec->rc_lq_pct);
    wr_u16_le(dst, &pos, rec->rc_age_ms);
    wr_u16_le(dst, &pos, rec->rc_switches);
    wr_u16_le(dst, &pos, rec->power_io_bits);
    wr_u8(dst, &pos, rec->reset_cause);
    wr_u8(dst, &pos, rec->cpu_load_pct);
    for (uint32_t i = 0u; i < LOG_NUM_RTOS_TASKS; ++i) {
        wr_u16_le(dst, &pos, rec->min_stack_words[i]);
    }

    *out_len = pos;
    return LOG_OK;
}

LogStatus log_unpack_telem_health(const uint8_t *src, size_t src_len,
                                  LogTelemHealth *out_rec)
{
    if (src == NULL || out_rec == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    if (src_len != LOG_TELEM_HEALTH_SIZE) {
        return LOG_ERR_BAD_LENGTH;
    }

    size_t pos = 0u;
    for (uint32_t i = 0u; i < LOG_NUM_CAN_BUSES; ++i) {
        out_rec->can_tec[i] = rd_u8(src, &pos);
    }
    for (uint32_t i = 0u; i < LOG_NUM_CAN_BUSES; ++i) {
        out_rec->can_rec[i] = rd_u8(src, &pos);
    }
    for (uint32_t i = 0u; i < LOG_NUM_CAN_BUSES; ++i) {
        out_rec->can_busoff_cnt[i] = rd_u16_le(src, &pos);
    }
    for (uint32_t i = 0u; i < LOG_NUM_CAN_BUSES; ++i) {
        out_rec->can_rx_drop[i] = rd_u16_le(src, &pos);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        out_rec->motor_state[i] = rd_u8(src, &pos);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        out_rec->t_mos_c[i] = rd_u8(src, &pos);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        out_rec->t_coil_c[i] = rd_u8(src, &pos);
    }
    out_rec->rc_rssi_dbm   = (int8_t)rd_u8(src, &pos);
    out_rec->rc_lq_pct     = rd_u8(src, &pos);
    out_rec->rc_age_ms     = rd_u16_le(src, &pos);
    out_rec->rc_switches   = rd_u16_le(src, &pos);
    out_rec->power_io_bits = rd_u16_le(src, &pos);
    out_rec->reset_cause   = rd_u8(src, &pos);
    out_rec->cpu_load_pct  = rd_u8(src, &pos);
    for (uint32_t i = 0u; i < LOG_NUM_RTOS_TASKS; ++i) {
        out_rec->min_stack_words[i] = rd_u16_le(src, &pos);
    }

    return LOG_OK;
}

LogStatus log_pack_bench_raw(const LogBenchRaw *rec, uint8_t *dst,
                             size_t dst_cap, size_t *out_len)
{
    if (rec == NULL || dst == NULL || out_len == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    *out_len = 0u;
    if (dst_cap < LOG_BENCH_RAW_SIZE) {
        return LOG_ERR_BUFFER_TOO_SMALL;
    }

    size_t pos = 0u;
    wr_u64_le(dst, &pos, rec->imu_ts_us);
    for (uint32_t i = 0u; i < 3u; ++i) {
        wr_f32_le(dst, &pos, rec->gyro_rad_s[i]);
    }
    for (uint32_t i = 0u; i < 3u; ++i) {
        wr_f32_le(dst, &pos, rec->accel_m_s2[i]);
    }
    wr_f32_le(dst, &pos, rec->imu_temp_c);
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        wr_f32_le(dst, &pos, rec->motor_pos_rad[i]);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        wr_f32_le(dst, &pos, rec->motor_vel_rad_s[i]);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        wr_u32_le(dst, &pos, rec->motor_rx_us[i]);
    }
    wr_u8(dst, &pos, rec->fresh_mask);
    for (uint32_t i = 0u; i < 3u; ++i) {
        wr_u8(dst, &pos, rec->reserved[i]);
    }
    wr_f32_le(dst, &pos, rec->pitch_base_rad);
    wr_f32_le(dst, &pos, rec->v_base_m_s);
    wr_f32_le(dst, &pos, rec->pitch_s10_rad);
    wr_f32_le(dst, &pos, rec->v_s10_m_s);

    *out_len = pos;
    return LOG_OK;
}

LogStatus log_unpack_bench_raw(const uint8_t *src, size_t src_len, LogBenchRaw *out_rec)
{
    if (src == NULL || out_rec == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    if (src_len != LOG_BENCH_RAW_SIZE) {
        return LOG_ERR_BAD_LENGTH;
    }

    size_t pos = 0u;
    out_rec->imu_ts_us = rd_u64_le(src, &pos);
    for (uint32_t i = 0u; i < 3u; ++i) {
        out_rec->gyro_rad_s[i] = rd_f32_le(src, &pos);
    }
    for (uint32_t i = 0u; i < 3u; ++i) {
        out_rec->accel_m_s2[i] = rd_f32_le(src, &pos);
    }
    out_rec->imu_temp_c = rd_f32_le(src, &pos);
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        out_rec->motor_pos_rad[i] = rd_f32_le(src, &pos);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        out_rec->motor_vel_rad_s[i] = rd_f32_le(src, &pos);
    }
    for (uint32_t i = 0u; i < LOG_NUM_AXES; ++i) {
        out_rec->motor_rx_us[i] = rd_u32_le(src, &pos);
    }
    out_rec->fresh_mask = rd_u8(src, &pos);
    for (uint32_t i = 0u; i < 3u; ++i) {
        out_rec->reserved[i] = rd_u8(src, &pos);
    }
    out_rec->pitch_base_rad = rd_f32_le(src, &pos);
    out_rec->v_base_m_s     = rd_f32_le(src, &pos);
    out_rec->pitch_s10_rad  = rd_f32_le(src, &pos);
    out_rec->v_s10_m_s      = rd_f32_le(src, &pos);

    return LOG_OK;
}

LogStatus log_pack_flight_record(const LogFlightRecord *rec, uint8_t *dst,
                                 size_t dst_cap, size_t *out_len)
{
    if (rec == NULL || dst == NULL || out_len == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    *out_len = 0u;
    if (dst_cap < LOG_FLIGHT_RECORD_SIZE) {
        return LOG_ERR_BUFFER_TOO_SMALL;
    }

    size_t pos = 0u;
    wr_u64_le(dst, &pos, rec->timestamp_us);
    wr_u32_le(dst, &pos, rec->seq);

    size_t fast_len = 0u;
    const LogStatus st = log_pack_telem_fast(&rec->fast, &dst[pos], dst_cap - pos, &fast_len);
    if (st != LOG_OK) {
        return st;
    }
    *out_len = pos + fast_len;
    return LOG_OK;
}

LogStatus log_unpack_flight_record(const uint8_t *src, size_t src_len,
                                   LogFlightRecord *out_rec)
{
    if (src == NULL || out_rec == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    if (src_len != LOG_FLIGHT_RECORD_SIZE) {
        return LOG_ERR_BAD_LENGTH;
    }

    size_t pos = 0u;
    out_rec->timestamp_us = rd_u64_le(src, &pos);
    out_rec->seq          = rd_u32_le(src, &pos);
    return log_unpack_telem_fast(&src[pos], src_len - pos, &out_rec->fast);
}

static LogStatus frame_payload(uint8_t msg_type, uint32_t seq, uint64_t timestamp_us,
                               uint8_t flags, const uint8_t *payload, uint16_t payload_len,
                               uint8_t *wire_buf, size_t wire_cap, size_t *wire_len)
{
    UsbLinkHeader hdr;
    hdr.ver_major    = USB_LINK_VER_MAJOR;
    hdr.ver_minor    = USB_LINK_VER_MINOR;
    hdr.msg_type     = msg_type;
    hdr.flags        = flags;
    hdr.seq          = seq;
    hdr.timestamp_us = timestamp_us;
    hdr.payload_len  = payload_len;

    const UsbLinkStatus st = usb_link_encode(&hdr, payload, wire_buf, wire_cap, wire_len);
    if (st == USB_LINK_ERR_BUFFER_TOO_SMALL) {
        return LOG_ERR_BUFFER_TOO_SMALL;
    }
    return (st == USB_LINK_OK) ? LOG_OK : LOG_ERR_LINK_ENCODE;
}

LogStatus log_frame_telem_fast(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                               const LogTelemFast *rec,
                               uint8_t *wire_buf, size_t wire_cap, size_t *wire_len)
{
    uint8_t payload[LOG_TELEM_FAST_SIZE];
    size_t plen = 0u;
    const LogStatus st = log_pack_telem_fast(rec, payload, sizeof(payload), &plen);
    if (st != LOG_OK) {
        return st;
    }
    return frame_payload((uint8_t)USB_MSG_TELEM_FAST, seq, timestamp_us, flags,
                         payload, (uint16_t)plen, wire_buf, wire_cap, wire_len);
}

LogStatus log_frame_telem_health(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                                 const LogTelemHealth *rec,
                                 uint8_t *wire_buf, size_t wire_cap, size_t *wire_len)
{
    uint8_t payload[LOG_TELEM_HEALTH_SIZE];
    size_t plen = 0u;
    const LogStatus st = log_pack_telem_health(rec, payload, sizeof(payload), &plen);
    if (st != LOG_OK) {
        return st;
    }
    return frame_payload((uint8_t)USB_MSG_TELEM_HEALTH, seq, timestamp_us, flags,
                         payload, (uint16_t)plen, wire_buf, wire_cap, wire_len);
}

LogStatus log_frame_bench_raw(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                              const LogBenchRaw *rec,
                              uint8_t *wire_buf, size_t wire_cap, size_t *wire_len)
{
    uint8_t payload[LOG_BENCH_RAW_SIZE];
    size_t plen = 0u;
    const LogStatus st = log_pack_bench_raw(rec, payload, sizeof(payload), &plen);
    if (st != LOG_OK) {
        return st;
    }
    return frame_payload((uint8_t)USB_MSG_BENCH_RAW_LOG, seq, timestamp_us, flags,
                         payload, (uint16_t)plen, wire_buf, wire_cap, wire_len);
}

LogStatus log_ring_init(LogFlightRing *ring, LogFlightRecord *storage,
                        uint32_t capacity, uint32_t post_fault_ticks)
{
    if (ring == NULL || storage == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    if (capacity == 0u) {
        return LOG_ERR_OUT_OF_RANGE;
    }
    memset(ring, 0, sizeof(*ring));
    ring->storage = storage;
    ring->capacity = capacity;
    ring->post_fault_ticks = post_fault_ticks;
    return LOG_OK;
}

LogStatus log_ring_push(LogFlightRing *ring, const LogFlightRecord *rec)
{
    if (ring == NULL || rec == NULL || ring->storage == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    if (ring->frozen != 0u) {
        return LOG_OK;
    }

    ring->storage[ring->head] = *rec;
    ring->head = (ring->head + 1u) % ring->capacity;
    if (ring->count < ring->capacity) {
        ring->count += 1u;
    }

    if (ring->fault_triggered != 0u) {
        if (ring->post_fault_remaining > 0u) {
            ring->post_fault_remaining -= 1u;
        }
        if (ring->post_fault_remaining == 0u) {
            ring->frozen = 1u;
        }
    }
    return LOG_OK;
}

void log_ring_trigger_fault(LogFlightRing *ring)
{
    if (ring == NULL || ring->fault_triggered != 0u) {
        return;
    }
    ring->fault_triggered = 1u;
    ring->post_fault_remaining = ring->post_fault_ticks;
    if (ring->post_fault_remaining == 0u) {
        ring->frozen = 1u;
    }
}

uint32_t log_ring_count(const LogFlightRing *ring)
{
    if (ring == NULL) {
        return 0u;
    }
    return ring->count;
}

uint8_t log_ring_is_frozen(const LogFlightRing *ring)
{
    if (ring == NULL) {
        return 0u;
    }
    return ring->frozen;
}

LogStatus log_ring_get(const LogFlightRing *ring, uint32_t logical_idx,
                       LogFlightRecord *out_rec)
{
    if (ring == NULL || out_rec == NULL || ring->storage == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    if (logical_idx >= ring->count) {
        return LOG_ERR_OUT_OF_RANGE;
    }

    const uint32_t oldest = (ring->count == ring->capacity) ? ring->head : 0u;
    const uint32_t phys_idx = (oldest + logical_idx) % ring->capacity;
    *out_rec = ring->storage[phys_idx];
    return LOG_OK;
}
