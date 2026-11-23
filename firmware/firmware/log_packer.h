/*
 * Telemetry & Flight-Recorder Binary Log Record Packer
 *
 * Implements portable little-endian packing/unpacking for TELEM_FAST (0x81, 176 B),
 * TELEM_HEALTH (0x82, 56 B), BENCH_RAW_LOG (0x85, 128 B), and the static pre-fault
 * circular flight recorder described in FIRMWARE_DESIGN_v2.md Sections 7.3 & 8.
 *
 * Pure C11, zero HAL/RTOS dependencies, zero dynamic memory, fixed-width types.
 */

#ifndef WBR_LOG_PACKER_H
#define WBR_LOG_PACKER_H

#include "usb_link.h"

#include <stddef.h>
#include <stdint.h>

#define LOG_NUM_AXES            6u
#define LOG_NUM_CAN_BUSES       3u
#define LOG_NUM_RTOS_TASKS      5u

#define LOG_TELEM_FAST_SIZE     176u
#define LOG_TELEM_HEALTH_SIZE   56u
#define LOG_BENCH_RAW_SIZE      128u
#define LOG_FLIGHT_RECORD_SIZE  (12u + LOG_TELEM_FAST_SIZE)

typedef enum {
    LOG_OK = 0,
    LOG_ERR_NULL_PTR = 1,
    LOG_ERR_BUFFER_TOO_SMALL = 2,
    LOG_ERR_BAD_LENGTH = 3,
    LOG_ERR_OUT_OF_RANGE = 4,
    LOG_ERR_LINK_ENCODE = 5
} LogStatus;

/* TELEM_FAST (0x81) — exact 176-byte wire payload from FIRMWARE_DESIGN_v2.md Section 7.3. */
typedef struct {
    uint8_t  fsm_state;
    uint8_t  s11_mode;
    uint8_t  s11_max_misses;
    uint8_t  flags;
    uint32_t fault_mask;
    float    euler_rad[3];                 /* pitch, roll, yaw */
    float    gyro_rad_s[3];                /* wx, wy, wz */
    float    v_est_m_s;
    float    x_est_m;
    float    leg_length_m[2];              /* L0 left, right */
    float    leg_theta_rad[2];             /* theta left, right */
    float    motor_pos_rad[LOG_NUM_AXES];
    float    motor_vel_rad_s[LOG_NUM_AXES];
    float    motor_tor_cmd_nm[LOG_NUM_AXES];
    float    motor_tor_fb_nm[LOG_NUM_AXES];
    uint16_t enc_age_us[LOG_NUM_AXES];
    float    s9_trim_rad;
    float    gyro_bias_y_rad_s;
    uint16_t v_bus_mv;
    int16_t  i_bus_ma;
} LogTelemFast;

/* TELEM_HEALTH (0x82) — exact 56-byte wire payload from FIRMWARE_DESIGN_v2.md Section 7.3. */
typedef struct {
    uint8_t  can_tec[LOG_NUM_CAN_BUSES];
    uint8_t  can_rec[LOG_NUM_CAN_BUSES];
    uint16_t can_busoff_cnt[LOG_NUM_CAN_BUSES];
    uint16_t can_rx_drop[LOG_NUM_CAN_BUSES];
    uint8_t  motor_state[LOG_NUM_AXES];
    uint8_t  t_mos_c[LOG_NUM_AXES];
    uint8_t  t_coil_c[LOG_NUM_AXES];
    int8_t   rc_rssi_dbm;
    uint8_t  rc_lq_pct;
    uint16_t rc_age_ms;
    uint16_t rc_switches;
    uint16_t power_io_bits;
    uint8_t  reset_cause;
    uint8_t  cpu_load_pct;
    uint16_t min_stack_words[LOG_NUM_RTOS_TASKS];
} LogTelemHealth;

/* BENCH_RAW_LOG (0x85) — exact 128-byte wire payload for offline estimator A/B replay. */
typedef struct {
    uint64_t imu_ts_us;
    float    gyro_rad_s[3];
    float    accel_m_s2[3];
    float    imu_temp_c;
    float    motor_pos_rad[LOG_NUM_AXES];
    float    motor_vel_rad_s[LOG_NUM_AXES];
    uint32_t motor_rx_us[LOG_NUM_AXES];
    uint8_t  fresh_mask;
    uint8_t  reserved[3];
    float    pitch_base_rad;
    float    v_base_m_s;
    float    pitch_s10_rad;
    float    v_s10_m_s;
} LogBenchRaw;

/* Flight recorder snapshot stored in static circular SRAM ring buffer. */
typedef struct {
    uint64_t     timestamp_us;
    uint32_t     seq;
    LogTelemFast fast;
} LogFlightRecord;

typedef struct {
    LogFlightRecord *storage;
    uint32_t         capacity;
    uint32_t         head;                /* Next write index */
    uint32_t         count;               /* Valid records stored (<= capacity) */
    uint32_t         post_fault_ticks;    /* Ticks to record after fault trigger */
    uint32_t         post_fault_remaining;
    uint8_t          fault_triggered;
    uint8_t          frozen;
} LogFlightRing;

/* Explicit little-endian payload packers & unpackers. */
LogStatus log_pack_telem_fast(const LogTelemFast *rec, uint8_t *dst, size_t dst_cap, size_t *out_len);
LogStatus log_unpack_telem_fast(const uint8_t *src, size_t src_len, LogTelemFast *out_rec);

LogStatus log_pack_telem_health(const LogTelemHealth *rec, uint8_t *dst, size_t dst_cap, size_t *out_len);
LogStatus log_unpack_telem_health(const uint8_t *src, size_t src_len, LogTelemHealth *out_rec);

LogStatus log_pack_bench_raw(const LogBenchRaw *rec, uint8_t *dst, size_t dst_cap, size_t *out_len);
LogStatus log_unpack_bench_raw(const uint8_t *src, size_t src_len, LogBenchRaw *out_rec);

LogStatus log_pack_flight_record(const LogFlightRecord *rec, uint8_t *dst, size_t dst_cap, size_t *out_len);
LogStatus log_unpack_flight_record(const uint8_t *src, size_t src_len, LogFlightRecord *out_rec);

/* Convenience helpers: pack record and frame with COBS + CRC-32C via usb_link_encode(). */
LogStatus log_frame_telem_fast(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                               const LogTelemFast *rec,
                               uint8_t *wire_buf, size_t wire_cap, size_t *wire_len);

LogStatus log_frame_telem_health(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                                 const LogTelemHealth *rec,
                                 uint8_t *wire_buf, size_t wire_cap, size_t *wire_len);

LogStatus log_frame_bench_raw(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                              const LogBenchRaw *rec,
                              uint8_t *wire_buf, size_t wire_cap, size_t *wire_len);

/* Static pre-fault / post-fault circular flight recorder (caller owns storage). */
LogStatus log_ring_init(LogFlightRing *ring, LogFlightRecord *storage,
                        uint32_t capacity, uint32_t post_fault_ticks);
LogStatus log_ring_push(LogFlightRing *ring, const LogFlightRecord *rec);
void      log_ring_trigger_fault(LogFlightRing *ring);
uint32_t  log_ring_count(const LogFlightRing *ring);
uint8_t   log_ring_is_frozen(const LogFlightRing *ring);
LogStatus log_ring_get(const LogFlightRing *ring, uint32_t logical_idx, LogFlightRecord *out_rec);

#endif /* WBR_LOG_PACKER_H */
