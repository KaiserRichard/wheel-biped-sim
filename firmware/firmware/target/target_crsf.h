/*
 * ExpressLRS CRSF Receiver UART Driver with CRC-8 DVB-S2 & 150 ms Failsafe Timeout
 *
 * Implements CRSF (420000 baud, 8N1) frame synchronization, CRC-8 DVB-S2 (poly 0xD5)
 * verification, 16x11-bit packed RC channel decoding (0x16), Link Statistics (0x14)
 * RSSI/LQ qualification, and 150 ms failsafe timeout feeding SafetyFsm radio-loss
 * policy c (RADIO_LOSS_BRAKE_BALANCE: 0.60 s local brake -> REST, keep balancing).
 *
 * Pinout: USART6 on PC6 (TX) / PC7 (RX), AF7, verified against CTRL rev4.
 */

#ifndef WBR_TARGET_CRSF_H
#define WBR_TARGET_CRSF_H

#include "target_hw.h"
#include "safety_fsm.h"

#include <stddef.h>
#include <stdint.h>

#define CRSF_SYNC_BYTE_FC               0xC8u
#define CRSF_SYNC_BYTE_TX               0xEEu
#define CRSF_SYNC_BYTE_HANDSET          0xEAu

#define CRSF_FRAMETYPE_LINK_STATISTICS  0x14u
#define CRSF_FRAMETYPE_RC_CHANNELS      0x16u

#define CRSF_NUM_CHANNELS               16u
#define CRSF_RC_PAYLOAD_LEN             22u
#define CRSF_LINK_STATS_PAYLOAD_LEN     10u
#define CRSF_MAX_FRAME_LEN              64u

#define CRSF_CH_VAL_MIN                 172u
#define CRSF_CH_VAL_MID                 992u
#define CRSF_CH_VAL_MAX                 1811u
#define CRSF_SW_HIGH_THRESH             1500u

/* Radio loss must not add a stricter torque-off threshold during braking. */
#define CRSF_NORMAL_TILT_LIMIT_DEG      30.0f

typedef struct {
    uint16_t channels[CRSF_NUM_CHANNELS];
    float    v_target_m_s;
    float    yaw_target_rad_s;
    uint8_t  radio_arm_sw;
    uint8_t  radio_estop_sw;
    uint8_t  stand_handoff_req;
    uint8_t  voluntary_stop_req;
    int8_t   rssi_dbm;
    uint8_t  lq_pct;
    uint16_t switch_bitmask;
    uint32_t radio_age_ms;
    uint8_t  failsafe_active;     /* 1 = >150 ms since last qualified fresh CRSF frame */
    uint8_t  active_radio_policy; /* SAFETY_RADIO_LOSS_BRAKE_BALANCE (policy c = 1u) */
    float    effective_tilt_limit_deg;
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t framing_errors;
} TargetCrsfSnapshot;

extern UART_HandleTypeDef g_huart_crsf;

/* Pure-C CRC-8 DVB-S2 (polynomial 0xD5, init 0x00) over CRSF [Type + Payload]. */
uint8_t target_crsf_crc8_dvb_s2(const uint8_t *data, size_t len);

/* Initialize USART6 at 420000 baud 8N1 and reset CRSF parser state (failsafe active until 1st frame). */
HAL_StatusTypeDef target_crsf_init(void);

/* Push one raw UART RX byte from ISR into the lock-free SPSC ring buffer. */
void target_crsf_isr_push_byte(uint8_t byte, uint32_t rx_us);

/* Drain RX ring buffer and parse complete CRSF frames (CRC-8 checked). */
void target_crsf_poll(uint32_t now_us);

/* Snapshot decoded pilot commands, link stats, and 150 ms failsafe timeout for the 500 Hz tick. */
void target_crsf_snapshot(uint32_t now_us, TargetCrsfSnapshot *out_snap);

/* Populate SafetyInputs radio fields with policy c and the normal tilt limit. */
void target_crsf_apply_to_safety(const TargetCrsfSnapshot *snap,
                                 SafetyConfig *safety_cfg,
                                 uint32_t *out_radio_age_ms,
                                 uint8_t *out_radio_arm_sw,
                                 uint8_t *out_radio_estop_sw);

#endif /* WBR_TARGET_CRSF_H */
