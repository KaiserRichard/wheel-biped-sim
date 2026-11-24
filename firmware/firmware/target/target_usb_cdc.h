/*
 * STM32H723 USB OTG Full-Speed CDC ACM Driver with COBS + CRC-32C Framing
 *
 * Bridges the STM32H7 HAL PCD peripheral on USB1_OTG_HS (internal FS PHY on
 * PA11/PA12, ASSUMED pending HW1) to the portable COBS + CRC-32C wire protocol
 * (`firmware/usb_link.h`) and telemetry binary record packer (`firmware/log_packer.h`).
 *
 * Implements:
 *   - Lock-free ISR-to-thread RX byte ring buffer fed from Bulk OUT endpoint 0x01
 *   - Streaming COBS + CRC-32C frame decoding via `UsbLinkRxParser` (`usb_link_rx_feed`)
 *   - CMD_TWIST (0x01, 52 B) decoding with monotonic sequence check, 50 Hz rate
 *     gate (MIN_INTERVAL_US = 20000 us), and 150 ms freshness timeout feeding S9
 *   - CMD_MODE (0x02) operator mode/arm/reset decoding
 *   - TIME_SYNC_REQ (0x03) -> TIME_SYNC_RESP (0x83) round-trip timestamp response
 *   - Outgoing COBS + CRC-32C framing for TELEM_FAST (0x81, 176 B),
 *     TELEM_HEALTH (0x82, 56 B), FAULT_SNAPSHOT (0x84, 188 B), and
 *     BENCH_RAW_LOG (0x85, 128 B) over Bulk IN endpoint 0x81
 */

#ifndef WBR_TARGET_USB_CDC_H
#define WBR_TARGET_USB_CDC_H

#include "target_hw.h"
#include "usb_link.h"
#include "log_packer.h"
#include <stddef.h>
#include <stdint.h>

#define TARGET_USB_CDC_OUT_EP           0x01u
#define TARGET_USB_CDC_IN_EP            0x81u
#define TARGET_USB_CDC_CMD_EP           0x82u
#define TARGET_USB_FS_MAX_PACKET_SIZE   64u

#define TARGET_USB_RX_RING_SIZE         1024u
#define TARGET_USB_TX_BUF_SIZE          512u

#define TARGET_USB_CMD_TWIST_SIZE       52u
#define TARGET_USB_CMD_MODE_SIZE        8u
#define TARGET_USB_TIME_SYNC_REQ_SIZE   8u
#define TARGET_USB_TIME_SYNC_RESP_SIZE  24u

#define TARGET_USB_MIN_TWIST_INTERVAL_US 20000u /* S12 50 Hz rate clamp */
#define TARGET_USB_HOST_TIMEOUT_MS       150u   /* 150 ms freshness TTL */

typedef struct {
    /* Latest valid CMD_TWIST (0x01) fields */
    uint64_t session_id;
    uint64_t sent_us;
    uint64_t pose_us;
    uint32_t pose_seq;
    float    v_cmd_m_s;
    float    yaw_rate_cmd_rad_s;
    float    height_cmd_m;
    float    roll_cmd_rad;
    float    pose_x_m;
    float    pose_y_m;
    float    pose_yaw_rad;

    /* Latest CMD_MODE (0x02) requests */
    uint8_t  req_mode;
    uint8_t  arm_req;
    uint8_t  stand_handoff_req;
    uint8_t  voluntary_stop_req;
    uint8_t  reset_fault_req;
    uint8_t  jump_req;
    uint8_t  bench_mode_active;

    /* Freshness & link diagnostics for BalanceTickInput */
    uint32_t host_age_ms;
    uint8_t  host_valid;
    uint32_t twist_frames_ok;
    uint32_t twist_rate_drops;
    uint32_t twist_seq_drops;
    uint32_t rx_crc_errors;
    uint32_t rx_proto_errors;
    uint32_t tx_frames_sent;
    uint32_t tx_frames_dropped;
} TargetUsbHostCmd;

extern PCD_HandleTypeDef g_hpcd_usb;

/* Initialize USB1_OTG_HS in Full-Speed internal PHY CDC device mode. */
HAL_StatusTypeDef target_usb_cdc_init(void);

/* Inject raw bytes into the RX ring buffer (called by HAL_PCD_DataOutStageCallback). */
void target_usb_cdc_rx_push_bytes_isr(const uint8_t *data, uint32_t len);

/* Poll and decode pending COBS+CRC32C frames from the USB RX ring buffer. */
void target_usb_cdc_poll(uint32_t now_us, uint64_t now_us64, TargetUsbHostCmd *out_cmd);

/* Frame arbitrary payload with COBS + CRC-32C (`usb_link_encode`) and transmit over Bulk IN. */
UsbLinkStatus target_usb_cdc_send_frame(uint8_t msg_type, uint8_t flags,
                                        uint64_t timestamp_us,
                                        const uint8_t *payload, uint16_t payload_len);

/* Convenience senders for TELEM_FAST (0x81), TELEM_HEALTH (0x82), BENCH_RAW_LOG (0x85), FAULT_SNAPSHOT (0x84). */
LogStatus target_usb_cdc_send_telem_fast(uint64_t timestamp_us, uint8_t flags,
                                         const LogTelemFast *rec);
LogStatus target_usb_cdc_send_telem_health(uint64_t timestamp_us, uint8_t flags,
                                           const LogTelemHealth *rec);
LogStatus target_usb_cdc_send_bench_raw(uint64_t timestamp_us, uint8_t flags,
                                        const LogBenchRaw *rec);
LogStatus target_usb_cdc_send_fault_snapshot(uint64_t timestamp_us,
                                             const LogFlightRecord *rec);

#endif /* WBR_TARGET_USB_CDC_H */
