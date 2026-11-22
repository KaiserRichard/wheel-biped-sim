/*
 * hea_proto: UART framing shared by the Linux bridge and the STM32 firmware.
 *
 * Wire format (all multi-byte fields little-endian):
 *
 *   +------+------+-----+------+---------+-----+-----------+---------+
 *   | 0xA5 | 0x5A | ver | type | seq u16 | len | payload   | crc u16 |
 *   +------+------+-----+------+---------+-----+-----------+---------+
 *     SOF (2 bytes)   |<------- CRC-16/CCITT-FALSE covers ------->|
 *
 * Pure C99, no heap, no OS dependency: the same file builds for the host
 * tests, the Linux bridge and FreeRTOS.
 */
#ifndef HEA_PROTO_H
#define HEA_PROTO_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HEA_SOF0 0xA5u
#define HEA_SOF1 0x5Au
#define HEA_PROTO_VERSION_LEGACY 1u
#define HEA_PROTO_VERSION_AGE 2u
#define HEA_PROTO_VERSION 3u
#define HEA_MAX_PAYLOAD 64u
#define HEA_HEADER_LEN 7u /* SOF0 SOF1 ver type seq(2) len */
#define HEA_CRC_LEN 2u
#define HEA_MAX_FRAME (HEA_HEADER_LEN + HEA_MAX_PAYLOAD + HEA_CRC_LEN)

/* Message types. 0x0_: Linux -> STM32, 0x8_: STM32 -> Linux. */
enum hea_msg_type {
    HEA_MSG_HEARTBEAT = 0x01,  /* hea_heartbeat */
    HEA_MSG_INFERENCE = 0x02,  /* hea_inference */
    HEA_MSG_ECHO_REQ = 0x03,   /* hea_echo_req */
    HEA_MSG_NET_STATUS = 0x04, /* hea_net_status, optional v3 */
    HEA_MSG_ECHO_RESP = 0x83,  /* hea_echo_resp */
    HEA_MSG_INF_RX = 0x86, /* hea_inf_rx, optional v3 */
    HEA_MSG_STATE_EVENT = 0x85, /* hea_state_event, optional v3 */
    HEA_MSG_MCU_STATUS = 0x84, /* hea_mcu_status */
};

/*
 * Clock domains: fields ending in _ns use one Pi monotonic clock (planned CLOCK_MONOTONIC_RAW);
 * mcu_*_us fields are the STM32 free-running timer. age_at_send_us is a
 * Pi-local duration, not an MCU timestamp. Cross-domain timestamps are never
 * subtracted without an explicit offset/drift estimate.
 */
struct hea_heartbeat {
    uint64_t linux_send_ns;
    uint64_t session_id; /* Sender incarnation; present only in version 3. */
};

struct hea_inference {
    uint64_t linux_input_ns; /* input sample acquired (AoI origin) */
    uint64_t linux_done_ns;  /* inference finished */
    uint32_t input_seq;      /* sequence number of the input sample */
    uint8_t class_id;
    uint8_t confidence_pct;
    uint32_t age_at_send_us; /* Pi input-to-send duration; absent in version 1. */
};

struct hea_echo_req {
    uint64_t linux_t1_ns;
};

/* Sequence is the existing shared u16 frame header counter. Unknown/saturated
 * age and RTT are UINT32_MAX. linux_send_ns is gateway-local, never MCU time. */
struct hea_net_status {
    uint64_t linux_send_ns;
    uint32_t result_age_ms;
    uint16_t loss_per_mille; /* 0..1000, sequence gaps in the recent result window. */
    uint32_t rtt_p95_ms;
};

struct hea_echo_resp {
    uint64_t linux_t1_ns; /* phase-1, echoed unchanged */
    uint32_t mcu_rx_us;   /* phase-2: STM32 time when the request was received */
    uint32_t mcu_tx_us;   /* phase-3: STM32 time just before the response is sent */
};

enum hea_mcu_state {
    HEA_MCU_INIT = 0,     /* no valid data yet */
    HEA_MCU_FRESH = 1,
    HEA_MCU_HOLD = 2,     /* data stale, still within grace */
    HEA_MCU_FAILSAFE = 3, /* latched until explicit rearm */
};

struct hea_mcu_status {
    uint32_t mcu_us;
    uint8_t state; /* enum hea_mcu_state */
    uint16_t missed_heartbeats;
    uint16_t rx_crc_errors;
};

/* Optional v3 MCU_STATUS diagnostics; base packer stays nine bytes. */
struct hea_mcu_status_extended {
    struct hea_mcu_status base;
    uint32_t last_capture_us;
    uint32_t capture_count;
    uint32_t rx_dropped;
    uint32_t uart_errors;
    uint32_t queue_dropped;
    uint32_t capture_lost;
    uint8_t clock_fallback; /* HSE requested but startup failed; absent in 33-byte status. */
    uint32_t state_event_lost;
    uint32_t uart_ore, uart_fe, uart_ne, uart_pe;
    uint32_t max_irq_masked_cycles;
    int32_t tick_rate_ppm; /* (SysTick elapsed us / TIM2 elapsed us - 1) * 1e6. */
    uint16_t tick_window_ms; /* Zero / INT32_MIN ppm: startup sample unavailable. */
};

/* Extended TIM2 times; UINT64_MAX means no accepted frame in that clock history. */
struct hea_state_event {
    uint64_t mcu_us;
    uint8_t old_state, new_state, reason;
    uint64_t last_hb_mcu_us, last_inf_mcu_us;
    uint32_t lost_count;
};

/* One accepted inference, age at supervisor evaluation (not report TX). */
struct hea_inf_rx {
    uint16_t wire_seq;
    uint64_t age_mcu_us, mcu_rx_us;
    uint8_t state;
    uint32_t lost_count;
};

struct hea_frame {
    uint8_t version;
    uint8_t type;
    uint16_t seq;
    uint8_t len;
    uint8_t payload[HEA_MAX_PAYLOAD];
};

/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout. */
uint16_t hea_crc16(const uint8_t *data, size_t len);
uint16_t hea_crc16_update(uint16_t crc, uint8_t byte);
int hea_version_supported(uint8_t version);

/* Same Pi clock only. Round duration up to us and saturate at UINT32_MAX.
 * Return -1 on clock regression/NULL output, leaving output untouched. */
int hea_age_at_send_us(uint64_t input_ns, uint64_t send_ns, uint32_t *age_us);

/*
 * Encode a frame into out (capacity out_cap). Returns the number of bytes
 * written, or 0 if len > HEA_MAX_PAYLOAD or out_cap is too small.
 */
size_t hea_encode(uint8_t type, uint16_t seq, const uint8_t *payload, uint8_t len,
                  uint8_t *out, size_t out_cap);

/* Byte-at-a-time decoder; safe to call from a UART RX task or ISR. */
enum hea_parse_state {
    HEA_P_SOF0,
    HEA_P_SOF1,
    HEA_P_HEADER,
    HEA_P_PAYLOAD,
    HEA_P_CRC,
};

struct hea_parser_stats {
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t len_errors;     /* declared len > HEA_MAX_PAYLOAD */
    uint32_t version_errors; /* CRC ok but unknown version */
    uint32_t bytes_dropped;  /* bytes discarded while hunting for SOF */
};

struct hea_parser {
    enum hea_parse_state state;
    uint8_t hdr[HEA_HEADER_LEN - 2]; /* ver type seq_lo seq_hi len */
    uint8_t idx;
    uint16_t crc;
    uint8_t crc_rx[HEA_CRC_LEN];
    struct hea_frame frame;
    struct hea_parser_stats stats;
};

void hea_parser_init(struct hea_parser *p);

/*
 * Feed one byte. Returns 1 when a complete, CRC-valid frame is available in
 * p->frame (valid until the next call), 0 otherwise.
 */
int hea_parser_feed(struct hea_parser *p, uint8_t byte);

/* Payload (de)serialisers. pack_* return payload length; unpack_* return 0 on success. */
uint8_t hea_pack_heartbeat(const struct hea_heartbeat *m, uint8_t *buf);
int hea_unpack_heartbeat(const struct hea_frame *f, struct hea_heartbeat *m);
uint8_t hea_pack_inference(const struct hea_inference *m, uint8_t *buf);
int hea_unpack_inference(const struct hea_frame *f, struct hea_inference *m);
uint8_t hea_pack_net_status(const struct hea_net_status *m, uint8_t *buf);
int hea_unpack_net_status(const struct hea_frame *f, struct hea_net_status *m);
uint8_t hea_pack_echo_req(const struct hea_echo_req *m, uint8_t *buf);
int hea_unpack_echo_req(const struct hea_frame *f, struct hea_echo_req *m);
uint8_t hea_pack_echo_resp(const struct hea_echo_resp *m, uint8_t *buf);
int hea_unpack_echo_resp(const struct hea_frame *f, struct hea_echo_resp *m);
uint8_t hea_pack_mcu_status(const struct hea_mcu_status *m, uint8_t *buf);
int hea_unpack_mcu_status(const struct hea_frame *f, struct hea_mcu_status *m);

uint8_t hea_pack_mcu_status_extended(const struct hea_mcu_status_extended *m, uint8_t *buf);
int hea_unpack_mcu_status_extended(const struct hea_frame *f, struct hea_mcu_status_extended *m);

uint8_t hea_pack_inf_rx(const struct hea_inf_rx *m, uint8_t *buf);
int hea_unpack_inf_rx(const struct hea_frame *f, struct hea_inf_rx *m);

uint8_t hea_pack_state_event(const struct hea_state_event *m, uint8_t *buf);
int hea_unpack_state_event(const struct hea_frame *f, struct hea_state_event *m);

#ifdef __cplusplus
}
#endif

#endif /* HEA_PROTO_H */
