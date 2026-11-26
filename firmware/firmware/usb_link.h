#ifndef WBR_USB_LINK_H
#define WBR_USB_LINK_H

#include <stddef.h>
#include <stdint.h>

/* Wire protocol constants from FIRMWARE_DESIGN_v2.md Section 7. */
#define USB_LINK_VER_MAJOR         1u
#define USB_LINK_VER_MINOR         0u
#define USB_LINK_HEADER_SIZE       18u
#define USB_LINK_CRC_SIZE          4u
#define USB_LINK_MAX_PAYLOAD       256u
#define USB_LINK_MAX_RAW_FRAME     (USB_LINK_HEADER_SIZE + USB_LINK_MAX_PAYLOAD + USB_LINK_CRC_SIZE)
/* COBS worst-case + 1 byte trailing 0x00 delimiter */
#define USB_LINK_MAX_WIRE_FRAME    (1u + USB_LINK_MAX_RAW_FRAME + (USB_LINK_MAX_RAW_FRAME / 254u) + 1u)
#define USB_LINK_DELIMITER         0x00u

#define USB_LINK_FLAG_ACK_REQ      (1u << 0u)
#define USB_LINK_FLAG_BENCH_MODE   (1u << 1u)

typedef enum {
    USB_MSG_CMD_TWIST      = 0x01u,
    USB_MSG_CMD_MODE       = 0x02u,
    USB_MSG_TIME_SYNC_REQ  = 0x03u,
    USB_MSG_PARAM_OP       = 0x04u,
    USB_MSG_CMD_MOTOR_ID   = 0x05u, /* FW10: MOTOR_ID image only, see bringup_mode.h */
    USB_MSG_TELEM_FAST     = 0x81u,
    USB_MSG_TELEM_HEALTH   = 0x82u,
    USB_MSG_TIME_SYNC_RESP = 0x83u,
    USB_MSG_FAULT_SNAPSHOT = 0x84u,
    USB_MSG_BENCH_RAW_LOG  = 0x85u
} UsbMsgType;

typedef enum {
    USB_LINK_OK = 0,
    USB_LINK_NEED_MORE = 1,
    USB_LINK_ERR_NULL_PTR = 2,
    USB_LINK_ERR_PAYLOAD_TOO_LARGE = 3,
    USB_LINK_ERR_BUFFER_TOO_SMALL = 4,
    USB_LINK_ERR_COBS = 5,
    USB_LINK_ERR_LENGTH = 6,
    USB_LINK_ERR_CRC = 7,
    USB_LINK_ERR_VERSION = 8,
    USB_LINK_ERR_OVERFLOW = 9
} UsbLinkStatus;

typedef struct {
    uint8_t  ver_major;
    uint8_t  ver_minor;
    uint8_t  msg_type;
    uint8_t  flags;
    uint32_t seq;
    uint64_t timestamp_us;
    uint16_t payload_len;
} UsbLinkHeader;

typedef struct {
    uint8_t  buf[USB_LINK_MAX_WIRE_FRAME];
    size_t   len;
    uint8_t  overflowed;
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t proto_errors;
    uint32_t overflow_errors;
} UsbLinkRxParser;

/* Encode header + payload + CRC-32C -> COBS + trailing 0x00 delimiter. */
UsbLinkStatus usb_link_encode(const UsbLinkHeader *hdr,
                              const uint8_t *payload,
                              uint8_t *out_wire, size_t out_cap,
                              size_t *out_len);

/* Decode one COBS-encoded frame (with or without trailing 0x00 delimiter).
 * Verifies COBS integrity, CRC-32C, ver_major == USB_LINK_VER_MAJOR, and payload length. */
UsbLinkStatus usb_link_decode(const uint8_t *wire_buf, size_t wire_len,
                              UsbLinkHeader *out_hdr,
                              uint8_t *out_payload, size_t out_payload_cap);

/* Streaming byte-by-byte receiver using a static buffer inside UsbLinkRxParser. */
void usb_link_rx_init(UsbLinkRxParser *rx);
UsbLinkStatus usb_link_rx_feed(UsbLinkRxParser *rx, uint8_t byte,
                               UsbLinkHeader *out_hdr,
                               uint8_t *out_payload, size_t out_payload_cap);

#endif /* WBR_USB_LINK_H */
