#include "usb_link.h"

#include "cobs.h"
#include "crc32c.h"

#include <string.h>

static void put_u16_le(uint8_t *dst, uint16_t v)
{
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8u) & 0xFFu);
}

static void put_u32_le(uint8_t *dst, uint32_t v)
{
    dst[0] = (uint8_t)(v & 0xFFu);
    dst[1] = (uint8_t)((v >> 8u) & 0xFFu);
    dst[2] = (uint8_t)((v >> 16u) & 0xFFu);
    dst[3] = (uint8_t)((v >> 24u) & 0xFFu);
}

static void put_u64_le(uint8_t *dst, uint64_t v)
{
    for (uint32_t i = 0u; i < 8u; ++i) {
        dst[i] = (uint8_t)((v >> (i * 8u)) & 0xFFu);
    }
}

static uint16_t get_u16_le(const uint8_t *src)
{
    return (uint16_t)((uint16_t)src[0] | ((uint16_t)src[1] << 8u));
}

static uint32_t get_u32_le(const uint8_t *src)
{
    return (uint32_t)src[0]
         | ((uint32_t)src[1] << 8u)
         | ((uint32_t)src[2] << 16u)
         | ((uint32_t)src[3] << 24u);
}

static uint64_t get_u64_le(const uint8_t *src)
{
    uint64_t v = 0u;
    for (uint32_t i = 0u; i < 8u; ++i) {
        v |= ((uint64_t)src[i] << (i * 8u));
    }
    return v;
}

UsbLinkStatus usb_link_encode(const UsbLinkHeader *hdr,
                              const uint8_t *payload,
                              uint8_t *out_wire, size_t out_cap,
                              size_t *out_len)
{
    if (hdr == NULL || out_wire == NULL || out_len == NULL) {
        return USB_LINK_ERR_NULL_PTR;
    }
    *out_len = 0u;
    if (hdr->payload_len > USB_LINK_MAX_PAYLOAD) {
        return USB_LINK_ERR_PAYLOAD_TOO_LARGE;
    }
    if (hdr->payload_len > 0u && payload == NULL) {
        return USB_LINK_ERR_NULL_PTR;
    }

    uint8_t raw[USB_LINK_MAX_RAW_FRAME];
    raw[0] = hdr->ver_major;
    raw[1] = hdr->ver_minor;
    raw[2] = hdr->msg_type;
    raw[3] = hdr->flags;
    put_u32_le(&raw[4], hdr->seq);
    put_u64_le(&raw[8], hdr->timestamp_us);
    put_u16_le(&raw[16], hdr->payload_len);

    if (hdr->payload_len > 0u) {
        memcpy(&raw[USB_LINK_HEADER_SIZE], payload, (size_t)hdr->payload_len);
    }

    const size_t body_len = USB_LINK_HEADER_SIZE + (size_t)hdr->payload_len;
    const uint32_t crc = crc32c_compute(raw, body_len);
    put_u32_le(&raw[body_len], crc);
    const size_t raw_len = body_len + USB_LINK_CRC_SIZE;

    if (out_cap < 2u) {
        return USB_LINK_ERR_BUFFER_TOO_SMALL;
    }

    size_t cobs_len = 0u;
    const CobsStatus cs = cobs_encode(raw, raw_len, out_wire, out_cap - 1u, &cobs_len);
    if (cs == COBS_ERR_BUFFER_TOO_SMALL) {
        return USB_LINK_ERR_BUFFER_TOO_SMALL;
    }
    if (cs != COBS_OK) {
        return USB_LINK_ERR_COBS;
    }

    out_wire[cobs_len] = USB_LINK_DELIMITER;
    *out_len = cobs_len + 1u;
    return USB_LINK_OK;
}

UsbLinkStatus usb_link_decode(const uint8_t *wire_buf, size_t wire_len,
                              UsbLinkHeader *out_hdr,
                              uint8_t *out_payload, size_t out_payload_cap)
{
    if (wire_buf == NULL || out_hdr == NULL) {
        return USB_LINK_ERR_NULL_PTR;
    }

    size_t cobs_len = wire_len;
    if (cobs_len > 0u && wire_buf[cobs_len - 1u] == USB_LINK_DELIMITER) {
        cobs_len -= 1u;
    }
    if (cobs_len == 0u) {
        return USB_LINK_ERR_COBS;
    }

    uint8_t raw[USB_LINK_MAX_RAW_FRAME];
    size_t raw_len = 0u;
    const CobsStatus cs = cobs_decode(wire_buf, cobs_len, raw, sizeof(raw), &raw_len);
    if (cs == COBS_ERR_BUFFER_TOO_SMALL) {
        return USB_LINK_ERR_OVERFLOW;
    }
    if (cs != COBS_OK) {
        return USB_LINK_ERR_COBS;
    }

    if (raw_len < (USB_LINK_HEADER_SIZE + USB_LINK_CRC_SIZE)) {
        return USB_LINK_ERR_LENGTH;
    }

    const size_t body_len = raw_len - USB_LINK_CRC_SIZE;
    const uint32_t expected_crc = get_u32_le(&raw[body_len]);
    const uint32_t actual_crc = crc32c_compute(raw, body_len);
    if (actual_crc != expected_crc) {
        return USB_LINK_ERR_CRC;
    }

    UsbLinkHeader hdr;
    hdr.ver_major    = raw[0];
    hdr.ver_minor    = raw[1];
    hdr.msg_type     = raw[2];
    hdr.flags        = raw[3];
    hdr.seq          = get_u32_le(&raw[4]);
    hdr.timestamp_us = get_u64_le(&raw[8]);
    hdr.payload_len  = get_u16_le(&raw[16]);

    if (hdr.ver_major != USB_LINK_VER_MAJOR) {
        return USB_LINK_ERR_VERSION;
    }

    const size_t avail_payload = body_len - USB_LINK_HEADER_SIZE;
    if (hdr.ver_minor == USB_LINK_VER_MINOR) {
        if ((size_t)hdr.payload_len != avail_payload) {
            return USB_LINK_ERR_LENGTH;
        }
    } else {
        /* Backward-compatible minor extension: ignore extra trailing bytes. */
        if ((size_t)hdr.payload_len > avail_payload) {
            return USB_LINK_ERR_LENGTH;
        }
    }

    if (hdr.payload_len > 0u) {
        if (out_payload == NULL) {
            return USB_LINK_ERR_NULL_PTR;
        }
        if ((size_t)hdr.payload_len > out_payload_cap) {
            return USB_LINK_ERR_BUFFER_TOO_SMALL;
        }
        memcpy(out_payload, &raw[USB_LINK_HEADER_SIZE], (size_t)hdr.payload_len);
    }

    *out_hdr = hdr;
    return USB_LINK_OK;
}

void usb_link_rx_init(UsbLinkRxParser *rx)
{
    if (rx == NULL) {
        return;
    }
    memset(rx, 0, sizeof(*rx));
}

UsbLinkStatus usb_link_rx_feed(UsbLinkRxParser *rx, uint8_t byte,
                               UsbLinkHeader *out_hdr,
                               uint8_t *out_payload, size_t out_payload_cap)
{
    if (rx == NULL || out_hdr == NULL) {
        return USB_LINK_ERR_NULL_PTR;
    }

    if (byte == USB_LINK_DELIMITER) {
        if (rx->overflowed != 0u) {
            rx->overflowed = 0u;
            rx->len = 0u;
            rx->overflow_errors += 1u;
            return USB_LINK_ERR_OVERFLOW;
        }
        if (rx->len == 0u) {
            /* Leading or back-to-back delimiter: flush line noise. */
            return USB_LINK_NEED_MORE;
        }
        const UsbLinkStatus st = usb_link_decode(rx->buf, rx->len, out_hdr,
                                                 out_payload, out_payload_cap);
        rx->len = 0u;
        if (st == USB_LINK_OK) {
            rx->frames_ok += 1u;
        } else if (st == USB_LINK_ERR_CRC) {
            rx->crc_errors += 1u;
        } else {
            rx->proto_errors += 1u;
        }
        return st;
    }

    if (rx->overflowed != 0u) {
        return USB_LINK_NEED_MORE;
    }

    if (rx->len >= (USB_LINK_MAX_WIRE_FRAME - 1u)) {
        rx->overflowed = 1u;
        return USB_LINK_NEED_MORE;
    }

    rx->buf[rx->len] = byte;
    rx->len += 1u;
    return USB_LINK_NEED_MORE;
}
