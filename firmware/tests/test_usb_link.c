#ifdef NDEBUG
#undef NDEBUG
#endif
#include "usb_link.h"

#include <assert.h>
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

static void test_round_trip_various_payloads(void)
{
    uint8_t wire[USB_LINK_MAX_WIRE_FRAME];
    uint8_t rx_payload[USB_LINK_MAX_PAYLOAD];
    size_t wire_len = 0u;

    /* 1. Empty payload (e.g. ping / zero-byte control frame) */
    UsbLinkHeader tx_hdr;
    tx_hdr.ver_major    = USB_LINK_VER_MAJOR;
    tx_hdr.ver_minor    = USB_LINK_VER_MINOR;
    tx_hdr.msg_type     = (uint8_t)USB_MSG_CMD_MODE;
    tx_hdr.flags        = USB_LINK_FLAG_ACK_REQ;
    tx_hdr.seq          = 42u;
    tx_hdr.timestamp_us = 0x0102030405060708ULL;
    tx_hdr.payload_len  = 0u;

    assert(usb_link_encode(&tx_hdr, NULL, wire, sizeof(wire), &wire_len) == USB_LINK_OK);
    assert(wire_len >= 2u);
    assert(wire[wire_len - 1u] == USB_LINK_DELIMITER);
    for (size_t i = 0u; i < wire_len - 1u; ++i) {
        assert(wire[i] != USB_LINK_DELIMITER);
    }

    UsbLinkHeader rx_hdr;
    memset(&rx_hdr, 0, sizeof(rx_hdr));
    assert(usb_link_decode(wire, wire_len, &rx_hdr, rx_payload, sizeof(rx_payload)) == USB_LINK_OK);
    assert(rx_hdr.ver_major == tx_hdr.ver_major);
    assert(rx_hdr.ver_minor == tx_hdr.ver_minor);
    assert(rx_hdr.msg_type == tx_hdr.msg_type);
    assert(rx_hdr.flags == tx_hdr.flags);
    assert(rx_hdr.seq == tx_hdr.seq);
    assert(rx_hdr.timestamp_us == tx_hdr.timestamp_us);
    assert(rx_hdr.payload_len == 0u);

    /* 2. Max payload (256 bytes, all non-zero to exercise 254-byte COBS block split) */
    uint8_t big_payload[USB_LINK_MAX_PAYLOAD];
    for (size_t i = 0u; i < sizeof(big_payload); ++i) {
        big_payload[i] = (uint8_t)((i % 255u) + 1u);
    }
    tx_hdr.msg_type    = (uint8_t)USB_MSG_TELEM_FAST;
    tx_hdr.flags       = USB_LINK_FLAG_BENCH_MODE;
    tx_hdr.seq         = 0xDEADBEEFu;
    tx_hdr.payload_len = (uint16_t)sizeof(big_payload);

    assert(usb_link_encode(&tx_hdr, big_payload, wire, sizeof(wire), &wire_len) == USB_LINK_OK);
    assert(usb_link_decode(wire, wire_len, &rx_hdr, rx_payload, sizeof(rx_payload)) == USB_LINK_OK);
    assert(rx_hdr.seq == 0xDEADBEEFu);
    assert(rx_hdr.payload_len == sizeof(big_payload));
    assert(memcmp(rx_payload, big_payload, sizeof(big_payload)) == 0);

    /* 3. All-zero payload (256 bytes) */
    memset(big_payload, 0x00, sizeof(big_payload));
    tx_hdr.seq += 1u;
    assert(usb_link_encode(&tx_hdr, big_payload, wire, sizeof(wire), &wire_len) == USB_LINK_OK);
    assert(usb_link_decode(wire, wire_len, &rx_hdr, rx_payload, sizeof(rx_payload)) == USB_LINK_OK);
    assert(memcmp(rx_payload, big_payload, sizeof(big_payload)) == 0);
}

static void test_error_detection(void)
{
    uint8_t payload[16];
    for (uint8_t i = 0u; i < 16u; ++i) {
        payload[i] = (uint8_t)(i + 1u);
    }
    uint8_t wire[USB_LINK_MAX_WIRE_FRAME];
    uint8_t out_payload[32];
    size_t wire_len = 0u;

    UsbLinkHeader hdr;
    hdr.ver_major    = USB_LINK_VER_MAJOR;
    hdr.ver_minor    = USB_LINK_VER_MINOR;
    hdr.msg_type     = (uint8_t)USB_MSG_CMD_TWIST;
    hdr.flags        = 0u;
    hdr.seq          = 7u;
    hdr.timestamp_us = 123456ULL;
    hdr.payload_len  = (uint16_t)sizeof(payload);

    assert(usb_link_encode(&hdr, payload, wire, sizeof(wire), &wire_len) == USB_LINK_OK);

    /* Bit flip in encoded payload -> CRC mismatch (or COBS error) */
    uint8_t corrupted[USB_LINK_MAX_WIRE_FRAME];
    memcpy(corrupted, wire, wire_len);
    corrupted[wire_len / 2u] ^= 0x01u;
    UsbLinkHeader rx_hdr;
    const UsbLinkStatus st = usb_link_decode(corrupted, wire_len, &rx_hdr,
                                             out_payload, sizeof(out_payload));
    assert(st == USB_LINK_ERR_CRC || st == USB_LINK_ERR_COBS);

    /* Flip a data byte cleanly inside payload (guaranteed CRC error, not COBS code byte) */
    memcpy(corrupted, wire, wire_len);
    corrupted[wire_len - 3u] ^= 0x04u;
    if (corrupted[wire_len - 3u] == 0u) {
        corrupted[wire_len - 3u] = 0x01u;
    }
    assert(usb_link_decode(corrupted, wire_len, &rx_hdr, out_payload, sizeof(out_payload))
           == USB_LINK_ERR_CRC);

    /* Incompatible ver_major -> USB_LINK_ERR_VERSION */
    hdr.ver_major = 2u;
    assert(usb_link_encode(&hdr, payload, wire, sizeof(wire), &wire_len) == USB_LINK_OK);
    assert(usb_link_decode(wire, wire_len, &rx_hdr, out_payload, sizeof(out_payload))
           == USB_LINK_ERR_VERSION);

    /* Oversized payload on encode */
    hdr.ver_major = USB_LINK_VER_MAJOR;
    hdr.payload_len = (uint16_t)(USB_LINK_MAX_PAYLOAD + 1u);
    assert(usb_link_encode(&hdr, payload, wire, sizeof(wire), &wire_len)
           == USB_LINK_ERR_PAYLOAD_TOO_LARGE);

    /* Output payload buffer too small on decode */
    hdr.payload_len = (uint16_t)sizeof(payload);
    assert(usb_link_encode(&hdr, payload, wire, sizeof(wire), &wire_len) == USB_LINK_OK);
    assert(usb_link_decode(wire, wire_len, &rx_hdr, out_payload, 8u)
           == USB_LINK_ERR_BUFFER_TOO_SMALL);
}

static void test_streaming_rx_parser(void)
{
    UsbLinkRxParser rx;
    usb_link_rx_init(&rx);

    UsbLinkHeader tx1 = { USB_LINK_VER_MAJOR, USB_LINK_VER_MINOR,
                          (uint8_t)USB_MSG_TIME_SYNC_REQ, 0u, 100u, 5000ULL, 8u };
    const uint8_t p1[8] = { 1u, 0u, 2u, 0u, 3u, 0u, 4u, 0u };
    uint8_t wire1[USB_LINK_MAX_WIRE_FRAME];
    size_t len1 = 0u;
    assert(usb_link_encode(&tx1, p1, wire1, sizeof(wire1), &len1) == USB_LINK_OK);

    UsbLinkHeader tx2 = { USB_LINK_VER_MAJOR, USB_LINK_VER_MINOR,
                          (uint8_t)USB_MSG_TIME_SYNC_RESP, 1u, 101u, 6000ULL, 4u };
    const uint8_t p2[4] = { 0xAAu, 0xBBu, 0xCCu, 0xDDu };
    uint8_t wire2[USB_LINK_MAX_WIRE_FRAME];
    size_t len2 = 0u;
    assert(usb_link_encode(&tx2, p2, wire2, sizeof(wire2), &len2) == USB_LINK_OK);

    UsbLinkHeader rx_hdr;
    uint8_t rx_payload[64];

    /* Leading 0x00 noise bytes are ignored cleanly */
    assert(usb_link_rx_feed(&rx, 0x00u, &rx_hdr, rx_payload, sizeof(rx_payload))
           == USB_LINK_NEED_MORE);
    assert(usb_link_rx_feed(&rx, 0x00u, &rx_hdr, rx_payload, sizeof(rx_payload))
           == USB_LINK_NEED_MORE);

    /* Feed frame 1 byte-by-byte */
    UsbLinkStatus st = USB_LINK_NEED_MORE;
    for (size_t i = 0u; i < len1; ++i) {
        st = usb_link_rx_feed(&rx, wire1[i], &rx_hdr, rx_payload, sizeof(rx_payload));
        if (i + 1u < len1) {
            assert(st == USB_LINK_NEED_MORE);
        }
    }
    assert(st == USB_LINK_OK);
    assert(rx_hdr.seq == 100u && rx_hdr.payload_len == 8u);
    assert(memcmp(rx_payload, p1, 8u) == 0);
    assert(rx.frames_ok == 1u);

    /* Feed an overflowing stream of non-zero bytes followed by 0x00 -> overflow error */
    for (size_t i = 0u; i < USB_LINK_MAX_WIRE_FRAME + 20u; ++i) {
        assert(usb_link_rx_feed(&rx, 0x55u, &rx_hdr, rx_payload, sizeof(rx_payload))
               == USB_LINK_NEED_MORE);
    }
    assert(usb_link_rx_feed(&rx, 0x00u, &rx_hdr, rx_payload, sizeof(rx_payload))
           == USB_LINK_ERR_OVERFLOW);
    assert(rx.overflow_errors == 1u);

    /* Parser recovers immediately on next valid frame */
    for (size_t i = 0u; i < len2; ++i) {
        st = usb_link_rx_feed(&rx, wire2[i], &rx_hdr, rx_payload, sizeof(rx_payload));
    }
    assert(st == USB_LINK_OK);
    assert(rx_hdr.seq == 101u && rx_hdr.payload_len == 4u);
    assert(memcmp(rx_payload, p2, 4u) == 0);
    assert(rx.frames_ok == 2u);
}

int main(void)
{
    test_round_trip_various_payloads();
    test_error_detection();
    test_streaming_rx_parser();
    printf("test_usb_link: ALL PASSED\n");
    return 0;
}
