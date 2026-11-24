#include "ph_rx.h"
#include <string.h>

static uint16_t get16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8)); }
static uint64_t get64(const uint8_t *p)
{
    uint64_t v = 0u; unsigned i;
    for (i = 0u; i < 8u; ++i) v |= (uint64_t)p[i] << (8u * i);
    return v;
}

void ph_rx_init(struct ph_rx *rx) { if (rx != NULL) memset(rx, 0, sizeof(*rx)); }

static void enqueue(struct ph_rx *rx, const struct ph_rx_event *ev)
{
    uint8_t next = (uint8_t)((rx->head + 1u) % PH_RX_QUEUE_CAPACITY);
    if (next == rx->tail) { if (rx->stats.queue_drops != UINT32_MAX) ++rx->stats.queue_drops; return; }
    rx->queue[rx->head] = *ev;
    rx->head = next; /* publish only after the complete fixed-size copy */
}

static void complete(struct ph_rx *rx)
{
    struct ph_rx_event ev;
    uint16_t wire_crc = get16(rx->frame + rx->expected - 2u);
    memset(&ev, 0, sizeof(ev));
    ev.received_at_mcu_us = rx->current_byte_mcu_us;
    if (rx->frame[2] != PH_VERSION || wire_crc != ph_crc16(rx->frame + 2u, (size_t)rx->expected - 4u)) {
        if (rx->stats.crc_errors != UINT32_MAX) ++rx->stats.crc_errors;
        return;
    }
    ev.seq = get16(rx->frame + 4u);
    if (rx->frame[3] == PH_TYPE && rx->frame[6] == PH_PAYLOAD_LEN &&
        ph_decode(rx->frame, rx->expected, &ev.seq, &ev.health)) {
        ev.kind = PH_RX_HEALTH; enqueue(rx, &ev);
    } else if (rx->frame[3] == PH_HEARTBEAT_TYPE && rx->frame[6] == 16u) {
        /* v3 HEARTBEAT payload: Pi send timestamp then sender session id.
         * Both are opaque to MCU timing; never compare them to local ticks. */
        ev.kind = PH_RX_HEARTBEAT; ev.session_id = get64(rx->frame + 15u); enqueue(rx, &ev);
    } else if (rx->stats.ignored_types != UINT32_MAX) ++rx->stats.ignored_types;
}

void ph_rx_feed_at(struct ph_rx *rx, uint8_t b, uint32_t received_at_mcu_us)
{
    if (rx == NULL) return;
    rx->current_byte_mcu_us = received_at_mcu_us;
    if (rx->used == 0u) {
        if (b == 0xa5u) { rx->frame[0] = b; rx->used = 1u; }
        return;
    }
    if (rx->used == 1u && b != 0x5au) {
        if (b == 0xa5u) rx->frame[0] = b; else rx->used = 0u;
        return;
    }
    rx->frame[rx->used++] = b;
    if (rx->used == 7u) {
        uint8_t len = rx->frame[6];
        if (len > 64u) { if (rx->stats.frame_errors != UINT32_MAX) ++rx->stats.frame_errors; rx->used = 0u; return; }
        rx->expected = (uint8_t)(9u + len);
    }
    if (rx->used != 0u && rx->expected != 0u && rx->used == rx->expected) {
        complete(rx); rx->used = 0u; rx->expected = 0u;
    }
    if (rx->used >= sizeof(rx->frame)) { if (rx->stats.frame_errors != UINT32_MAX) ++rx->stats.frame_errors; rx->used=0u; rx->expected=0u; }
}

void ph_rx_feed(struct ph_rx *rx, uint8_t b) { ph_rx_feed_at(rx, b, 0u); }

int ph_rx_pop(struct ph_rx *rx, struct ph_rx_event *out)
{
    uint8_t tail;
    if (rx == NULL || out == NULL) return 0;
    tail = rx->tail;
    if (tail == rx->head) return 0;
    *out = rx->queue[tail];
    rx->tail = (uint8_t)((tail + 1u) % PH_RX_QUEUE_CAPACITY);
    return 1;
}
