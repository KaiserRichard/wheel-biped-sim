#ifndef PH_RX_H
#define PH_RX_H

#include <stdint.h>
#include "../perception_health.h"

#define PH_RX_QUEUE_CAPACITY 4u
#define PH_HEARTBEAT_TYPE 0x01u
#define PH_RX_MAX_FRAME 73u /* 7-byte header + 64-byte maximum payload + CRC */

enum ph_rx_kind { PH_RX_NONE = 0, PH_RX_HEALTH = 1, PH_RX_HEARTBEAT = 2 };
struct ph_rx_event {
    enum ph_rx_kind kind;
    uint16_t seq;
    uint32_t received_at_mcu_us;
    uint64_t session_id;
    struct ph_health health;
};
struct ph_rx_stats { uint32_t crc_errors, frame_errors, ignored_types, queue_drops; };
struct ph_rx {
    uint8_t frame[PH_RX_MAX_FRAME];
    uint8_t used;
    uint8_t expected;
    uint32_t current_byte_mcu_us;
    struct ph_rx_event queue[PH_RX_QUEUE_CAPACITY];
    /* SPSC indices: producer is the UART RX ISR; consumer is one task. The
     * platform must provide naturally atomic 8-bit accesses and prevent a
     * second producer/consumer. Do not call pop from an ISR or feed from tasks. */
    volatile uint8_t head, tail;
    struct ph_rx_stats stats;
};
void ph_rx_init(struct ph_rx *rx);
/* Feed one UART byte from exactly one producer context (normally RX ISR). */
void ph_rx_feed(struct ph_rx *rx, uint8_t byte);
/* Preferred ISR entry: supply the MCU timer sample captured with this byte.
 * The complete-frame event retains the sample from its final wire byte. */
void ph_rx_feed_at(struct ph_rx *rx, uint8_t byte, uint32_t received_at_mcu_us);
/* Nonblocking single consumer handoff. Returns 1 when an event was copied. */
int ph_rx_pop(struct ph_rx *rx, struct ph_rx_event *out);
#endif
