/* Bench-only Pi<->MCU link test logic (Nucleo-F446RE stand-in for the STM32H7).
 * DIAGNOSTIC tooling around the CANDIDATE codec in ../../link_proto. No balance,
 * actuator or controller code. Portable C99: built for the MCU and for the host
 * simulator in ../host. */
#ifndef LINK_BENCH_H
#define LINK_BENCH_H
#include <stddef.h>
#include <stdint.h>
#include "link_proto.h"

/* Test-only provisioning. The SPEC requires a trusted local bind; on the bench
 * the "trusted action" is flashing this firmware with these constants, and the
 * Pi script uses the same values. Not a real session/mapping handshake. */
#define BENCH_SESSION 0x5742524C494E4B31ULL /* "WBRLINK1" */
#define BENCH_MAPPING_REVISION 0xF446u
#define BENCH_TRANSFORM_REVISION 0xF446u
#define BENCH_STATUS_PERIOD_US 50000u /* 20 Hz heartbeat status, SPEC rate */
#define BENCH_ADVISORY_STALE_US 300000u
#define BENCH_ADVISORY_ECHO_PERIOD_US 100000u

typedef void (*bench_tx_fn)(void *ctx, const uint8_t *data, size_t n);
/* Optional hardware-timing hook: called once per packet that passed COBS/CRC/layout
 * decoding, before the receiver policy runs (firmware toggles a marker pin). */
typedef void (*bench_mark_fn)(void *ctx);

typedef struct {
 lp_stream stream;
 lp_receiver rx;
 uint64_t boot_id;      /* McuStatus header session (MCU boot ID) */
 uint32_t status_seq;   /* McuStatus header sequence */
 uint64_t now_us;       /* MCU monotonic us of the event being processed */
 uint64_t action_us;    /* last fallback action (rejection or entry into a withdraw state) */
 uint64_t last_status_us;
 lp_state reported_state;
 uint32_t accepted, rejected, status_sent, tx_dropped;
 uint32_t advisory_seq, advisory_echo_seq, advisory_accepted, advisory_rejected;
 uint64_t advisory_last_us, advisory_boot_us, advisory_echo_us;
 uint8_t advisory_value, advisory_stale, advisory_have_seq;
 bench_tx_fn tx;
 void *tx_ctx;
 bench_mark_fn mark; /* may be NULL; set after bench_init */
} bench_port;

void bench_init(bench_port *p, uint64_t boot_id, bench_tx_fn tx, void *tx_ctx);
/* Feed received bytes; every decoded packet or wire error gets an immediate
 * McuStatus reply. now_us is MCU boot monotonic microseconds. */
void bench_rx(bench_port *p, const uint8_t *data, size_t n, uint64_t now_us);
/* Call as often as possible: runs the silence/staleness rule, sends a status
 * on any state change and a 20 Hz heartbeat status. */
void bench_poll(bench_port *p, uint64_t now_us);
/* 1 when the receiver holds an accepted link (any state except NO_LINK). */
int bench_linked(const bench_port *p);
/* One-hot LED mask for the most recently accepted, non-stale advisory.
 * Bit 0=GREEN, 1=YELLOW, 2=RED, 3=BLUE. Zero means no live advisory. */
uint8_t bench_led_mask(const bench_port *p);
#endif
