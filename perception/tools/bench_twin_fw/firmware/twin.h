#ifndef BENCH_TWIN_H
#define BENCH_TWIN_H
#include <stddef.h>
#include <stdint.h>
#include "dm.h"
#include "link_proto.h"
/* SOURCE-SUPPORTED bench configuration: tools/link_bench/firmware/bench.h:14,18.
 * Bench provisioning, not a real session handshake. */
#define TWIN_SESSION 0x5742524c494e4b31ULL
#define TWIN_STALE_MS 300u
#define TWIN_FEEDBACK_BASE 0x700u /* ASSUMED arbitration IDs, not DM-jump configuration */
#define TWIN_LINE_CAP 128u
typedef struct { uint32_t seq; uint8_t id, special; dm_command cmd; } twin_command;
int twin_parse(const char *line,twin_command *c);
size_t twin_format(char *out,size_t cap,uint32_t ms,const dm_feedback *f);
typedef void (*twin_tx)(void *ctx,const uint8_t *d,size_t n);
typedef void (*twin_on_command)(void *ctx,const twin_command *c);
typedef struct {
 lp_stream stream;
 uint32_t now,last,echo_at,seq,echo_seq,accepted,rejected,parse_errors,other;
 uint32_t text_ok,rx_gap,mux_timeout,binary_bytes,binary_overflow,echo_encode_drop,text_overflow,byte_at;
 uint64_t boot_id,uptime_ms;
 uint8_t have,value,mode,overflow;
 size_t used;
 char line[TWIN_LINE_CAP];
 twin_tx tx;
 twin_on_command command;
 void *ctx;
} twin_link;
void twin_init(twin_link *s,uint64_t boot_id,twin_tx tx,twin_on_command command,void *ctx);
void twin_feed(twin_link *s,uint8_t byte,uint32_t now);
void twin_rx_gap(twin_link *s);
void twin_poll(twin_link *s,uint32_t now);
uint8_t twin_leds(const twin_link *s);
#endif
