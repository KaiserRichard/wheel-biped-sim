#include "bench.h"
#include <assert.h>
#include <string.h>

static void discard_tx(void *ctx, const uint8_t *data, size_t n) {
 (void)ctx; (void)data; (void)n;
}

static size_t advisory_wire(uint8_t *wire, uint32_t seq, uint8_t value) {
 lp_message m;
 memset(&m, 0, sizeof(m));
 m.type = LP_TERRAIN_ADVISORY;
 m.session = BENCH_SESSION;
 m.seq = seq;
 m.data.advisory.advisory = value;
 return lp_pack(&m, wire, LP_MAX_WIRE);
}

int main(void) {
 static const uint8_t expected[] = {1u, 2u, 4u, 8u};
 bench_port p;
 uint8_t wire[LP_MAX_WIRE];
 size_t n;
 uint64_t now = 1000u;
 unsigned i;

 bench_init(&p, 1u, discard_tx, NULL);
 assert(bench_led_mask(&p) == 0u); /* no advisory / no link */
 for (i = 0; i < 4; ++i) {
  n = advisory_wire(wire, i + 1u, (uint8_t)i);
  assert(n > 1u);
  bench_rx(&p, wire, n, now += 100u);
  assert(bench_led_mask(&p) == expected[i]);
 }

 /* A damaged CRC is rejected without replacing the accepted advisory. */
 n = advisory_wire(wire, 5u, 0u);
 assert(n > 2u);
 wire[n - 2u] ^= 0x40u;
 bench_rx(&p, wire, n, now += 100u);
 assert(bench_led_mask(&p) == expected[3]);

 /* A valid packet with an old sequence is also rejected without changing it. */
 n = advisory_wire(wire, 3u, 2u);
 bench_rx(&p, wire, n, now += 100u);
 assert(bench_led_mask(&p) == expected[3]);

 bench_poll(&p, now + BENCH_ADVISORY_STALE_US);
 assert(bench_led_mask(&p) == 0u);
 return 0;
}
