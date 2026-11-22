/* See bench.h. Status fields follow SPEC.md McuStatus; controller-owned fields
 * (faults, fsm, armed, operator_enable, battery) are zero/invalid because this
 * bench MCU has no controller. */
#include "bench.h"
#include <string.h>

static void send_status(bench_port *p) {
 lp_message m; lp_status *s = &m.data.status; uint8_t wire[LP_MAX_WIRE]; size_t n;
 memset(&m, 0, sizeof(m));
 m.type = LP_MCU_STATUS; m.session = p->boot_id; m.seq = ++p->status_seq;
 s->sample_mcu_ns = p->now_us * 1000u;
 s->last_health_seq = p->rx.health_seq;
 s->last_estimate_seq = p->rx.estimate_seq;
 s->last_ref_seq = p->rx.ref_seq;
 s->action_mcu_ns = p->action_us * 1000u;
 s->state = (uint8_t)p->rx.state;
 s->reason = (uint8_t)p->rx.reason;
 n = lp_pack(&m, wire, sizeof(wire));
 if (n == 0) { p->tx_dropped++; return; }
 p->tx(p->tx_ctx, wire, n);
 p->status_sent++;
 p->last_status_us = p->now_us;
 p->reported_state = p->rx.state;
}

static int advisory_stale(bench_port *p) {
 uint64_t reference = p->advisory_last_us ? p->advisory_last_us : p->advisory_boot_us;
 p->advisory_stale = (uint8_t)(p->now_us - reference >= BENCH_ADVISORY_STALE_US);
 return p->advisory_stale;
}

static void send_echo(bench_port *p, uint32_t seq, uint8_t advisory, lp_reason result) {
 lp_message m; lp_advisory_echo *e=&m.data.echo; uint8_t wire[LP_MAX_WIRE]; size_t n;
 memset(&m,0,sizeof(m)); m.type=LP_ADVISORY_ECHO; m.session=p->boot_id; m.seq=++p->advisory_echo_seq;
 e->sample_mcu_ns=p->now_us*1000u; e->advisory_seq=seq; e->advisory=advisory;
 e->result=(uint8_t)result; e->accepted_count=p->advisory_accepted; e->rejected_count=p->advisory_rejected;
 e->stale=(uint8_t)advisory_stale(p);
 e->ms_since_accepted=p->advisory_last_us ? (uint32_t)((p->now_us-p->advisory_last_us)/1000u) : UINT32_MAX;
 n=lp_pack(&m,wire,sizeof(wire));
 if(!n) { p->tx_dropped++; return; }
 p->tx(p->tx_ctx,wire,n); p->advisory_echo_us=p->now_us;
}

static int withdrawn(lp_state s) { return s == LP_NO_LINK || s == LP_STALE || s == LP_REJECTED; }

static void on_packet(void *ctx, const lp_message *m) {
 bench_port *p = ctx;
 if(m->type==LP_TERRAIN_ADVISORY) {
  const lp_advisory *a=&m->data.advisory; lp_reason why=LP_REASON_NONE;
  if(p->mark) p->mark(p->tx_ctx);
  if(m->session!=BENCH_SESSION) why=LP_REASON_SESSION;
  else if(p->advisory_have_seq && !lp_seq_newer(m->seq,p->advisory_seq)) why=LP_REASON_SEQUENCE;
  else if(a->advisory>3 || a->plane_valid>1) why=LP_REASON_FIELDS;
  if(why==LP_REASON_NONE) {
   p->advisory_seq=m->seq; p->advisory_have_seq=1; p->advisory_value=a->advisory;
   p->advisory_last_us=p->now_us; p->advisory_accepted++; p->accepted++;
  } else { p->advisory_rejected++; p->rejected++; }
  send_echo(p,m->seq,a->advisory,why); return;
 }
 if (p->mark) p->mark(p->tx_ctx);
 if (lp_receiver_accept(&p->rx, (uint32_t)p->now_us, m)) p->accepted++;
 else { p->rejected++; p->action_us = p->now_us; }
 send_status(p);
}

static void on_error(void *ctx) {
 bench_port *p = ctx;
 lp_reason why=p->stream.last_reason;
 lp_receiver_reject(&p->rx, why);
 p->rejected++; p->action_us = p->now_us;
 if(p->stream.last_type==LP_TERRAIN_ADVISORY) {
  p->advisory_rejected++;
  send_echo(p,p->stream.last_seq,p->stream.last_advisory,why);
 }
 send_status(p);
}

void bench_init(bench_port *p, uint64_t boot_id, bench_tx_fn tx, void *tx_ctx) {
 memset(p, 0, sizeof(*p));
 p->boot_id = boot_id; p->tx = tx; p->tx_ctx = tx_ctx;
 lp_stream_init(&p->stream);
 lp_receiver_bind(&p->rx, BENCH_SESSION, BENCH_MAPPING_REVISION, BENCH_TRANSFORM_REVISION);
 p->reported_state = p->rx.state;
}

void bench_rx(bench_port *p, const uint8_t *data, size_t n, uint64_t now_us) {
 p->now_us = now_us;
 lp_stream_feed(&p->stream, data, n, on_packet, on_error, p);
}

void bench_poll(bench_port *p, uint64_t now_us) {
 lp_state before = p->rx.state;
 p->now_us = now_us;
 if(!p->advisory_boot_us) p->advisory_boot_us=now_us;
 (void)advisory_stale(p);
 lp_receiver_update(&p->rx, (uint32_t)now_us);
 if (p->rx.state != before && withdrawn(p->rx.state)) p->action_us = now_us;
 if (p->rx.state != p->reported_state || now_us - p->last_status_us >= BENCH_STATUS_PERIOD_US)
  send_status(p);
 if(p->advisory_stale && now_us-p->advisory_echo_us>=BENCH_ADVISORY_ECHO_PERIOD_US)
  send_echo(p,p->advisory_seq,p->advisory_value,LP_REASON_NONE);
}

int bench_linked(const bench_port *p) { return p->rx.state != LP_NO_LINK; }

uint8_t bench_led_mask(const bench_port *p) {
 if (!p->advisory_have_seq || p->advisory_stale ||
     p->now_us - p->advisory_last_us >= BENCH_ADVISORY_STALE_US)
  return 0;
 switch (p->advisory_value) {
  case 0: return 1u << 0; /* NONE -> GREEN */
  case 1: return 1u << 1; /* JUMP_CANDIDATE -> YELLOW */
  case 2: return 1u << 2; /* STOP_CANDIDATE -> RED */
  case 3: return 1u << 3; /* UNCERTAIN -> BLUE */
  default: return 0;
 }
}
