#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <stdbool.h>
#include <stdint.h>
#include "hea_proto.h"

#define SUPERVISOR_TRANSITION_DEPTH 16u
#define SUPERVISOR_INF_RX_DEPTH 16u

/* Step 2: duration-based source-age estimate; all local ticks are microseconds. */
typedef enum {
    SUPERVISOR_STATE_INIT = 0,
    SUPERVISOR_STATE_FRESH,
    SUPERVISOR_STATE_HOLD,
    SUPERVISOR_STATE_FAILSAFE
} supervisor_state_t;

typedef enum {
    SUPERVISOR_REASON_NONE = 0,
    SUPERVISOR_REASON_READY,
    SUPERVISOR_REASON_RESULT_SILENCE,
    SUPERVISOR_REASON_HEARTBEAT_TIMEOUT,
    SUPERVISOR_REASON_RESULT_TIMEOUT,
    SUPERVISOR_REASON_REARM,
    SUPERVISOR_REASON_CLOCK_REGRESSION,
    SUPERVISOR_REASON_SESSION_CHANGE,
    SUPERVISOR_REASON_INVALID_CONFIG,
    SUPERVISOR_REASON_SOURCE_AGE_HOLD,
    SUPERVISOR_REASON_SOURCE_AGE_TIMEOUT
} supervisor_reason_t;

typedef enum {
    SUPERVISOR_SEQ_DUPLICATE = 0,
    SUPERVISOR_SEQ_NEWER,
    SUPERVISOR_SEQ_OLDER,
    SUPERVISOR_SEQ_AMBIGUOUS
} supervisor_seq_order_t;

typedef enum {
    SUPERVISOR_EVENT_NONE = 0,
    SUPERVISOR_EVENT_HEARTBEAT,
    SUPERVISOR_EVENT_RESULT,
    SUPERVISOR_EVENT_OTHER, /* Valid Pi ECHO_REQ; advances wire sequence only. */
    SUPERVISOR_EVENT_NET_STATUS /* age_at_send_us carries u32 ms converted to us. */
} supervisor_event_kind_t;

typedef struct {
    uint64_t heartbeat_timeout_ticks;
    uint64_t result_hold_ticks;
    uint64_t result_failsafe_ticks;
    uint64_t rearm_healthy_ticks;
    uint32_t transit_bound_us; /* PROVISIONAL send-to-receipt allowance, not proven. */
} supervisor_config_t;

typedef struct {
    supervisor_event_kind_t kind;
    bool valid;
    uint16_t wire_seq; /* One shared per-Pi counter, across message types. */
    uint32_t input_seq; /* Used only for RESULT; independent input counter. */
    uint64_t received_at_ticks; /* Actual MCU receipt boundary, not queue dispatch. */
    uint64_t generation; /* Capture at receipt/handoff; never restamp at dispatch. */
    uint64_t session_id; /* HEARTBEAT only; independent of local generation. */
    bool has_session_id; /* False for protocol v1/v2. */
    uint64_t age_at_send_us; /* RESULT: <= UINT32_MAX; NET_STATUS: <= UINT32_MAX * 1000. */
    bool has_age_at_send; /* False only when source age is absent (legacy v1). */
} supervisor_event_t;

typedef struct {
    struct hea_inf_rx inf_rx[SUPERVISOR_INF_RX_DEPTH];
    uint32_t inf_rx_lost;
    unsigned inf_rx_head, inf_rx_count;
    bool result_accepted; /* Reset each update; set only at accepted-result boundary. */
    struct hea_state_event transitions[SUPERVISOR_TRANSITION_DEPTH];
    uint32_t transition_lost;
    unsigned transition_head, transition_count;
    uint64_t accepted_hb_us, accepted_inf_us;
    supervisor_state_t state;
    supervisor_reason_t last_transition_reason;
    supervisor_config_t config;
    uint64_t last_time_ticks;
    uint64_t epoch_start_ticks;
    uint64_t last_heartbeat_ticks;
    uint64_t last_result_ticks;
    uint64_t healthy_since_ticks;
    uint64_t generation;
    uint64_t rejected_generation_events; /* Saturating count of stale queue events. */
    uint64_t session_id;
    uint16_t last_wire_seq;
    uint32_t last_input_seq;
    bool configured;
    bool has_session_id;
    bool has_wire_seq;
    bool has_input_seq;
    bool has_heartbeat;
    bool has_result;
    bool healthy_interval;
    uint32_t last_age_at_send_us;
    uint64_t source_age_estimate_us; /* Saturates at UINT64_MAX; valid with has_result. */
    bool receipt_silence_fallback; /* Last accepted RESULT has no source age. */
    bool net_required; /* First NET_STATUS enables monitoring, retained across rearm/session. */
    bool has_net_status;
    uint64_t last_net_status_ticks;
    uint64_t net_age_at_send_us;
} supervisor_t;

/* Extend a raw uint32 microsecond timer from the previous extended sample.
 * Initialize previous to the first raw sample. Sample strictly within 2^31 us;
 * regression/ambiguous gaps and uint64 overflow return false without writing out.
 * Capture extended receipt time before queueing; never extend old queued samples. */
bool supervisor_extend_mcu_us(uint64_t previous, uint32_t raw, uint64_t *out);

supervisor_seq_order_t supervisor_seq_compare16(uint16_t current, uint16_t last);
supervisor_seq_order_t supervisor_seq_compare(uint32_t current, uint32_t last);

/* No default thresholds. Invalid config leaves an unrearmable FAILSAFE. */
bool supervisor_init(supervisor_t *sv, const supervisor_config_t *config,
                     uint64_t now_ticks);

/* Single-owner API. Call even without traffic. Expiry precedes event handling.
 * Times must be nondecreasing, extended MCU-local microseconds; no wrap.
 * NONE/NULL means a timer evaluation. Invalid events never renew timers.
 * An event must carry receipt time <= now, within this epoch, and the generation
 * captured at receipt. Generation mismatch is rejected and counted. */
supervisor_state_t supervisor_update(supervisor_t *sv,
                                     const supervisor_event_t *event,
                                     uint64_t now_ticks);

/* Local explicit request, not a wire command. Only succeeds from FAILSAFE
 * after uninterrupted healthy observation; clears receipt/health history,
 * preserves sequence high-water marks, and returns to safe INIT.
 * Increments generation, rejecting queued pre-rearm events even in the same tick.
 * The owner must serialize generation capture with update/rearm. Generation must
 * not wrap; reset the whole queue/core before UINT64_MAX is exhausted. */
bool supervisor_rearm(supervisor_t *sv, uint64_t now_ticks);

/* Caller serializes pop with update/rearm; nonblocking static ring. */
bool supervisor_pop_inf_rx(supervisor_t *sv, struct hea_inf_rx *out);
bool supervisor_pop_transition(supervisor_t *sv, struct hea_state_event *out);

/* Only FRESH permits new simulated output; HOLD policy is a port decision.
 * INIT and FAILSAFE must select the local safe output. */
const char *supervisor_state_to_str(supervisor_state_t state);

#endif
