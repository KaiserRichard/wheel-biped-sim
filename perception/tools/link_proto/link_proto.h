/* CANDIDATE: not an accepted interface; STM32 firmware owner review required.
 * All constants/layouts/budgets are PROPOSED. See SPEC.md. */
#ifndef LINK_PROTO_H
#define LINK_PROTO_H
#include <stddef.h>
#include <stdint.h>
#define LP_MAX_RAW 192u
#define LP_MAX_ENCODED 193u
#define LP_MAX_WIRE 194u
#define LP_VERSION 1u
#define LP_HEALTH_SIZE 80u
#define LP_REF_SIZE 44u
#define LP_STATUS_SIZE 44u
#define LP_ADVISORY_SIZE 42u
#define LP_ADVISORY_ECHO_SIZE 27u
#define LP_LINK_US 150000u
#define LP_AGE_US 150000u
#define LP_LEASE_US 100000u
#define LP_UNCERTAINTY_NS 10000000u
#define LP_BODY_FRAME 1u
#define LP_ODOM_FRAME 2u
#define LP_REQUIRED_FLAGS 0x7fu
#define LP_RESOURCE_FAULT 0x80u
#define LP_UNCERTAINTY_AVAILABLE 0x100u
/* flags: initialized, tracking, finite pose, finite velocity, reviewed frame,
 * reviewed calibration, clock map valid, resource fault, uncertainty available.
 * These are assertions only; this health-only format carries no pose/covariance. */
typedef enum { LP_HEALTH=1, LP_BODY_REF=2, LP_MCU_STATUS=3, LP_TERRAIN_ADVISORY=4, LP_ADVISORY_ECHO=5 } lp_type;
typedef enum { LP_STARTING, LP_INITIALIZING, LP_TRACKING, LP_LOST, LP_FAULT } lp_tracking;
typedef enum { LP_NO_LINK, LP_LINK_OK_NO_ESTIMATE, LP_ESTIMATE_VALID,
 LP_STALE, LP_REJECTED, LP_RESET_PENDING_REENABLE } lp_state;
typedef enum { LP_REASON_NONE, LP_REASON_WIRE, LP_REASON_SESSION,
 LP_REASON_SEQUENCE, LP_REASON_FIELDS, LP_REASON_AGE, LP_REASON_RESET,
 LP_REASON_DISABLED, LP_REASON_ASSOCIATION, LP_REASON_SILENCE,
 LP_REASON_LEASE } lp_reason;
typedef struct {
 uint32_t estimate_seq, map_epoch, transform_revision, mapping_revision;
 uint8_t reset_counter, tracking, estimator_id, source_domain;
 uint16_t flags, frame_id, child_frame_id;
 int8_t quality; /* -1 failed, 0 unknown, 1..100 graded; no invented confidence */
 uint32_t reasons;
 uint64_t source_ns, mapped_mcu_ns, pi_send_ns, age_at_send_ns;
 uint32_t uncertainty_ns;
} lp_health;
typedef struct {
 uint32_t estimate_seq, map_epoch, mapping_revision;
 uint16_t frame_id;
 uint8_t reset_counter;
 uint64_t pi_creation_ns, mapped_mcu_creation_ns;
 uint32_t lease_us;
 float forward_m_s, yaw_rad_s;
} lp_body_ref;
typedef struct {
 uint64_t sample_mcu_ns;
 uint32_t last_health_seq, last_estimate_seq, last_ref_seq, faults;
 uint64_t action_mcu_ns;
 uint16_t battery_mv;
 uint8_t fsm, armed, operator_enable, state, reason, battery_valid;
} lp_status;
typedef struct { uint64_t pi_monotonic_ns, frame_stamp_ns; uint8_t advisory, plane_valid; float confidence, distance_m, height_m, width_m, plane_pitch_rad, plane_height_m; } lp_advisory;
typedef struct { uint64_t sample_mcu_ns; uint32_t advisory_seq; uint8_t advisory, result; uint32_t accepted_count, rejected_count; uint8_t stale; uint32_t ms_since_accepted; } lp_advisory_echo;
typedef struct {
 uint8_t type;
 uint64_t session;
 uint32_t seq;
 union { lp_health health; lp_body_ref ref; lp_status status; lp_advisory advisory; lp_advisory_echo echo; } data;
} lp_message;
uint32_t lp_crc32c(const uint8_t *data, size_t n);
size_t lp_cobs_encode(const uint8_t *src, size_t n, uint8_t *dst, size_t cap);
size_t lp_cobs_decode(const uint8_t *src, size_t n, uint8_t *dst, size_t cap);
/* Zero means failure. unpack consumes decoded bytes including CRC. */
size_t lp_pack(const lp_message *m, uint8_t *wire, size_t cap);
int lp_unpack(const uint8_t *raw, size_t n, lp_message *m);
typedef void (*lp_packet_fn)(void *context, const lp_message *m);
typedef void (*lp_error_fn)(void *context);
typedef struct { uint8_t encoded[LP_MAX_ENCODED]; size_t used;
 int dropping; uint32_t errors, packets, last_seq; lp_reason last_reason;
 uint8_t last_type, last_advisory; } lp_stream;
void lp_stream_init(lp_stream *s);
void lp_stream_feed(lp_stream *s, const uint8_t *data, size_t n,
 lp_packet_fn packet, lp_error_fn error, void *context);
int lp_seq_newer(uint32_t candidate, uint32_t previous);
typedef struct {
 lp_state state; lp_reason reason;
 uint64_t session;
 uint32_t mapping_revision, transform_revision;
 uint32_t last_link_us, health_seq, ref_seq, estimate_seq;
 uint32_t estimate_mcu_us, uncertainty_us, ref_start_us, ref_lease_us;
 uint64_t source_ns, mapped_ns;
 uint32_t map_epoch;
 uint8_t reset_counter, source_domain;
 int estimate_usable;
 int bound, have_link, have_health, have_estimate, have_ref_seq;
 int pending_reenable, enabled, reference_active;
 float forward_m_s, yaw_rad_s;
} lp_receiver;
void lp_receiver_init(lp_receiver *r);
/* Trusted local action; never automatically bind an on-wire session.
 * Call on Pi reboot/resync or MCU boot, with reviewed mapping/extrinsic revision. */
void lp_receiver_bind(lp_receiver *r, uint64_t session,
 uint32_t mapping_revision, uint32_t transform_revision);
void lp_receiver_reject(lp_receiver *r, lp_reason reason);
/* Call regularly even when no bytes arrive. All expiry uses MCU monotonic us. */
void lp_receiver_update(lp_receiver *r, uint32_t now_mcu_us);
int lp_receiver_accept(lp_receiver *r, uint32_t now_mcu_us, const lp_message *m);
/* A new explicit local enable action, after current accepted fresh health. */
int lp_receiver_enable(lp_receiver *r, uint32_t now_mcu_us);
void lp_receiver_disable(lp_receiver *r);
#endif
