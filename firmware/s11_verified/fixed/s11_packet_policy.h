#ifndef S11_PACKET_POLICY_H
#define S11_PACKET_POLICY_H

#include <stdatomic.h>
#include <stdint.h>

/* Axis order is left front, rear, wheel, then right front, rear, wheel.
 *
 * Age derivation and logged budget (A3, A5, B9):
 *   CAN replies carry no hardware acquisition timestamp or sequence field, so
 *   age_ms is an estimate of RX arrival age relative to the current control
 *   tick using wrap-safe unsigned modular subtraction
 *   (see s11_age_ms_u32 / s11_age_ms_u16). Physical sample acquisition phase
 *   within the drive/bridge period remains UNKNOWN pending bench measurement.
 *
 *   Derived default age ledger (at 500 Hz control dt = 2.0 ms):
 *     - Sensor + sample/control release phase (S11 matched fresh max
 *       2.997048 ms in review2/CHECK.md, excluding holds):        <= 3.000 ms
 *     - CAN 6-axis reply burst drain (81% 1-bus load, REVIEW.md):   1.700 ms
 *       (split-bus alternatives: 0.890 ms @ 40.5%, 0.620 ms @ 27%)
 *     - 1 ms bridge transfer hop (REVIEW.md A11):                  1.000 ms
 *     - Host/ISR scheduling jitter margin (INFERRED):               0.300 ms
 *     => max_transport_age_ms (arrival ceiling, zero holds)       = 6.000 ms
 *     - predict_hold_ms (2 missed 2 ms ticks, matches orig)       = 4.000 ms
 *     - cut_hold_ms (5 missed 2 ms ticks; 2-tick LIMIT @ 6,8 ms)  = 10.000 ms
 *     => max_total_age_ms = max_transport_age_ms + cut_hold_ms    = 16.000 ms
 *     => max_predict_ms = cut_hold_ms (consistent w/ LIMIT, A12)  = 10.000 ms
 *   Note: S11's reported 8.815548 ms peak includes 3 missed ticks (6.0 ms hold)
 *   on top of ~2.816 ms arrival age; holds are NOT double-counted in arrival.
 *
 * Bit precedence (A2, A12):
 *   1. healthy (bit i): drive/hardware status (enable, UV/OV, fault nibble).
 *      Evaluated unconditionally when status is known (status_unknown bit == 0).
 *      Clearing healthy immediately latches S11_DRIVE_FAULT and overrides
 *      fresh, valid, TIMEOUT, INVALID, and STUCK. An axis that has never
 *      replied to S11IsrMailbox has status_unknown bit set and holds ARMING
 *      rather than latching DRIVE_FAULT.
 *   2. fresh (bit i): indicates a newly arrived reply for axis i (further
 *      verified against seq[i] when sequence numbers are supplied).
 *   3. valid (bit i): transport/protocol/CRC validity for a fresh reply.
 *      Evaluated only when healthy and fresh are both set for axis i.
 *
 * ISR / concurrency contract (A10):
 *   - S11IsrMailbox uses a portable C11 ownership-safe 3-slot (triple-buffer)
 *     scheme with _Atomic uint32_t slot exchange (memory_order_acq_rel).
 *     Contract: single writer per axis (CAN RX ISR) and single reader
 *     (control loop); writer and reader never access the same S11AxisSlot
 *     concurrently (zero C11 data races per WG14 N1570 5.1.2.4p25).
 *     Initialization is tracked per slot independently of gen, so gen wrap at
 *     UINT32_MAX -> 0 never resets freshness or health.
 *   - Legacy s11_packets() / s11_packets_ex() copy caller arrays onto the
 *     stack, but cannot exclude a concurrent ISR from mutating caller-owned
 *     arrays during that copy: callers not using S11IsrMailbox must pass a
 *     thread/ISR-private copy or invoke inside a target critical section.
 *
 * ABI / integration note:
 *   sizeof(S11PacketPolicy) and sizeof(S11PacketFrame) differ from orig/.
 *   Downstream callers must update struct layouts, lifecycle (s11_init/s11_arm/
 *   s11_reset), frame->valid, NAN fault frames, and wheel-angle wrapping.
 */

enum {
    S11_AXES = 6,
    S11_PERIOD_MS = 2,
    S11_PREDICT_MISSES = 2,
    S11_LIMIT_MISSES = 4,
    S11_CUT_MISSES = 5,
    S11_CUT_AGE_MS = 10,
    S11_STUCK_MS = 10,
    S11_ARM_GROUPS = 3,
    S11_INVALID_LATCH_COUNT = 3,
    S11_STUCK_MIN_TICKS = 2
};

enum S11Mode {
    S11_FRESH = 0,
    S11_PREDICT,
    S11_LIMIT,
    S11_TIMEOUT,
    S11_INVALID,
    S11_DRIVE_FAULT,
    S11_STUCK,
    S11_DISARMED,
    S11_ARMING
};

enum S11ArmState {
    S11_STATE_DISARMED = 0,
    S11_STATE_ARMING = 1,
    S11_STATE_ARMED = 2
};

typedef struct {
    double predict_hold_ms;        /* Max hold duration in PREDICT (default 4.0 ms) */
    double cut_hold_ms;            /* Hold duration at which TIMEOUT latches (default 10.0 ms) */
    double max_transport_age_ms;   /* Max RX arrival age, from ledger (default 6.0 ms) */
    double max_total_age_ms;       /* Hard ceiling on arrival + hold age (default 16.0 ms) */
    double max_predict_ms;         /* Extrapolation cap, consistent with LIMIT window (default 10.0 ms) */
    double max_dt_ms;              /* Upper sanity bound for caller dt_ms (default 50.0 ms) */
    double hip_angle_limit_rad;    /* Hip mechanical limit + margin (INFERRED default 3.5 rad) */
    double sat_angle_rad;          /* Full-scale saturated code magnitude (default 12.5 rad) */
    double wheel_angle_max_rad;    /* Wheel multi-turn wrap half-range (default 12.5 rad) */
    double hip_rate_limit_rad_s;   /* Hip rate limit (~1.2 * 20.9 rad/s at 24 V = 25.08) */
    double wheel_rate_limit_rad_s; /* Wheel rate limit (~1.2 * 33.5 rad/s at 24 V = 40.20) */
    double bus_voltage_v;          /* Reference bus voltage for rate scaling (default 24.0 V) */
    double stuck_ms;               /* Max unchanged duration while moving (default 10.0 ms) */
    double stuck_rate_thresh;      /* Motion rate threshold for stuck timer (default 0.12 rad/s) */
    double angle_lsb_rad;          /* Quantization LSB for displacement stuck check (INFERRED 25/65536 rad) */
    unsigned arm_groups;           /* Consecutive valid 6-axis groups to arm (default 3) */
    unsigned invalid_latch_count;  /* Consecutive invalid packets to latch INVALID (default 3) */
    unsigned stuck_min_ticks;      /* Consecutive unchanged ticks with expected > 2 LSB (default 2) */
} S11Config;

typedef struct {
    double angle;
    double unwrapped_angle;
    double rate;
    double age_ms;
    double transport_age_ms;
    double hold_ms;
    double unchanged_ms;
    double stuck_expected_rad;
    unsigned misses;
    unsigned invalid_streak;
    unsigned unchanged_ticks;
    uint32_t last_seq;
    int has_seq;
    int seen;
} S11Axis;

typedef struct {
    S11Axis axis[S11_AXES];
    enum S11Mode latched;
    unsigned latched_max_misses;
    enum S11ArmState arm_state;
    unsigned arm_count;
    int config_initialized;
    S11Config config;
} S11PacketPolicy;

typedef struct {
    double angle[S11_AXES];
    double unwrapped_angle[S11_AXES];
    double rate[S11_AXES];
    double authority;
    unsigned max_misses;
    enum S11Mode mode;
    int valid;
} S11PacketFrame;

/* Ownership-safe triple-buffered single-writer (CAN RX ISR) / single-reader
 * (control loop) mailbox. Writer and reader exchange slot indices via _Atomic
 * uint32_t and never access the same S11AxisSlot concurrently. */
typedef struct {
    int valid;
    int healthy;
    int initialized;
    uint32_t stamp_ticks;
    uint32_t msg_seq;
    double angle;
    double rate;
} S11AxisSlot;

typedef struct {
    _Atomic uint32_t gen;      /* monotonic write counter (even = stable, odd = writing) */
    _Atomic uint32_t shared;   /* encoded slot ((idx ^ 2) & 3) | dirty (0x4) */
    uint32_t writer_idx;       /* private to writer (0..2) */
    uint32_t reader_idx_xor1;  /* private to reader (slot = reader_idx_xor1 ^ 1) */
    uint32_t msg_seq;          /* private to writer */
    S11AxisSlot slot[3];
} S11AxisMailbox;

typedef struct {
    S11AxisMailbox axis[S11_AXES];
    uint32_t last_seen_seq[S11_AXES];
    unsigned last_seen_mask;
    unsigned last_healthy_mask;
} S11IsrMailbox;

typedef struct {
    unsigned fresh;
    unsigned valid;
    unsigned healthy;
    unsigned status_unknown;   /* bits set for axes that have never written status */
    double age_ms[S11_AXES];
    double angle[S11_AXES];
    double rate[S11_AXES];
    uint32_t seq[S11_AXES];
    int has_seq;
} S11InputSnapshot;

/* Configuration and lifecycle management (A2, A6). */
S11Config s11_default_config(void);
void s11_init(S11PacketPolicy *state);
void s11_init_with_config(S11PacketPolicy *state, const S11Config *cfg);
void s11_reset(S11PacketPolicy *state);
int s11_arm(S11PacketPolicy *state);
void s11_disarm(S11PacketPolicy *state);
int s11_set_bus_voltage(S11PacketPolicy *state, double bus_voltage_v);

/* Wrap-safe timestamp age helpers (A9). */
double s11_age_ms_u32(uint32_t now_ticks, uint32_t stamp_ticks, double ticks_per_ms);
double s11_age_ms_u16(uint16_t now_ticks, uint16_t stamp_ticks, double ticks_per_ms);

/* Wheel angle wrap/unwrap helpers (A7). */
double s11_wrap_angle(double angle, double half_range);
double s11_unwrap_delta(double prev_wrapped, double cur_wrapped, double half_range);

/* Tear-free ISR mailbox helpers (A10). */
void s11_mailbox_init(S11IsrMailbox *mb);
void s11_mailbox_write_axis(S11IsrMailbox *mb, unsigned axis, int valid, int healthy,
                            uint32_t stamp_ticks, double angle, double rate);
unsigned s11_mailbox_snapshot(S11IsrMailbox *mb, uint32_t now_ticks, double ticks_per_ms,
                              S11InputSnapshot *out);

/* Extended packet policy step with explicit dt_ms and optional per-axis seq[] (A4, A11). */
void s11_packets_ex(S11PacketPolicy *state, unsigned fresh, unsigned valid,
                    unsigned healthy, const double age_ms[S11_AXES],
                    const double angle[S11_AXES], const double rate[S11_AXES],
                    double dt_ms, const uint32_t seq[S11_AXES],
                    S11PacketFrame *frame);

/* Snapshot convenience wrapper combining A4, A9, A10, A11. */
void s11_packets_snapshot(S11PacketPolicy *state, const S11InputSnapshot *snap,
                          double dt_ms, S11PacketFrame *frame);

/* Legacy 500 Hz (2 ms) entry point preserved for API compatibility.
 * Note: zero-initialized state starts DISARMED; call s11_arm() to arm. */
void s11_packets(S11PacketPolicy *state, unsigned fresh, unsigned valid,
                 unsigned healthy, const double age_ms[S11_AXES],
                 const double angle[S11_AXES], const double rate[S11_AXES],
                 S11PacketFrame *frame);

#endif
