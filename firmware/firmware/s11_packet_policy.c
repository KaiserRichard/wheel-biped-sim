#include "s11_packet_policy.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static const double S11_EPS = 1e-9;
static const double S11_HIP_NOLOAD_24V = 20.9;
static const double S11_WHEEL_NOLOAD_24V = 33.5;
static const double S11_RATE_MARGIN = 1.2;
static const double S11_REF_BUS_V = 24.0;

static uint8_t s11_is_fault(enum S11Mode m)
{
    return (m == S11_TIMEOUT || m == S11_INVALID
         || m == S11_DRIVE_FAULT || m == S11_STUCK) ? 1u : 0u;
}

static uint32_t s11_mode_severity(enum S11Mode m)
{
    switch (m) {
    case S11_FRESH:       return 0u;
    case S11_PREDICT:     return 1u;
    case S11_LIMIT:       return 2u;
    case S11_DEGRADED:    return 3u;
    case S11_TIMEOUT:     return 4u;
    case S11_STUCK:       return 5u;
    case S11_INVALID:     return 6u;
    case S11_DRIVE_FAULT: return 7u;
    case S11_DISARMED:    return 8u;
    case S11_ARMING:      return 9u;
    }
    return 10u;
}

static void s11_fill_invalid_frame(S11PacketFrame *frame, enum S11Mode mode,
                                   uint32_t max_misses)
{
    frame->mode = mode;
    frame->authority = 0.0;
    frame->valid = 0u;
    frame->degraded_mask = 0u;
    frame->advisory_mask = 0u;
    frame->max_misses = max_misses;
    for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
        frame->angle[i] = NAN;
        frame->unwrapped_angle[i] = NAN;
        frame->rate[i] = NAN;
    }
}

S11Config s11_default_config(void)
{
    S11Config c;
    c.predict_hold_ms = 4.0;
    c.cut_hold_ms = 10.0;
    c.max_transport_age_ms = 6.0;
    c.max_total_age_ms = 16.0;
    c.max_predict_ms = 10.0;
    c.max_dt_ms = 50.0;
    c.hip_angle_limit_rad = 3.5;
    c.sat_angle_rad = 12.5;
    c.wheel_angle_max_rad = 12.5;
    c.bus_voltage_v = S11_REF_BUS_V;
    c.hip_rate_limit_rad_s = S11_RATE_MARGIN * S11_HIP_NOLOAD_24V;
    c.wheel_rate_limit_rad_s = S11_RATE_MARGIN * S11_WHEEL_NOLOAD_24V;
    c.stuck_ms = (double)S11_STUCK_MS;
    c.stuck_rate_thresh = 0.12;
    c.angle_lsb_rad = 25.0 / 65536.0;
    c.arm_groups = (uint32_t)S11_ARM_GROUPS;
    c.invalid_latch_count = (uint32_t)S11_INVALID_LATCH_COUNT;
    c.stuck_min_ticks = (uint32_t)S11_STUCK_MIN_TICKS;
    c.frozen_detect_enable = 1u;
    c.frozen_hip_mirror = 1u;
    c.frozen_hip = s11fz_default_config(0);
    c.frozen_wheel = s11fz_default_config(1);
    return c;
}

static void s11_ensure_config(S11PacketPolicy *state)
{
    if (state->config_initialized == 0u) {
        state->config = s11_default_config();
        state->config_initialized = 1u;
    }
}

void s11_init(S11PacketPolicy *state)
{
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->config = s11_default_config();
    state->config_initialized = 1u;
    state->arm_state = S11_STATE_DISARMED;
    s11fz_gate_init(&state->frozen_gate);
}

void s11_init_with_config(S11PacketPolicy *state, const S11Config *cfg)
{
    if (state == NULL) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->config = (cfg != NULL) ? *cfg : s11_default_config();
    state->config_initialized = 1u;
    state->arm_state = S11_STATE_DISARMED;
    s11fz_gate_init(&state->frozen_gate);
}

void s11_frozen_imu_update(S11PacketPolicy *state, double gyro_x, double gyro_y,
                           double acc_norm, double dt_ms)
{
    if (state != NULL && isfinite(gyro_x) && isfinite(gyro_y)
        && isfinite(acc_norm) && isfinite(dt_ms) && dt_ms > 0.0) {
        state->frozen_is_gated = (uint8_t)s11fz_gate_update(
            &state->frozen_gate, gyro_x, gyro_y, acc_norm, dt_ms);
    }
}

uint32_t s11_partner_wheel(uint32_t axis)
{
    return (axis == 2u) ? 5u : ((axis == 5u) ? 2u : 0xFFu);
}

/* Double faults that cannot be handled by substitution or holding geometry:
 * both wheels, both hips of one leg, or three or more axes. */
static uint8_t s11_frozen_is_double(uint32_t mask)
{
    uint32_t n = 0u;
    for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
        n += (mask >> i) & 1u;
    }
    return ((mask & 0x24u) == 0x24u || (mask & 0x03u) == 0x03u
            || (mask & 0x18u) == 0x18u || n >= 3u) ? 1u : 0u;
}

/* Run the STEP3RC frozen-encoder detector on one fresh, valid sample.
 * Returns 1 on the tick the axis becomes CONFIRMED; *gross = 1 if by GROSS. */
static uint8_t s11_frozen_step_axis(S11PacketPolicy *state, uint32_t i, uint8_t wheel,
                                    double angle, double rate, double dt_ms,
                                    uint8_t *gross)
{
    S11Axis *axis = &state->axis[i];
    const uint32_t bit = 1u << i;
    *gross = 0u;
    if (state->config.frozen_detect_enable == 0u) {
        state->advisory_mask &= ~bit;
        return 0u;
    }
    const S11FzConfig *fc = (wheel != 0u) ? &state->config.frozen_wheel
                                          : &state->config.frozen_hip;
    /* Time since this axis' previous accepted sample (missed ticks included). */
    const double sample_dt = dt_ms + ((axis->seen != 0u) ? axis->hold_ms : 0.0);
    const S11FzState fz = s11fz_step(fc, &axis->frozen, angle, rate, sample_dt,
                                     (state->frozen_is_gated != 0u) ? 1 : 0);
    if (fz == S11FZ_ADVISORY) {
        state->advisory_mask |= bit;
    } else {
        state->advisory_mask &= ~bit;
    }
    if (fz == S11FZ_CONFIRMED && (state->frozen_mask & bit) == 0u) {
        *gross = (axis->frozen.reason == S11FZ_R_GROSS) ? 1u : 0u;
        return 1u;
    }
    return 0u;
}

void s11_reset(S11PacketPolicy *state)
{
    if (state == NULL) {
        return;
    }
    s11_ensure_config(state);
    const S11Config saved = state->config;
    memset(state, 0, sizeof(*state));
    state->config = saved;
    state->config_initialized = 1u;
    state->arm_state = S11_STATE_DISARMED;
    s11fz_gate_init(&state->frozen_gate);
}

int32_t s11_arm(S11PacketPolicy *state)
{
    if (state == NULL) {
        return 0;
    }
    s11_ensure_config(state);
    if (state->latched != S11_FRESH) {
        return 0;
    }
    if (state->arm_state == S11_STATE_DISARMED) {
        state->arm_state = S11_STATE_ARMING;
        state->arm_count = 0u;
    }
    return 1;
}

void s11_disarm(S11PacketPolicy *state)
{
    if (state == NULL) {
        return;
    }
    s11_ensure_config(state);
    state->arm_state = S11_STATE_DISARMED;
    state->arm_count = 0u;
}

int32_t s11_set_bus_voltage(S11PacketPolicy *state, double bus_voltage_v)
{
    if (state == NULL || !isfinite(bus_voltage_v) || bus_voltage_v <= 0.0) {
        return -1;
    }
    s11_ensure_config(state);
    const double scale = bus_voltage_v / S11_REF_BUS_V;
    state->config.bus_voltage_v = bus_voltage_v;
    state->config.hip_rate_limit_rad_s = S11_RATE_MARGIN * S11_HIP_NOLOAD_24V * scale;
    state->config.wheel_rate_limit_rad_s = S11_RATE_MARGIN * S11_WHEEL_NOLOAD_24V * scale;
    return 0;
}

double s11_age_ms_u32(uint32_t now_ticks, uint32_t stamp_ticks, double ticks_per_ms)
{
    if (!isfinite(ticks_per_ms) || ticks_per_ms <= 0.0) {
        return NAN;
    }
    const uint32_t delta = (uint32_t)(now_ticks - stamp_ticks);
    return (double)delta / ticks_per_ms;
}

double s11_age_ms_u16(uint16_t now_ticks, uint16_t stamp_ticks, double ticks_per_ms)
{
    if (!isfinite(ticks_per_ms) || ticks_per_ms <= 0.0) {
        return NAN;
    }
    const uint16_t delta = (uint16_t)((uint32_t)now_ticks - (uint32_t)stamp_ticks);
    return (double)delta / ticks_per_ms;
}

double s11_wrap_angle(double angle, double half_range)
{
    if (!isfinite(angle) || !isfinite(half_range) || half_range <= 0.0) {
        return NAN;
    }
    const double period = 2.0 * half_range;
    double w = fmod(angle + half_range, period);
    if (w < 0.0) {
        w += period;
    }
    w -= half_range;
    if (w >= half_range) {
        w -= period;
    }
    if (w == 0.0 && signbit(w)) {
        w = 0.0; /* normalize -0.0 */
    }
    return w;
}

double s11_unwrap_delta(double prev_wrapped, double cur_wrapped, double half_range)
{
    if (!isfinite(prev_wrapped) || !isfinite(cur_wrapped)
        || !isfinite(half_range) || half_range <= 0.0) {
        return NAN;
    }
    return s11_wrap_angle(cur_wrapped - prev_wrapped, half_range);
}

void s11_mailbox_init(S11IsrMailbox *mb)
{
    if (mb == NULL) {
        return;
    }
    memset(mb, 0, sizeof(*mb));
    for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
        atomic_init(&mb->axis[i].gen, 0u);
        atomic_init(&mb->axis[i].shared, 0u);
    }
}

void s11_mailbox_write_axis(S11IsrMailbox *mb, uint32_t axis,
                            uint8_t valid, uint8_t healthy,
                            uint32_t stamp_ticks, double angle, double rate)
{
    if (mb == NULL || axis >= (uint32_t)S11_AXES) {
        return;
    }
    S11AxisMailbox *m = &mb->axis[axis];
    const uint32_t g = atomic_load_explicit(&m->gen, memory_order_relaxed);
    atomic_store_explicit(&m->gen, g + 1u, memory_order_relaxed);

    const uint32_t w = m->writer_idx;
    S11AxisSlot *dst = &m->slot[w];
    dst->valid = (valid != 0u) ? 1u : 0u;
    dst->healthy = (healthy != 0u) ? 1u : 0u;
    dst->initialized = 1u;
    dst->stamp_ticks = stamp_ticks;
    m->msg_seq += 1u;
    dst->msg_seq = m->msg_seq;
    dst->angle = angle;
    dst->rate = rate;

    /* Ownership-safe triple-buffer publish: writer and reader never touch the
     * same slot concurrently. Slot index is XOR-2 encoded so zero-init = 2. */
    const uint32_t pub = (w ^ 2u) | 0x4u;
    const uint32_t prev = atomic_exchange_explicit(&m->shared, pub, memory_order_acq_rel);
    m->writer_idx = (prev & 0x3u) ^ 2u;

    atomic_store_explicit(&m->gen, g + 2u, memory_order_release);
}

uint32_t s11_mailbox_snapshot(S11IsrMailbox *mb, uint32_t now_ticks,
                              double ticks_per_ms, S11InputSnapshot *out)
{
    if (out == NULL) {
        return 0x3Fu;
    }
    memset(out, 0, sizeof(*out));
    out->has_seq = 1u;
    if (mb == NULL) {
        out->status_unknown = 0x3Fu;
        return 0x3Fu;
    }

    for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
        const uint32_t bit = 1u << i;
        S11AxisMailbox *m = &mb->axis[i];
        uint32_t r = m->reader_idx_xor1 ^ 1u;
        const uint32_t sh = atomic_load_explicit(&m->shared, memory_order_acquire);
        if ((sh & 0x4u) != 0u) {
            const uint32_t prev = atomic_exchange_explicit(&m->shared, r ^ 2u,
                                                           memory_order_acq_rel);
            r = (prev & 0x3u) ^ 2u;
            m->reader_idx_xor1 = r ^ 1u;
        }

        const S11AxisSlot *src = &m->slot[r];
        if (src->initialized == 0u) {
            /* Axis has never written a reply: mark status unknown rather than
             * synthesizing an unhealthy drive fault during startup. */
            out->status_unknown |= bit;
            out->angle[i] = NAN;
            out->rate[i] = NAN;
            out->age_ms[i] = NAN;
            continue;
        }

        if (src->healthy != 0u) {
            out->healthy |= bit;
            mb->last_healthy_mask |= bit;
        } else {
            mb->last_healthy_mask &= ~bit;
        }
        if (src->valid != 0u) {
            out->valid |= bit;
        }
        out->angle[i] = src->angle;
        out->rate[i] = src->rate;
        out->seq[i] = src->msg_seq;
        out->age_ms[i] = s11_age_ms_u32(now_ticks, src->stamp_ticks, ticks_per_ms);

        if ((mb->last_seen_mask & bit) == 0u || src->msg_seq != mb->last_seen_seq[i]) {
            out->fresh |= bit;
            mb->last_seen_seq[i] = src->msg_seq;
            mb->last_seen_mask |= bit;
        }
    }
    return 0u;
}

static uint8_t s11_sample_valid(const S11Config *cfg, const S11Axis *axis,
                                uint8_t wheel, uint8_t valid_bit,
                                double age_ms, double angle, double rate,
                                double dt_ms)
{
    if (valid_bit == 0u) {
        return 0u;
    }
    if (!isfinite(angle) || !isfinite(rate) || !isfinite(age_ms)) {
        return 0u;
    }
    if (age_ms < 0.0 || age_ms > cfg->max_transport_age_ms + S11_EPS) {
        return 0u;
    }
    /* A5: reject stale or out-of-order replies whose acquisition time is not newer. */
    if (axis->seen != 0u && age_ms >= (axis->age_ms + dt_ms) - S11_EPS) {
        return 0u;
    }
    /* A6 & A7: plausibility and saturation checks. */
    if (wheel == 0u) {
        if (fabs(angle) >= cfg->sat_angle_rad || fabs(angle) >= cfg->hip_angle_limit_rad) {
            return 0u;
        }
        if (fabs(rate) >= cfg->hip_rate_limit_rad_s) {
            return 0u;
        }
    } else {
        if (angle < -cfg->wheel_angle_max_rad || angle >= cfg->wheel_angle_max_rad) {
            return 0u;
        }
        if (fabs(rate) >= cfg->wheel_rate_limit_rad_s) {
            return 0u;
        }
    }
    return 1u;
}

static enum S11Mode s11_classify_hold(const S11Config *cfg, const S11Axis *axis)
{
    if (axis->seen == 0u) {
        return S11_TIMEOUT;
    }
    if (axis->hold_ms >= cfg->cut_hold_ms - S11_EPS
        || axis->age_ms > cfg->max_total_age_ms + S11_EPS) {
        return S11_TIMEOUT;
    }
    if (axis->misses == 0u && axis->hold_ms <= S11_EPS) {
        return S11_FRESH;
    }
    if (axis->hold_ms <= cfg->predict_hold_ms + S11_EPS) {
        return S11_PREDICT;
    }
    return S11_LIMIT;
}

static void s11_accept_sample(const S11Config *cfg, S11Axis *axis, uint8_t wheel,
                              double age_ms, double angle, double rate,
                              const uint32_t *seq_ptr)
{
    if (wheel != 0u) {
        if (axis->seen != 0u) {
            const double delta = s11_unwrap_delta(axis->angle, angle,
                                                  cfg->wheel_angle_max_rad);
            axis->unwrapped_angle += delta;
        } else {
            axis->unwrapped_angle = angle;
        }
    } else {
        axis->unwrapped_angle = angle;
    }
    axis->angle = angle;
    axis->rate = rate;
    axis->transport_age_ms = age_ms;
    axis->hold_ms = 0.0;
    axis->age_ms = age_ms;
    axis->misses = 0u;
    axis->invalid_streak = 0u;
    axis->seen = 1u;
    if (seq_ptr != NULL) {
        axis->last_seq = *seq_ptr;
        axis->has_seq = 1u;
    }
}

static void s11_packets_internal(S11PacketPolicy *state, uint32_t fresh, uint32_t valid,
                                 uint32_t healthy, uint32_t status_unknown,
                                 const double age_ms[S11_AXES],
                                 const double angle[S11_AXES], const double rate[S11_AXES],
                                 double dt_ms, const uint32_t seq[S11_AXES],
                                 S11PacketFrame *frame)
{
    if (frame == NULL) {
        return;
    }
    if (state == NULL) {
        s11_fill_invalid_frame(frame, S11_INVALID, 0u);
        return;
    }
    s11_ensure_config(state);
    const S11Config *cfg = &state->config;

    /* A1: latched frame returns NAN data, valid=0, preserved max_misses. */
    if (state->latched != S11_FRESH) {
        s11_fill_invalid_frame(frame, state->latched, state->latched_max_misses);
        return;
    }

    /* A2: DISARMED state outputs no authority and valid=0. */
    if (state->arm_state == S11_STATE_DISARMED) {
        s11_fill_invalid_frame(frame, S11_DISARMED, 0u);
        return;
    }

    /* A11: reject invalid dt_ms fail-safely. */
    if (!isfinite(dt_ms) || dt_ms <= 0.0 || dt_ms > cfg->max_dt_ms) {
        state->latched = S11_INVALID;
        state->latched_max_misses = 0u;
        s11_fill_invalid_frame(frame, S11_INVALID, 0u);
        return;
    }

    /* A10: snapshot caller arrays onto stack before processing.
     * Note: callers not using S11IsrMailbox must ensure caller-owned arrays are
     * not concurrently mutated by an ISR during this copy. */
    double local_age[S11_AXES];
    double local_angle[S11_AXES];
    double local_rate[S11_AXES];
    uint32_t local_seq[S11_AXES];
    const uint8_t have_arrays = (age_ms != NULL && angle != NULL && rate != NULL) ? 1u : 0u;
    for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
        local_age[i] = (have_arrays != 0u) ? age_ms[i] : NAN;
        local_angle[i] = (have_arrays != 0u) ? angle[i] : NAN;
        local_rate[i] = (have_arrays != 0u) ? rate[i] : NAN;
        local_seq[i] = (seq != NULL) ? seq[i] : 0u;
    }

    /* A2: ARMING state requires cfg->arm_groups consecutive full valid 6-axis groups. */
    if (state->arm_state == S11_STATE_ARMING) {
        for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
            const uint32_t bit = 1u << i;
            if ((status_unknown & bit) == 0u && (healthy & bit) == 0u) {
                state->arm_count = 0u;
                state->latched = S11_DRIVE_FAULT;
                state->latched_max_misses = 0u;
                s11_fill_invalid_frame(frame, S11_DRIVE_FAULT, 0u);
                return;
            }
        }

        uint8_t all_good = 1u;
        for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
            const uint32_t bit = 1u << i;
            const uint8_t wheel = ((i % 3u) == 2u) ? 1u : 0u;
            S11Axis *axis = &state->axis[i];
            uint8_t axis_fresh = ((status_unknown & bit) == 0u && (fresh & bit) != 0u) ? 1u : 0u;
            if (axis_fresh != 0u && seq != NULL && axis->has_seq != 0u
                && local_seq[i] == axis->last_seq) {
                axis_fresh = 0u;
            }

            if (axis_fresh != 0u
                && s11_sample_valid(cfg, axis, wheel, ((valid & bit) != 0u) ? 1u : 0u,
                                    local_age[i], local_angle[i],
                                    local_rate[i], dt_ms) != 0u) {
                uint8_t gross = 0u;
                if (s11_frozen_step_axis(state, i, wheel, local_angle[i], local_rate[i],
                                         dt_ms, &gross) != 0u) {
                    /* Torque is not yet enabled while arming: a frozen axis
                     * keeps the robot unarmed (latched STUCK). */
                    state->frozen_mask |= bit;
                    state->arm_count = 0u;
                    state->latched = S11_STUCK;
                    state->latched_max_misses = 0u;
                    s11_fill_invalid_frame(frame, S11_STUCK, 0u);
                    return;
                }
                s11_accept_sample(cfg, axis, wheel, local_age[i], local_angle[i],
                                  local_rate[i], (seq != NULL) ? &local_seq[i] : NULL);
            } else {
                all_good = 0u;
                if (axis->seen != 0u) {
                    axis->hold_ms += dt_ms;
                    axis->age_ms = axis->transport_age_ms + axis->hold_ms;
                }
            }
        }

        if (all_good != 0u) {
            state->arm_count += 1u;
            const uint32_t needed = (cfg->arm_groups > 0u) ? cfg->arm_groups : 1u;
            if (state->arm_count >= needed) {
                state->arm_state = S11_STATE_ARMED;
                frame->mode = S11_FRESH;
                frame->authority = 1.0;
                frame->valid = 1u;
                frame->max_misses = 0u;
                for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
                    frame->angle[i] = state->axis[i].angle;
                    frame->unwrapped_angle[i] = state->axis[i].unwrapped_angle;
                    frame->rate[i] = state->axis[i].rate;
                }
                return;
            }
        } else {
            state->arm_count = 0u;
        }
        s11_fill_invalid_frame(frame, S11_ARMING, 0u);
        return;
    }

    /* S11_STATE_ARMED */
    enum S11Mode mode = S11_FRESH;
    uint32_t max_misses = 0u;
    uint32_t new_frozen = 0u;
    uint32_t new_gross = 0u;

    for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
        const uint32_t bit = 1u << i;
        const uint8_t wheel = ((i % 3u) == 2u) ? 1u : 0u;
        S11Axis *axis = &state->axis[i];
        enum S11Mode current = S11_FRESH;

        uint8_t axis_fresh = ((status_unknown & bit) == 0u && (fresh & bit) != 0u) ? 1u : 0u;
        if (axis_fresh != 0u && seq != NULL && axis->has_seq != 0u
            && local_seq[i] == axis->last_seq) {
            axis_fresh = 0u; /* A4: mailbox not overwritten -> treat as miss */
        }

        if ((status_unknown & bit) == 0u && (healthy & bit) == 0u) {
            /* A12: healthy has highest precedence; latches immediately. */
            current = S11_DRIVE_FAULT;
        } else if (axis_fresh != 0u) {
            if (s11_sample_valid(cfg, axis, wheel, ((valid & bit) != 0u) ? 1u : 0u,
                                 local_age[i], local_angle[i],
                                 local_rate[i], dt_ms) == 0u) {
                /* A5, A6, A7, A8: do not corrupt stored state; count as miss and
                 * latch INVALID only after cfg->invalid_latch_count in a row. */
                axis->invalid_streak += 1u;
                axis->hold_ms += dt_ms;
                axis->age_ms = axis->transport_age_ms + axis->hold_ms;
                axis->misses += 1u;
                const uint32_t cut_inv = (cfg->invalid_latch_count > 0u)
                                       ? cfg->invalid_latch_count : 1u;
                if (axis->invalid_streak >= cut_inv) {
                    current = S11_INVALID;
                } else {
                    current = s11_classify_hold(cfg, axis);
                }
            } else {
                uint8_t gross = 0u;
                if (s11_frozen_step_axis(state, i, wheel, local_angle[i], local_rate[i],
                                         dt_ms, &gross) != 0u) {
                    new_frozen |= bit;
                    new_gross += gross;
                }
                s11_accept_sample(cfg, axis, wheel, local_age[i], local_angle[i],
                                  local_rate[i], (seq != NULL) ? &local_seq[i] : NULL);
            }
        } else {
            axis->hold_ms += dt_ms;
            axis->age_ms = axis->transport_age_ms + axis->hold_ms;
            axis->misses += 1u;
            current = s11_classify_hold(cfg, axis);
        }

        if (s11_is_fault(current) == 0u) {
            const enum S11Mode hold_mode = s11_classify_hold(cfg, axis);
            if (s11_mode_severity(hold_mode) > s11_mode_severity(current)) {
                current = hold_mode;
            }
        }

        if (s11_mode_severity(current) > s11_mode_severity(mode)) {
            mode = current;
        }
        if (axis->misses > max_misses) {
            max_misses = axis->misses;
        }

        /* A7 & A12: extrapolate consistently with hold_ms up to max_predict_ms,
         * and wrap predicted wheel angle into [-wheel_angle_max_rad, +wheel_angle_max_rad). */
        const double predict_ms = (axis->hold_ms < cfg->max_predict_ms)
                                ? axis->hold_ms : cfg->max_predict_ms;
        const double delta_rad = 0.001 * predict_ms * axis->rate;
        double pred_angle = axis->angle + delta_rad;
        if (wheel != 0u) {
            pred_angle = s11_wrap_angle(pred_angle, cfg->wheel_angle_max_rad);
        }
        frame->angle[i] = pred_angle;
        frame->unwrapped_angle[i] = axis->unwrapped_angle + delta_rad;
        frame->rate[i] = axis->rate;
    }

    /* FW6 frozen-encoder reaction (STEP3RC B6). A newly CONFIRMED axis records
     * its substitution reference; one axis -> DEGRADED (controlled stop, frame
     * stays valid); double faults or two simultaneous GROSS -> latched STUCK. */
    if (new_frozen != 0u) {
        for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
            if ((new_frozen & (1u << i)) == 0u) {
                continue;
            }
            const uint32_t p = s11_partner_wheel(i);
            if (p != 0xFFu) {
                const uint32_t k = (i == 2u) ? 0u : 1u;
                state->sub_self_unwrapped[k] = state->axis[i].unwrapped_angle;
                state->sub_partner_unwrapped[k] = state->axis[p].unwrapped_angle;
            } else {
                state->hip_hold_angle[i] = state->axis[i].angle;
                state->hip_mirror_ref[i] = state->axis[(i + 3u) % 6u].angle;
            }
        }
        state->frozen_mask |= new_frozen;
    }
    if (state->frozen_mask != 0u
        && (new_gross >= 2u || s11_frozen_is_double(state->frozen_mask) != 0u)
        && s11_mode_severity(S11_STUCK) > s11_mode_severity(mode)) {
        mode = S11_STUCK;
    }

    if (s11_is_fault(mode) != 0u) {
        state->latched = mode;
        state->latched_max_misses = max_misses;
        s11_fill_invalid_frame(frame, mode, max_misses);
        frame->degraded_mask = state->frozen_mask;
        return;
    }

    const uint8_t any_limit = (mode == S11_LIMIT) ? 1u : 0u;
    if (state->frozen_mask != 0u) {
        for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
            if ((state->frozen_mask & (1u << i)) == 0u) {
                continue;
            }
            const uint32_t p = s11_partner_wheel(i);
            if (p != 0xFFu) {
                /* Wheel: partner wheel's angle increment and rate (zero yaw). */
                const uint32_t k = (i == 2u) ? 0u : 1u;
                const double u = state->sub_self_unwrapped[k]
                               + (frame->unwrapped_angle[p] - state->sub_partner_unwrapped[k]);
                frame->unwrapped_angle[i] = u;
                frame->angle[i] = s11_wrap_angle(u, cfg->wheel_angle_max_rad);
                frame->rate[i] = frame->rate[p];
            } else if (cfg->frozen_hip_mirror != 0u
                       && (state->frozen_mask & (1u << ((i + 3u) % 6u))) == 0u) {
                /* Hip, mirror option: other leg's same-role hip increment and rate. */
                const uint32_t m = (i + 3u) % 6u;
                frame->angle[i] = state->hip_hold_angle[i]
                                + (frame->angle[m] - state->hip_mirror_ref[i]);
                frame->unwrapped_angle[i] = frame->angle[i];
                frame->rate[i] = frame->rate[m];
            } else {
                /* Hip: hold the geometry at confirmation, no rate feedback. */
                frame->angle[i] = state->hip_hold_angle[i];
                frame->unwrapped_angle[i] = state->hip_hold_angle[i];
                frame->rate[i] = 0.0;
            }
        }
        if (s11_mode_severity(S11_DEGRADED) > s11_mode_severity(mode)) {
            mode = S11_DEGRADED;
        }
    }

    frame->mode = mode;
    frame->max_misses = max_misses;
    frame->valid = 1u;
    frame->authority = (any_limit != 0u) ? 0.75 : 1.0;
    frame->degraded_mask = state->frozen_mask;
    frame->advisory_mask = state->advisory_mask;
}

void s11_packets_ex(S11PacketPolicy *state, uint32_t fresh, uint32_t valid,
                    uint32_t healthy, const double age_ms[S11_AXES],
                    const double angle[S11_AXES], const double rate[S11_AXES],
                    double dt_ms, const uint32_t seq[S11_AXES],
                    S11PacketFrame *frame)
{
    s11_packets_internal(state, fresh, valid, healthy, 0u, age_ms, angle, rate,
                         dt_ms, seq, frame);
}

void s11_packets_snapshot(S11PacketPolicy *state, const S11InputSnapshot *snap,
                          double dt_ms, S11PacketFrame *frame)
{
    if (snap == NULL) {
        if (frame != NULL) {
            s11_fill_invalid_frame(frame, S11_INVALID, 0u);
        }
        return;
    }
    s11_packets_internal(state, snap->fresh, snap->valid, snap->healthy,
                         snap->status_unknown, snap->age_ms, snap->angle,
                         snap->rate, dt_ms,
                         (snap->has_seq != 0u) ? snap->seq : NULL, frame);
}

void s11_packets(S11PacketPolicy *state, uint32_t fresh, uint32_t valid,
                 uint32_t healthy, const double age_ms[S11_AXES],
                 const double angle[S11_AXES], const double rate[S11_AXES],
                 S11PacketFrame *frame)
{
    s11_packets_ex(state, fresh, valid, healthy, age_ms, angle, rate,
                   (double)S11_PERIOD_MS, NULL, frame);
}
