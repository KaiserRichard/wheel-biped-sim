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
    case S11_TIMEOUT:     return 3u;
    case S11_STUCK:       return 4u;
    case S11_INVALID:     return 5u;
    case S11_DRIVE_FAULT: return 6u;
    case S11_DISARMED:    return 7u;
    case S11_ARMING:      return 8u;
    }
    return 9u;
}

static void s11_fill_invalid_frame(S11PacketFrame *frame, enum S11Mode mode,
                                   uint32_t max_misses)
{
    frame->mode = mode;
    frame->authority = 0.0;
    frame->valid = 0u;
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
    c.max_transport_age_ms = 12.0;
    c.max_total_age_ms = 20.0;
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
    }
}

void s11_mailbox_write_axis(S11IsrMailbox *mb, uint32_t axis,
                            uint8_t valid, uint8_t healthy,
                            uint32_t stamp_ticks, double angle, double rate)
{
    if (mb == NULL || axis >= (uint32_t)S11_AXES) {
        return;
    }
    S11AxisMailbox *slot = &mb->axis[axis];
    const uint32_t g = atomic_load_explicit(&slot->gen, memory_order_relaxed);
    atomic_store_explicit(&slot->gen, g + 1u, memory_order_release);
    atomic_thread_fence(memory_order_acq_rel);

    slot->valid = (valid != 0u) ? 1u : 0u;
    slot->healthy = (healthy != 0u) ? 1u : 0u;
    slot->stamp_ticks = stamp_ticks;
    slot->angle = angle;
    slot->rate = rate;
    slot->msg_seq += 1u;

    atomic_store_explicit(&slot->gen, g + 2u, memory_order_release);
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
        return 0x3Fu;
    }

    uint32_t torn_mask = 0u;
    for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
        const uint32_t bit = 1u << i;
        S11AxisMailbox *slot = &mb->axis[i];
        uint8_t ok = 0u;
        uint32_t g1 = 0u;
        uint8_t v = 0u;
        uint8_t h = 0u;
        uint32_t st = 0u;
        uint32_t ms = 0u;
        double a = NAN;
        double r = NAN;

        for (uint32_t attempt = 0u; attempt < 8u; ++attempt) {
            g1 = atomic_load_explicit(&slot->gen, memory_order_acquire);
            if ((g1 & 1u) != 0u) {
                continue;
            }
            v = slot->valid;
            h = slot->healthy;
            st = slot->stamp_ticks;
            ms = slot->msg_seq;
            a = slot->angle;
            r = slot->rate;
            atomic_thread_fence(memory_order_acquire);
            const uint32_t g2 = atomic_load_explicit(&slot->gen, memory_order_acquire);
            if (g1 == g2) {
                ok = 1u;
                break;
            }
        }

        if (ok == 0u) {
            torn_mask |= bit;
            if ((mb->last_healthy_mask & bit) != 0u) {
                out->healthy |= bit;
            }
            out->angle[i] = NAN;
            out->rate[i] = NAN;
            out->age_ms[i] = NAN;
            continue;
        }

        if (g1 == 0u) {
            out->angle[i] = NAN;
            out->rate[i] = NAN;
            out->age_ms[i] = NAN;
            continue;
        }

        if (h != 0u) {
            out->healthy |= bit;
            mb->last_healthy_mask |= bit;
        } else {
            mb->last_healthy_mask &= ~bit;
        }
        if (v != 0u) {
            out->valid |= bit;
        }
        out->angle[i] = a;
        out->rate[i] = r;
        out->seq[i] = ms;
        out->age_ms[i] = s11_age_ms_u32(now_ticks, st, ticks_per_ms);

        if ((mb->last_seen_mask & bit) == 0u || ms != mb->last_seen_seq[i]) {
            out->fresh |= bit;
            mb->last_seen_seq[i] = ms;
            mb->last_seen_mask |= bit;
        }
    }
    return torn_mask;
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
    if (axis->seen != 0u && age_ms >= (axis->age_ms + dt_ms) - S11_EPS) {
        return 0u;
    }
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

static uint8_t s11_update_stuck(const S11Config *cfg, S11Axis *axis,
                                double angle, double rate, double dt_ms)
{
    const double step_expected = fabs(rate) * (0.001 * dt_ms);
    const uint8_t moving = (fabs(rate) >= cfg->stuck_rate_thresh
                         || step_expected > 2.0 * cfg->angle_lsb_rad) ? 1u : 0u;
    if (axis->seen != 0u && angle == axis->angle && moving != 0u) {
        axis->unchanged_ms += dt_ms;
        axis->unchanged_ticks += 1u;
        axis->stuck_expected_rad += step_expected;
    } else {
        axis->unchanged_ms = 0.0;
        axis->unchanged_ticks = 0u;
        axis->stuck_expected_rad = 0.0;
    }
    if (axis->unchanged_ms >= cfg->stuck_ms - S11_EPS) {
        return 1u;
    }
    if (axis->stuck_expected_rad > 2.0 * cfg->angle_lsb_rad
        && axis->unchanged_ticks >= cfg->stuck_min_ticks) {
        return 1u;
    }
    return 0u;
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

void s11_packets_ex(S11PacketPolicy *state, uint32_t fresh, uint32_t valid,
                    uint32_t healthy, const double age_ms[S11_AXES],
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

    if (state->latched != S11_FRESH) {
        s11_fill_invalid_frame(frame, state->latched, state->latched_max_misses);
        return;
    }

    if (state->arm_state == S11_STATE_DISARMED) {
        s11_fill_invalid_frame(frame, S11_DISARMED, 0u);
        return;
    }

    if (!isfinite(dt_ms) || dt_ms <= 0.0 || dt_ms > cfg->max_dt_ms) {
        state->latched = S11_INVALID;
        state->latched_max_misses = 0u;
        s11_fill_invalid_frame(frame, S11_INVALID, 0u);
        return;
    }

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

    if (state->arm_state == S11_STATE_ARMING) {
        for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
            const uint32_t bit = 1u << i;
            if ((healthy & bit) == 0u) {
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
            uint8_t axis_fresh = ((fresh & bit) != 0u) ? 1u : 0u;
            if (axis_fresh != 0u && seq != NULL && axis->has_seq != 0u
                && local_seq[i] == axis->last_seq) {
                axis_fresh = 0u;
            }

            if (axis_fresh != 0u
                && s11_sample_valid(cfg, axis, wheel, ((valid & bit) != 0u) ? 1u : 0u,
                                    local_age[i], local_angle[i],
                                    local_rate[i], dt_ms) != 0u) {
                if (s11_update_stuck(cfg, axis, local_angle[i], local_rate[i], dt_ms) != 0u) {
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

    for (uint32_t i = 0u; i < (uint32_t)S11_AXES; ++i) {
        const uint32_t bit = 1u << i;
        const uint8_t wheel = ((i % 3u) == 2u) ? 1u : 0u;
        S11Axis *axis = &state->axis[i];
        enum S11Mode current = S11_FRESH;

        uint8_t axis_fresh = ((fresh & bit) != 0u) ? 1u : 0u;
        if (axis_fresh != 0u && seq != NULL && axis->has_seq != 0u
            && local_seq[i] == axis->last_seq) {
            axis_fresh = 0u;
        }

        if ((healthy & bit) == 0u) {
            current = S11_DRIVE_FAULT;
        } else if (axis_fresh != 0u) {
            if (s11_sample_valid(cfg, axis, wheel, ((valid & bit) != 0u) ? 1u : 0u,
                                 local_age[i], local_angle[i],
                                 local_rate[i], dt_ms) == 0u) {
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
                if (s11_update_stuck(cfg, axis, local_angle[i], local_rate[i], dt_ms) != 0u) {
                    current = S11_STUCK;
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

    if (s11_is_fault(mode) != 0u) {
        state->latched = mode;
        state->latched_max_misses = max_misses;
        s11_fill_invalid_frame(frame, mode, max_misses);
        return;
    }

    frame->mode = mode;
    frame->max_misses = max_misses;
    frame->valid = 1u;
    frame->authority = (mode == S11_LIMIT) ? 0.75 : 1.0;
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
    s11_packets_ex(state, snap->fresh, snap->valid, snap->healthy,
                   snap->age_ms, snap->angle, snap->rate, dt_ms,
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
