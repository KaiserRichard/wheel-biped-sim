/*
 * TRAC3: floor-grip estimate and friction-aware host-command limiter.
 * See grip_limit.h. Development values (simulation), not hardware constants.
 */
#include "control/grip_limit.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define GRIP_G_M_S2 9.81f
/* Development defaults; compile-time overridable for simulation tuning sweeps. */
#ifndef GRIP_K_MARGIN
/* TRAC4: 0.60 / 1.25. Keeps the TRAC3 0.6*mu*g budget for a host hint up to
 * 25 % above the true friction (sweep: 1.25x hint, 0/96 falls). */
#define GRIP_K_MARGIN 0.48f
#endif
#ifndef GRIP_JERK_TIME_S
#define GRIP_JERK_TIME_S 0.30f   /* 0.15 s let a mu 0.1 brake lock the wheels (TRAC3 tuning) */
#endif

static float grip_clamp(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

GripLimitConfig grip_limit_default_config(void)
{
    GripLimitConfig c;
    c.k_margin        = GRIP_K_MARGIN;
    c.mu_limit_on     = 0.50f;
    c.mu_min          = 0.08f;
    c.mu_slip_cap     = 0.30f;
    c.hold_s          = 5.0f;
    c.recover_per_s   = 0.05f;
    c.util_tau_s      = 0.10f;
    c.util_peak_decay = 0.20f;
    c.demo_s          = 0.20f;
    c.jerk_time_s     = GRIP_JERK_TIME_S;
    c.kp_1_s          = 3.0f;
    return c;
}

void grip_limit_init(GripLimit *g, const GripLimitConfig *cfg)
{
    if (g == NULL) {
        return;
    }
    memset(g, 0, sizeof(*g));
    g->cfg = (cfg != NULL) ? *cfg : grip_limit_default_config();
    g->mu_slip = 1.0f;
    g->mu_used = 1.0f;
}

static void grip_update_used(GripLimit *g)
{
    g->mu_used = g->mu_slip;
    if (g->mu_hint > 0.0f && g->mu_hint < g->mu_used) {
        g->mu_used = g->mu_hint;
    }
}

void grip_limit_set_hint(GripLimit *g, float mu_hint)
{
    if (g == NULL) {
        return;
    }
    g->mu_hint = (isfinite(mu_hint) && mu_hint > 0.0f && mu_hint < 1.0f) ? mu_hint : 0.0f;
    grip_update_used(g);
}

void grip_limit_observe(GripLimit *g, float dt_s, uint8_t eligible,
                        uint8_t slip_alarm, float wheel_accel_m_s2)
{
    if (g == NULL || !(dt_s > 0.0f)) {
        return;
    }
    const GripLimitConfig *c = &g->cfg;
    const uint8_t slip = (eligible != 0u && slip_alarm != 0u) ? 1u : 0u;

    if (eligible != 0u && slip == 0u && isfinite(wheel_accel_m_s2)) {
        /* No-slip utilisation: the wheels track the inertial reference, so the
         * filtered wheel acceleration is acceleration the floor carried. */
        const float u = fabsf(wheel_accel_m_s2) / GRIP_G_M_S2;
        g->util_lpf += (dt_s / (c->util_tau_s + dt_s)) * (u - g->util_lpf);
        g->util_peak -= c->util_peak_decay * dt_s;
        if (g->util_lpf > g->util_peak) {
            g->util_peak = g->util_lpf;
        }
        if (g->mu_slip < 1.0f && g->util_lpf > g->mu_slip) {
            g->demo_t_s += dt_s;
            if (g->demo_t_s >= c->demo_s) {
                g->mu_slip = grip_clamp(g->util_lpf, c->mu_min, 1.0f);
            }
        } else {
            g->demo_t_s = 0.0f;
        }
    } else {
        g->demo_t_s = 0.0f;
    }
    if (g->util_peak < 0.0f) {
        g->util_peak = 0.0f;
    }

    if (slip != 0u) {
        /* The floor gave way at (about) the utilisation seen just before. */
        const float obs = grip_clamp(g->util_peak, c->mu_min, c->mu_slip_cap);
        if (obs < g->mu_slip) {
            g->mu_slip = obs;
        }
        g->hold_left_s = c->hold_s;
        if (g->prev_slip == 0u && g->slip_events_n < UINT32_MAX) {
            g->slip_events_n++;
        }
    } else if (g->hold_left_s > 0.0f) {
        g->hold_left_s -= dt_s;
    } else if (g->mu_slip < 1.0f) {
        g->mu_slip += c->recover_per_s * dt_s;
        if (g->mu_slip > 1.0f) {
            g->mu_slip = 1.0f;
        }
    }
    g->prev_slip = slip;
    grip_update_used(g);
}

float grip_limit_accel_max(const GripLimit *g)
{
    if (g == NULL || !(g->mu_used <= g->cfg.mu_limit_on)) {
        return 0.0f;
    }
    return g->cfg.k_margin * g->mu_used * GRIP_G_M_S2;
}

uint8_t grip_limit_ref(GripLimit *g, float dt_s, uint8_t enable,
                       float v_cmd, float a_cmd, float j_cmd,
                       float *v_out, float *a_out, float *j_out)
{
    if (v_out == NULL || a_out == NULL || j_out == NULL) {
        return 0u;
    }
    *v_out = v_cmd;
    *a_out = a_cmd;
    *j_out = j_cmd;
    if (g == NULL || !(dt_s > 0.0f) || !isfinite(v_cmd) || !isfinite(a_cmd)
        || !isfinite(j_cmd)) {
        return 0u;
    }
    const GripLimitConfig *c = &g->cfg;
    /* TRAC4: inclusive, so a 0.5 hint (e.g. 1.25 x a true 0.4) still limits. */
    const uint8_t low = (enable != 0u && g->mu_used <= c->mu_limit_on) ? 1u : 0u;
    /* While releasing (estimate recovered), keep the last-applicable limit at
     * the recovered value so the tracker rejoins the command smoothly. */
    const float a_max = c->k_margin * g->mu_used * GRIP_G_M_S2;
    const float j_max = a_max / c->jerk_time_s;

    if (enable == 0u) {
        g->active = 0u;
    } else if (g->active == 0u && low != 0u
               && (fabsf(a_cmd) > a_max || fabsf(j_cmd) > j_max)) {
        g->active = 1u;   /* tracker starts from the last passed-through sample */
    }

    if (g->active == 0u) {
        g->v_m_s = v_cmd;
        g->a_m_s2 = a_cmd;
        return 0u;
    }

    const float e = v_cmd - g->v_m_s;
    const float a_des = grip_clamp(grip_clamp(a_cmd, -a_max, a_max) + c->kp_1_s * e,
                                   -a_max, a_max);
    const float a_prev = g->a_m_s2;
    /* Jerk-limited towards a_des; a lower limit (new low-grip evidence in the
     * middle of a command) applies at once rather than at the jerk rate. */
    g->a_m_s2 = grip_clamp(a_prev + grip_clamp(a_des - a_prev, -j_max * dt_s, j_max * dt_s),
                           -a_max, a_max);
    g->v_m_s += g->a_m_s2 * dt_s;
    *v_out = g->v_m_s;
    *a_out = g->a_m_s2;
    *j_out = (g->a_m_s2 - a_prev) / dt_s;
    if (g->limit_ticks_n < UINT32_MAX) {
        g->limit_ticks_n++;
    }
    /* Rejoin the command once synchronised and the command is within limits
     * (or no low grip is known any more). */
    if (fabsf(g->v_m_s - v_cmd) < 1e-3f && fabsf(g->a_m_s2 - a_cmd) < 0.05f
        && (low == 0u || (fabsf(a_cmd) <= a_max && fabsf(j_cmd) <= j_max))) {
        g->active = 0u;
    }
    return 1u;
}
