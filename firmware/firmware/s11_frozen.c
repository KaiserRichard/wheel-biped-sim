#include "s11_frozen.h"

#include <math.h>
#include <string.h>

S11FzConfig s11fz_default_config(int wheel)
{
    S11FzConfig c;
    c.lsb_rad = wheel ? (24.0 / 65535.0) : (25.0 / 65536.0);
    c.rate_deadband = 90.0 / 4095.0;
    c.gross_lsb = 10.0;
    c.rect_lsb = 7.0;
    c.rect_window_ms = 8.0;
    c.net_lsb = 5.0;
    c.advisory_lsb = 6.0;
    c.run_cap_ms = 64.0;
    return c;
}

void s11fz_gate_init(S11FzGate *g)
{
    g->gyro_rad_s = 0.7;
    g->acc_dev_m_s2 = 8.0;
    g->hold_ms = 60.0;
    g->left_ms = 0.0;
}

int s11fz_gate_update(S11FzGate *g, double gyro_x, double gyro_y, double acc_norm, double dt_ms)
{
    const int raw = (fabs(gyro_x) > g->gyro_rad_s || fabs(gyro_y) > g->gyro_rad_s
                     || fabs(acc_norm - 9.81) > g->acc_dev_m_s2) ? 1 : 0;
    if (raw) {
        g->left_ms = g->hold_ms + dt_ms; /* gated for this sample and hold_ms after */
    }
    const int gated = g->left_ms > 1e-9 ? 1 : 0;
    g->left_ms = fmax(0.0, g->left_ms - dt_ms);
    return gated;
}

void s11fz_reset(S11FzAxis *a)
{
    memset(a, 0, sizeof(*a));
}

static void clear_run(S11FzAxis *a)
{
    a->n = 0u;
    a->head = 0u;
    if (a->state != S11FZ_CONFIRMED) {
        a->state = S11FZ_OK;
        a->reason = S11FZ_R_NONE;
    }
}

static void confirm(S11FzAxis *a, S11FzReason r)
{
    if (a->state != S11FZ_CONFIRMED) {
        a->state = S11FZ_CONFIRMED;
        a->reason = r;
    }
}

S11FzState s11fz_step(const S11FzConfig *c, S11FzAxis *a, double angle, double rate,
                      double dt_ms, int gated)
{
    if (!a->seen) {
        a->seen = 1u;
        a->last_angle = angle;
        clear_run(a);
        return a->state;
    }
    if (angle != a->last_angle || a->restart) {
        /* New code (healthy motion) or first sample after a gate: start a new run.
         * This sample's rate belongs to the transition, not to the run. */
        a->last_angle = angle;
        a->restart = 0u;
        clear_run(a);
        return a->state;
    }
    double v = 0.0;
    if (rate > c->rate_deadband) {
        v = rate - c->rate_deadband;
    } else if (rate < -c->rate_deadband) {
        v = rate + c->rate_deadband;
    }
    const double d = v * 0.001 * dt_ms / c->lsb_rad;

    /* GROSS: no healthy explanation even inside an impact (healthy max 6.84 LSB). */
    const double g_ratio = fabs(d) / c->gross_lsb;
    if (g_ratio > a->peak_ratio[0]) a->peak_ratio[0] = g_ratio;
    if (fabs(d) >= c->gross_lsb) {
        confirm(a, S11FZ_R_GROSS);
    }
    if (gated) {
        a->restart = 1u;
        clear_run(a);
        return a->state;
    }

    a->d[a->head] = d;
    a->dt[a->head] = dt_ms;
    a->head = (a->head + 1u) % S11FZ_RING;
    if (a->n < S11FZ_RING) a->n++;

    double rect8 = 0.0, net = 0.0, rect_run = 0.0, t = 0.0;
    for (uint32_t i = 0u; i < a->n; ++i) {
        const uint32_t j = (a->head + S11FZ_RING - 1u - i) % S11FZ_RING;
        t += a->dt[j];
        if (t > c->run_cap_ms + 1e-9) break;
        if (t <= c->rect_window_ms + 1e-9) rect8 += fabs(a->d[j]);
        net += a->d[j];
        rect_run += fabs(a->d[j]);
    }
    net = fabs(net);
    const double r[3] = { rect8 / c->rect_lsb, net / c->net_lsb, rect_run / c->advisory_lsb };
    for (int i = 0; i < 3; ++i) {
        if (r[i] > a->peak_ratio[i + 1]) a->peak_ratio[i + 1] = r[i];
    }
    if (rect8 >= c->rect_lsb) {
        confirm(a, S11FZ_R_RECT);
    } else if (net >= c->net_lsb) {
        confirm(a, S11FZ_R_NET);
    } else if (a->state != S11FZ_CONFIRMED) {
        if (rect_run >= c->advisory_lsb) {
            a->state = S11FZ_ADVISORY;
            a->reason = S11FZ_R_ADVISORY;
        } else {
            a->state = S11FZ_OK;
            a->reason = S11FZ_R_NONE;
        }
    }
    return a->state;
}
