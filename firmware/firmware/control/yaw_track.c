/*
 * YAW1: optional heading loop and path follower. See yaw_track.h.
 */
#include "control/yaw_track.h"

#include <math.h>
#include <string.h>

static float yt_clamp(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

YawTrackConfig yaw_track_default_config(void)
{
    YawTrackConfig c;
    /* YAW1 FIL gain scan (out/tune_scan.log): 0.4 m/s, R 0.5 m, mu 0.38, external
     * pose, both mass models; best worst-case max error over 24 combinations. */
    c.heading_kp_1_s   = 8.0f;
    c.max_rate_rad_s   = 2.0f;
    c.max_accel_rad_s2 = 6.0f;
    c.xtrack_k_1_s     = 3.0f;
    c.xtrack_max_rad   = 0.7f;
    c.min_speed_m_s    = 0.10f;
    c.ff_preview_s     = 0.20f;
    c.ext_pose_timeout_s = 0.10f;  /* 50 ticks at 500 Hz */
    c.reverse_speed_m_s  = 0.05f;
    return c;
}

void yaw_track_init(YawTrack *t, const YawTrackConfig *cfg)
{
    if (t == NULL) {
        return;
    }
    memset(t, 0, sizeof(*t));
    t->cfg = (cfg != NULL) ? *cfg : yaw_track_default_config();
    /* FW7B: keep custom configs inside the ranges the step relies on */
    const YawTrackConfig d = yaw_track_default_config();
    if (!(t->cfg.min_speed_m_s >= YAW_TRACK_MIN_SPEED_FLOOR)) {
        t->cfg.min_speed_m_s = YAW_TRACK_MIN_SPEED_FLOOR;
    }
    if (!(t->cfg.ext_pose_timeout_s > 0.0f) || !isfinite(t->cfg.ext_pose_timeout_s)) {
        t->cfg.ext_pose_timeout_s = d.ext_pose_timeout_s;
    }
    if (!(t->cfg.reverse_speed_m_s > 0.0f) || !isfinite(t->cfg.reverse_speed_m_s)) {
        t->cfg.reverse_speed_m_s = d.reverse_speed_m_s;
    }
    t->mode = (uint8_t)YAW_MODE_RATE;
}

float yaw_wrap_pi(float a)
{
    if (!isfinite(a)) {
        return 0.0f;
    }
    float w = fmodf(a + YAW_TRACK_PI, YAW_TRACK_TWO_PI);
    if (w < 0.0f) {
        w += YAW_TRACK_TWO_PI;
    }
    w -= YAW_TRACK_PI;
    /* float rounding can land exactly on +pi; keep the half-open interval */
    if (w >= YAW_TRACK_PI) {
        w -= YAW_TRACK_TWO_PI;
    }
    if (w < -YAW_TRACK_PI) {
        w = -YAW_TRACK_PI;
    }
    return w;
}

float yaw_heading_rate(float kp, float target_rad, float yaw_rad, float ff_rad_s)
{
    return ff_rad_s + kp * yaw_wrap_pi(target_rad - yaw_rad);
}

float yaw_rate_limit(float prev, float cmd, float max_rate, float max_accel, float dt)
{
    if (!isfinite(cmd)) {
        cmd = 0.0f;
    }
    if (!isfinite(prev)) {
        prev = 0.0f;
    }
    const float c = yt_clamp(cmd, -max_rate, max_rate);
    const float dmax = max_accel * dt;
    return prev + yt_clamp(c - prev, -dmax, dmax);
}

void yaw_seg_pose(const YawPathSeg *g, float s, float *x, float *y, float *psi)
{
    const float k = g->curvature_1_m;
    const float p = g->psi0_rad + k * s;
    float px;
    float py;
    if (fabsf(k) < 1e-6f) {
        px = g->x0_m + s * cosf(g->psi0_rad);
        py = g->y0_m + s * sinf(g->psi0_rad);
    } else {
        /* p(s) = c - (1/k) n(s), n = (-sin psi, cos psi), c = p0 + (1/k) n0 */
        px = g->x0_m + (sinf(p) - sinf(g->psi0_rad)) / k;
        py = g->y0_m - (cosf(p) - cosf(g->psi0_rad)) / k;
    }
    if (x != NULL) { *x = px; }
    if (y != NULL) { *y = py; }
    if (psi != NULL) { *psi = yaw_wrap_pi(p); }
}

float yaw_seg_project(const YawPathSeg *g, float s_hint, float px, float py, float *e_y)
{
    const float k = g->curvature_1_m;
    float s;
    float e;
    if (fabsf(k) < 1e-6f) {
        const float dx = px - g->x0_m;
        const float dy = py - g->y0_m;
        const float c0 = cosf(g->psi0_rad);
        const float s0 = sinf(g->psi0_rad);
        s = dx * c0 + dy * s0;
        e = -dx * s0 + dy * c0;
    } else {
        const float inv = 1.0f / k;
        const float cx = g->x0_m - inv * sinf(g->psi0_rad);
        const float cy = g->y0_m + inv * cosf(g->psi0_rad);
        const float rx = px - cx;
        const float ry = py - cy;
        const float rn = sqrtf(rx * rx + ry * ry);
        if (rn < 1e-6f) {
            /* at the centre every point is closest: keep the hint */
            s = s_hint;
            e = (k > 0.0f) ? fabsf(inv) : -fabsf(inv);
        } else {
            const float psi_c = (k > 0.0f) ? atan2f(rx, -ry) : atan2f(-rx, ry);
            const float psi_h = g->psi0_rad + k * s_hint;
            s = s_hint + yaw_wrap_pi(psi_c - psi_h) / k;
            e = (k > 0.0f) ? (fabsf(inv) - rn) : (rn - fabsf(inv));
        }
    }
    if (e_y != NULL) {
        *e_y = e;
    }
    return s;
}

void yaw_path_begin(YawTrack *t, float x0, float y0, float psi0, uint8_t closed)
{
    if (t == NULL) {
        return;
    }
    t->n_seg = 0u;
    t->closed = (closed != 0u) ? 1u : 0u;
    t->seg_idx = 0u;
    t->s_seg_m = 0.0f;
    memset(t->seg, 0, sizeof(t->seg));
    memset(&t->diag, 0, sizeof(t->diag));
    t->seg[0].x0_m = x0;
    t->seg[0].y0_m = y0;
    t->seg[0].psi0_rad = yaw_wrap_pi(psi0);
}

uint8_t yaw_path_append(YawTrack *t, float length_m, float curvature_1_m)
{
    if (t == NULL || t->n_seg >= YAW_TRACK_MAX_SEGS
        || !isfinite(length_m) || !(length_m > 0.0f) || !isfinite(curvature_1_m)) {
        return 0u;
    }
    YawPathSeg *g = &t->seg[t->n_seg];
    if (t->n_seg > 0u) {
        const YawPathSeg *p = &t->seg[t->n_seg - 1u];
        yaw_seg_pose(p, p->length_m, &g->x0_m, &g->y0_m, &g->psi0_rad);
        t->seg_start_s_m[t->n_seg] = t->seg_start_s_m[t->n_seg - 1u] + p->length_m;
    } else {
        t->seg_start_s_m[0] = 0.0f;
    }
    g->length_m = length_m;
    g->curvature_1_m = curvature_1_m;
    t->n_seg++;
    return 1u;
}

uint8_t yaw_path_figure8(YawTrack *t, float x0, float y0, float psi0, float radius_m,
                         uint8_t closed)
{
    if (t == NULL || !isfinite(radius_m) || !(radius_m > 0.05f)) {
        return 0u;
    }
    yaw_path_begin(t, x0, y0, psi0, closed);
    const float L = YAW_TRACK_TWO_PI * radius_m;
    return (uint8_t)(yaw_path_append(t, L, 1.0f / radius_m)
                     & yaw_path_append(t, L, -1.0f / radius_m));
}

float yaw_path_length(const YawTrack *t)
{
    if (t == NULL || t->n_seg == 0u) {
        return 0.0f;
    }
    return t->seg_start_s_m[t->n_seg - 1u] + t->seg[t->n_seg - 1u].length_m;
}

float yaw_path_curvature_at(const YawTrack *t, float s_total_m)
{
    if (t == NULL || t->n_seg == 0u || !isfinite(s_total_m)) {
        return 0.0f;
    }
    const float L = yaw_path_length(t);
    float s = s_total_m;
    if (t->closed != 0u && L > 0.0f) {
        s = fmodf(s, L);
        if (s < 0.0f) {
            s += L;
        }
    }
    for (uint8_t i = 0u; i + 1u < t->n_seg; ++i) {
        if (s < t->seg_start_s_m[i + 1u]) {
            return t->seg[i].curvature_1_m;
        }
    }
    return t->seg[t->n_seg - 1u].curvature_1_m;
}

void yaw_path_track(YawTrack *t, float px, float py, float psi)
{
    if (t == NULL || t->n_seg == 0u) {
        return;
    }
    float e = 0.0f;
    float s = yaw_seg_project(&t->seg[t->seg_idx], t->s_seg_m, px, py, &e);
    /* Advance across segment ends (at most one full lap per tick). Progress is
     * local and monotone in segment index, so the figure-8 crossing point is not
     * ambiguous. */
    for (uint8_t n = 0u; n < t->n_seg && s > t->seg[t->seg_idx].length_m; ++n) {
        const float over = s - t->seg[t->seg_idx].length_m;
        if (t->seg_idx + 1u < t->n_seg) {
            t->seg_idx++;
        } else if (t->closed != 0u) {
            t->seg_idx = 0u;
        } else {
            s = t->seg[t->seg_idx].length_m;
            t->diag.done = 1u;
            break;
        }
        s = yaw_seg_project(&t->seg[t->seg_idx], over, px, py, &e);
    }
    if (t->diag.done != 0u) {
        s = t->seg[t->seg_idx].length_m;
        (void)yaw_seg_project(&t->seg[t->seg_idx], s, px, py, &e);
    }
    t->s_seg_m = s;
    const YawPathSeg *g = &t->seg[t->seg_idx];
    float psi_p = 0.0f;
    yaw_seg_pose(g, s, NULL, NULL, &psi_p);
    t->diag.e_y_m = e;
    t->diag.e_psi_rad = yaw_wrap_pi(psi - psi_p);
    t->diag.s_total_m = t->seg_start_s_m[t->seg_idx] + s;
    t->diag.kappa_1_m = g->curvature_1_m;
    t->diag.seg_idx = t->seg_idx;
}

void yaw_track_reset_odom(YawTrack *t, float x, float y, float yaw_now_est, float yaw_set)
{
    if (t == NULL) {
        return;
    }
    t->x_m = x;
    t->y_m = y;
    /* FW7B: odometry heading = yaw_set now; later estimator increments are added
     * to it. A non-finite yaw_now_est keeps the YAW1 meaning (offset = yaw_set). */
    t->psi_odom_rad = yaw_wrap_pi(yaw_set);
    t->est_yaw_ref_rad = isfinite(yaw_now_est) ? yaw_now_est : 0.0f;
}

void yaw_track_set_ext_pose(YawTrack *t, float x, float y, float yaw, uint8_t valid)
{
    if (t == NULL) {
        return;
    }
    const uint8_t ok = (valid != 0u && isfinite(x) && isfinite(y) && isfinite(yaw)) ? 1u : 0u;
    t->ext_pose_valid = ok;
    t->ext_pose_new = ok;
    t->ext_x_m = x;
    t->ext_y_m = y;
    t->ext_yaw_rad = yaw;
    if (ok != 0u) {
        t->ext_pose_age_ticks = 0u;
        t->diag.ext_pose_stale = 0u;
    }
}

float yaw_track_step(YawTrack *t, float est_yaw_rad, float est_v_m_s,
                     float host_rate_rad_s, float dt_s, uint8_t active)
{
    if (t == NULL) {
        return host_rate_rad_s;
    }
    /* Dead reckoning (always, so the pose is current when a mode is engaged).
     * FW7B: the heading is kept wrapped and advanced by estimator increments
     * (a difference of two nearby floats is exact), so a large unwrapped
     * estimator yaw adds no rounding here. */
    const uint8_t est_ok = (isfinite(est_yaw_rad) && isfinite(est_v_m_s)) ? 1u : 0u;
    if (est_ok != 0u) {
        float d = est_yaw_rad - t->est_yaw_ref_rad;
        if (d >= YAW_TRACK_PI || d < -YAW_TRACK_PI) {
            d = yaw_wrap_pi(d);
        }
        t->est_yaw_ref_rad = est_yaw_rad;
        float p = t->psi_odom_rad + d;
        if (p >= YAW_TRACK_PI || p < -YAW_TRACK_PI) {
            p = yaw_wrap_pi(p);
        }
        t->psi_odom_rad = p;
        t->x_m += est_v_m_s * cosf(p) * dt_s;
        t->y_m += est_v_m_s * sinf(p) * dt_s;
    }
    /* FW7B: external pose. A new valid sample re-seats the odometry pose once;
     * the age limit clears a stream that stopped and flags it. */
    t->diag.ext_pose_used = 0u;
    if (t->ext_pose_valid != 0u) {
        if ((float)t->ext_pose_age_ticks * dt_s >= t->cfg.ext_pose_timeout_s) {
            t->ext_pose_valid = 0u;
            t->ext_pose_new = 0u;
            t->diag.ext_pose_stale = 1u;
        } else {
            if (t->ext_pose_new != 0u && est_ok != 0u) {
                t->x_m = t->ext_x_m;
                t->y_m = t->ext_y_m;
                t->psi_odom_rad = yaw_wrap_pi(t->ext_yaw_rad);
                t->ext_pose_new = 0u;
                t->diag.ext_pose_used = 1u;
            }
            if (t->ext_pose_age_ticks < UINT16_MAX) {
                t->ext_pose_age_ticks++;
            }
        }
    }
    const uint8_t tracking = (t->mode == (uint8_t)YAW_MODE_HEADING
                              || t->mode == (uint8_t)YAW_MODE_PATH) ? 1u : 0u;
    /* A non-finite host rate is passed through so the safety FSM still sees it.
     * Not active (not BALANCING): immediate pass-through, the FSM owns it. */
    if (active == 0u || est_ok == 0u || !isfinite(host_rate_rad_s)) {
        t->rate_handoff = 0u;
        t->reverse_hold = 0u;
        t->diag.rate_handoff = 0u;
        t->diag.reverse_hold = 0u;
        t->rate_cmd_rad_s = isfinite(host_rate_rad_s) ? host_rate_rad_s : 0.0f;
        return host_rate_rad_s;
    }
    if (tracking == 0u) {
        t->reverse_hold = 0u;
        t->diag.reverse_hold = 0u;
        if (t->rate_handoff != 0u) {
            /* FW7B: active HEADING/PATH -> RATE: slew to the host rate (no
             * magnitude clamp), then hand over bit-exactly. */
            const float dmax = t->cfg.max_accel_rad_s2 * dt_s;
            const float dlt = host_rate_rad_s - t->rate_cmd_rad_s;
            if (dmax > 0.0f && isfinite(dmax) && fabsf(dlt) > dmax) {
                t->rate_cmd_rad_s += (dlt > 0.0f) ? dmax : -dmax;
                t->diag.rate_handoff = 1u;
                return t->rate_cmd_rad_s;
            }
            t->rate_handoff = 0u;
        }
        t->diag.rate_handoff = 0u;
        t->rate_cmd_rad_s = host_rate_rad_s;
        return host_rate_rad_s;
    }
    t->diag.rate_handoff = 0u;
    const float px = t->x_m;
    const float py = t->y_m;
    const float psi = t->psi_odom_rad;
    float cmd;
    if (t->mode == (uint8_t)YAW_MODE_HEADING) {
        t->reverse_hold = 0u;
        cmd = yaw_heading_rate(t->cfg.heading_kp_1_s, t->heading_target_rad, psi,
                               host_rate_rad_s);
    } else if (t->n_seg == 0u) {
        cmd = 0.0f;
    } else {
        yaw_path_track(t, px, py, psi);
        /* FW7B: reverse travel (with hysteresis): latch the heading, no
         * cross-track correction (its sign assumes forward travel). */
        if (t->reverse_hold != 0u) {
            if (est_v_m_s > -0.5f * t->cfg.reverse_speed_m_s) {
                t->reverse_hold = 0u;
            }
        } else if (est_v_m_s < -t->cfg.reverse_speed_m_s) {
            t->reverse_hold = 1u;
            t->reverse_hold_psi_rad = psi;
        }
        if (t->diag.done != 0u) {
            /* End of an open path: hold the final path heading. */
            cmd = t->cfg.heading_kp_1_s * yaw_wrap_pi(-t->diag.e_psi_rad);
        } else if (t->reverse_hold != 0u) {
            cmd = yaw_heading_rate(t->cfg.heading_kp_1_s, t->reverse_hold_psi_rad, psi, 0.0f);
        } else {
            /* |v| in the denominator, floored; the feed-forward keeps the signed
             * speed and the preview only looks ahead for forward travel. Below
             * -reverse_speed the reverse hold above takes over. */
            const float v_den = fmaxf(fabsf(est_v_m_s),
                                      fmaxf(t->cfg.min_speed_m_s, YAW_TRACK_MIN_SPEED_FLOOR));
            const float corr = yt_clamp(atanf(t->cfg.xtrack_k_1_s * t->diag.e_y_m / v_den),
                                        -t->cfg.xtrack_max_rad, t->cfg.xtrack_max_rad);
            /* psi_d - psi = (psi_p - corr) - psi = -e_psi - corr */
            const float k_ff = yaw_path_curvature_at(
                t, t->diag.s_total_m + fmaxf(est_v_m_s, 0.0f) * t->cfg.ff_preview_s);
            cmd = k_ff * est_v_m_s
                + t->cfg.heading_kp_1_s * yaw_wrap_pi(-t->diag.e_psi_rad - corr);
        }
    }
    t->diag.reverse_hold = t->reverse_hold;
    t->rate_cmd_rad_s = yaw_rate_limit(t->rate_cmd_rad_s, cmd, t->cfg.max_rate_rad_s,
                                       t->cfg.max_accel_rad_s2, dt_s);
    t->rate_handoff = 1u;
    return t->rate_cmd_rad_s;
}
