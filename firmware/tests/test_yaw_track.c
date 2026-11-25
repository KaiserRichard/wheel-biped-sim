/*
 * YAW1 host tests: heading wrap at +-pi, heading loop + rate limit, path-follower
 * geometry (line/arc pose and projection, figure-8 construction, progress across
 * the crossing point and segment ends), a kinematic closed loop, and the
 * default-off pass-through inside balance_ctrl.
 */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "control/balance_ctrl.h"
#include "control/yaw_track.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(expr)                                                       \
    do {                                                                  \
        if (!(expr)) {                                                    \
            fprintf(stderr, "%s:%d: %s: Assertion `%s' failed.\n",        \
                    __FILE__, __LINE__, __func__, #expr);                 \
            abort();                                                      \
        }                                                                 \
    } while (0)

#define NEAR(a, b, tol) CHECK(fabsf((float)(a) - (float)(b)) <= (float)(tol))

static const float PI_F = YAW_TRACK_PI;

static void test_wrap(void)
{
    NEAR(yaw_wrap_pi(0.0f), 0.0f, 1e-7f);
    NEAR(yaw_wrap_pi(1.0f), 1.0f, 1e-6f);
    NEAR(yaw_wrap_pi(-1.0f), -1.0f, 1e-6f);
    /* +pi maps to -pi (half-open [-pi, pi)) */
    NEAR(yaw_wrap_pi(PI_F), -PI_F, 1e-6f);
    NEAR(yaw_wrap_pi(-PI_F), -PI_F, 1e-6f);
    NEAR(yaw_wrap_pi(PI_F - 0.001f), PI_F - 0.001f, 1e-5f);
    NEAR(yaw_wrap_pi(PI_F + 0.001f), -PI_F + 0.001f, 1e-5f);
    NEAR(yaw_wrap_pi(2.0f * PI_F + 0.1f), 0.1f, 1e-5f);
    NEAR(yaw_wrap_pi(-2.0f * PI_F - 0.1f), -0.1f, 1e-5f);
    NEAR(yaw_wrap_pi(3.0f * PI_F), -PI_F, 1e-5f);
    NEAR(yaw_wrap_pi(-7.5f), -7.5f + 2.0f * PI_F, 1e-5f);
    NEAR(yaw_wrap_pi(100.0f * PI_F + 0.25f), 0.25f, 1e-3f);
    CHECK(yaw_wrap_pi(NAN) == 0.0f);
    CHECK(yaw_wrap_pi(INFINITY) == 0.0f);
    /* every output in [-pi, pi) and equal to the input modulo 2 pi */
    for (int i = -20000; i <= 20000; ++i) {
        const float a = (float)i * 0.0031f;
        const float w = yaw_wrap_pi(a);
        CHECK(w >= -PI_F && w < PI_F);
        const float k = roundf((a - w) / (2.0f * PI_F));
        NEAR(a - w, k * 2.0f * PI_F, 2e-4f);
    }
    printf("  wrap: ok\n");
}

static void test_heading_loop_and_limits(void)
{
    /* short way across the +-pi seam: target +3.0, yaw -3.0 -> error -0.283 rad */
    const float r = yaw_heading_rate(2.0f, 3.0f, -3.0f, 0.0f);
    NEAR(r, 2.0f * (6.0f - 2.0f * PI_F), 1e-5f);
    CHECK(r < 0.0f);
    NEAR(yaw_heading_rate(2.0f, -3.0f, 3.0f, 0.0f), -r, 1e-5f);
    NEAR(yaw_heading_rate(1.0f, 0.5f, 0.5f, 0.3f), 0.3f, 1e-7f);

    /* magnitude clamp, then slew */
    NEAR(yaw_rate_limit(0.0f, 10.0f, 2.0f, 6.0f, 0.002f), 0.012f, 1e-7f);
    NEAR(yaw_rate_limit(1.995f, 10.0f, 2.0f, 6.0f, 0.002f), 2.0f, 1e-6f);
    NEAR(yaw_rate_limit(0.0f, -0.005f, 2.0f, 6.0f, 0.002f), -0.005f, 1e-7f);
    NEAR(yaw_rate_limit(0.0f, NAN, 2.0f, 6.0f, 0.002f), 0.0f, 1e-7f);

    /* closed heading loop through yaw_track_step, ideal yaw-rate plant, crossing
     * the seam from -3.0 to +3.0 the short way and never exceeding the limits */
    YawTrack t;
    YawTrackConfig c = yaw_track_default_config();
    c.max_rate_rad_s = 1.0f;
    c.max_accel_rad_s2 = 4.0f;
    yaw_track_init(&t, &c);
    t.mode = YAW_MODE_HEADING;
    t.heading_target_rad = 3.0f;
    float yaw = -3.0f;           /* unwrapped estimator yaw, as S10 integrates it */
    float prev = 0.0f;
    float min_yaw = yaw;
    for (int k = 0; k < 2500; ++k) {
        const float cmd = yaw_track_step(&t, yaw, 0.0f, 0.0f, 0.002f, 1u);
        CHECK(fabsf(cmd) <= 1.0f + 1e-6f);
        CHECK(fabsf(cmd - prev) <= 4.0f * 0.002f + 1e-6f);
        prev = cmd;
        yaw += cmd * 0.002f;
        if (yaw < min_yaw) { min_yaw = yaw; }
    }
    NEAR(yaw_wrap_pi(yaw - 3.0f), 0.0f, 1e-3f);
    CHECK(yaw < -3.0f);            /* went through -pi, not the long way round */
    CHECK(min_yaw > -3.0f - 0.40f); /* 0.283 rad needed + small overshoot */
    printf("  heading loop + rate limit: ok\n");
}

static void test_segment_geometry(void)
{
    /* straight line at 30 deg */
    const float h = 0.5235988f;
    YawPathSeg ln = { 1.0f, 2.0f, h, 3.0f, 0.0f };
    float x, y, p, e;
    yaw_seg_pose(&ln, 2.0f, &x, &y, &p);
    NEAR(x, 1.0f + 2.0f * cosf(h), 1e-5f);
    NEAR(y, 2.0f + 2.0f * sinf(h), 1e-5f);
    NEAR(p, h, 1e-6f);
    /* point 0.1 m to the left of s = 1 */
    float s = yaw_seg_project(&ln, 0.0f, 1.0f + cosf(h) - 0.1f * sinf(h),
                              2.0f + sinf(h) + 0.1f * cosf(h), &e);
    NEAR(s, 1.0f, 1e-5f);
    NEAR(e, 0.1f, 1e-5f);

    /* left arc R = 0.5 from origin heading 0: quarter turn ends at (0.5, 0.5, pi/2) */
    YawPathSeg arc = { 0.0f, 0.0f, 0.0f, PI_F, 2.0f };
    yaw_seg_pose(&arc, 0.25f * PI_F, &x, &y, &p);
    NEAR(x, 0.5f, 1e-5f);
    NEAR(y, 0.5f, 1e-5f);
    NEAR(p, 0.5f * PI_F, 1e-5f);
    /* full circle returns to the start */
    yaw_seg_pose(&arc, PI_F, &x, &y, &p);
    NEAR(x, 0.0f, 1e-5f);
    NEAR(y, 0.0f, 1e-5f);
    /* outside the left circle (right of path) -> e < 0; inside -> e > 0 */
    s = yaw_seg_project(&arc, 0.0f, 0.0f, -0.02f, &e);
    NEAR(s, 0.0f, 1e-5f);
    NEAR(e, -0.02f, 1e-5f);
    s = yaw_seg_project(&arc, 0.785f, 0.45f, 0.5f, &e);  /* inside, near s = pi/4 */
    NEAR(s, 0.25f * PI_F, 1e-5f);
    NEAR(e, 0.05f, 1e-5f);
    /* right arc (k < 0), centre (0, -0.5): inside a right-turning circle the
     * robot is right of the path -> e < 0 */
    YawPathSeg rarc = { 0.0f, 0.0f, 0.0f, PI_F, -2.0f };
    yaw_seg_pose(&rarc, 0.25f * PI_F, &x, &y, &p);
    NEAR(x, 0.5f, 1e-5f);
    NEAR(y, -0.5f, 1e-5f);
    NEAR(p, -0.5f * PI_F, 1e-5f);
    s = yaw_seg_project(&rarc, 0.785f, 0.45f, -0.5f, &e);
    NEAR(s, 0.25f * PI_F, 1e-5f);
    NEAR(e, -0.05f, 1e-5f);
    /* hint-local unwrapping: near the end of a full circle s stays near L, not 0 */
    s = yaw_seg_project(&arc, PI_F - 0.01f, 0.0f, 0.0f, &e);
    NEAR(s, PI_F, 1e-4f);
    printf("  segment geometry: ok\n");
}

static void test_figure8_and_progress(void)
{
    YawTrack t;
    yaw_track_init(&t, NULL);
    const float R = 0.5f;
    CHECK(yaw_path_figure8(&t, 0.0f, 0.0f, 0.0f, R, 0u) == 1u);
    CHECK(t.n_seg == 2u);
    NEAR(yaw_path_length(&t), 4.0f * PI_F * R, 1e-5f);
    /* both circles start at the crossing point with heading 0 */
    NEAR(t.seg[1].x0_m, 0.0f, 1e-5f);
    NEAR(t.seg[1].y0_m, 0.0f, 1e-5f);
    NEAR(t.seg[1].psi0_rad, 0.0f, 1e-5f);
    NEAR(t.seg_start_s_m[1], 2.0f * PI_F * R, 1e-5f);
    NEAR(yaw_path_curvature_at(&t, 0.1f), 2.0f, 1e-6f);
    NEAR(yaw_path_curvature_at(&t, 2.0f * PI_F * R + 0.1f), -2.0f, 1e-6f);
    NEAR(yaw_path_curvature_at(&t, 100.0f), -2.0f, 1e-6f);  /* open: clamps */
    /* reject bad radius */
    YawTrack bad;
    yaw_track_init(&bad, NULL);
    CHECK(yaw_path_figure8(&bad, 0.0f, 0.0f, 0.0f, 0.0f, 0u) == 0u);
    CHECK(yaw_path_figure8(&bad, 0.0f, 0.0f, 0.0f, NAN, 0u) == 0u);
    CHECK(yaw_path_append(&bad, -1.0f, 0.0f) == 0u);

    /* walk the path with a +1 cm left offset in 2 mm steps (one 500 Hz tick at
     * 1 m/s); progress must be monotone, change segment exactly once at the
     * crossing, and report e_y = +1 cm throughout */
    const float L = yaw_path_length(&t);
    float last_s = -1.0f;
    int seg_changes = 0;
    uint8_t last_seg = 0u;
    for (float s = 0.0f; s < L - 0.004f; s += 0.002f) {
        const uint8_t gi = (uint8_t)((s < 2.0f * PI_F * R) ? 0u : 1u);
        float x, y, p;
        yaw_seg_pose(&t.seg[gi], s - t.seg_start_s_m[gi], &x, &y, &p);
        x += -0.01f * sinf(p);
        y += 0.01f * cosf(p);
        yaw_path_track(&t, x, y, p);
        CHECK(t.diag.s_total_m >= last_s - 1e-4f);
        NEAR(t.diag.s_total_m, s, 2e-3f);
        NEAR(t.diag.e_y_m, 0.01f, 1e-4f);
        NEAR(t.diag.e_psi_rad, 0.0f, 1e-3f);
        if (t.diag.seg_idx != last_seg) { seg_changes++; last_seg = t.diag.seg_idx; }
        last_s = t.diag.s_total_m;
    }
    CHECK(seg_changes == 1);
    CHECK(t.diag.done == 0u);
    /* past the end of the open path: done, progress clamped at L */
    yaw_path_track(&t, 0.05f, 0.0f, 0.0f);
    CHECK(t.diag.done == 1u);
    NEAR(t.diag.s_total_m, L, 1e-4f);

    /* closed path wraps back to segment 0 */
    YawTrack c;
    yaw_track_init(&c, NULL);
    CHECK(yaw_path_figure8(&c, 0.0f, 0.0f, 0.0f, R, 1u) == 1u);
    c.seg_idx = 1u;
    c.s_seg_m = 2.0f * PI_F * R - 0.01f;
    yaw_path_track(&c, 0.02f, 0.0f, 0.0f);
    CHECK(c.diag.seg_idx == 0u);
    CHECK(c.diag.done == 0u);
    NEAR(c.diag.s_total_m, 0.02f, 1e-3f);
    printf("  figure-8 construction + progress tracking: ok\n");
}

/* Unicycle with an ideal yaw-rate response, starting 5 cm right of the path with
 * a 0.3 rad heading error. Pure kinematics, no balance dynamics. Returns the max
 * |e_y| after the first 1.5 m. */
static float kinematic_run(const YawTrackConfig *c, int *ticks)
{
    YawTrack t;
    yaw_track_init(&t, c);
    CHECK(yaw_path_figure8(&t, 0.0f, 0.0f, 0.0f, 0.5f, 0u) == 1u);
    t.mode = YAW_MODE_PATH;
    float x = 0.0f, y = -0.05f, psi = 0.3f, r = 0.0f;
    const float v = 0.3f, dt = 0.002f;
    float max_late = 0.0f;
    int k = 0;
    for (; k < 40000 && t.diag.done == 0u; ++k) {
        yaw_track_set_ext_pose(&t, x, y, psi, 1u);
        r = yaw_track_step(&t, psi, v, 0.0f, dt, 1u);
        CHECK(fabsf(r) <= t.cfg.max_rate_rad_s + 1e-5f);
        psi += r * dt;
        x += v * cosf(psi) * dt;
        y += v * sinf(psi) * dt;
        if (t.diag.s_total_m > 1.5f && fabsf(t.diag.e_y_m) > max_late) {
            max_late = fabsf(t.diag.e_y_m);
        }
    }
    CHECK(t.diag.done == 1u);
    *ticks = k;
    return max_late;
}

static void test_kinematic_closed_loop(void)
{
    /* The crossing is a curvature step of 2/R = 4 1/m, i.e. a yaw-rate step of
     * 1.2 rad/s at 0.3 m/s, slewed at 6 rad/s^2 over 0.2 s. For an ideal yaw-rate
     * plant the centred preview is half of that, 0.1 s: sub-mm error. */
    YawTrackConfig c = yaw_track_default_config();
    c.ff_preview_s = 0.10f;
    int k = 0;
    const float ideal = kinematic_run(&c, &k);
    CHECK(ideal < 0.001f);
    printf("  kinematic closed loop (preview 0.1 s): max |e_y| after 1.5 m = %.3f mm (%d ticks): ok\n",
           (double)(ideal * 1000.0f), k);
    /* The firmware default (0.2 s) is tuned in FIL for the lag of the real yaw
     * loop, so on an ideal plant it leads by 0.1 s: still converges, mm level. */
    const float dflt = kinematic_run(NULL, &k);
    CHECK(dflt < 0.005f);
    printf("  kinematic closed loop (default gains): max |e_y| after 1.5 m = %.3f mm: ok\n",
           (double)(dflt * 1000.0f));

    /* onboard odometry: x/y integrate est_v along est_yaw + offset */
    YawTrack o;
    yaw_track_init(&o, NULL);
    yaw_track_reset_odom(&o, 0.0f, 0.0f, 1.0f, 0.0f);  /* est yaw 1.0 == path yaw 0 */
    for (int i = 0; i < 500; ++i) {
        (void)yaw_track_step(&o, 1.0f + 0.5f * PI_F, 0.2f, 0.0f, 0.002f, 0u);
    }
    NEAR(o.x_m, 0.0f, 1e-4f);
    NEAR(o.y_m, 0.2f, 1e-4f);
}

static void test_default_off_passthrough(void)
{
    YawTrack t;
    yaw_track_init(&t, NULL);
    CHECK(t.mode == YAW_MODE_RATE);
    CHECK(yaw_path_figure8(&t, 0.0f, 0.0f, 0.0f, 0.5f, 0u) == 1u);
    /* RATE mode returns the host value bit-exactly, even with a path loaded */
    const float vals[] = { 0.0f, -0.0f, 0.3f, -1.7f, 12.5f, 1e-30f };
    for (size_t i = 0; i < sizeof(vals) / sizeof(vals[0]); ++i) {
        const float r = yaw_track_step(&t, 0.1f, 0.2f, vals[i], 0.002f, 1u);
        CHECK(memcmp(&r, &vals[i], sizeof(r)) == 0);
    }
    const float nan_in = yaw_track_step(&t, 0.1f, 0.2f, NAN, 0.002f, 1u);
    CHECK(isnan(nan_in));
    /* PATH mode while inactive (not BALANCING) also passes the host value */
    t.mode = YAW_MODE_PATH;
    NEAR(yaw_track_step(&t, 0.1f, 0.2f, 0.25f, 0.002f, 0u), 0.25f, 0.0f);
    /* non-finite host rate in an active mode is passed through for the FSM */
    CHECK(isnan(yaw_track_step(&t, 0.1f, 0.2f, NAN, 0.002f, 1u)));

    /* balance_ctrl: default RATE; invalid mode falls back to RATE; heading wrapped */
    static BalanceCtrl ctrl;
    BalanceCtrlConfig cfg = balance_ctrl_default_config();
    balance_ctrl_init(&ctrl, &cfg);
    CHECK(balance_ctrl_yaw_track(&ctrl)->mode == YAW_MODE_RATE);
    balance_ctrl_set_yaw_mode(&ctrl, 7u, 0.0f);
    CHECK(ctrl.yaw_track.mode == YAW_MODE_RATE);
    balance_ctrl_set_yaw_mode(&ctrl, YAW_MODE_HEADING, 4.0f);
    CHECK(ctrl.yaw_track.mode == YAW_MODE_HEADING);
    NEAR(ctrl.yaw_track.heading_target_rad, 4.0f - 2.0f * PI_F, 1e-5f);
    balance_ctrl_set_yaw_mode(&ctrl, YAW_MODE_RATE, 0.0f);
    CHECK(ctrl.yaw_track.mode == YAW_MODE_RATE);
    printf("  default-off pass-through: ok\n");
}

/* ---------------------------------------------------------------- FW7B tests */

/* distance from (x, y) to the figure-8 of radius R (circles centred (0, +-R)) */
static float fig8_dist(float x, float y, float R)
{
    const float d1 = fabsf(hypotf(x, y - R) - R);
    const float d2 = fabsf(hypotf(x, y + R) - R);
    return (d1 < d2) ? d1 : d2;
}

/* Kinematic figure-8 run (R 0.5 m, 0.3 m/s) on the robot's own dead reckoning.
 * The estimator yaw lives in its own frame (offset 5 rad from the path frame) and
 * may carry a gyro bias and a speed scale error. ext_ticks > 0 feeds the truth
 * pose as external pose for that many ticks, then stops the stream.
 * Returns the max truth distance to the path after the first 1.5 m. */
typedef struct {
    float speed_scale;   /* est_v = v * scale */
    float gyro_bias;     /* rad/s added to the estimator yaw rate */
    int   ext_ticks;     /* external pose ticks at the start (0 = none) */
    /* outputs */
    int   ticks;
    int   stale_tick;    /* first tick with diag.ext_pose_stale (-1 = never) */
    int   ext_used_after_stop; /* ticks with ext_pose_used after the stream stopped */
    float odom_end_err;
    float max_err;
} DrRun;

static void dr_run(DrRun *r)
{
    YawTrack t;
    yaw_track_init(&t, NULL);
    CHECK(yaw_path_figure8(&t, 0.0f, 0.0f, 0.0f, 0.5f, 0u) == 1u);
    t.mode = YAW_MODE_PATH;
    const float v = 0.3f, dt = 0.002f, frame = 5.0f;
    float x = 0.0f, y = -0.05f, psi = 0.3f;
    float est_yaw = psi + frame;
    /* odometry starts at the true pose (same as the FIL reset at the start) */
    yaw_track_reset_odom(&t, x, y, est_yaw, psi);
    r->max_err = 0.0f;
    r->stale_tick = -1;
    r->ext_used_after_stop = 0;
    int k = 0;
    for (; k < 40000 && t.diag.done == 0u; ++k) {
        if (k < r->ext_ticks) {
            yaw_track_set_ext_pose(&t, x, y, psi, 1u);
        }
        const float cmd = yaw_track_step(&t, est_yaw, v * r->speed_scale, 0.0f, dt, 1u);
        CHECK(isfinite(cmd) && fabsf(cmd) <= t.cfg.max_rate_rad_s + 1e-5f);
        CHECK(t.psi_odom_rad >= -PI_F && t.psi_odom_rad < PI_F);
        if (k < r->ext_ticks) {
            CHECK(t.diag.ext_pose_used == 1u);
            CHECK(t.x_m == x && t.y_m == y);   /* pose re-seated from the sample */
        } else if (r->ext_ticks > 0) {
            if (t.diag.ext_pose_used != 0u) { r->ext_used_after_stop++; }
            if (r->stale_tick < 0 && t.diag.ext_pose_stale != 0u) { r->stale_tick = k; }
        }
        psi += cmd * dt;
        est_yaw += (cmd + r->gyro_bias) * dt;
        x += v * cosf(psi) * dt;
        y += v * sinf(psi) * dt;
        if (t.diag.s_total_m > 1.5f) {
            const float e = fig8_dist(x, y, 0.5f);
            if (e > r->max_err) { r->max_err = e; }
        }
    }
    CHECK(t.diag.done == 1u);
    r->ticks = k;
    r->odom_end_err = hypotf(t.x_m - x, t.y_m - y);
}

static void test_ext_pose_timeout(void)
{
    /* 1. stream for 2 s, then stop: never re-used, stale within the timeout */
    DrRun r = { 1.03f, 0.003f, 1000, 0, 0, 0, 0.0f, 0.0f };
    dr_run(&r);
    CHECK(r.ext_used_after_stop == 0);
    const int timeout_ticks = (int)lroundf(yaw_track_default_config().ext_pose_timeout_s / 0.002f);
    CHECK(r.stale_tick >= 0);
    CHECK(r.stale_tick - (r.ext_ticks - 1) <= timeout_ticks + 1);
    CHECK(r.stale_tick - (r.ext_ticks - 1) >= timeout_ticks - 1);
    /* falls back to dead reckoning from the last sample and still finishes; with
     * the same estimator errors it is no worse than dead reckoning from the start */
    DrRun dr_only = { 1.03f, 0.003f, 0, 0, 0, 0, 0.0f, 0.0f };
    dr_run(&dr_only);
    printf("  [info] ext 2 s then DR: max err %.2f mm; DR only: %.2f mm\n",
           (double)(r.max_err * 1000.0f), (double)(dr_only.max_err * 1000.0f));
    CHECK(r.max_err < 0.10f);
    CHECK(r.max_err <= dr_only.max_err + 1e-3f);
    printf("  ext pose: stream stopped at tick %d, stale at tick %d (+%d ticks, timeout %d), "
           "fallback max err %.2f mm: ok\n", r.ext_ticks - 1, r.stale_tick,
           r.stale_tick - (r.ext_ticks - 1), timeout_ticks, (double)(r.max_err * 1000.0f));

    /* 2. unit level: a frozen pose is applied once; stale flag cleared by the next
     * valid sample; valid = 0 drops the stream without the stale flag */
    YawTrack t;
    yaw_track_init(&t, NULL);
    yaw_track_set_ext_pose(&t, 1.0f, 2.0f, 0.5f, 1u);
    (void)yaw_track_step(&t, 0.0f, 0.0f, 0.0f, 0.002f, 0u);
    CHECK(t.diag.ext_pose_used == 1u);
    NEAR(t.x_m, 1.0f, 0.0f);
    NEAR(t.psi_odom_rad, 0.5f, 1e-7f);
    /* now drive 0.2 m/s along +x of the path frame (est yaw -0.5 => heading 0):
     * the pose moves on from (1, 2) instead of sticking to the sample */
    for (int i = 0; i < 49; ++i) {
        (void)yaw_track_step(&t, -0.5f, 0.2f, 0.0f, 0.002f, 0u);
        CHECK(t.diag.ext_pose_used == 0u);
        CHECK(t.ext_pose_valid == 1u && t.diag.ext_pose_stale == 0u);
    }
    (void)yaw_track_step(&t, -0.5f, 0.2f, 0.0f, 0.002f, 0u);
    CHECK(t.ext_pose_valid == 0u && t.diag.ext_pose_stale == 1u);
    NEAR(t.x_m, 1.0f + 0.2f * 50.0f * 0.002f, 1e-5f);
    NEAR(t.y_m, 2.0f, 1e-5f);
    yaw_track_set_ext_pose(&t, 0.0f, 0.0f, 0.0f, 1u);
    CHECK(t.diag.ext_pose_stale == 0u);
    yaw_track_set_ext_pose(&t, 0.0f, 0.0f, 0.0f, 0u);
    (void)yaw_track_step(&t, -0.5f, 0.2f, 0.0f, 0.002f, 0u);
    CHECK(t.ext_pose_valid == 0u && t.diag.ext_pose_stale == 0u && t.diag.ext_pose_used == 0u);
    /* non-finite sample is rejected */
    yaw_track_set_ext_pose(&t, NAN, 0.0f, 0.0f, 1u);
    CHECK(t.ext_pose_valid == 0u);
    printf("  ext pose unit checks: ok\n");
}

static void test_dead_reckoning_closed_loop(void)
{
    /* ideal estimator (exact speed, no bias), estimator yaw in its own frame */
    DrRun a = { 1.0f, 0.0f, 0, 0, 0, 0, 0.0f, 0.0f };
    dr_run(&a);
    CHECK(a.max_err < 0.005f);
    CHECK(a.odom_end_err < 0.002f);
    /* 3 % speed scale error + 0.003 rad/s gyro bias: odometry drifts, the follower
     * tracks the odometry; the truth error stays at the drift level */
    DrRun b = { 1.03f, 0.003f, 0, 0, 0, 0, 0.0f, 0.0f };
    dr_run(&b);
    CHECK(b.max_err < 0.10f);
    /* the errors are really exercised (a speed scale error alone shrinks the
     * figure-8 about its start = end point, so the end error mostly cancels) */
    CHECK(b.max_err > 0.01f && b.max_err > 5.0f * a.max_err);
    printf("  dead-reckoning closed loop: ideal max err %.2f mm (odom end %.2f mm); "
           "3%% speed + 3 mrad/s bias max err %.1f mm (odom end %.1f mm): ok\n",
           (double)(a.max_err * 1000.0f), (double)(a.odom_end_err * 1000.0f),
           (double)(b.max_err * 1000.0f), (double)(b.odom_end_err * 1000.0f));
}

static void test_rate_handoff(void)
{
    const float dt = 0.002f;
    YawTrack t;
    yaw_track_init(&t, NULL);
    const float dmax = t.cfg.max_accel_rad_s2 * dt;
    /* HEADING with a 2 rad error at fixed yaw: output ramps to max_rate */
    t.mode = YAW_MODE_HEADING;
    t.heading_target_rad = 2.0f;
    float out = 0.0f;
    for (int i = 0; i < 400; ++i) {
        out = yaw_track_step(&t, 0.0f, 0.0f, 0.0f, dt, 1u);
    }
    NEAR(out, t.cfg.max_rate_rad_s, 1e-6f);
    /* switch to RATE while BALANCING, host 0: slew down at max_accel */
    t.mode = YAW_MODE_RATE;
    float prev = out;
    int n = 0;
    for (;; ++n) {
        out = yaw_track_step(&t, 0.0f, 0.0f, 0.0f, dt, 1u);
        CHECK(fabsf(out - prev) <= dmax + 1e-6f);
        prev = out;
        if (t.diag.rate_handoff == 0u) { break; }
        CHECK(n < 1000);
    }
    const float zero = 0.0f;
    CHECK(memcmp(&out, &zero, sizeof(out)) == 0);
    CHECK(n >= (int)(2.0f / dmax) - 1 && n <= (int)(2.0f / dmax) + 1);
    /* after the hand-off: RATE->RATE is bit-exact again */
    const float vals[] = { 0.0f, -0.0f, 0.3f, -1.7f, 12.5f, 1e-30f };
    for (size_t i = 0; i < sizeof(vals) / sizeof(vals[0]); ++i) {
        const float r = yaw_track_step(&t, 0.0f, 0.0f, vals[i], dt, 1u);
        CHECK(memcmp(&r, &vals[i], sizeof(r)) == 0);
    }
    printf("  HEADING->RATE while BALANCING: %d ticks slew to the host rate, then bit-exact: ok\n", n);

    /* host rate moving during the hand-off: follows it, ends exactly on it */
    yaw_track_init(&t, NULL);
    t.mode = YAW_MODE_HEADING;
    t.heading_target_rad = -2.0f;
    for (int i = 0; i < 400; ++i) { (void)yaw_track_step(&t, 0.0f, 0.0f, 0.0f, dt, 1u); }
    t.mode = YAW_MODE_RATE;
    float host = 0.5f;
    prev = t.rate_cmd_rad_s;
    for (int i = 0; i < 2000 && (i == 0 || t.diag.rate_handoff != 0u); ++i) {
        host = 0.5f + 0.2f * sinf(0.01f * (float)i);
        out = yaw_track_step(&t, 0.0f, 0.0f, host, dt, 1u);
        CHECK(fabsf(out - prev) <= dmax + 1e-6f);
        prev = out;
    }
    CHECK(t.diag.rate_handoff == 0u);
    CHECK(memcmp(&out, &host, sizeof(out)) == 0);
    /* a non-finite host rate during a hand-off is passed through for the FSM */
    yaw_track_init(&t, NULL);
    t.mode = YAW_MODE_PATH;
    CHECK(yaw_path_figure8(&t, 0.0f, 0.0f, 0.0f, 0.5f, 0u) == 1u);
    t.heading_target_rad = 0.0f;
    yaw_track_set_ext_pose(&t, 0.0f, 0.2f, 0.0f, 1u);   /* 20 cm left: turn right */
    for (int i = 0; i < 200; ++i) { (void)yaw_track_step(&t, 0.0f, 0.0f, 0.0f, dt, 1u); }
    CHECK(fabsf(t.rate_cmd_rad_s) > 0.5f);
    t.mode = YAW_MODE_RATE;
    CHECK(isnan(yaw_track_step(&t, 0.0f, 0.0f, NAN, dt, 1u)));
    CHECK(t.diag.rate_handoff == 0u);

    /* not BALANCING (active = 0): immediate bit-exact pass-through, also mid hand-off */
    yaw_track_init(&t, NULL);
    t.mode = YAW_MODE_HEADING;
    t.heading_target_rad = 2.0f;
    for (int i = 0; i < 400; ++i) { (void)yaw_track_step(&t, 0.0f, 0.0f, 0.0f, dt, 1u); }
    t.mode = YAW_MODE_RATE;
    (void)yaw_track_step(&t, 0.0f, 0.0f, 0.0f, dt, 1u);
    CHECK(t.diag.rate_handoff == 1u);
    const float h = -0.75f;
    out = yaw_track_step(&t, 0.0f, 0.0f, h, dt, 0u);
    CHECK(memcmp(&out, &h, sizeof(out)) == 0);
    CHECK(t.diag.rate_handoff == 0u);
    /* and back in BALANCING the hand-off is over: bit-exact */
    out = yaw_track_step(&t, 0.0f, 0.0f, 1.25f, dt, 1u);
    NEAR(out, 1.25f, 0.0f);
    /* HEADING mode, leaving BALANCING: immediate pass-through */
    yaw_track_init(&t, NULL);
    t.mode = YAW_MODE_HEADING;
    t.heading_target_rad = 2.0f;
    for (int i = 0; i < 400; ++i) { (void)yaw_track_step(&t, 0.0f, 0.0f, 0.0f, dt, 1u); }
    out = yaw_track_step(&t, 0.0f, 0.0f, 0.0f, dt, 0u);
    CHECK(memcmp(&out, &zero, sizeof(out)) == 0);
    printf("  RATE hand-off: moving host, NaN, not-BALANCING pass-through: ok\n");
}

/* straight path along +x; robot 5 cm right of it, heading 0, driving at v */
static float reverse_run(float v, float reverse_speed, int ticks, uint8_t *hold)
{
    YawTrack t;
    YawTrackConfig c = yaw_track_default_config();
    c.reverse_speed_m_s = reverse_speed;
    yaw_track_init(&t, &c);
    yaw_path_begin(&t, -5.0f, 0.0f, 0.0f, 0u);
    CHECK(yaw_path_append(&t, 20.0f, 0.0f) == 1u);
    t.mode = YAW_MODE_PATH;
    float x = 0.0f, y = -0.05f, psi = 0.0f;
    yaw_track_reset_odom(&t, x, y, psi, psi);
    for (int k = 0; k < ticks; ++k) {
        const float r = yaw_track_step(&t, psi, v, 0.0f, 0.002f, 1u);
        psi += r * 0.002f;
        x += v * cosf(psi) * 0.002f;
        y += v * sinf(psi) * 0.002f;
    }
    *hold = t.diag.reverse_hold;
    return y;
}

static void test_reverse(void)
{
    uint8_t hold = 0u;
    /* the YAW1 law in reverse (hold disabled by a huge threshold) steers away */
    const float y_old = reverse_run(-0.2f, 100.0f, 1500, &hold);
    CHECK(hold == 0u);
    CHECK(fabsf(y_old) > 0.10f);
    /* FW7B: reverse hold keeps the heading; the offset does not grow */
    const float y_new = reverse_run(-0.2f, 0.05f, 1500, &hold);
    CHECK(hold == 1u);
    NEAR(y_new, -0.05f, 1e-4f);
    /* forward at the same speed still converges onto the path */
    const float y_fwd = reverse_run(0.2f, 0.05f, 1500, &hold);
    CHECK(hold == 0u);
    CHECK(fabsf(y_fwd) < 0.005f);
    printf("  reverse: YAW1 law |e_y| 5 -> %.0f mm in 3 s; hold keeps %.1f mm; forward %.2f mm: ok\n",
           (double)(fabsf(y_old) * 1000.0f), (double)(fabsf(y_new) * 1000.0f),
           (double)(fabsf(y_fwd) * 1000.0f));

    /* hysteresis: engage below -0.05, release above -0.025; heading latched at entry */
    YawTrack t;
    yaw_track_init(&t, NULL);
    yaw_path_begin(&t, -5.0f, 0.0f, 0.0f, 0u);
    CHECK(yaw_path_append(&t, 20.0f, 0.0f) == 1u);
    t.mode = YAW_MODE_PATH;
    yaw_track_reset_odom(&t, 0.0f, -0.05f, 0.0f, 0.0f);
    (void)yaw_track_step(&t, 0.0f, -0.04f, 0.0f, 0.002f, 1u);
    CHECK(t.diag.reverse_hold == 0u);
    (void)yaw_track_step(&t, 0.1f, -0.06f, 0.0f, 0.002f, 1u);
    CHECK(t.diag.reverse_hold == 1u);
    NEAR(t.reverse_hold_psi_rad, 0.1f, 1e-6f);
    (void)yaw_track_step(&t, 0.1f, -0.03f, 0.0f, 0.002f, 1u);
    CHECK(t.diag.reverse_hold == 1u);
    (void)yaw_track_step(&t, 0.1f, -0.02f, 0.0f, 0.002f, 1u);
    CHECK(t.diag.reverse_hold == 0u);
    /* leaving BALANCING clears the hold */
    (void)yaw_track_step(&t, 0.1f, -0.3f, 0.0f, 0.002f, 1u);
    CHECK(t.diag.reverse_hold == 1u);
    (void)yaw_track_step(&t, 0.1f, -0.3f, 0.0f, 0.002f, 0u);
    CHECK(t.diag.reverse_hold == 0u);
    printf("  reverse hold hysteresis: ok\n");
}

static void test_large_yaw_precision(void)
{
    /* Estimator yaw far from zero (about 6400 turns); the odometry is reset to path
     * heading 0.1. YAW1 formed est_yaw + (0.1 - yaw0) in float: two roundings at
     * ulp(40000) = 3.9 mrad. FW7B folds exact increments into a wrapped heading. */
    const float yaw0 = 40000.3f;
    YawTrack t;
    yaw_track_init(&t, NULL);
    yaw_track_reset_odom(&t, 0.0f, 0.0f, yaw0, 0.1f);
    /* 1. straight 1 m at constant estimator yaw */
    for (int i = 0; i < 2500; ++i) {
        (void)yaw_track_step(&t, yaw0, 0.2f, 0.0f, 0.002f, 0u);
    }
    const float yaw1_err = fabsf(yaw_wrap_pi((yaw0 + (0.1f - yaw0)) - 0.1f));
    NEAR(t.psi_odom_rad, 0.1f, 1e-7f);
    /* 2500 float Euler sums: along-track rounding ~1e-4 m; direction exact */
    NEAR(t.x_m, cosf(0.1f), 5e-4f);
    NEAR(t.y_m, sinf(0.1f), 5e-4f);
    NEAR(atan2f(t.y_m, t.x_m), 0.1f, 3e-5f);
    printf("  [info] YAW1 form heading err %.3f mrad, FW7B direction err %.4f mrad\n",
           (double)(yaw1_err * 1000.0f), (double)(fabsf(atan2f(t.y_m, t.x_m) - 0.1f) * 1000.0f));
    CHECK(yaw1_err > 1e-4f);   /* the case really exercises the YAW1 defect */
    /* 2. turning: estimator yaw advances in exactly representable steps; the
     * wrapped heading equals 0.1 + (est - yaw0) mod 2 pi to float precision */
    float est = yaw0;
    const float step = 0.0078125f;   /* 2^-7, exact at this magnitude */
    double max_err = 0.0;
    for (int i = 0; i < 4000; ++i) {   /* 31.25 rad, about 5 turns */
        est += step;
        (void)yaw_track_step(&t, est, 0.0f, 0.0f, 0.002f, 0u);
        CHECK(t.psi_odom_rad >= -PI_F && t.psi_odom_rad < PI_F);
        const double truth = 0.1 + ((double)est - (double)yaw0);
        double e = fmod((double)t.psi_odom_rad - truth, 2.0 * 3.14159265358979323846);
        if (e > 3.14159265358979323846) { e -= 2.0 * 3.14159265358979323846; }
        if (e < -3.14159265358979323846) { e += 2.0 * 3.14159265358979323846; }
        if (fabs(e) > max_err) { max_err = fabs(e); }
    }
    CHECK(max_err < 2e-5);
    /* 3. an estimator re-wrap (jump by -2 pi * k) does not move the heading */
    const float before = t.psi_odom_rad;
    (void)yaw_track_step(&t, yaw_wrap_pi(est), 0.0f, 0.0f, 0.002f, 0u);
    NEAR(yaw_wrap_pi(t.psi_odom_rad - before), 0.0f, 5e-3f);  /* ulp(40000) */
    /* 3b. balance_ctrl feeds fmod(S10 yaw, 2 pi): the input jumps by 2 pi at the
     * seam; the heading must be continuous to float resolution */
    {
        YawTrack s;
        yaw_track_init(&s, NULL);
        yaw_track_reset_odom(&s, 0.0f, 0.0f, 6.2f, 0.0f);
        double yd = 6.2;
        double max_seam = 0.0;
        for (int i = 0; i < 2000; ++i) {   /* +1 rad/s for 4 s, crosses 2 pi once */
            yd += 0.002;
            (void)yaw_track_step(&s, (float)fmod(yd, 6.283185307179586), 0.0f, 0.0f, 0.002f, 0u);
            double e = (double)s.psi_odom_rad - (yd - 6.2);
            e -= 6.283185307179586 * floor((e + 3.141592653589793) / 6.283185307179586);
            if (fabs(e) > max_seam) { max_seam = fabs(e); }
        }
        CHECK(max_seam < 1e-5);
    }
    /* 4. HEADING mode at large yaw: the error is the wrapped heading error */
    t.mode = YAW_MODE_HEADING;
    t.heading_target_rad = t.psi_odom_rad;
    const float r = yaw_track_step(&t, yaw_wrap_pi(est), 0.0f, 0.0f, 0.002f, 1u);
    NEAR(r, 0.0f, 1e-6f);
    printf("  large yaw (%.0f rad): straight-line heading exact (YAW1 form err %.2f mrad), "
           "5-turn max err %.2e rad: ok\n", (double)yaw0, (double)(yaw1_err * 1000.0f), max_err);
}

static void test_config_guards(void)
{
    /* N8: the 2 pi constant equals 2 * pi in float */
    CHECK(YAW_TRACK_TWO_PI == 2.0f * YAW_TRACK_PI);
    /* min_speed floor: zero speed, zero cross-track error, min_speed 0 -> finite */
    YawTrackConfig c = yaw_track_default_config();
    c.min_speed_m_s = 0.0f;
    c.ext_pose_timeout_s = -1.0f;
    c.reverse_speed_m_s = NAN;
    YawTrack t;
    yaw_track_init(&t, &c);
    NEAR(t.cfg.min_speed_m_s, YAW_TRACK_MIN_SPEED_FLOOR, 0.0f);
    NEAR(t.cfg.ext_pose_timeout_s, 0.1f, 0.0f);
    NEAR(t.cfg.reverse_speed_m_s, 0.05f, 0.0f);
    CHECK(yaw_path_figure8(&t, 0.0f, 0.0f, 0.0f, 0.5f, 0u) == 1u);
    t.mode = YAW_MODE_PATH;
    t.cfg.min_speed_m_s = 0.0f;   /* poked after init (FIL hook path) */
    const float r = yaw_track_step(&t, 0.0f, 0.0f, 0.0f, 0.002f, 1u);
    CHECK(isfinite(r));
    NEAR(r, 0.0f, 1e-6f);
    printf("  config guards (2 pi, speed floor, timeouts): ok\n");
}

int main(void)
{
    test_wrap();
    test_heading_loop_and_limits();
    test_segment_geometry();
    test_figure8_and_progress();
    test_kinematic_closed_loop();
    test_default_off_passthrough();
    test_ext_pose_timeout();
    test_dead_reckoning_closed_loop();
    test_rate_handoff();
    test_reverse();
    test_large_yaw_precision();
    test_config_guards();
    printf("test_yaw_track: ALL PASSED\n");
    return 0;
}
