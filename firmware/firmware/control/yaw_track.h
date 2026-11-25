/*
 * YAW1: optional heading loop and path follower (yaw angle -> yaw-rate command).
 *
 * The existing firmware takes a host yaw-RATE target only. This module adds two
 * optional modes on top of it; the output is always a yaw-rate target that goes
 * into the unchanged safety FSM (authority scaling, brake ramp) and H5 yaw PI.
 *
 *   YAW_MODE_RATE    (0, default): host yaw-rate target passed through untouched.
 *   YAW_MODE_HEADING (1): r = r_ff + kp * wrap(psi_target - psi), limited.
 *   YAW_MODE_PATH    (2): cross-track + heading-error law on a chained path of
 *                         line/arc segments:
 *       psi_d = psi_path(s) - atan(k_e * e_y / max(|v|, v_min))
 *       r     = kappa(s + v*T_p) * v + kp * wrap(psi_d - psi)    then limited.
 *
 * "Limited" = magnitude clamp max_rate and slew clamp max_accel (per tick).
 *
 * Pose (FW7B): the controller always uses the onboard dead-reckoning pose
 * (estimator v and yaw increments; heading kept wrapped in [-pi, pi) so a long
 * run with a large unwrapped estimator yaw loses no precision here). A valid
 * external pose re-seats that pose once, on the tick after it arrives, so a
 * frozen external pose is never re-used; between updates and after the stream
 * stops the robot runs on dead reckoning. If no valid update arrives for
 * ext_pose_timeout_s, ext_pose_valid is cleared and diag.ext_pose_stale is set
 * (cleared by the next valid update).
 *
 * Mode switches (FW7B): HEADING/PATH -> RATE while active (BALANCING) slews from
 * the last tracker command to the host rate at max_accel (no magnitude clamp),
 * then passes the host rate through bit-exactly. When not active the host rate is
 * passed through immediately (the safety FSM owns stop/rest/fault yaw handling).
 * RATE mode that was already RATE on the previous tick is a bit-exact
 * pass-through. RATE -> HEADING/PATH starts the slew from the last host rate.
 *
 * Reverse (FW7B): the cross-track law assumes forward travel. In PATH mode with
 * est_v < -reverse_speed_m_s the cross-track correction and curvature
 * feed-forward are disabled and the heading at reverse entry is held
 * (diag.reverse_hold); forward following resumes above -reverse_speed_m_s / 2.
 *
 * End of an open path: diag.done latches and the final path heading is held; to
 * run the path again, rebuild it (yaw_path_begin / yaw_path_figure8).
 * rate_cmd_rad_s is the sanitised last output (a non-finite host rate is stored
 * as 0 there, but returned unchanged).
 *
 * Pure C11, float only, no dynamic memory, no HAL.
 */
#ifndef WBR_CONTROL_YAW_TRACK_H
#define WBR_CONTROL_YAW_TRACK_H

#include <stdint.h>

#define YAW_TRACK_MAX_SEGS 16u
#define YAW_TRACK_PI       3.14159265358979f
#define YAW_TRACK_TWO_PI   6.28318530717959f

enum {
    YAW_MODE_RATE    = 0,
    YAW_MODE_HEADING = 1,
    YAW_MODE_PATH    = 2
};

typedef struct {
    float x0_m;          /* start position (path/odometry frame) */
    float y0_m;
    float psi0_rad;      /* start heading */
    float length_m;      /* > 0 */
    float curvature_1_m; /* signed: + = left turn, 0 = straight line */
} YawPathSeg;

typedef struct {
    float heading_kp_1_s;   /* yaw-rate per rad heading error */
    float max_rate_rad_s;   /* |r| clamp */
    float max_accel_rad_s2; /* dr/dt clamp */
    float xtrack_k_1_s;     /* Stanley-type cross-track gain */
    float xtrack_max_rad;   /* clamp of the cross-track heading correction */
    float min_speed_m_s;    /* speed floor in the cross-track term */
    float ff_preview_s;     /* curvature feed-forward read at s + v*preview, so a
                             * slew-limited rate is centred on a curvature step */
    float ext_pose_timeout_s; /* FW7B: external pose age limit */
    float reverse_speed_m_s;  /* FW7B: PATH reverse-hold threshold (> 0) */
} YawTrackConfig;

/* Floor applied to min_speed_m_s in the cross-track denominator (FW7B). */
#define YAW_TRACK_MIN_SPEED_FLOOR 0.01f

typedef struct {
    /* Per-tick outputs / diagnostics (path mode) */
    float   e_y_m;          /* signed cross-track error, + = robot left of path */
    float   e_psi_rad;      /* wrap(psi - psi_path) */
    float   s_total_m;      /* progress along the whole path */
    float   kappa_1_m;      /* path curvature at the projection */
    uint8_t seg_idx;
    uint8_t done;           /* reached the end of an open path */
    /* FW7B flags */
    uint8_t ext_pose_used;  /* a valid external pose re-seated the pose this tick */
    uint8_t ext_pose_stale; /* external pose timed out; running on dead reckoning */
    uint8_t reverse_hold;   /* PATH: reversing, cross-track off, heading held */
    uint8_t rate_handoff;   /* RATE: slewing from the tracker command to host */
} YawTrackDiag;

typedef struct {
    YawTrackConfig cfg;
    uint8_t   mode;
    float     heading_target_rad;
    float     rate_cmd_rad_s;  /* limited output of the last active tick */
    /* odometry pose */
    float     x_m, y_m;
    float     psi_odom_rad;    /* FW7B: odometry heading, wrapped [-pi, pi) */
    float     est_yaw_ref_rad; /* FW7B: estimator yaw already folded into psi_odom */
    uint8_t   ext_pose_valid;
    uint8_t   ext_pose_new;    /* FW7B: not yet applied to the odometry pose */
    uint16_t  ext_pose_age_ticks;
    float     ext_x_m, ext_y_m, ext_yaw_rad;
    /* FW7B mode-switch / reverse state */
    uint8_t   rate_handoff;
    uint8_t   reverse_hold;
    float     reverse_hold_psi_rad;
    /* path */
    YawPathSeg seg[YAW_TRACK_MAX_SEGS];
    uint8_t   n_seg;
    uint8_t   closed;
    uint8_t   seg_idx;
    float     s_seg_m;
    float     seg_start_s_m[YAW_TRACK_MAX_SEGS];
    YawTrackDiag diag;
} YawTrack;

YawTrackConfig yaw_track_default_config(void);
void  yaw_track_init(YawTrack *t, const YawTrackConfig *cfg);

/* Wrap any finite angle into [-pi, pi). Non-finite input returns 0. */
float yaw_wrap_pi(float a);
/* Heading P law with feed-forward, unlimited. */
float yaw_heading_rate(float kp, float target_rad, float yaw_rad, float ff_rad_s);
/* Clamp |cmd| to max_rate, then slew from prev by at most max_accel*dt. */
float yaw_rate_limit(float prev, float cmd, float max_rate, float max_accel, float dt);

/* Path construction: begin at a pose, then append chained segments. */
void    yaw_path_begin(YawTrack *t, float x0, float y0, float psi0, uint8_t closed);
uint8_t yaw_path_append(YawTrack *t, float length_m, float curvature_1_m);
/* Figure-8: left circle then right circle of radius r, both through the start
 * pose (the crossing point). Total length 4*pi*r. Returns 1 on success. */
uint8_t yaw_path_figure8(YawTrack *t, float x0, float y0, float psi0, float radius_m,
                         uint8_t closed);
float   yaw_path_length(const YawTrack *t);
/* Curvature at a total path position (wraps on a closed path, clamps on open). */
float   yaw_path_curvature_at(const YawTrack *t, float s_total_m);

/* Segment geometry */
void yaw_seg_pose(const YawPathSeg *g, float s, float *x, float *y, float *psi);
/* Local projection around s_hint: returns s (may leave [0, L]) and signed e_y. */
float yaw_seg_project(const YawPathSeg *g, float s_hint, float px, float py, float *e_y);

/* Advance the progress tracker to the projection of (px, py); fills t->diag. */
void yaw_path_track(YawTrack *t, float px, float py, float psi);

/* Odometry & external pose */
void yaw_track_reset_odom(YawTrack *t, float x, float y, float yaw_now_est, float yaw_set);
void yaw_track_set_ext_pose(YawTrack *t, float x, float y, float yaw, uint8_t valid);

/* One 500 Hz tick. Integrates odometry (always) and applies a new external pose,
 * then, if active != 0 and the mode is HEADING or PATH, returns the limited
 * yaw-rate command. Otherwise returns host_rate_rad_s unchanged (and resets the
 * slew state to it), except for the active HEADING/PATH -> RATE hand-off slew. */
float yaw_track_step(YawTrack *t, float est_yaw_rad, float est_v_m_s,
                     float host_rate_rad_s, float dt_s, uint8_t active);

#endif /* WBR_CONTROL_YAW_TRACK_H */
