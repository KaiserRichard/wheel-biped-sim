/*
 * FW8 host tests for the confirmed should-fix findings of the firmware safety review.
 *   arm_stop        F-04: arm switch LOW while balancing brakes (CONTROLLED_STOP)
 *                         instead of a latched torque-off FAULT.
 *   startup_motion  F-06: rotation during the S10 startup window must not be
 *                         learned as gyro bias.
 *   startup_gravity F-06: a startup mean specific force far above 1 g must not
 *                         be accepted as a gravity reference.
 * (F-05 is covered by test_target_hal fatal_halt_repeat.)
 */
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "safety_fsm.h"
#include "control/s10_estimator.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(expr)                                                       \
    do {                                                                  \
        if (!(expr)) {                                                    \
            fprintf(stderr, "%s:%d: %s: check `%s' failed.\n",            \
                    __FILE__, __LINE__, __func__, #expr);                 \
            abort();                                                      \
        }                                                                 \
    } while (0)

static SafetyInputs clean_inputs(void)
{
    SafetyInputs in;
    memset(&in, 0, sizeof(in));
    in.dt_ms              = 2u;
    in.boot_self_test_ok  = 1u;
    in.dump_continuity_ok = 1u;
    in.v_bus_v            = 0.5f;
    in.v_batt_ocv_v       = 24.0f;
    in.imu_est_valid      = 1u;
    in.ground_left        = 1u;
    in.ground_right       = 1u;
    in.radio_age_ms       = 4u;
    in.host_age_ms        = 20u;
    in.host_valid         = 1u;
    return in;
}

static SafetyInputs balancing_inputs(void)
{
    SafetyInputs in = clean_inputs();
    in.k1_closed        = 1u;
    in.k2_closed        = 1u;
    in.v_bus_v          = 24.0f;
    in.drives_healthy_6 = 1u;
    in.radio_arm_sw     = 1u;
    in.pitch_deg        = 1.0f;
    return in;
}

static void reach_balancing(SafetyFsm *fsm, SafetyOutput *out)
{
    safety_fsm_init(fsm);
    SafetyInputs in = clean_inputs();
    in.local_reset_req = 1u;
    in.local_arm_req   = 1u;
    safety_fsm_step(fsm, &in, out);
    CHECK(out->state == SAFETY_STATE_PRECHARGE);
    in.local_reset_req  = 0u;
    in.local_arm_req    = 0u;
    in.k1_closed        = 1u;
    in.k2_closed        = 1u;
    in.precharge_dv_ok  = 1u;
    in.v_bus_v          = 24.0f;
    in.drives_healthy_6 = 1u;
    in.radio_arm_sw     = 1u;
    safety_fsm_step(fsm, &in, out);
    CHECK(out->state == SAFETY_STATE_ARMED);
    in = balancing_inputs();
    in.stand_handoff_req = 1u;
    safety_fsm_step(fsm, &in, out);
    CHECK(out->state == SAFETY_STATE_BALANCING);
}

/* F-04 */
static void arm_stop(void)
{
    SafetyFsm fsm;
    SafetyOutput out;
    reach_balancing(&fsm, &out);

    /* Drive at 0.3 m/s so the brake ramp is observable. */
    SafetyInputs in = balancing_inputs();
    in.v_target_m_s = 0.3f;
    in.v_est_m_s    = 0.3f;
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_BALANCING);

    /* Pilot drops the arm switch with a fresh radio link and no other fault. */
    in.radio_arm_sw = 0u;
    safety_fsm_step(&fsm, &in, &out);
    printf("arm LOW while balancing -> state=%d reason=%d torque=%u\n",
           (int)out.state, (int)out.reason, (unsigned)out.torque_enable);
    CHECK(out.state == SAFETY_STATE_CONTROLLED_STOP);
    CHECK(out.reason == SAFETY_REASON_ARM_SW_DROPPED);
    CHECK(out.torque_enable == 1u);

    /* Holding arm LOW keeps braking on the ramp (never restarts it, never cuts). */
    float prev_v = fsm.v_ref_m_s;
    uint32_t steps = 0u;
    while (out.state == SAFETY_STATE_CONTROLLED_STOP && steps < 2000u) {
        in.v_est_m_s = fsm.v_ref_m_s;
        in.pitch_deg = 0.5f;
        safety_fsm_step(&fsm, &in, &out);
        CHECK(out.state != SAFETY_STATE_FAULT);
        CHECK(fsm.v_ref_m_s <= prev_v + 1e-6f);
        prev_v = fsm.v_ref_m_s;
        if (out.state == SAFETY_STATE_CONTROLLED_STOP) {
            CHECK(out.torque_enable == 1u);
        }
        ++steps;
    }
    printf("braked to rest in %u ms -> state=%d\n", (unsigned)(steps * 2u), (int)out.state);
    CHECK(out.state == SAFETY_STATE_REST);
    CHECK(out.torque_enable == 1u);

    /* At REST with arm LOW and a fresh radio: existing clean disarm. */
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_DISARMED);
    CHECK(out.reason == SAFETY_REASON_CLEAN_DISARM);

    /* The handset torque cut is still immediate: radio e-stop switch -> FAULT. */
    reach_balancing(&fsm, &out);
    in = balancing_inputs();
    in.radio_arm_sw   = 0u;
    in.radio_estop_sw = 1u;
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_FAULT);
    CHECK(out.torque_enable == 0u);

    /* Tilt still cuts torque from the arm-drop stop. */
    reach_balancing(&fsm, &out);
    in = balancing_inputs();
    in.radio_arm_sw = 0u;
    safety_fsm_step(&fsm, &in, &out);
    CHECK(out.state == SAFETY_STATE_CONTROLLED_STOP);
    in.pitch_deg = 80.0f;
    for (int i = 0; i < 100 && out.state != SAFETY_STATE_FAULT; ++i) {
        safety_fsm_step(&fsm, &in, &out);
    }
    CHECK(out.state == SAFETY_STATE_FAULT);
    CHECK(out.reason == SAFETY_REASON_TILT_LIMIT);
}

/* F-06 helpers: S10 fed at 1 kHz with a fixed IMU reading. */
static void feed(S10Estimator *s, int32_t *tick, int32_t n,
                 const double gyro[3], const double force[3])
{
    for (int32_t i = 0; i < n; ++i) {
        s10_imu(s, (*tick)++, gyro, force);
    }
}

static void ready_encoder(S10Estimator *s, int32_t tick)
{
    S10Encoder enc;
    memset(&enc, 0, sizeof(enc));
    s10_encoder(s, tick, 0, &enc);
}

static void startup_motion(void)
{
    const double lever[3] = {0.0, 0.0, 0.0};
    const double g = 9.80665;
    const double level[3] = {0.0, 0.0, g};
    const double still[3] = {0.0, 0.002, 0.0};    /* true gyro bias 0.002 rad/s */
    const double turning[3] = {0.0, 0.302, 0.0};  /* 0.3 rad/s real pitch rotation */
    S10Estimator s;
    int32_t tick = 1;

    /* Robot is being rotated (handled) for the whole startup window. */
    s10_init(&s, 0.0603, lever);
    feed(&s, &tick, S10_STARTUP, turning, level);
    ready_encoder(&s, tick - 1);
    printf("rotating startup: ready=%d bias_pitch=%.4f rad/s\n",
           (int)s10_ready(&s), s.bias[1]);
    CHECK(s10_ready(&s) == 0);   /* must not qualify while moving */
    CHECK(s.startup_failed == 0); /* not latched: it may still qualify later */

    /* Once held still for a full window, it qualifies with the true bias. */
    feed(&s, &tick, S10_STARTUP, still, level);
    ready_encoder(&s, tick - 1);
    printf("after still window: ready=%d bias_pitch=%.4f rad/s\n",
           (int)s10_ready(&s), s.bias[1]);
    CHECK(s10_ready(&s) == 1);
    CHECK(fabs(s.bias[1] - 0.002) < 1e-9);

    /* A single jolt in the middle of an otherwise still window restarts it. */
    s10_init(&s, 0.0603, lever);
    tick = 1;
    feed(&s, &tick, 50, still, level);
    feed(&s, &tick, 1, turning, level);
    feed(&s, &tick, S10_STARTUP - 51, still, level);
    ready_encoder(&s, tick - 1);
    CHECK(s10_ready(&s) == 0);
    feed(&s, &tick, 51, still, level);
    ready_encoder(&s, tick - 1);
    CHECK(s10_ready(&s) == 1);
}

static void startup_gravity(void)
{
    const double lever[3] = {0.0, 0.0, 0.0};
    const double g = 9.80665;
    const double heavy[3] = {0.0, 0.0, 1.30 * g};  /* shaken / mis-scaled */
    const double gyro[3] = {0.0, 0.0, 0.0};
    S10Estimator s;
    int32_t tick = 1;

    s10_init(&s, 0.0603, lever);
    feed(&s, &tick, S10_STARTUP, gyro, heavy);
    ready_encoder(&s, tick - 1);
    printf("1.30 g startup: ready=%d\n", (int)s10_ready(&s));
    CHECK(s10_ready(&s) == 0);

    /* Statically tilted 20 deg at 1 g is a valid reference (stand gate decides later). */
    const double tilt = 20.0 * 0.017453292519943295;
    const double tilted[3] = {-g * sin(tilt), 0.0, g * cos(tilt)};
    s10_init(&s, 0.0603, lever);
    tick = 1;
    feed(&s, &tick, S10_STARTUP, gyro, tilted);
    ready_encoder(&s, tick - 1);
    CHECK(s10_ready(&s) == 1);
    CHECK(fabs(fabs(s.pitch) - tilt) < 1e-6);

    /* Free fall never qualifies. */
    const double fall[3] = {0.0, 0.0, 0.1 * g};
    s10_init(&s, 0.0603, lever);
    tick = 1;
    feed(&s, &tick, S10_STARTUP, gyro, fall);
    ready_encoder(&s, tick - 1);
    CHECK(s10_ready(&s) == 0);
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: test_fw8_safety <arm_stop|startup_motion|startup_gravity>\n");
        return 2;
    }
    if (strcmp(argv[1], "arm_stop") == 0) {
        arm_stop();
    } else if (strcmp(argv[1], "startup_motion") == 0) {
        startup_motion();
    } else if (strcmp(argv[1], "startup_gravity") == 0) {
        startup_gravity();
    } else {
        fprintf(stderr, "unknown scenario %s\n", argv[1]);
        return 2;
    }
    puts("PASS");
    return 0;
}
