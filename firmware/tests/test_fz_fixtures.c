/* Frozen position code + reported-rate fixtures from S11_DETECTOR_SPEC.md section 4.1,
 * every phase, both signs, hip and wheel LSB, dt = 1 and 2 ms. Position is frozen from
 * the first sample. Each row states the required outcome and worst-case latency of the
 * STEP3RC design (see STEP3RC_REPORT.md section B for the comparison with the spec).
 * Usage: test_fz_fixtures [--survey]   (survey prints CSV of every trial) */
#include "s11_frozen.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

typedef enum { EXP_CONFIRM, EXP_ADVISORY, EXP_OK } Expect;

typedef struct {
    const char *name;
    double pat[10];
    int len;
    int dt_ms;
    Expect expect;
    double max_ms;      /* CONFIRM: latest confirm; ADVISORY: latest advisory */
    double spec_ms;     /* spec deadline (<0: spec says no trip / accepted) */
} Fixture;

#define P2(a, b) {a, b}, 2
static const Fixture FIX[] = {
    {"1a", P2(-20., 20.), 1, EXP_CONFIRM, 1, 4},
    {"1a", P2(-20., 20.), 2, EXP_CONFIRM, 2, 4},
    {"2a", {20., -20., -20., 20.}, 4, 1, EXP_CONFIRM, 1, 4},
    {"2a", {20., -20., -20., 20.}, 4, 2, EXP_CONFIRM, 2, 4},
    {"1c", P2(-.5, 1.), 1, EXP_CONFIRM, 4, 4},
    {"1c", P2(-.5, 1.), 2, EXP_CONFIRM, 4, 4},
    {"large_0p5", P2(-.5, .5), 1, EXP_CONFIRM, 6, -1},
    {"large_0p5", P2(-.5, .5), 2, EXP_CONFIRM, 6, 4},
    {"uni_0p15", {.15}, 1, 1, EXP_CONFIRM, 15, 10},
    {"uni_0p15", {.15}, 1, 2, EXP_CONFIRM, 16, 10},
    {"slow_0p1", {.1}, 1, 1, EXP_CONFIRM, 25, 20},
    {"slow_0p1", {.1}, 1, 2, EXP_CONFIRM, 26, 20},
    {"3a", {-.2, -.2, .2, .2, .2}, 5, 1, EXP_CONFIRM, 55, 15},
    {"3a", {-.2, -.2, .2, .2, .2}, 5, 2, EXP_CONFIRM, 60, 15},
    {"4b", {-.1, .1, .1, .1, .1}, 5, 1, EXP_CONFIRM, 43, 16},
    {"4b", {-.1, .1, .1, .1, .1}, 5, 2, EXP_CONFIRM, 46, 16},
    {"3b", {-.12, -.12, -.12, -.12, -.12, .12, .12, .12, .12, .12}, 10, 1, EXP_ADVISORY, 24, 10},
    {"3b", {-.12, -.12, -.12, -.12, -.12, .12, .12, .12, .12, .12}, 10, 2, EXP_ADVISORY, 24, 10},
    {"2b", {.2, .2, -.2, -.2}, 4, 1, EXP_ADVISORY, 13, -1},
    {"2b", {.2, .2, -.2, -.2}, 4, 2, EXP_ADVISORY, 14, 12},
    {"4a", {-.1, -.1, -.1, -.1, -.1, .1, .1, .1, .1, .1}, 10, 1, EXP_ADVISORY, 30, -1},
    {"4a", {-.1, -.1, -.1, -.1, -.1, .1, .1, .1, .1, .1}, 10, 2, EXP_ADVISORY, 30, 18},
    {"1b", P2(-.15, .15), 1, EXP_ADVISORY, 18, -1},
    {"1b", P2(-.15, .15), 2, EXP_ADVISORY, 18, -1},
    {"chatter_250hz", P2(-.2, .2), 1, EXP_ADVISORY, 13, -1},
    {"chatter_250hz", P2(-.2, .2), 2, EXP_ADVISORY, 14, -1},
    {"stand_noise", P2(-.005, .005), 1, EXP_OK, 0, -1},
    {"stand_noise", P2(-.005, .005), 2, EXP_OK, 0, -1},
    {"one_count", P2(-.0219, .0219), 2, EXP_OK, 0, -1},
};

static int run_trial(const Fixture *f, int wheel, double sign, int phase,
                     double *t_confirm, double *t_adv, S11FzReason *why)
{
    S11FzConfig c = s11fz_default_config(wheel);
    S11FzAxis a;
    s11fz_reset(&a);
    const double angle = 0.123456;  /* frozen code value */
    const int n = 1000 / f->dt_ms;  /* 1 s */
    *t_confirm = -1; *t_adv = -1; *why = S11FZ_R_NONE;
    s11fz_step(&c, &a, angle, 0.0, f->dt_ms, 0);  /* last healthy sample = onset */
    for (int k = 1; k <= n; ++k) {
        const double r = sign * f->pat[(phase + k - 1) % f->len];
        const S11FzState s = s11fz_step(&c, &a, angle, r, f->dt_ms, 0);
        if (s == S11FZ_ADVISORY && *t_adv < 0) *t_adv = k * f->dt_ms;
        if (s == S11FZ_CONFIRMED) { *t_confirm = k * f->dt_ms; *why = a.reason; break; }
    }
    return 0;
}

int main(int argc, char **argv)
{
    const int survey = argc > 1 && strcmp(argv[1], "--survey") == 0;
    int failures = 0, trials = 0;
    if (survey) printf("fixture,dt_ms,wheel,sign,phase,confirm_ms,advisory_ms,reason\n");
    printf("%s", survey ? "" : "fixture dt expect worst_ms limit_ms spec_ms result\n");
    for (size_t i = 0; i < sizeof(FIX) / sizeof(FIX[0]); ++i) {
        const Fixture *f = &FIX[i];
        double worst = 0; int bad = 0;
        for (int wheel = 0; wheel < 2; ++wheel)
        for (int s = 0; s < 2; ++s)
        for (int ph = 0; ph < f->len; ++ph) {
            double tc, ta; S11FzReason why;
            run_trial(f, wheel, s ? -1.0 : 1.0, ph, &tc, &ta, &why);
            trials++;
            if (survey) printf("%s,%d,%d,%d,%d,%g,%g,%d\n", f->name, f->dt_ms, wheel, s ? -1 : 1, ph, tc, ta, why);
            if (f->expect == EXP_CONFIRM) {
                if (tc < 0 || tc > f->max_ms) bad = 1;
                worst = fmax(worst, tc < 0 ? 1e9 : tc);
            } else if (f->expect == EXP_ADVISORY) {
                if (tc >= 0 || ta < 0 || ta > f->max_ms) bad = 1;
                worst = fmax(worst, ta < 0 ? 1e9 : ta);
            } else {
                if (tc >= 0 || ta >= 0) bad = 1;
            }
        }
        failures += bad;
        if (!survey) {
            printf("%-14s %d %-8s %6g %6g %6g %s\n", f->name, f->dt_ms,
                   f->expect == EXP_CONFIRM ? "CONFIRM" : f->expect == EXP_ADVISORY ? "ADVISORY" : "OK",
                   worst >= 1e9 ? -1 : worst, f->max_ms, f->spec_ms, bad ? "FAIL" : "pass");
        }
    }
    if (!survey) printf("%d trials, %d fixture rows failed\n", trials, failures);
    return failures ? 1 : 0;
}
