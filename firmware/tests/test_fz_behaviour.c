/* Behaviour checks: gate holds and resets evidence, GROSS bypasses the gate, CONFIRMED
 * latches until reset, a code change clears ADVISORY, quantised slow walk and stand
 * noise stay OK for 10 s, and an 8 ms window uses time (dt = 1/2/3 ms). */
#include "s11_frozen.h"

#include <math.h>
#include <stdio.h>

static int fails = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); fails++; } else printf("pass: %s\n", msg); } while (0)

int main(void)
{
    const S11FzConfig hip = s11fz_default_config(0);
    S11FzAxis a; S11FzGate g;

    /* 1. Gate: a 3a-type frozen dither during a held transient never accumulates. */
    s11fz_reset(&a); s11fz_gate_init(&g);
    const double p3a[5] = {-.2, -.2, .2, .2, .2};
    int worst = S11FZ_OK;
    for (int k = 0; k < 500; ++k) {
        const double gy = (k % 25 == 0) ? 1.0 : 0.0;   /* a pitch-rate spike every 50 ms */
        const int gated = s11fz_gate_update(&g, 0.0, gy, 9.81, 2.0);
        const int s = s11fz_step(&hip, &a, 1.0, p3a[k % 5], 2.0, gated);
        if (s > worst) worst = s;
    }
    CHECK(worst == S11FZ_OK, "evidence resets while gate is held (spike every 50 ms < 60 ms hold)");

    /* 2. Gate hold length: raw spike at sample 0 keeps the gate for 60 ms after it. */
    s11fz_gate_init(&g);
    int last_gated = -1;
    for (int k = 0; k < 50; ++k) {
        if (s11fz_gate_update(&g, 0.0, k == 0 ? 2.0 : 0.0, 9.81, 2.0)) last_gated = k;
    }
    CHECK(last_gated == 30, "gate held through sample 30 (60 ms) after a single spike");

    /* 3. GROSS bypasses the gate. */
    s11fz_reset(&a);
    s11fz_step(&hip, &a, 1.0, 0.0, 2.0, 1);
    s11fz_step(&hip, &a, 1.0, 20.0, 2.0, 1);
    CHECK(a.state == S11FZ_CONFIRMED && a.reason == S11FZ_R_GROSS, "GROSS confirms while gated");

    /* 4. Latch: CONFIRMED survives code changes until reset. */
    s11fz_step(&hip, &a, 1.001, 0.0, 2.0, 0);
    s11fz_step(&hip, &a, 1.002, 0.0, 2.0, 0);
    CHECK(a.state == S11FZ_CONFIRMED, "CONFIRMED latched across code changes");
    s11fz_reset(&a);
    CHECK(a.state == S11FZ_OK, "explicit reset clears the latch");

    /* 5. ADVISORY clears on a code change and does not latch. */
    s11fz_reset(&a);
    s11fz_step(&hip, &a, 1.0, 0.0, 2.0, 0);
    for (int k = 0; k < 20; ++k) s11fz_step(&hip, &a, 1.0, (k & 1) ? .2 : -.2, 2.0, 0);
    const int was_adv = a.state == S11FZ_ADVISORY;
    s11fz_step(&hip, &a, 1.0 + hip.lsb_rad, .2, 2.0, 0);
    CHECK(was_adv && a.state == S11FZ_OK, "ADVISORY on sustained chatter, cleared by a code change");

    /* 6. Quantised slow walk: 0.1 rad/s, code advances 1 LSB every 4 ms, 10 s. */
    s11fz_reset(&a);
    worst = S11FZ_OK;
    double pos = 0.0;
    for (int k = 0; k < 5000; ++k) {
        pos += 0.1 * 0.002;
        const double code = floor(pos / hip.lsb_rad) * hip.lsb_rad;
        const int s = s11fz_step(&hip, &a, code, 0.1, 2.0, 0);
        if (s > worst) worst = s;
    }
    CHECK(worst == S11FZ_OK, "quantised 0.1 rad/s walk stays OK for 10 s");

    /* 7. Stand noise below one velocity count, 10 s. */
    s11fz_reset(&a);
    worst = S11FZ_OK;
    for (int k = 0; k < 5000; ++k) {
        const int s = s11fz_step(&hip, &a, 1.0, (k % 3 - 1) * 0.0219, 2.0, 0);
        if (s > worst) worst = s;
    }
    CHECK(worst == S11FZ_OK, "stationary noise within one velocity count stays OK for 10 s");

    /* 8. Windows are time-based: constant 0.5 rad/s at dt = 1, 2, 3 ms confirms by RECT/NET
     *    between 4 and 8 ms (8 ms window at 0.478 rad/s effective = 10 LSB). */
    for (int dt = 1; dt <= 3; ++dt) {
        s11fz_reset(&a);
        s11fz_step(&hip, &a, 1.0, 0.0, dt, 0);
        double t = -1;
        for (int k = 1; k <= 100 && t < 0; ++k)
            if (s11fz_step(&hip, &a, 1.0, 0.5, dt, 0) == S11FZ_CONFIRMED) t = k * dt;
        char msg[96];
        snprintf(msg, sizeof msg, "dt=%d ms: 0.5 rad/s frozen confirms at %.0f ms (<= 8)", dt, t);
        CHECK(t > 0 && t <= 8, msg);
    }
    printf("%d failure(s)\n", fails);
    return fails ? 1 : 0;
}
