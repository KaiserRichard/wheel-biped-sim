/* Replays 110 healthy firmware-in-the-loop runs (976k samples at 500 Hz, all 6 axes):
 * S11FIX4 FIL/push/stand/run/step logs (no IMU) and STEP3RC F3 grid, slow run-up and
 * leg-pulse runs with IMU. Data is cut at the first fault/body contact. Requires zero
 * CONFIRMED and zero ADVISORY samples and prints the worst metric/threshold ratios.
 * Usage: test_fz_healthy_replay <healthy_corpus.bin> */
#include "s11_frozen.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const int WHEEL[6] = {0, 0, 1, 0, 0, 1};

static double angle_of(int ax, unsigned code)
{
    return WHEEL[ax] ? code * (24.0 / 65535.0) - 12.0 : code * (25.0 / 65536.0) - 12.5;
}

static double rate_of(int ax, unsigned code)
{
    const double half = WHEEL[ax] ? 45.0 : 30.0;
    return code * (2.0 * half / 4095.0) - half;
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "usage: %s corpus.bin\n", argv[0]); return 2; }
    FILE *f = fopen(argv[1], "rb");
    if (!f) { perror(argv[1]); return 2; }
    char magic[4]; uint32_t nruns;
    if (fread(magic, 1, 4, f) != 4 || memcmp(magic, "S11C", 4) != 0 || fread(&nruns, 4, 1, f) != 1) {
        fprintf(stderr, "bad corpus header\n"); return 2;
    }
    const char *names[4] = {"GROSS", "RECT8", "NET", "ADVISORY"};
    double worst[4] = {0}; char worst_run[4][128] = {{0}};
    unsigned long samples = 0, gated = 0, confirmed = 0, advisory = 0;
    for (uint32_t r = 0; r < nruns; ++r) {
        uint32_t nl, n; uint8_t has_imu; char name[128] = {0};
        if (fread(&nl, 4, 1, f) != 1 || nl >= sizeof(name) || fread(name, 1, nl, f) != nl
            || fread(&n, 4, 1, f) != 1 || fread(&has_imu, 1, 1, f) != 1) {
            fprintf(stderr, "truncated corpus at run %u\n", r); return 2;
        }
        int16_t *rec = malloc((size_t)n * 15u * sizeof(int16_t));
        if (!rec || fread(rec, sizeof(int16_t), (size_t)n * 15u, f) != (size_t)n * 15u) {
            fprintf(stderr, "truncated data in %s\n", name); return 2;
        }
        S11FzAxis ax[6]; S11FzConfig cfg[6]; S11FzGate gate;
        s11fz_gate_init(&gate);
        for (int i = 0; i < 6; ++i) { s11fz_reset(&ax[i]); cfg[i] = s11fz_default_config(WHEEL[i]); }
        for (uint32_t k = 0; k < n; ++k) {
            const int16_t *p = rec + (size_t)k * 15u;
            const int g = s11fz_gate_update(&gate, p[12] * 1e-3, p[13] * 1e-3, p[14] * 1e-2, 2.0);
            gated += (unsigned long)g;
            for (int i = 0; i < 6; ++i) {
                const S11FzState s = s11fz_step(&cfg[i], &ax[i], angle_of(i, (uint16_t)p[i]),
                                                rate_of(i, (uint16_t)p[6 + i]), 2.0, g);
                if (s == S11FZ_CONFIRMED) {
                    if (confirmed++ < 10) printf("CONFIRMED %s axis %d sample %u reason %d\n", name, i, k, ax[i].reason);
                    s11fz_reset(&ax[i]);
                } else if (s == S11FZ_ADVISORY) {
                    if (advisory++ < 10) printf("ADVISORY %s axis %d sample %u\n", name, i, k);
                }
            }
            samples++;
        }
        for (int i = 0; i < 6; ++i)
            for (int m = 0; m < 4; ++m)
                if (ax[i].peak_ratio[m] > worst[m]) {
                    worst[m] = ax[i].peak_ratio[m];
                    snprintf(worst_run[m], sizeof(worst_run[m]), "%s axis %d", name, i);
                }
        free(rec);
    }
    fclose(f);
    printf("runs %u, samples %lu, gated fraction %.4f\n", nruns, samples, (double)gated / samples);
    for (int m = 0; m < 4; ++m)
        printf("worst %-8s metric/threshold %.3f (margin %.2fx)  %s\n", names[m], worst[m],
               worst[m] > 0 ? 1.0 / worst[m] : 0.0, worst_run[m]);
    printf("CONFIRMED samples %lu, ADVISORY samples %lu\n", confirmed, advisory);
    return (confirmed || advisory) ? 1 : 0;
}
