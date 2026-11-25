/*
 * Five-Bar Forward Kinematics and Virtual Model Control (VMC)
 *
 * Adapted from DM-jump (User/Algorithm/VMC/VMC_calc.c) with S11 continuous
 * circle-intersection branch fix (S11_packet_tolerance/firmware_copy/src/VMC_calc.c):
 *   Copyright (c) 2024-2026 Trần Nguyên Bình
 *   Project: DM-jump (https://github.com/nguyenbinh-shark/DM-jump), MIT License
 *   Reference: dmBots/wheel-legged (https://github.com/dmBots/wheel-legged)
 *   See firmware/DM_JUMP_LICENSE.txt for the full MIT license notice.
 */

#include "VMC_calc.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static void vmc_calc_1_common(vmc_leg_t *vmc, const VmcInsState *ins, float dt)
{
    if (vmc == NULL || ins == NULL || dt <= 0.0f) {
        return;
    }

    const float pitch_rad = ins->Pitch;
    const float pitch_gyro = ins->Gyro[0];

    /* 1. Knee coordinates B and D from hip angles phi1 and phi4 */
    vmc->YD = vmc->l4 * sinf(vmc->phi4);
    vmc->YB = vmc->l1 * sinf(vmc->phi1);
    vmc->XD = vmc->l5 + vmc->l4 * cosf(vmc->phi4);
    vmc->XB = vmc->l1 * cosf(vmc->phi1);

    const float dx = vmc->XD - vmc->XB;
    const float dy = vmc->YD - vmc->YB;
    vmc->lBD = sqrtf(dx * dx + dy * dy);

    /* 2. Intersect lower-link circles and select the CAD lower assembly branch.
     *    Selecting the intersection with larger Y is continuous across vertical knee lines. */
    if (vmc->lBD > 1e-6f) {
        const float half = 0.5f * vmc->lBD;
        const float disc = vmc->l2 * vmc->l2 - half * half;
        const float h = (disc > 0.0f) ? sqrtf(disc) : 0.0f;
        const float midX = 0.5f * (vmc->XB + vmc->XD);
        const float midY = 0.5f * (vmc->YB + vmc->YD);
        const float perpX = -dy / vmc->lBD;
        const float perpY =  dx / vmc->lBD;
        const float c1X = midX + h * perpX;
        const float c1Y = midY + h * perpY;
        const float c2X = midX - h * perpX;
        const float c2Y = midY - h * perpY;
        if (c1Y >= c2Y) {
            vmc->XC = c1X;
            vmc->YC = c1Y;
        } else {
            vmc->XC = c2X;
            vmc->YC = c2Y;
        }
        vmc->phi2 = atan2f(vmc->YC - vmc->YB, vmc->XC - vmc->XB);
        vmc->phi3 = atan2f(vmc->YC - vmc->YD, vmc->XC - vmc->XD);
    }

    /* 3. Virtual polar leg length L0 and angle phi0 */
    const float cx_mid = vmc->XC - vmc->l5 / 2.0f;
    vmc->L0 = sqrtf(cx_mid * cx_mid + vmc->YC * vmc->YC);
    vmc->phi0 = atan2f(vmc->YC, cx_mid);
    vmc->alpha = VMC_PI / 2.0f - vmc->phi0;

    if (vmc->first_flag == 0u) {
        vmc->last_phi0 = vmc->phi0;
        vmc->last_L0 = vmc->L0;
        vmc->last_d_L0 = 0.0f;
        vmc->first_flag = 1u;
    }

    vmc->d_phi0 = (vmc->phi0 - vmc->last_phi0) / dt;
    const float sin32 = sinf(vmc->phi3 - vmc->phi2);
    const float denom = vmc->L0 * sin32;
    if (fabsf(denom) > 1e-6f) {
        vmc->j12 = (vmc->l1 * cosf(vmc->phi0 - vmc->phi3) * sinf(vmc->phi1 - vmc->phi2)) / denom;
        vmc->j22 = (vmc->l4 * cosf(vmc->phi0 - vmc->phi2) * sinf(vmc->phi3 - vmc->phi4)) / denom;
    } else {
        vmc->j12 = 0.0f;
        vmc->j22 = 0.0f;
    }
    vmc->d_alpha = -(vmc->j12 * vmc->qd1 + vmc->j22 * vmc->qd4);

    vmc->theta = vmc->alpha - pitch_rad;
    vmc->d_theta = vmc->d_alpha - pitch_gyro;

    vmc->last_phi0 = vmc->phi0;

    vmc->d_L0 = (vmc->L0 - vmc->last_L0) / dt;
    vmc->dd_L0 = (vmc->d_L0 - vmc->last_d_L0) / dt;

    vmc->last_d_L0 = vmc->d_L0;
    vmc->last_L0 = vmc->L0;

    vmc->dd_theta = (vmc->d_theta - vmc->last_d_theta) / dt;
    vmc->last_d_theta = vmc->d_theta;
}

void VMC_init(vmc_leg_t *vmc)
{
    if (vmc == NULL) {
        return;
    }
    memset(vmc, 0, sizeof(*vmc));
    vmc->l5 = 0.07900003157632506f;   /* H3 CAD hip spacing */
    vmc->l1 = 0.07249993968718672f;   /* H3 CAD upper link */
    vmc->l2 = 0.14000000823631453f;   /* H3 CAD lower link */
    vmc->l3 = 0.14000000823631453f;
    vmc->l4 = 0.07249993968718672f;
}

void VMC_calc_1_right(vmc_leg_t *vmc, const VmcInsState *ins, float dt)
{
    vmc_calc_1_common(vmc, ins, dt);
}

void VMC_calc_1_left(vmc_leg_t *vmc, const VmcInsState *ins, float dt)
{
    vmc_calc_1_common(vmc, ins, dt);
}

void VMC_calc_2(vmc_leg_t *vmc)
{
    if (vmc == NULL) {
        return;
    }
    const float sin32 = sinf(vmc->phi3 - vmc->phi2);
    const float denom_ang = vmc->L0 * sin32;
    if (fabsf(sin32) > 1e-6f && fabsf(denom_ang) > 1e-6f) {
        vmc->j11 = (vmc->l1 * sinf(vmc->phi0 - vmc->phi3) * sinf(vmc->phi1 - vmc->phi2)) / sin32;
        vmc->j12 = (vmc->l1 * cosf(vmc->phi0 - vmc->phi3) * sinf(vmc->phi1 - vmc->phi2)) / denom_ang;
        vmc->j21 = (vmc->l4 * sinf(vmc->phi0 - vmc->phi2) * sinf(vmc->phi3 - vmc->phi4)) / sin32;
        vmc->j22 = (vmc->l4 * cosf(vmc->phi0 - vmc->phi2) * sinf(vmc->phi3 - vmc->phi4)) / denom_ang;
    } else {
        vmc->j11 = 0.0f;
        vmc->j12 = 0.0f;
        vmc->j21 = 0.0f;
        vmc->j22 = 0.0f;
    }

    vmc->torque_set[0] = vmc->j11 * vmc->F0 + vmc->j12 * vmc->Tp;
    vmc->torque_set[1] = vmc->j21 * vmc->F0 + vmc->j22 * vmc->Tp;
}

uint8_t ground_detectionR(vmc_leg_t *vmc)
{
    if (vmc == NULL || fabsf(vmc->L0) <= 1e-6f) {
        return 1u;
    }
    vmc->FN = vmc->F0 * cosf(vmc->theta) + vmc->Tp * sinf(vmc->theta) / vmc->L0 + 6.0f;
    return (vmc->FN < 5.0f) ? 1u : 0u;
}

uint8_t ground_detectionL(vmc_leg_t *vmc)
{
    if (vmc == NULL || fabsf(vmc->L0) <= 1e-6f) {
        return 1u;
    }
    vmc->FN = vmc->F0 * cosf(vmc->theta) + vmc->Tp * sinf(vmc->theta) / vmc->L0 + 6.0f;
    return (vmc->FN < 5.0f) ? 1u : 0u;
}

float LQR_K_calc(const float coe[4], float len)
{
    return coe[0] * len * len * len + coe[1] * len * len + coe[2] * len + coe[3];
}
