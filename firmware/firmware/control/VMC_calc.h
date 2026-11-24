/*
 * Five-Bar Forward Kinematics and Virtual Model Control (VMC)
 *
 * Adapted from DM-jump (User/Algorithm/VMC/VMC_calc.h) with S11 continuous
 * circle-intersection branch fix (S11_packet_tolerance/firmware_copy/src/VMC_calc.h):
 *   Copyright (c) 2024-2026 Trần Nguyên Bình
 *   Project: DM-jump (DM-jump), MIT License
 *   Reference: dmBots/wheel-legged (https://github.com/dmBots/wheel-legged)
 *   See firmware/DM_JUMP_LICENSE.txt for the full MIT license notice.
 *
 * Pure C11, zero HAL/RTOS dependencies, zero dynamic memory.
 */

#ifndef WBR_CONTROL_VMC_CALC_H
#define WBR_CONTROL_VMC_CALC_H

#include <stdint.h>

#define VMC_PI              3.141592653589793f

/* Leg length PID parameters (used during jump FSM phases). */
#define LEG_PID_KP          700.0f
#define LEG_PID_KI          0.0f
#define LEG_PID_KD          2500.0f
#define LEG_PID_MAX_OUT     130.0f
#define LEG_PID_MAX_IOUT    0.0f

typedef struct {
    float   Gyro[3];       /* [0] pitch rate, [1] roll rate, [2] yaw rate (DM-jump convention) */
    float   Pitch;         /* rad */
    float   Roll;          /* rad */
    float   YawTotalAngle; /* rad */
    uint8_t ins_flag;
} VmcInsState;

typedef struct {
    /* Fixed link lengths (m) - H3 CAD geometry */
    float l5;
    float l1;
    float l2;
    float l3;
    float l4;

    /* Joint coordinates in sagittal leg frame */
    float XB, YB;
    float XD, YD;
    float XC, YC;

    /* Virtual polar leg coordinates */
    float L0;
    float phi0;
    float alpha;
    float d_alpha;

    float lBD;

    float d_phi0;
    float last_phi0;

    float phi2;
    float phi3;
    float phi1;
    float phi4;
    float qd1;
    float qd4;

    /* Jacobian transpose J^T entries */
    float j11, j12;
    float j21, j22;
    float torque_set[2];

    /* Virtual Cartesian task-space force and torque */
    float F0;
    float Tp;

    /* LQR balance states */
    float theta;
    float d_theta;
    float last_d_theta;
    float dd_theta;

    float d_L0;
    float dd_L0;
    float last_L0;
    float last_d_L0;

    float FN;

    uint8_t first_flag;
    uint8_t leg_flag;
} vmc_leg_t;

void VMC_init(vmc_leg_t *vmc);
void VMC_calc_1_right(vmc_leg_t *vmc, const VmcInsState *ins, float dt);
void VMC_calc_1_left(vmc_leg_t *vmc, const VmcInsState *ins, float dt);
void VMC_calc_2(vmc_leg_t *vmc);
uint8_t ground_detectionR(vmc_leg_t *vmc);
uint8_t ground_detectionL(vmc_leg_t *vmc);
float LQR_K_calc(const float coe[4], float len);

#endif /* WBR_CONTROL_VMC_CALC_H */
