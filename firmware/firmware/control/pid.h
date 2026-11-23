/*
 * Classic PID Controller (Position & Delta) with Anti-Windup
 *
 * Adapted from DM-jump (User/Algorithm/PID/pid.h):
 *   Copyright (c) 2024-2026 Trần Nguyên Bình
 *   Project: DM-jump (DM-jump), MIT License
 *   Reference: dmBots/wheel-legged (https://github.com/dmBots/wheel-legged)
 *   See firmware/DM_JUMP_LICENSE.txt for the full MIT license notice.
 *
 * Pure C11, zero HAL/RTOS dependencies, zero dynamic memory.
 */

#ifndef WBR_CONTROL_PID_H
#define WBR_CONTROL_PID_H

#include <stdint.h>

enum PidMode {
    PID_POSITION = 0,
    PID_DELTA = 1
};

typedef struct {
    uint8_t mode;

    float Kp;
    float Ki;
    float Kd;

    float max_out;
    float max_iout;

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];
    float error[3];
} PidTypeDef;

void PID_init(PidTypeDef *pid, uint8_t mode, const float gains[3],
              float max_out, float max_iout);
float PID_Calc(PidTypeDef *pid, float ref, float set);
void PID_clear(PidTypeDef *pid);

#endif /* WBR_CONTROL_PID_H */
