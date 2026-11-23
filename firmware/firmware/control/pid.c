/*
 * Classic PID Controller (Position & Delta) with Anti-Windup
 *
 * Adapted from DM-jump (User/Algorithm/PID/pid.c):
 *   Copyright (c) 2024-2026 Trần Nguyên Bình
 *   Project: DM-jump (DM-jump), MIT License
 *   Reference: dmBots/wheel-legged (https://github.com/dmBots/wheel-legged)
 *   See firmware/DM_JUMP_LICENSE.txt for the full MIT license notice.
 */

#include "pid.h"

#include <stddef.h>

static float limit_max(float value, float max_abs)
{
    if (value > max_abs) {
        return max_abs;
    }
    if (value < -max_abs) {
        return -max_abs;
    }
    return value;
}

void PID_init(PidTypeDef *pid, uint8_t mode, const float gains[3],
              float max_out, float max_iout)
{
    if (pid == NULL || gains == NULL) {
        return;
    }
    pid->mode = mode;
    pid->Kp = gains[0];
    pid->Ki = gains[1];
    pid->Kd = gains[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = 0.0f;
    pid->Dbuf[1] = 0.0f;
    pid->Dbuf[2] = 0.0f;
    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->out = 0.0f;
    pid->fdb = 0.0f;
    pid->set = 0.0f;
}

float PID_Calc(PidTypeDef *pid, float ref, float set)
{
    if (pid == NULL) {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;

    if (pid->mode == (uint8_t)PID_POSITION) {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = pid->error[0] - pid->error[1];
        pid->Dout = pid->Kd * pid->Dbuf[0];

        pid->Iout = limit_max(pid->Iout, pid->max_iout);
        pid->out = limit_max(pid->Pout + pid->Iout + pid->Dout, pid->max_out);
    } else if (pid->mode == (uint8_t)PID_DELTA) {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = pid->error[0] - 2.0f * pid->error[1] + pid->error[2];
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out = limit_max(pid->out + pid->Pout + pid->Iout + pid->Dout, pid->max_out);
    }
    return pid->out;
}

void PID_clear(PidTypeDef *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->error[0] = 0.0f;
    pid->error[1] = 0.0f;
    pid->error[2] = 0.0f;
    pid->Dbuf[0] = 0.0f;
    pid->Dbuf[1] = 0.0f;
    pid->Dbuf[2] = 0.0f;
    pid->out = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->fdb = 0.0f;
    pid->set = 0.0f;
}
