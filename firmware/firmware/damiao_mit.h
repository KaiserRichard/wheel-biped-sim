/*
 * DAMIAO DM4310 / DM6215 Pure-C MIT CAN Frame Codec
 *
 * Adapted from DM-jump (User/Devices/DM_Motor/dm4310_drv.h/.c):
 *   Copyright (c) 2024-2026 Trần Nguyên Bình
 *   Project: DM-jump (https://github.com/nguyenbinh-shark/DM-jump), MIT License
 *   Reference: dmBots/wheel-legged (https://github.com/dmBots/wheel-legged)
 *   See firmware/DM_JUMP_LICENSE.txt for the full MIT license notice.
 *
 * Pure C11, zero HAL/RTOS dependencies, zero dynamic memory, fixed-width types.
 */

#ifndef WBR_DAMIAO_MIT_H
#define WBR_DAMIAO_MIT_H

#include <stdint.h>

#define DAMIAO_CAN_DLC          8u
#define DAMIAO_NUM_AXES         6u

#define DAMIAO_CMD_ENABLE_BYTE  0xFCu
#define DAMIAO_CMD_DISABLE_BYTE 0xFDu

/* Drive status nibble values (rx[0] >> 4).
 * 0xD (comm loss) is verified in S11_REPORT.md:54; exact healthy-enabled code
 * on installed drive firmware must be confirmed on bench (0x1 is illustrative). */
#define DAMIAO_DRIVE_STATUS_DISABLED        0x0u
#define DAMIAO_DRIVE_STATUS_ENABLED_DEFAULT 0x1u
#define DAMIAO_DRIVE_STATUS_COMM_LOSS       0xDu

typedef enum {
    DAMIAO_MOTOR_DM4310 = 0,
    DAMIAO_MOTOR_DM6215 = 1
} DamiaoMotorType;

typedef enum {
    DAMIAO_OK = 0,
    DAMIAO_ERR_NULL_PTR = 1,
    DAMIAO_ERR_NONFINITE = 2,
    DAMIAO_ERR_INVALID_LIMITS = 3,
    DAMIAO_ERR_BAD_DLC = 4,
    DAMIAO_ERR_INVALID_AXIS = 5
} DamiaoStatus;

typedef struct {
    float p_min;
    float p_max;
    float v_min;
    float v_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    float t_min;
    float t_max;
    float cmd_torque_clamp; /* Firmware safety torque clamp (7.0 Nm hip, 2.0 Nm wheel) */
} DamiaoLimits;

typedef struct {
    uint8_t         axis_index;
    uint8_t         can_bus;       /* 1 = FDCAN1, 2 = FDCAN2, 3 = FDCAN3 */
    uint16_t        cmd_can_id;    /* 0x001..0x006 */
    uint16_t        reply_can_id;  /* 0x011..0x016 */
    DamiaoMotorType motor_type;
    DamiaoLimits    limits;
} DamiaoAxisConfig;

typedef struct {
    float pos;
    float vel;
    float kp;
    float kd;
    float torque;
} DamiaoMitCmd;

typedef struct {
    uint8_t  motor_id;       /* Low nibble of byte 0 (0..15) */
    uint8_t  status_nibble;  /* High nibble of byte 0 (0..15) */
    uint16_t p_raw;          /* 16-bit raw position code */
    uint16_t v_raw;          /* 12-bit raw velocity code */
    uint16_t t_raw;          /* 12-bit raw torque code */
    float    pos;            /* rad */
    float    vel;            /* rad/s */
    float    torque;         /* N*m */
    float    t_mos_c;        /* MOSFET temperature (deg C) */
    float    t_coil_c;       /* Coil temperature (deg C) */
} DamiaoMitFeedback;

/* Preset limits matching DM-jump dm4310_drv.h:34-55 & FIRMWARE_DESIGN_v2.md Section 4.2. */
DamiaoLimits damiao_limits_dm4310(void);
DamiaoLimits damiao_limits_dm6215(void);

/* Retrieve unified 6-axis bus/ID/limit configuration (axis 0..5). */
DamiaoStatus damiao_get_axis_config(uint8_t axis, DamiaoAxisConfig *out_cfg);

/* Fixed-point quantization helpers (clamped to [x_min, x_max], rounded to nearest). */
uint16_t damiao_float_to_uint(float x, float x_min, float x_max, uint8_t bits);
float damiao_uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);

/* Pack 8-byte MIT command frame using codec limits (clamps to codec min/max). */
DamiaoStatus damiao_pack_mit_cmd(const DamiaoLimits *lim,
                                 const DamiaoMitCmd *cmd,
                                 uint8_t out_frame[DAMIAO_CAN_DLC]);

/* Pack 8-byte MIT command frame while also enforcing firmware cmd_torque_clamp. */
DamiaoStatus damiao_pack_mit_cmd_clamped(const DamiaoLimits *lim,
                                         const DamiaoMitCmd *cmd,
                                         uint8_t out_frame[DAMIAO_CAN_DLC]);

/* Unpack 8-byte MIT command frame back to physical units. */
DamiaoStatus damiao_unpack_mit_cmd(const DamiaoLimits *lim,
                                   const uint8_t in_frame[DAMIAO_CAN_DLC],
                                   DamiaoMitCmd *out_cmd);

/* Pack 8-byte motor feedback reply frame (for host SIL/HIL test vectors). */
DamiaoStatus damiao_pack_feedback(const DamiaoLimits *lim,
                                  const DamiaoMitFeedback *fb,
                                  uint8_t out_frame[DAMIAO_CAN_DLC]);

/* Unpack 8-byte motor feedback reply frame. Rejects dlc != 8. */
DamiaoStatus damiao_unpack_feedback(const DamiaoLimits *lim,
                                    const uint8_t in_frame[DAMIAO_CAN_DLC],
                                    uint8_t dlc,
                                    DamiaoMitFeedback *out_fb);

/* Build 8-byte DAMIAO enable (0xFF..FC) and disable (0xFF..FD) frames. */
void damiao_pack_enable(uint8_t out_frame[DAMIAO_CAN_DLC]);
void damiao_pack_disable(uint8_t out_frame[DAMIAO_CAN_DLC]);

/* Check if decoded status nibble indicates a healthy enabled drive. */
uint8_t damiao_status_is_healthy(uint8_t status_nibble, uint8_t expected_enabled_code);

#endif /* WBR_DAMIAO_MIT_H */
