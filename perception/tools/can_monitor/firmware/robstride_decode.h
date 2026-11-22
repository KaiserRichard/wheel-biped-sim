#ifndef CAN_MONITOR_ROBSTRIDE_DECODE_H
#define CAN_MONITOR_ROBSTRIDE_DECODE_H

#include <stdint.h>

enum {
  RS_TYPE_GET_ID = 0, RS_TYPE_CONTROL = 1, RS_TYPE_FEEDBACK = 2,
  RS_TYPE_ENABLE = 3, RS_TYPE_STOP = 4, RS_TYPE_SET_ZERO = 6,
  RS_TYPE_READ_PARAM = 17, RS_TYPE_WRITE_PARAM = 18,
  RS_TYPE_FAULT_FB = 21, RS_TYPE_SAVE_PARAM = 22, RS_TYPE_SET_BAUD = 23,
  RS_TYPE_ACTIVE_RPT = 24, RS_TYPE_PROTOCOL_SW = 25, RS_TYPE_READ_VER = 26,
  RS_PARAM_RUN_MODE = 0x7005, RS_PARAM_SPD_REF = 0x700A
};

typedef enum {
  RS_DECODE_OTHER = 0, RS_DECODE_FEEDBACK, RS_DECODE_COMMAND,
  RS_DECODE_PARAM_READ, RS_DECODE_PARAM_WRITE, RS_DECODE_FAULT,
  RS_DECODE_INVALID
} rs_decode_kind_t;

typedef struct {
  rs_decode_kind_t kind;
  uint8_t mode, fault, motor_id, master_id;
  uint8_t has_speed, has_run_mode, run_mode;
  float position, velocity, torque, temperature, kp, kd, commanded_speed;
  uint16_t param_index;
  uint32_t fault_detail;
} rs_decoded_t;

uint32_t rs_ext_build_id(uint8_t comm_type, uint16_t data2, uint8_t target_id);
int rs_decode_frame(uint32_t id, int extended, const uint8_t *data,
                    uint8_t len, rs_decoded_t *out);

#endif
