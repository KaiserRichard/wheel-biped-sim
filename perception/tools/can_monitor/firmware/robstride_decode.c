#include "robstride_decode.h"
#include <string.h>

#define P_MIN (-12.57f)
#define P_MAX (12.57f)
#define FB_V_MIN (-30.0f)
#define FB_V_MAX (30.0f)
#define CMD_V_MIN (-20.0f)
#define CMD_V_MAX (20.0f)
#define T_MIN (-60.0f)
#define T_MAX (60.0f)
#define KP_MAX (5000.0f)
#define KD_MAX (100.0f)

static float uint_to_float(uint16_t code, float lo, float hi) {
  return (float)code * (hi - lo) / 65535.0f + lo;
}

uint32_t rs_ext_build_id(uint8_t type, uint16_t data2, uint8_t target) {
  return (((uint32_t)type & 0x1fu) << 24) |
         (((uint32_t)data2 & 0xffffu) << 8) | target;
}

int rs_decode_frame(uint32_t id, int extended, const uint8_t *d,
                    uint8_t len, rs_decoded_t *o) {
  if (o == NULL) return -1;
  memset(o, 0, sizeof(*o));
  if (!extended || id > 0x1fffffffu || d == NULL) return -1;
  const uint8_t type = (uint8_t)((id >> 24) & 0x1fu);
  if (type == RS_TYPE_FEEDBACK || type == RS_TYPE_ACTIVE_RPT) {
    if (len != 8u) return -1;
    uint16_t p = (uint16_t)(((uint16_t)d[0] << 8) | d[1]);
    uint16_t v = (uint16_t)(((uint16_t)d[2] << 8) | d[3]);
    uint16_t t = (uint16_t)(((uint16_t)d[4] << 8) | d[5]);
    uint16_t temp = (uint16_t)(((uint16_t)d[6] << 8) | d[7]);
    o->kind = RS_DECODE_FEEDBACK;
    o->mode = (uint8_t)((id >> 22) & 3u);
    o->fault = (uint8_t)((id >> 16) & 0x3fu);
    o->motor_id = (uint8_t)(id >> 8);
    o->master_id = (uint8_t)id;
    o->position = uint_to_float(p, P_MIN, P_MAX);
    o->velocity = uint_to_float(v, FB_V_MIN, FB_V_MAX);
    o->torque = uint_to_float(t, T_MIN, T_MAX);
    o->temperature = (float)temp / 10.0f;
  } else if (type == RS_TYPE_CONTROL || type == RS_TYPE_GET_ID ||
             type == RS_TYPE_ENABLE || type == RS_TYPE_STOP ||
             type == RS_TYPE_SET_ZERO || type == RS_TYPE_SAVE_PARAM ||
             type == RS_TYPE_SET_BAUD || type == RS_TYPE_PROTOCOL_SW ||
             type == RS_TYPE_READ_VER) {
    if (len != 8u) return -1;
    o->kind = RS_DECODE_COMMAND;
    o->motor_id = (uint8_t)id;
    if (type == RS_TYPE_CONTROL) {
      o->torque = uint_to_float((uint16_t)(id >> 8), T_MIN, T_MAX);
      o->position = uint_to_float((uint16_t)(((uint16_t)d[0] << 8) | d[1]), P_MIN, P_MAX);
      o->velocity = uint_to_float((uint16_t)(((uint16_t)d[2] << 8) | d[3]), CMD_V_MIN, CMD_V_MAX);
      o->kp = uint_to_float((uint16_t)(((uint16_t)d[4] << 8) | d[5]), 0.0f, KP_MAX);
      o->kd = uint_to_float((uint16_t)(((uint16_t)d[6] << 8) | d[7]), 0.0f, KD_MAX);
      o->commanded_speed = o->velocity;
      o->has_speed = 1;
    }
  } else if (type == RS_TYPE_READ_PARAM || type == RS_TYPE_WRITE_PARAM) {
    if (len != 8u) return -1;
    o->kind = type == RS_TYPE_READ_PARAM ? RS_DECODE_PARAM_READ : RS_DECODE_PARAM_WRITE;
    o->motor_id = (uint8_t)id;
    o->master_id = (uint8_t)(id >> 8);
    o->param_index = (uint16_t)(d[0] | ((uint16_t)d[1] << 8));
    if (type == RS_TYPE_WRITE_PARAM && o->param_index == RS_PARAM_RUN_MODE) {
      o->has_run_mode = 1;
      o->run_mode = d[4];
    } else if (type == RS_TYPE_WRITE_PARAM && o->param_index == RS_PARAM_SPD_REF) {
      memcpy(&o->commanded_speed, d + 4, sizeof(float));
      o->has_speed = 1;
    }
  } else if (type == RS_TYPE_FAULT_FB) {
    if (len != 8u) return -1;
    o->kind = RS_DECODE_FAULT;
    o->motor_id = (uint8_t)id;
    o->fault_detail = ((uint32_t)d[0] << 24) | ((uint32_t)d[1] << 16) |
                      ((uint32_t)d[2] << 8) | d[3];
  } else {
    o->kind = RS_DECODE_OTHER;
  }
  return 0;
}
