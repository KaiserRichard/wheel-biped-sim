#include "../firmware/robstride_decode.h"
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint16_t to_u16(float x, float lo, float hi) {
  return (uint16_t)((x - lo) * 65535.0f / (hi - lo));
}

int main(void) {
  rs_decoded_t o;
  uint8_t fb[8];
  uint16_t p = to_u16(1.25f, -12.57f, 12.57f);
  uint16_t v = to_u16(-3.0f, -30.0f, 30.0f);
  uint16_t t = to_u16(8.0f, -60.0f, 60.0f);
  uint16_t temp = 427u;
  fb[0] = (uint8_t)(p >> 8); fb[1] = (uint8_t)p;
  fb[2] = (uint8_t)(v >> 8); fb[3] = (uint8_t)v;
  fb[4] = (uint8_t)(t >> 8); fb[5] = (uint8_t)t;
  fb[6] = (uint8_t)(temp >> 8); fb[7] = (uint8_t)temp;
  uint32_t id = rs_ext_build_id(RS_TYPE_FEEDBACK, (uint16_t)((2u << 14) | (5u << 8) | 1u), 0xFDu);
  assert(id == (((uint32_t)RS_TYPE_FEEDBACK << 24) | (2u << 22) | (5u << 16) | (1u << 8) | 0xFDu));
  assert(rs_decode_frame(id, 1, fb, 8, &o) == 0);
  assert(o.kind == RS_DECODE_FEEDBACK && o.mode == 2 && o.fault == 5);
  assert(o.motor_id == 1 && o.master_id == 0xFD);
  assert(fabsf(o.position - 1.25f) < 0.001f);
  assert(fabsf(o.velocity + 3.0f) < 0.001f);
  assert(fabsf(o.torque - 8.0f) < 0.002f);
  assert(fabsf(o.temperature - 42.7f) < 0.001f);

  id = rs_ext_build_id(RS_TYPE_ACTIVE_RPT, (uint16_t)((1u << 14) | (3u << 8) | 1u), 0xFDu);
  assert(rs_decode_frame(id, 1, fb, 8, &o) == 0 && o.kind == RS_DECODE_FEEDBACK);

  uint8_t param[8] = {0x05, 0x70, 0, 0, 2, 0, 0, 0};
  id = rs_ext_build_id(RS_TYPE_WRITE_PARAM, 0x00FDu, 1u);
  assert(rs_decode_frame(id, 1, param, 8, &o) == 0);
  assert(o.kind == RS_DECODE_PARAM_WRITE && o.has_run_mode && o.run_mode == 2);
  float speed = -6.25f;
  param[0] = 0x0a; param[1] = 0x70;
  memcpy(param + 4, &speed, sizeof(speed));
  assert(rs_decode_frame(id, 1, param, 8, &o) == 0);
  assert(o.has_speed && fabsf(o.commanded_speed - speed) < 0.0001f);

  uint16_t tq = to_u16(5.0f, -60.0f, 60.0f);
  uint16_t cp = to_u16(-2.0f, -12.57f, 12.57f);
  uint16_t cv = to_u16(4.5f, -20.0f, 20.0f);
  uint16_t kp = to_u16(800.0f, 0.0f, 5000.0f);
  uint16_t kd = to_u16(12.0f, 0.0f, 100.0f);
  id = rs_ext_build_id(RS_TYPE_CONTROL, tq, 1u);
  fb[0] = (uint8_t)(cp >> 8); fb[1] = (uint8_t)cp;
  fb[2] = (uint8_t)(cv >> 8); fb[3] = (uint8_t)cv;
  fb[4] = (uint8_t)(kp >> 8); fb[5] = (uint8_t)kp;
  fb[6] = (uint8_t)(kd >> 8); fb[7] = (uint8_t)kd;
  assert(rs_decode_frame(id, 1, fb, 8, &o) == 0 && o.kind == RS_DECODE_COMMAND);
  assert(fabsf(o.torque - 5.0f) < 0.002f && fabsf(o.position + 2.0f) < 0.001f);
  assert(fabsf(o.velocity - 4.5f) < 0.001f && fabsf(o.kp - 800.0f) < 0.1f);
  assert(fabsf(o.kd - 12.0f) < 0.01f);
  id = rs_ext_build_id(RS_TYPE_ENABLE, 0x00FDu, 1u);
  assert(rs_decode_frame(id, 1, fb, 8, &o) == 0 && o.kind == RS_DECODE_COMMAND && !o.has_speed);
  id = rs_ext_build_id(RS_TYPE_STOP, 0x00FDu, 1u);
  assert(rs_decode_frame(id, 1, fb, 8, &o) == 0 && o.kind == RS_DECODE_COMMAND);

  id = rs_ext_build_id(RS_TYPE_READ_PARAM, 0x00FDu, 1u);
  assert(rs_decode_frame(id, 1, param, 8, &o) == 0 && o.kind == RS_DECODE_PARAM_READ);
  id = rs_ext_build_id(RS_TYPE_FAULT_FB, 0, 1);
  memset(fb, 0, sizeof(fb)); fb[0] = 0x12; fb[1] = 0x34; fb[2] = 0x56; fb[3] = 0x78;
  assert(rs_decode_frame(id, 1, fb, 8, &o) == 0);
  assert(o.kind == RS_DECODE_FAULT && o.fault_detail == 0x12345678u);
  assert(rs_decode_frame(id, 1, fb, 7, &o) != 0);
  assert(rs_decode_frame(id, 0, fb, 8, &o) != 0);
  puts("decoder: 6 cases passed (control/admin commands, feedback, active report, parameter read/write, fault, rejects)");
  return 0;
}
