#ifndef BENCH_TWIN_ROBSTRIDE_H
#define BENCH_TWIN_ROBSTRIDE_H
/* OPTIONAL legacy AGV codec only; never the WBR runtime. Owner session,
 * 2026-10-30, SOURCE-SUPPORTED: AGV 1Mbit/s, ID 1 per bus, master 0xFD,
 * velocity mode. This codec does NOT implement that velocity-mode lifecycle. */
#ifndef BENCH_OPTIONAL_AGV_CODEC
#error "RobStride is optional AGV tooling: compile with BENCH_OPTIONAL_AGV_CODEC=1"
#endif
#include <stdint.h>
typedef struct { float position, velocity, torque, temperature; uint8_t error; } rs_feedback;
uint32_t rs_control_id(uint8_t motor, float torque);
void rs_pack_control(uint8_t out[8], float torque);
int rs_unpack_control(uint32_t id, const uint8_t in[8], uint8_t *motor, float *torque);
void rs_pack_feedback(uint8_t out[8], const rs_feedback *v);
int rs_unpack_feedback(uint32_t id, const uint8_t in[8], rs_feedback *v);
#endif
