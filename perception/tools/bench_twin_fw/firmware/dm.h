/* DM-jump 6855ef8, paths relative to that tree; SOURCE-SUPPORTED unless noted. */
#ifndef BENCH_DM_H
#define BENCH_DM_H
#include <stdint.h>
/* DM_Motor/dm4310_drv.h:29-31; only MIT implemented in this bench. */
#define DM_MIT 0x000u
#define DM_POS 0x100u
#define DM_SPEED 0x200u
/* DM_Motor/dm4310_drv.c:153-160,176-183. Save-zero absent in supplied source. */
#define DM_ENABLE 0xfcu
#define DM_DISABLE 0xfdu
typedef struct { float p, v, kp, kd, t; } dm_command;
typedef struct { float p, v, t; uint8_t id, state, tmos, tcoil; } dm_feedback;
typedef struct { float pmax, vmax, tmax; } dm_range;
extern const uint8_t dm_ids[4];
int dm_index(uint32_t id);
dm_range dm_limits(uint8_t id);
uint32_t dm_encode(float x, float lo, float hi, unsigned bits);
float dm_decode(uint32_t x, float lo, float hi, unsigned bits);
void dm_pack(uint8_t id, const dm_command *c, uint8_t d[8]);
void dm_unpack(uint8_t id, const uint8_t d[8], dm_command *c);
void dm_pack_feedback(const dm_feedback *f, uint8_t d[8]);
void dm_unpack_feedback(const uint8_t d[8], dm_feedback *f);
void dm_special(uint8_t code, uint8_t d[8]);
int dm_special_code(const uint8_t d[8]);
typedef struct { dm_command cmd; float p, v, t; uint8_t id, enabled; } dm_motor;
void dm_motor_init(dm_motor *m, uint8_t id);
void dm_motor_receive(dm_motor *m, const uint8_t d[8]);
void dm_motor_step(dm_motor *m, float dt);
void dm_motor_feedback(const dm_motor *m, dm_feedback *f);
#endif
