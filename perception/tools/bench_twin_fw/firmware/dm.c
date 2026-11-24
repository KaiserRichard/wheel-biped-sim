/* Independently implemented from DM-jump 6855ef8 (MIT, Tran Nguyen Binh).
 * SOURCE-SUPPORTED wire constants cite original file:line below. */
#include "dm.h"
/* chassisR_task.c:105-107 (CAN1 hips 6,8/wheel 0), chassisL_task.c:89 (CAN2 wheel 1).
 * ASSUMED reduced bench topology collapses these four unique IDs onto one wire. */
const uint8_t dm_ids[4] = {6, 8, 0, 1};
int dm_index(uint32_t id) { for (int i=0;i<4;i++) if(id==dm_ids[i]) return i; return -1; }
dm_range dm_limits(uint8_t id) {
 /* DM_Motor/dm4310_drv.h:34-55. */
 dm_range r = id==0 || id==1 ? (dm_range){12,45,18} : (dm_range){12.5f,30,10};
 return r;
}
static float clip(float x,float lo,float hi) { return x<lo?lo:x>hi?hi:x; }
uint32_t dm_encode(float x,float lo,float hi,unsigned bits) {
 /* dm4310_drv.c:50-55, with ASSUMED bench input clipping before conversion. */
 return (uint32_t)((clip(x,lo,hi)-lo)*(float)((1u<<bits)-1u)/(hi-lo));
}
float dm_decode(uint32_t x,float lo,float hi,unsigned bits) {
 /* dm4310_drv.c:65-70. */
 return (float)x*(hi-lo)/(float)((1u<<bits)-1u)+lo;
}
void dm_pack(uint8_t id,const dm_command *c,uint8_t d[8]) {
 dm_range r=dm_limits(id);
 /* dm4310_drv.h:38-41,50-53 (kp 0..500, kd 0..5).
  * dm4310_drv.c:204-217,238-251 (16-bit p; 12-bit v,kp,kd,t). */
 uint32_t p=dm_encode(c->p,-r.pmax,r.pmax,16),v=dm_encode(c->v,-r.vmax,r.vmax,12);
 uint32_t kp=dm_encode(c->kp,0,500,12),kd=dm_encode(c->kd,0,5,12),t=dm_encode(c->t,-r.tmax,r.tmax,12);
 d[0]=p>>8; d[1]=p; d[2]=v>>4; d[3]=(v<<4)|(kp>>8);
 d[4]=kp; d[5]=kd>>4; d[6]=(kd<<4)|(t>>8); d[7]=t;
}
void dm_unpack(uint8_t id,const uint8_t d[8],dm_command *c) {
 dm_range r=dm_limits(id);
 c->p=dm_decode((d[0]<<8)|d[1],-r.pmax,r.pmax,16);
 c->v=dm_decode((d[2]<<4)|(d[3]>>4),-r.vmax,r.vmax,12);
 c->kp=dm_decode(((d[3]&15)<<8)|d[4],0,500,12);
 c->kd=dm_decode((d[5]<<4)|(d[6]>>4),0,5,12);
 c->t=dm_decode(((d[6]&15)<<8)|d[7],-r.tmax,r.tmax,12);
}
void dm_pack_feedback(const dm_feedback *f,uint8_t d[8]) {
 /* Inverse of dm4310_drv.c:106-115,129-138; state/id nibbles, p16,v12,t12, two u8 Celsius. */
 dm_range r=dm_limits(f->id);
 uint32_t p=dm_encode(f->p,-r.pmax,r.pmax,16),v=dm_encode(f->v,-r.vmax,r.vmax,12),t=dm_encode(f->t,-r.tmax,r.tmax,12);
 d[0]=(f->state<<4)|(f->id&15); d[1]=p>>8; d[2]=p;
 d[3]=v>>4; d[4]=(v<<4)|(t>>8); d[5]=t; d[6]=f->tmos; d[7]=f->tcoil;
}
void dm_unpack_feedback(const uint8_t d[8],dm_feedback *f) {
 f->id=d[0]&15; f->state=d[0]>>4; dm_range r=dm_limits(f->id);
 f->p=dm_decode((d[1]<<8)|d[2],-r.pmax,r.pmax,16);
 f->v=dm_decode((d[3]<<4)|(d[4]>>4),-r.vmax,r.vmax,12);
 f->t=dm_decode(((d[4]&15)<<8)|d[5],-r.tmax,r.tmax,12);
 f->tmos=d[6]; f->tcoil=d[7];
}
void dm_special(uint8_t code,uint8_t d[8]) { for(int i=0;i<7;i++) d[i]=255; d[7]=code; }
int dm_special_code(const uint8_t d[8]) {
 for(int i=0;i<7;i++) if(d[i]!=255) return 0;
 return d[7]==DM_ENABLE || d[7]==DM_DISABLE ? d[7] : 0;
}
void dm_motor_init(dm_motor *m,uint8_t id) { *m=(dm_motor){0}; m->id=id; }
void dm_motor_receive(dm_motor *m,const uint8_t d[8]) {
 int code=dm_special_code(d);
 if(code) { m->enabled=code==DM_ENABLE; m->cmd=(dm_command){0}; m->t=0; }
 else dm_unpack(m->id,d,&m->cmd);
}
void dm_motor_step(dm_motor *m,float dt) {
 /* ASSUMED bench-only first-order lag: tau_v=.05s, v_target=t_eff/.2.
  * MIT spring/damper/feedforward, saturation, explicit Euler; not a motor fit.
  * Fixed 1ms firmware step; disabled motor coasts toward zero velocity. */
 dm_range r=dm_limits(m->id);
 m->t=m->enabled?clip(m->cmd.kp*(m->cmd.p-m->p)+m->cmd.kd*(m->cmd.v-m->v)+m->cmd.t,-r.tmax,r.tmax):0;
 m->v=clip(m->v+dt*(m->t/.2f-m->v)/.05f,-r.vmax,r.vmax);
 m->p=clip(m->p+dt*m->v,-r.pmax,r.pmax);
}
void dm_motor_feedback(const dm_motor *m,dm_feedback *f) {
 /* ASSUMED state codes 0=disabled,1=enabled, temperature 25 C (synthetic).
  * Source only defines a raw state nibble, not code meanings. */
 *f=(dm_feedback){m->p,m->v,m->t,m->id,m->enabled?1:0,25,25};
}
