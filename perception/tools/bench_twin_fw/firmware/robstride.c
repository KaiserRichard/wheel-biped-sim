/* OPTIONAL AGV CODEC ONLY, excluded by default. No AGV velocity-mode runtime.
 * SOURCE-SUPPORTED: tools/can_monitor/firmware/robstride_decode.c:4-16
 * (position +/-12.57, feedback velocity +/-30, torque +/-60, u16 scaling),
 * :19-21 (ID fields), :32-44 (feedback bytes/temp/fault), :54-58 (control).
 * Types 1/2: tools/can_monitor/firmware/robstride_decode.h:7.
 * Zero code 0x7fff: docs/lessons/01_can_and_robstride.md:196-203. */
#include "robstride.h"
#define clamp(x,l,h) ((x)<(l)?(l):((x)>(h)?(h):(x)))
static uint16_t enc(float x,float lo,float hi) { return (uint16_t)(((clamp(x,lo,hi)-lo)*65535.0f)/(hi-lo)); }
static float dec(uint16_t x,float lo,float hi) { return (float)x*(hi-lo)/65535.0f+lo; }
static uint16_t be(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]<<8|p[1]); }
uint32_t rs_control_id(uint8_t motor,float torque) { return (1u<<24)|((uint32_t)enc(torque,-60,60)<<8)|motor; }
void rs_pack_control(uint8_t o[8],float t) { o[0]=o[2]=0x7f; o[1]=o[3]=0xff; o[4]=o[5]=o[6]=o[7]=0; (void)t; }
int rs_unpack_control(uint32_t id,const uint8_t in[8],uint8_t *motor,float *torque) { if(((id>>24)&31)!=1) return 0; *motor=(uint8_t)id; *torque=dec((uint16_t)(id>>8),-60,60); (void)in; return 1; }
void rs_pack_feedback(uint8_t o[8],const rs_feedback *v) { uint16_t x[4]={enc(v->position,-12.57f,12.57f),enc(v->velocity,-30,30),enc(v->torque,-60,60),(uint16_t)(v->temperature*10)}; for(int i=0;i<4;i++){o[2*i]=(uint8_t)(x[i]>>8);o[2*i+1]=(uint8_t)x[i];} }
int rs_unpack_feedback(uint32_t id,const uint8_t in[8],rs_feedback *v) { if(((id>>24)&31)!=2)return 0; v->position=dec(be(in),-12.57f,12.57f);v->velocity=dec(be(in+2),-30,30);v->torque=dec(be(in+4),-60,60);v->temperature=(float)be(in+6)/10.0f;v->error=(uint8_t)((id>>16)&63);return 1; }
