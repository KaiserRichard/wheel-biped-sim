/* Port of DM-jump driver by Tran Nguyen Binh, MIT license; see THIRD_PARTY.md.
   The six function bodies below are extracted mechanically, comments removed.
   Wheel constants become variables to test BOTH original and SDK ranges. */
#include <stdio.h>
#include <stdint.h>
#include <math.h>
typedef int hcan_t;
#define FDCAN_DLC_BYTES_8 8
typedef struct { uint16_t id,state; int p_int,v_int,t_int; float pos,vel,tor,Tmos,Tcoil; } motor_fbpara_t;
typedef struct { motor_fbpara_t para; } Joint_Motor_t;
typedef Joint_Motor_t Wheel_Motor_t;
static uint8_t captured[8];
static float P_MIN2=-12, P_MAX2=12, V_MIN2=-45, V_MAX2=45, T_MIN2=-18, T_MAX2=18;
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
void canx_send_data(hcan_t* h, uint16_t id, uint8_t* d, int n) { (void)h; (void)id; for(int i=0;i<n;i++) captured[i]=d[i]; }
#define MIT_MODE        0x000
#define P_MIN           -12.5f
#define P_MAX           12.5f
#define V_MIN           -30.0f
#define V_MAX           30.0f
#define KP_MIN          0.0f
#define KP_MAX          500.0f
#define KD_MIN          0.0f
#define KD_MAX          5.0f
#define T_MIN           -10.0f
#define T_MAX           10.0f
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id + MIT_MODE;

    pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
    vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
    kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0x0F) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0x0F) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;
    
    canx_send_data(hcan, id, data, 8);
}

void mit_ctrl2(hcan_t* hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id + MIT_MODE;

    pos_tmp = float_to_uint(pos,  P_MIN2,  P_MAX2,  16);
    vel_tmp = float_to_uint(vel,  V_MIN2,  V_MAX2,  12);
    kp_tmp  = float_to_uint(kp,   KP_MIN2, KP_MAX2, 12);
    kd_tmp  = float_to_uint(kd,   KD_MIN2, KD_MAX2, 12);
    tor_tmp = float_to_uint(torq, T_MIN2,  T_MAX2,  12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0x0F) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0x0F) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;
    
    canx_send_data(hcan, id, data, 8);
}

void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len)
{ 
    if (data_len == FDCAN_DLC_BYTES_8)
    {
        motor->para.id    = (rx_data[0]) & 0x0F;
        motor->para.state = (rx_data[0]) >> 4;
        motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
        motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
        motor->para.t_int = ((rx_data[4] & 0x0F) << 8) | rx_data[5];
        motor->para.pos   = uint_to_float(motor->para.p_int, P_MIN, P_MAX, 16);  
        motor->para.vel   = uint_to_float(motor->para.v_int, V_MIN, V_MAX, 12);  
        motor->para.tor   = uint_to_float(motor->para.t_int, T_MIN, T_MAX, 12);  
        motor->para.Tmos  = (float)(rx_data[6]);                                  
        motor->para.Tcoil = (float)(rx_data[7]);                                  
    }
}

void dm6215_fbdata(Wheel_Motor_t *motor, uint8_t *rx_data, uint32_t data_len)
{ 
    if (data_len == FDCAN_DLC_BYTES_8)
    {
        motor->para.id    = (rx_data[0]) & 0x0F;
        motor->para.state = (rx_data[0]) >> 4;
        motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
        motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
        motor->para.t_int = ((rx_data[4] & 0x0F) << 8) | rx_data[5];
        motor->para.pos   = uint_to_float(motor->para.p_int, P_MIN2, P_MAX2, 16); 
        motor->para.vel   = uint_to_float(motor->para.v_int, V_MIN2, V_MAX2, 12); 
        motor->para.tor   = uint_to_float(motor->para.t_int, T_MIN2, T_MAX2, 12); 
        motor->para.Tmos  = (float)(rx_data[6]);
        motor->para.Tcoil = (float)(rx_data[7]);
    }
}

int main(void) {
 int idx,profile; float x[5];
 puts("case,hex,position,velocity,torque");
 while(scanf("%d %d %f %f %f %f %f",&idx,&profile,&x[0],&x[1],&x[2],&x[3],&x[4])==7) {
   P_MIN2=profile==1?-12.5f:-12.0f; P_MAX2=-P_MIN2;
   T_MIN2=profile==1?-10.0f:-18.0f; T_MAX2=-T_MIN2;
   float lo[]={profile?P_MIN2:P_MIN,profile?V_MIN2:V_MIN,0,0,profile?T_MIN2:T_MIN};
   float hi[]={-lo[0],-lo[1],500,5,-lo[4]};
   for(int k=0;k<5;k++) { if(x[k]<lo[k]) x[k]=lo[k]; if(x[k]>hi[k]) x[k]=hi[k]; }
   if(profile) mit_ctrl2(0,1,x[0],x[1],x[2],x[3],x[4]);
   else mit_ctrl(0,1,x[0],x[1],x[2],x[3],x[4]);
   printf("%d,",idx); for(int k=0;k<8;k++) printf("%02X",captured[k]);
   uint8_t fb[]={0x11,captured[0],captured[1],captured[2],(uint8_t)((captured[3]&0xF0)|(captured[6]&15)),captured[7],30,31};
   Joint_Motor_t m={0};
   if(profile) dm6215_fbdata(&m,fb,8); else dm4310_fbdata(&m,fb,8);
   printf(",%.9g,%.9g,%.9g\n",m.para.pos,m.para.vel,m.para.tor);
 }
 return 0;
}
