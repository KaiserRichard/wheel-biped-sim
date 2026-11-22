#include "link_proto.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint32_t rng;
static uint32_t next(void) { rng^=rng<<13; rng^=rng>>17; rng^=rng<<5; return rng; }
static void packet(void *p,const lp_message *m) { (void)p;(void)m; }
static void error(void *p) { (void)p; }
int main(int argc,char **argv) {
 unsigned long n=argc>1?strtoul(argv[1],0,10):1200,i; rng=argc>2?(uint32_t)strtoul(argv[2],0,10):20261018u;
 for(i=0;i<n;i++) { lp_message m={0},out; uint8_t wire[LP_MAX_WIRE],raw[LP_MAX_RAW],mut[LP_MAX_WIRE]; size_t len,k; lp_stream s; m.type=LP_MCU_STATUS;m.session=((uint64_t)next()<<32)|next();m.seq=next();m.data.status.sample_mcu_ns=next();m.data.status.battery_mv=(uint16_t)next();len=lp_pack(&m,wire,sizeof wire);if(len){memcpy(mut,wire,len);if(i%3==0)mut[next()%(len-1)]^=(uint8_t)(1u<<(next()%8));if(i%5==0)len=next()%(len+1);k=lp_cobs_decode(mut,len?len-1:0,raw,sizeof raw);if(k&&lp_unpack(raw,k,&out)){uint8_t again[LP_MAX_WIRE],again_raw[LP_MAX_RAW];size_t z=lp_pack(&out,again,sizeof again),q;if(!z)return 2;q=lp_cobs_decode(again,z-1,again_raw,sizeof again_raw);if(q!=k||memcmp(raw,again_raw,k))return 3;}lp_stream_init(&s);lp_stream_feed(&s,mut,len,packet,error,0);} }
 printf("link_proto C mutations: %lu seed=%u\n",n,rng); return 0;
}
