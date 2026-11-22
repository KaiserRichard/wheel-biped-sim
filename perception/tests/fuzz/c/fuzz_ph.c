#include "perception_health.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

static uint32_t rng;
static uint32_t next(void) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }
int main(int argc, char **argv) {
 unsigned long n=argc>1?strtoul(argv[1],0,10):1200, i; rng=argc>2?(uint32_t)strtoul(argv[2],0,10):20261018u;
 for(i=0;i<n;i++) { uint8_t b[PH_MAX_FRAME]; size_t len,j; struct ph_health h={next(),next(),(uint8_t)next(),(uint16_t)next(),(uint8_t)(next()&1),(uint16_t)next(),(uint8_t)next()}, out; uint16_t seq=0; len=ph_encode((uint16_t)next(),&h,b,sizeof b); if(len) { if(i%3==0) b[next()%len]^=(uint8_t)(1u<<(next()%8)); if(i%5==0) len=next()%(len+1); if(ph_decode(b,len,&seq,&out)) { uint8_t again[PH_MAX_FRAME]; size_t m=ph_encode(seq,&out,again,sizeof again); if(m!=len) return 2; for(j=0;j<len;j++) if(again[j]!=b[j]) return 3; } } }
 printf("perception_health C mutations: %lu seed=%u\n",n,rng); return 0;
}
