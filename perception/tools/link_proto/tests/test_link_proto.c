/* CANDIDATE: synthetic host verification only; all choices PROPOSED. */
#include "link_proto.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static lp_message health(uint32_t seq,uint32_t est,uint32_t us) {
 lp_message m; lp_health *h; memset(&m,0,sizeof(m));
 m.type=LP_HEALTH; m.session=0x0102030405060708ULL; m.seq=seq; h=&m.data.health;
 h->estimate_seq=est; h->map_epoch=3; h->transform_revision=4; h->mapping_revision=5;
 h->reset_counter=6; h->tracking=LP_TRACKING; h->estimator_id=1; h->source_domain=1;
 h->flags=LP_REQUIRED_FLAGS; h->frame_id=LP_ODOM_FRAME; h->child_frame_id=LP_BODY_FRAME;
 h->quality=0; h->source_ns=(uint64_t)us*1000+10000000000ULL;
 h->mapped_mcu_ns=(uint64_t)us*1000; h->pi_send_ns=h->source_ns+1000000;
 h->age_at_send_ns=1000000; h->uncertainty_ns=1000000; return m;
}
static lp_message reference(void) {
 lp_message m; lp_body_ref *v; memset(&m,0,sizeof(m));
 m.type=LP_BODY_REF; m.session=0x0102030405060708ULL; m.seq=9; v=&m.data.ref;
 v->estimate_seq=7; v->map_epoch=3; v->mapping_revision=5; v->frame_id=LP_BODY_FRAME; v->reset_counter=6;
 v->pi_creation_ns=10002000000ULL; v->mapped_mcu_creation_ns=2000000;
 v->lease_us=100000; v->forward_m_s=0.25f; v->yaw_rad_s=-0.5f; return m;
}
static lp_message status(void) {
 lp_message m; lp_status *s; memset(&m,0,sizeof(m)); m.type=LP_MCU_STATUS; m.session=42; m.seq=10; s=&m.data.status;
 s->sample_mcu_ns=3000000; s->last_health_seq=8; s->last_estimate_seq=7; s->last_ref_seq=9;
 s->action_mcu_ns=2000000; s->battery_mv=24000; s->fsm=2; s->armed=1; s->operator_enable=1;
 s->state=LP_ESTIMATE_VALID; s->battery_valid=1; return m;
}
static lp_message advisory(void) {
 lp_message m; lp_advisory *a; memset(&m,0,sizeof(m)); m.type=LP_TERRAIN_ADVISORY;
 m.session=0x0102030405060708ULL; m.seq=11; a=&m.data.advisory;
 a->pi_monotonic_ns=10002000000ULL; a->frame_stamp_ns=2000000000ULL; a->advisory=1; a->plane_valid=1;
 a->confidence=.75f; a->distance_m=1.25f; a->height_m=.08f; a->width_m=.3f; a->plane_pitch_rad=-.1f; a->plane_height_m=.42f; return m;
}
static lp_message echo(void) {
 lp_message m; lp_advisory_echo *e; memset(&m,0,sizeof(m)); m.type=LP_ADVISORY_ECHO;
 m.session=42; m.seq=12; e=&m.data.echo; e->sample_mcu_ns=3000000; e->advisory_seq=11;
 e->advisory=1; e->result=0; e->accepted_count=9; e->rejected_count=2; e->ms_since_accepted=15; return m;
}
static void bind(lp_receiver *r) { lp_receiver_bind(r,0x0102030405060708ULL,5,4); }
static void ready(lp_receiver *r) { lp_message h=health(8,7,1000); bind(r); assert(lp_receiver_accept(r,2000,&h)); assert(lp_receiver_enable(r,2000)); }
static void golden(void) {
 FILE *f=fopen("tests/vectors.txt","r"); char line[1200],name[80],hex[1000]; unsigned count=0;
 assert(f);
 while(fgets(line,sizeof(line),f)) {
  uint8_t expected[LP_MAX_WIRE],wire[LP_MAX_WIRE],raw[LP_MAX_RAW]; size_t i,n,len; lp_message m,decoded;
  if(line[0]=='#' || line[0]=='\n') continue;
  assert(sscanf(line,"%79s %999s",name,hex)==2);
  m=strcmp(name,"health")==0?health(8,7,1000):strcmp(name,"ref")==0?reference():strcmp(name,"status")==0?status():strcmp(name,"advisory")==0?advisory():echo();
  n=strlen(hex)/2; assert(n<=sizeof(expected));
  for(i=0;i<n;i++) { char byte[3]={hex[2*i],hex[2*i+1],0}; expected[i]=(uint8_t)strtoul(byte,NULL,16); }
  len=lp_pack(&m,wire,sizeof(wire)); assert(len==n); assert(memcmp(expected,wire,n)==0);
  len=lp_cobs_decode(wire,n-1,raw,sizeof(raw)); assert(lp_unpack(raw,len,&decoded));
  assert(lp_pack(&decoded,wire,sizeof(wire))==n); assert(memcmp(wire,expected,n)==0);
  count++;
 }
 assert(count==5); fclose(f);
}
static void cobs(void) {
 uint8_t src[600],enc[610],out[600]; size_t n,len; unsigned kind;
 assert(lp_crc32c((const uint8_t *)"123456789",9)==0xe3069283u);
 assert(lp_crc32c(NULL,0)==0);
 for(kind=0;kind<3;kind++) for(n=0;n<=sizeof(src);n++) {
  size_t i; for(i=0;i<n;i++) src[i]=kind==0?0:kind==1?1:(uint8_t)i;
  len=lp_cobs_encode(src,n,enc,sizeof(enc)); assert(len!=0);
  assert(lp_cobs_decode(enc,len,out,sizeof(out))==n); assert(memcmp(src,out,n)==0);
  assert(lp_cobs_encode(src,n,enc,len-1)==0);
 }
 { const uint8_t bad[]={3,1},zero[]={2,0}; assert(lp_cobs_decode(bad,2,out,sizeof(out))==0); assert(lp_cobs_decode(zero,2,out,sizeof(out))==0); }
 memset(src,1,254); len=lp_cobs_encode(src,254,enc,sizeof(enc)); assert(len==256 && enc[0]==255 && enc[255]==1);
 assert(lp_cobs_decode(enc,len,out,253)==0);
}
typedef struct { lp_receiver r; uint32_t now; unsigned packets,errors; } context;
static void packet(void *v,const lp_message *m) { context *c=v; c->packets++; (void)lp_receiver_accept(&c->r,c->now,m); }
static void error(void *v) { context *c=v; c->errors++; lp_receiver_reject(&c->r,LP_REASON_WIRE); }
static void streaming(void) {
 uint8_t wire[LP_MAX_WIRE],raw[LP_MAX_RAW],bad[400]; lp_message m=health(8,7,1000),out;
 size_t n=lp_pack(&m,wire,sizeof(wire)),split,len,i; lp_stream s; context c;
 for(split=0;split<=n;split++) {
  memset(&c,0,sizeof(c)); bind(&c.r); c.now=2000; lp_stream_init(&s);
  lp_stream_feed(&s,wire,split,packet,error,&c); lp_stream_feed(&s,wire+split,n-split,packet,error,&c);
  assert(c.packets==1 && c.errors==0);
 }
 memset(&c,0,sizeof(c)); bind(&c.r); c.now=2000; lp_stream_init(&s);
 for(i=0;i<n;i++) lp_stream_feed(&s,wire+i,1,packet,error,&c);
 assert(c.packets==1); assert(c.r.last_link_us==2000);
 /* Missing delimiter yields no packet and no timer refresh. */
 lp_stream_feed(&s,wire,n-1,packet,error,&c); assert(c.packets==1);
 c.now=5000; lp_stream_feed(&s,wire,n,packet,error,&c); assert(c.errors==1 && c.r.last_link_us==2000);
 /* Recovery at next delimiter; duplicate packet does not refresh. */
 lp_stream_feed(&s,wire,n,packet,error,&c); assert(c.packets==2 && c.r.last_link_us==2000);
 memcpy(bad,wire,n); bad[10]^=0x40; lp_stream_feed(&s,bad,n,packet,error,&c); assert(c.errors==2 && c.r.last_link_us==2000);
 memset(bad,1,sizeof(bad)); bad[sizeof(bad)-1]=0; lp_stream_feed(&s,bad,sizeof(bad),packet,error,&c); assert(c.errors==3);
 /* Raw decoded record > cap, valid COBS but rejected at decode bound. */
 memset(bad,1,193); len=lp_cobs_encode(bad,193,wire,sizeof(wire)-1); assert(len==0); /* cap is insufficient */
 { uint8_t big[200]; len=lp_cobs_encode(bad,193,big,sizeof(big)-1); big[len]=0; lp_stream_feed(&s,big,len+1,packet,error,&c); assert(c.errors==4); }
 n=lp_pack(&m,wire,sizeof(wire));
 len=lp_cobs_decode(wire,n-1,raw,sizeof(raw)); assert(!lp_unpack(raw,len-1,&out));
 raw[0]=2; assert(!lp_unpack(raw,len,&out));
 lp_stream_feed(&s,wire,n,packet,error,&c); assert(c.packets==3);
 /* Truncate body, then delimit and recover. */
 lp_stream_feed(&s,wire,n/2,packet,error,&c); bad[0]=0; lp_stream_feed(&s,bad,1,packet,error,&c); assert(c.errors==5);
 lp_stream_feed(&s,wire,n,packet,error,&c); assert(c.packets==4);
}
static void policy(void) {
 lp_receiver r; lp_message h,v; uint32_t link;
 lp_receiver_init(&r); lp_receiver_update(&r,0); assert(r.state==LP_NO_LINK);
 h=health(8,7,1000); bind(&r); h.data.health.flags=0;
 assert(lp_receiver_accept(&r,2000,&h)); assert(!lp_receiver_enable(&r,2000));
 assert(r.state==LP_RESET_PENDING_REENABLE);
 ready(&r); assert(r.state==LP_ESTIMATE_VALID);
 v=reference(); assert(lp_receiver_accept(&r,2000,&v)); assert(r.reference_active);
 link=r.last_link_us; assert(!lp_receiver_accept(&r,3000,&v)); assert(r.last_link_us==link && !r.reference_active);
 h=health(9,8,3000); assert(lp_receiver_accept(&r,3000,&h)); assert(r.state==LP_ESTIMATE_VALID);
 h.seq=8; assert(!lp_receiver_accept(&r,4000,&h)); assert(r.reason==LP_REASON_SEQUENCE && r.last_link_us==3000);
 /* Invalid fields and mapping never renew link/estimate. */
 h.seq=10; h.data.health.frame_id=99; assert(!lp_receiver_accept(&r,4000,&h)); assert(r.last_link_us==3000);
 h.data.health.frame_id=LP_ODOM_FRAME; h.data.health.uncertainty_ns=LP_UNCERTAINTY_NS+1;
 assert(!lp_receiver_accept(&r,4000,&h)); assert(r.last_link_us==3000);
 h.data.health.uncertainty_ns=1000000; h.data.health.mapping_revision=99;
 assert(!lp_receiver_accept(&r,4000,&h)); assert(r.last_link_us==3000);
 /* New on-wire boot cannot bind itself. */
 h.session++; assert(!lp_receiver_accept(&r,4000,&h)); assert(r.reason==LP_REASON_SESSION && r.last_link_us==3000);
 lp_receiver_bind(&r,h.session,5,4); h.data.health.mapping_revision=5;
 assert(lp_receiver_accept(&r,4000,&h)); assert(r.state==LP_RESET_PENDING_REENABLE && !r.enabled);
 assert(lp_receiver_enable(&r,4000));
 /* Reset, including reset counter wrap, requires a fresh local action. */
 h.seq++; h.data.health.reset_counter=255; assert(lp_receiver_accept(&r,5000,&h));
 assert(r.state==LP_RESET_PENDING_REENABLE && !r.enabled); assert(lp_receiver_enable(&r,5000));
 h.seq++; h.data.health.reset_counter=0; assert(lp_receiver_accept(&r,6000,&h)); assert(!r.enabled && r.pending_reenable);
 assert(lp_receiver_enable(&r,6000)); h.seq++; h.data.health.map_epoch++;
 assert(lp_receiver_accept(&r,7000,&h)); assert(r.pending_reenable);
 /* Frozen estimate with fresh heartbeats: accepted link timer advances,
  * estimate source does not; MCU update still goes stale. */
 ready(&r); h=health(9,7,1000);
 assert(lp_receiver_accept(&r,50000,&h)); h.seq++;
 assert(lp_receiver_accept(&r,100000,&h)); h.seq++;
 assert(lp_receiver_accept(&r,140000,&h)); assert(r.estimate_mcu_us==1000);
 lp_receiver_update(&r,152001); assert(r.state==LP_STALE && !r.reference_active);
 assert(!lp_receiver_accept(&r,152002,&h)); assert(r.last_link_us==140000);
 lp_receiver_update(&r,152002); assert(r.state==LP_STALE);
 lp_receiver_update(&r,290000); assert(r.state==LP_NO_LINK && !r.enabled);
 /* Silence recovery cannot automatically resume. */
 h=health(12,8,290000); assert(lp_receiver_accept(&r,290001,&h)); assert(r.state==LP_RESET_PENDING_REENABLE);
 /* Rewriting frozen estimate stamps is rejected. */
 ready(&r); h=health(9,7,3000); assert(!lp_receiver_accept(&r,4000,&h)); assert(r.last_link_us==2000);
 /* Healthy link, estimator explicitly lost; no valid estimate/reference. */
 ready(&r); h=health(9,7,1000); h.data.health.tracking=LP_LOST; h.data.health.flags=0;
 assert(lp_receiver_accept(&r,3000,&h)); assert(r.state==LP_LINK_OK_NO_ESTIMATE && !r.estimate_usable);
 h.seq++; h.data.health.estimate_seq=6; assert(!lp_receiver_accept(&r,4000,&h));
 /* Ref expiry from creation, never receipt; heartbeats cannot extend it. */
 ready(&r); v=reference(); assert(lp_receiver_accept(&r,50000,&v));
 h=health(9,8,80000); assert(lp_receiver_accept(&r,80000,&h));
 lp_receiver_update(&r,101000); assert(!r.reference_active && r.reason==LP_REASON_LEASE);
 ready(&r); v=reference(); v.data.ref.lease_us=100001; assert(!lp_receiver_accept(&r,2000,&v));
 ready(&r); v=reference(); v.data.ref.estimate_seq++; assert(!lp_receiver_accept(&r,2000,&v));
 ready(&r); v=reference(); assert(!lp_receiver_accept(&r,102000,&v)); assert(r.last_link_us==2000);
 ready(&r); v=reference(); v.data.ref.mapped_mcu_creation_ns=3000000; assert(!lp_receiver_accept(&r,2000,&v));
 ready(&r); lp_receiver_disable(&r); v=reference(); assert(!lp_receiver_accept(&r,2000,&v));
}
static void wrap(void) {
 lp_receiver r; lp_message h,v; uint32_t now=0xfffff000u;
 assert(lp_seq_newer(0,0xffffffffu)); assert(!lp_seq_newer(0xffffffffu,0));
 assert(!lp_seq_newer(0x80000000u,0)); assert(!lp_seq_newer(1,1));
 bind(&r); h=health(0xffffffffu,0xffffffffu,now);
 assert(lp_receiver_accept(&r,now,&h)); assert(lp_receiver_enable(&r,now));
 v=reference(); v.data.ref.estimate_seq=0xffffffffu; v.data.ref.mapped_mcu_creation_ns=(uint64_t)now*1000;
 v.seq=0xffffffffu; assert(lp_receiver_accept(&r,now,&v));
 v.seq=0; v.data.ref.mapped_mcu_creation_ns=(uint64_t)(now+1000u)*1000;
 assert(lp_receiver_accept(&r,now+1000u,&v));
 h.seq=0; h.data.health.estimate_seq=0; h.data.health.mapped_mcu_ns=((uint64_t)now+5000u)*1000;
 h.data.health.source_ns+=5000000; assert(lp_receiver_accept(&r,now+5000u,&h));
 lp_receiver_update(&r,now+101000u); assert(!r.reference_active);
 lp_receiver_update(&r,now+155000u); assert(r.state==LP_NO_LINK);
 bind(&r); h=health(1,1,now); assert(lp_receiver_accept(&r,now,&h));
 h.seq=2; assert(lp_receiver_accept(&r,now+100000u,&h));
 lp_receiver_update(&r,now+151001u); assert(r.state==LP_STALE);
}
static void nonfinite(void) {
 const uint32_t bits[]={0x7fc00000u,0x7f800000u,0xff800000u,0x7f800001u};
 size_t i,j; uint8_t wire[LP_MAX_WIRE],raw[LP_MAX_RAW]; lp_message m,out; lp_receiver r;
 for(i=0;i<4;i++) for(j=0;j<2;j++) {
  size_t n,len; m=reference();
  if(j==0) memcpy(&m.data.ref.forward_m_s,&bits[i],4); else memcpy(&m.data.ref.yaw_rad_s,&bits[i],4);
  n=lp_pack(&m,wire,sizeof(wire)); assert(n); len=lp_cobs_decode(wire,n-1,raw,sizeof(raw));
  assert(!lp_unpack(raw,len,&out)); ready(&r); assert(!lp_receiver_accept(&r,2000,&m)); assert(r.last_link_us==2000 && !r.reference_active);
 }
}

static void advisory_validation(void) {
 uint8_t wire[LP_MAX_WIRE],raw[LP_MAX_RAW]; lp_message m=advisory(),out; size_t n,len; uint32_t nanbits=0x7fc00000u;
 m.data.advisory.advisory=4; assert(lp_pack(&m,wire,sizeof(wire))==0);
 m=advisory(); m.data.advisory.plane_valid=2; assert(lp_pack(&m,wire,sizeof(wire))==0);
 m=advisory(); memcpy(&m.data.advisory.confidence,&nanbits,4); assert(lp_pack(&m,wire,sizeof(wire))==0);
 m=advisory();
 n=lp_pack(&m,wire,sizeof(wire)); len=lp_cobs_decode(wire,n-1,raw,sizeof(raw));
 raw[32]=4; /* payload advisory offset 16 */
 { uint32_t crc=lp_crc32c(raw,len-4); memcpy(raw+len-4,&crc,4); }
 assert(!lp_unpack(raw,len,&out));
 len=lp_cobs_decode(wire,n-1,raw,sizeof(raw)); raw[33]=2; /* plane_valid */
 { uint32_t crc=lp_crc32c(raw,len-4); memcpy(raw+len-4,&crc,4); }
 assert(!lp_unpack(raw,len,&out));
 len=lp_cobs_decode(wire,n-1,raw,sizeof(raw)); memcpy(raw+34,&nanbits,4); /* first float */
 { uint32_t crc=lp_crc32c(raw,len-4); memcpy(raw+len-4,&crc,4); }
 assert(!lp_unpack(raw,len,&out));
 len=lp_cobs_decode(wire,n-1,raw,sizeof(raw)); raw[2]=41; raw[3]=0;
 { uint32_t crc=lp_crc32c(raw,len-4); memcpy(raw+len-4,&crc,4); }
 assert(!lp_unpack(raw,len,&out));
}

static void boundaries(void) {
 lp_receiver r; lp_message h,v,out; uint8_t raw[LP_MAX_RAW],wire[LP_MAX_WIRE],tiny[1]; size_t n,len;
 uint32_t before; unsigned i;
 ready(&r); h=health(9,8,3000); h.data.health.quality=-1;
 assert(lp_receiver_accept(&r,3000,&h)); assert(r.state==LP_LINK_OK_NO_ESTIMATE);
 ready(&r); h=health(9,8,3000); h.data.health.flags|=LP_RESOURCE_FAULT;
 assert(lp_receiver_accept(&r,3000,&h)); assert(!r.estimate_usable);
 for(i=0;i<6;i++) {
  ready(&r); before=r.last_link_us; h=health(9,8,3000);
  switch(i) {
   case 0: h.data.health.tracking=255; break;
   case 1: h.data.health.source_domain=0; break;
   case 2: h.data.health.flags=0xffff; break;
   case 3: h.data.health.child_frame_id=9; break;
   case 4: h.data.health.transform_revision=99; break;
   default: h.data.health.mapped_mcu_ns=5000000; break;
  }
  assert(!lp_receiver_accept(&r,4000,&h)); assert(r.last_link_us==before && r.health_seq==8 && r.estimate_seq==7);
 }
 ready(&r); h=health(9,7,1000); h.data.health.uncertainty_ns=0;
 assert(!lp_receiver_accept(&r,4000,&h)); assert(r.last_link_us==2000);
 h=health(9,8,3000); h.data.health.source_domain=2;
 assert(!lp_receiver_accept(&r,4000,&h)); assert(r.last_link_us==2000);
 /* Exact source-age limit allowed; one microsecond beyond is stale. */
 ready(&r); h=health(9,7,1000); assert(lp_receiver_accept(&r,150000,&h));
 assert(r.state==LP_ESTIMATE_VALID); lp_receiver_update(&r,150001); assert(r.state==LP_STALE);
 /* Exact link deadline and maximum uncertainty, rounded upward. */
 ready(&r); lp_receiver_update(&r,152000); assert(r.state==LP_NO_LINK);
 bind(&r); h=health(8,7,1000); h.data.health.uncertainty_ns=LP_UNCERTAINTY_NS;
 assert(lp_receiver_accept(&r,141000,&h)); assert(lp_receiver_enable(&r,141000));
 lp_receiver_update(&r,141001); assert(r.state==LP_STALE);
 ready(&r); v=reference(); v.data.ref.lease_us=0;
 assert(!lp_receiver_accept(&r,2000,&v)); assert(r.last_link_us==2000);
 /* Syntactically good CRC cannot bypass version/length/reserved checks. */
 h=health(8,7,1000); n=lp_pack(&h,wire,sizeof(wire));
 assert(lp_pack(&h,tiny,sizeof(tiny))==0); len=lp_cobs_decode(wire,n-1,raw,sizeof(raw));
 for(i=0;i<5;i++) {
  uint8_t changed[LP_MAX_RAW]; uint32_t crc; unsigned j;
  memcpy(changed,raw,len);
  switch(i) { case 0: changed[0]=2; break; case 1: changed[1]=99; break;
   case 2: changed[2]++; break; case 3: changed[16+27]=1; break;
   default: changed[16+79]=1; break; }
  crc=lp_crc32c(changed,len-4); for(j=0;j<4;j++) changed[len-4+j]=(uint8_t)(crc>>(8*j));
  assert(!lp_unpack(changed,len,&out));
 }
 /* Every individual wire-byte mutation is rejected at framing/CRC, without
  * overrunning a buffer or accepting a different message. */
 for(i=0;i<n-1;i++) {
  uint8_t changed[LP_MAX_WIRE]; memcpy(changed,wire,n); changed[i]^=0x80;
  len=lp_cobs_decode(changed,n-1,raw,sizeof(raw));
  assert(len==0 || !lp_unpack(raw,len,&out));
 }
 /* Coalesced distinct health records preserve packet boundaries. */
 {
  lp_stream stream; context c; uint8_t both[LP_MAX_WIRE*2]; size_t first,second;
  memset(&c,0,sizeof(c)); bind(&c.r); c.now=4000; lp_stream_init(&stream);
  h=health(8,7,1000); first=lp_pack(&h,both,sizeof(both));
  h=health(9,8,3000); second=lp_pack(&h,both+first,sizeof(both)-first);
  lp_stream_feed(&stream,both,first+second,packet,error,&c);
  assert(c.packets==2 && c.errors==0 && c.r.estimate_seq==8);
 }
}

int main(void) {
 golden(); cobs(); streaming(); policy(); wrap(); nonfinite(); advisory_validation(); boundaries();
 puts("CANDIDATE: C codec, golden vectors and synthetic receiver faults PASS"); return 0;
}
