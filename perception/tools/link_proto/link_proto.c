/* CANDIDATE; PROPOSED implementation, not an accepted firmware interface. */
#include "link_proto.h"
#include <float.h>
#include <string.h>
#include <limits.h>
typedef char lp_requires_binary32[(sizeof(float)==4 && FLT_RADIX==2 && FLT_MANT_DIG==24 && FLT_MAX_EXP==128 && CHAR_BIT==8)?1:-1];
static void put(uint8_t *p, uint64_t v, size_t n) { size_t i; for(i=0;i<n;i++) { p[i]=(uint8_t)v; v>>=8; } }
static uint64_t get(const uint8_t *p, size_t n) { uint64_t v=0; size_t i; for(i=0;i<n;i++) v|=(uint64_t)p[i]<<(8*i); return v; }
static void putfloat(uint8_t *p, float f) { uint32_t u; memcpy(&u,&f,4); put(p,u,4); }
static float getfloat(const uint8_t *p) { uint32_t u=(uint32_t)get(p,4); float f; memcpy(&f,&u,4); return f; }
static int finitefloat(float f) { uint32_t u; memcpy(&u,&f,4); return (u & 0x7f800000u)!=0x7f800000u; }
uint32_t lp_crc32c(const uint8_t *data, size_t n) {
 uint32_t crc=0xffffffffu; size_t i; unsigned j;
 for(i=0;i<n;i++) { crc^=data[i]; for(j=0;j<8;j++) crc=(crc>>1)^((crc&1u)?0x82f63b78u:0u); }
 return crc^0xffffffffu;
}
size_t lp_cobs_encode(const uint8_t *s,size_t n,uint8_t *d,size_t cap) {
 size_t i, out=1, codepos=0; uint8_t code=1;
 if(cap==0) return 0;
 for(i=0;i<n;i++) {
  if(s[i]==0) { d[codepos]=code; codepos=out; if(out>=cap) return 0; out++; code=1; }
  else { if(out>=cap) return 0; d[out++]=s[i]; code++;
   if(code==255) { d[codepos]=code; codepos=out; if(out>=cap) return 0; out++; code=1; }
  }
 }
 d[codepos]=code; return out;
}
size_t lp_cobs_decode(const uint8_t *s,size_t n,uint8_t *d,size_t cap) {
 size_t in=0,out=0,k; uint8_t code;
 if(n==0) return 0;
 while(in<n) {
  code=s[in++]; if(code==0 || (size_t)(code-1)>n-in) return 0;
  for(k=1;k<code;k++) { if(out>=cap || s[in]==0) return 0; d[out++]=s[in++]; }
  if(code!=255 && in<n) { if(out>=cap) return 0; d[out++]=0; }
 }
 return out;
}
static size_t payload_size(uint8_t type) { return type==LP_HEALTH?LP_HEALTH_SIZE:type==LP_BODY_REF?LP_REF_SIZE:type==LP_MCU_STATUS?LP_STATUS_SIZE:type==LP_TERRAIN_ADVISORY?LP_ADVISORY_SIZE:type==LP_ADVISORY_ECHO?LP_ADVISORY_ECHO_SIZE:0; }
size_t lp_pack(const lp_message *m,uint8_t *wire,size_t cap) {
 uint8_t raw[LP_MAX_RAW],*p=raw+16; size_t n=payload_size(m->type),enc;
 if(n==0 || cap<2) return 0;
 if(m->type==LP_TERRAIN_ADVISORY) {
  const lp_advisory *a=&m->data.advisory;
  if(a->advisory>3 || a->plane_valid>1 || !finitefloat(a->confidence) || !finitefloat(a->distance_m) ||
   !finitefloat(a->height_m) || !finitefloat(a->width_m) || !finitefloat(a->plane_pitch_rad) || !finitefloat(a->plane_height_m)) return 0;
 }
 if(m->type==LP_ADVISORY_ECHO) {
  const lp_advisory_echo *e=&m->data.echo;
  if(e->advisory>3 || e->result>LP_REASON_LEASE || e->stale>1) return 0;
 }
 memset(raw,0,sizeof(raw)); raw[0]=LP_VERSION; raw[1]=m->type;
 put(raw+2,n,2); put(raw+4,m->session,8); put(raw+12,m->seq,4);
 if(m->type==LP_HEALTH) {
  const lp_health *h=&m->data.health;
  put(p,h->estimate_seq,4); put(p+4,h->map_epoch,4); put(p+8,h->transform_revision,4); put(p+12,h->mapping_revision,4);
  p[16]=h->reset_counter; p[17]=h->tracking; p[18]=h->estimator_id; p[19]=h->source_domain;
  put(p+20,h->flags,2); put(p+22,h->frame_id,2); put(p+24,h->child_frame_id,2); p[26]=(uint8_t)h->quality;
  put(p+28,h->reasons,4); put(p+32,h->source_ns,8); put(p+40,h->mapped_mcu_ns,8);
  put(p+48,h->pi_send_ns,8); put(p+56,h->age_at_send_ns,8); put(p+64,h->uncertainty_ns,4);
 } else if(m->type==LP_BODY_REF) {
  const lp_body_ref *r=&m->data.ref;
  put(p,r->estimate_seq,4); put(p+4,r->map_epoch,4); put(p+8,r->mapping_revision,4);
  put(p+12,r->frame_id,2); p[14]=r->reset_counter;
  put(p+16,r->pi_creation_ns,8); put(p+24,r->mapped_mcu_creation_ns,8); put(p+32,r->lease_us,4);
  putfloat(p+36,r->forward_m_s); putfloat(p+40,r->yaw_rad_s);
 } else if(m->type==LP_MCU_STATUS) {
  const lp_status *s=&m->data.status;
  put(p,s->sample_mcu_ns,8); put(p+8,s->last_health_seq,4); put(p+12,s->last_estimate_seq,4);
  put(p+16,s->last_ref_seq,4); put(p+20,s->faults,4); put(p+24,s->action_mcu_ns,8);
  put(p+32,s->battery_mv,2); p[34]=s->fsm; p[35]=s->armed; p[36]=s->operator_enable;
  p[37]=s->state; p[38]=s->reason; p[39]=s->battery_valid;
 } else if(m->type==LP_TERRAIN_ADVISORY) {
  const lp_advisory *a=&m->data.advisory;
  put(p,a->pi_monotonic_ns,8); put(p+8,a->frame_stamp_ns,8); p[16]=a->advisory; p[17]=a->plane_valid;
  putfloat(p+18,a->confidence); putfloat(p+22,a->distance_m); putfloat(p+26,a->height_m); putfloat(p+30,a->width_m);
  putfloat(p+34,a->plane_pitch_rad); putfloat(p+38,a->plane_height_m);
 } else {
  const lp_advisory_echo *e=&m->data.echo;
  put(p,e->sample_mcu_ns,8); put(p+8,e->advisory_seq,4); p[12]=e->advisory; p[13]=e->result;
  put(p+14,e->accepted_count,4); put(p+18,e->rejected_count,4); p[22]=e->stale; put(p+23,e->ms_since_accepted,4);
 }
 put(raw+16+n,lp_crc32c(raw,16+n),4);
 enc=lp_cobs_encode(raw,20+n,wire,cap-1); if(enc==0) return 0; wire[enc]=0; return enc+1;
}
int lp_unpack(const uint8_t *raw,size_t n,lp_message *m) {
 const uint8_t *p; size_t len,i;
 if(n<20 || n>LP_MAX_RAW || raw[0]!=LP_VERSION) return 0;
 p=raw+16;
 len=payload_size(raw[1]); if(len==0 || get(raw+2,2)!=len || n!=20+len) return 0;
 if(get(raw+n-4,4)!=lp_crc32c(raw,n-4)) return 0;
 memset(m,0,sizeof(*m)); m->type=raw[1]; m->session=get(raw+4,8); m->seq=(uint32_t)get(raw+12,4);
 if(m->type==LP_HEALTH) {
  lp_health *h=&m->data.health;
  if(p[27]!=0) return 0;
  for(i=68;i<80;i++) if(p[i]!=0) return 0;
  h->estimate_seq=(uint32_t)get(p,4); h->map_epoch=(uint32_t)get(p+4,4);
  h->transform_revision=(uint32_t)get(p+8,4); h->mapping_revision=(uint32_t)get(p+12,4);
  h->reset_counter=p[16]; h->tracking=p[17]; h->estimator_id=p[18]; h->source_domain=p[19];
  h->flags=(uint16_t)get(p+20,2); h->frame_id=(uint16_t)get(p+22,2); h->child_frame_id=(uint16_t)get(p+24,2);
  h->quality=p[26]==255?-1:(int8_t)(p[26]<=127?p[26]:127);
  if(p[26]>100 && p[26]!=255) return 0;
  h->reasons=(uint32_t)get(p+28,4); h->source_ns=get(p+32,8); h->mapped_mcu_ns=get(p+40,8);
  h->pi_send_ns=get(p+48,8); h->age_at_send_ns=get(p+56,8); h->uncertainty_ns=(uint32_t)get(p+64,4);
 } else if(m->type==LP_BODY_REF) {
  lp_body_ref *r=&m->data.ref;
  if(p[15]!=0) return 0;
  r->estimate_seq=(uint32_t)get(p,4); r->map_epoch=(uint32_t)get(p+4,4); r->mapping_revision=(uint32_t)get(p+8,4);
  r->frame_id=(uint16_t)get(p+12,2); r->reset_counter=p[14];
  r->pi_creation_ns=get(p+16,8); r->mapped_mcu_creation_ns=get(p+24,8); r->lease_us=(uint32_t)get(p+32,4);
  r->forward_m_s=getfloat(p+36); r->yaw_rad_s=getfloat(p+40);
  if(!finitefloat(r->forward_m_s) || !finitefloat(r->yaw_rad_s)) return 0;
 } else if(m->type==LP_MCU_STATUS) {
  lp_status *s=&m->data.status;
  for(i=40;i<44;i++) if(p[i]!=0) return 0;
  s->sample_mcu_ns=get(p,8); s->last_health_seq=(uint32_t)get(p+8,4); s->last_estimate_seq=(uint32_t)get(p+12,4);
  s->last_ref_seq=(uint32_t)get(p+16,4); s->faults=(uint32_t)get(p+20,4); s->action_mcu_ns=get(p+24,8);
  s->battery_mv=(uint16_t)get(p+32,2); s->fsm=p[34]; s->armed=p[35]; s->operator_enable=p[36];
  s->state=p[37]; s->reason=p[38]; s->battery_valid=p[39];
  if(s->armed>1 || s->operator_enable>1 || s->battery_valid>1 || s->state>LP_RESET_PENDING_REENABLE || s->reason>LP_REASON_LEASE) return 0;
 } else if(m->type==LP_TERRAIN_ADVISORY) {
  lp_advisory *a=&m->data.advisory;
  a->pi_monotonic_ns=get(p,8); a->frame_stamp_ns=get(p+8,8); a->advisory=p[16]; a->plane_valid=p[17];
  a->confidence=getfloat(p+18); a->distance_m=getfloat(p+22); a->height_m=getfloat(p+26); a->width_m=getfloat(p+30);
  a->plane_pitch_rad=getfloat(p+34); a->plane_height_m=getfloat(p+38);
  if(a->advisory>3 || a->plane_valid>1 || !finitefloat(a->confidence) || !finitefloat(a->distance_m) || !finitefloat(a->height_m) || !finitefloat(a->width_m) || !finitefloat(a->plane_pitch_rad) || !finitefloat(a->plane_height_m)) return 0;
 } else {
  lp_advisory_echo *e=&m->data.echo;
  e->sample_mcu_ns=get(p,8); e->advisory_seq=(uint32_t)get(p+8,4); e->advisory=p[12]; e->result=p[13];
  e->accepted_count=(uint32_t)get(p+14,4); e->rejected_count=(uint32_t)get(p+18,4); e->stale=p[22]; e->ms_since_accepted=(uint32_t)get(p+23,4);
  if(e->advisory>3 || e->result>LP_REASON_LEASE || e->stale>1) return 0;
 }
 return 1;
}
void lp_stream_init(lp_stream *s) { memset(s,0,sizeof(*s)); }
void lp_stream_feed(lp_stream *s,const uint8_t *data,size_t n,lp_packet_fn packet,lp_error_fn error,void *context) {
 size_t i,len; uint8_t raw[LP_MAX_RAW]; lp_message m;
 for(i=0;i<n;i++) {
  if(data[i]==0) {
   if(s->dropping || s->used!=0) {
    len=s->dropping?0:lp_cobs_decode(s->encoded,s->used,raw,sizeof(raw));
    s->last_type=len>=2?raw[1]:0;
    s->last_seq=len>=16?(uint32_t)get(raw+12,4):0;
    s->last_advisory=s->last_type==LP_TERRAIN_ADVISORY && len>32?raw[32]:0;
    if(len!=0 && lp_unpack(raw,len,&m)) { s->packets++; if(packet) packet(context,&m); }
    else { s->errors++; s->last_reason=LP_REASON_WIRE;
     if(s->last_type==LP_TERRAIN_ADVISORY && len>=20 && get(raw+len-4,4)==lp_crc32c(raw,len-4)) s->last_reason=LP_REASON_FIELDS;
     if(error) error(context); }
   }
   s->used=0; s->dropping=0;
  } else if(!s->dropping) {
   if(s->used==sizeof(s->encoded)) { s->dropping=1; s->used=0; }
   else s->encoded[s->used++]=data[i];
  }
 }
}
int lp_seq_newer(uint32_t a,uint32_t b) { uint32_t d=a-b; return d!=0 && d<0x80000000u; }
static void withdraw(lp_receiver *r) { r->reference_active=0; r->forward_m_s=0; r->yaw_rad_s=0; }
void lp_receiver_init(lp_receiver *r) { memset(r,0,sizeof(*r)); r->state=LP_NO_LINK; }
void lp_receiver_bind(lp_receiver *r,uint64_t session,uint32_t mapping,uint32_t transform) {
 lp_receiver_init(r); r->bound=1; r->session=session; r->mapping_revision=mapping;
 r->transform_revision=transform; r->pending_reenable=1; r->state=LP_RESET_PENDING_REENABLE;
}
void lp_receiver_reject(lp_receiver *r,lp_reason reason) { withdraw(r); r->reason=reason; r->state=LP_REJECTED; }
static int fresh(const lp_receiver *r,uint32_t now) {
 uint32_t age=now-r->estimate_mcu_us;
 return r->have_estimate && r->estimate_usable && age<0x80000000u && age<=LP_AGE_US && r->uncertainty_us<=LP_AGE_US-age;
}
void lp_receiver_update(lp_receiver *r,uint32_t now) {
 if(!r->have_link || (uint32_t)(now-r->last_link_us)>=LP_LINK_US) {
  withdraw(r); r->state=LP_NO_LINK; r->reason=LP_REASON_SILENCE;
  r->enabled=0; r->pending_reenable=1; return;
 }
 if(r->estimate_usable && !fresh(r,now)) { withdraw(r); r->state=LP_STALE; r->reason=LP_REASON_AGE; return; }
 if(r->reference_active && (uint32_t)(now-r->ref_start_us)>=r->ref_lease_us) {
  withdraw(r); r->reason=LP_REASON_LEASE;
 }
}
static int reject(lp_receiver *r,lp_reason why) {
 lp_state before=r->state; lp_receiver_reject(r,why);
 if(why==LP_REASON_AGE && before==LP_STALE) r->state=LP_STALE;
 return 0;
}
int lp_receiver_accept(lp_receiver *r,uint32_t now,const lp_message *m) {
 lp_receiver_update(r,now);
 if(!r->bound || m->session!=r->session) return reject(r,LP_REASON_SESSION);
 if(m->type==LP_HEALTH) {
  const lp_health *h=&m->data.health; uint32_t age,stamp,unc;
  int valid,changed;
  if(r->have_health && !lp_seq_newer(m->seq,r->health_seq)) return reject(r,LP_REASON_SEQUENCE);
  if(h->tracking>LP_FAULT || h->estimator_id==0 || h->source_domain<1 || h->source_domain>3 ||
   h->quality < -1 || h->quality>100 || (h->flags & ~0x1ffu)!=0 ||
   h->frame_id!=LP_ODOM_FRAME || h->child_frame_id!=LP_BODY_FRAME ||
   h->transform_revision!=r->transform_revision || h->mapping_revision!=r->mapping_revision)
   return reject(r,LP_REASON_FIELDS);
  valid=(h->flags & LP_REQUIRED_FLAGS)==LP_REQUIRED_FLAGS && !(h->flags & LP_RESOURCE_FAULT) && h->tracking==LP_TRACKING && h->quality!=-1;
  changed=r->have_health && (h->reset_counter!=r->reset_counter || h->map_epoch!=r->map_epoch);
  stamp=(uint32_t)(h->mapped_mcu_ns/1000u);
  unc=h->uncertainty_ns/1000u+(h->uncertainty_ns%1000u!=0);
  age=now-stamp;
  if(r->have_estimate && h->source_domain!=r->source_domain) return reject(r,LP_REASON_FIELDS);
  if(valid && (h->uncertainty_ns>LP_UNCERTAINTY_NS || age>=0x80000000u || age>LP_AGE_US || unc>LP_AGE_US-age || h->age_at_send_ns>150000000u)) return reject(r,LP_REASON_AGE);
  if(!changed && r->have_estimate) {
   if(h->estimate_seq==r->estimate_seq) {
    if(h->source_ns!=r->source_ns || h->mapped_mcu_ns!=r->mapped_ns || h->source_domain!=r->source_domain) return reject(r,LP_REASON_SEQUENCE);
    /* A heartbeat may only increase the uncertainty of a frozen estimate. */
    if(unc<r->uncertainty_us) return reject(r,LP_REASON_AGE);
   } else if(!lp_seq_newer(h->estimate_seq,r->estimate_seq) || h->source_ns<=r->source_ns || !lp_seq_newer(stamp,r->estimate_mcu_us)) return reject(r,LP_REASON_SEQUENCE);
  }
  if(changed) { withdraw(r); r->enabled=0; r->pending_reenable=1; r->have_estimate=0; r->have_ref_seq=0; }
  r->health_seq=m->seq; r->have_health=1; r->last_link_us=now; r->have_link=1;
  r->map_epoch=h->map_epoch; r->reset_counter=h->reset_counter;
  if(valid) {
   r->have_estimate=1; r->estimate_usable=1; r->source_domain=h->source_domain; r->estimate_seq=h->estimate_seq; r->source_ns=h->source_ns; r->mapped_ns=h->mapped_mcu_ns;
   r->estimate_mcu_us=stamp; r->uncertainty_us=unc;
  } else { r->estimate_usable=0; withdraw(r); }
  r->reason=changed?LP_REASON_RESET:LP_REASON_NONE;
  r->state=r->pending_reenable?LP_RESET_PENDING_REENABLE:valid?LP_ESTIMATE_VALID:LP_LINK_OK_NO_ESTIMATE;
  return 1;
 }
 if(m->type==LP_BODY_REF) {
  const lp_body_ref *v=&m->data.ref; uint32_t stamp=(uint32_t)(v->mapped_mcu_creation_ns/1000u),age=now-stamp;
  if(!r->enabled || r->pending_reenable || !fresh(r,now) || r->state!=LP_ESTIMATE_VALID) return reject(r,LP_REASON_DISABLED);
  if(r->have_ref_seq && !lp_seq_newer(m->seq,r->ref_seq)) return reject(r,LP_REASON_SEQUENCE);
  if(!finitefloat(v->forward_m_s) || !finitefloat(v->yaw_rad_s) || v->frame_id!=LP_BODY_FRAME || v->lease_us==0 || v->lease_us>LP_LEASE_US) return reject(r,LP_REASON_FIELDS);
  if(v->estimate_seq!=r->estimate_seq || v->map_epoch!=r->map_epoch || v->reset_counter!=r->reset_counter || v->mapping_revision!=r->mapping_revision) return reject(r,LP_REASON_ASSOCIATION);
  if(age>=0x80000000u || age>=v->lease_us || r->uncertainty_us>=v->lease_us-age) return reject(r,LP_REASON_AGE);
  r->ref_seq=m->seq; r->have_ref_seq=1; r->ref_start_us=stamp; r->ref_lease_us=v->lease_us-r->uncertainty_us;
  r->reference_active=1; r->forward_m_s=v->forward_m_s; r->yaw_rad_s=v->yaw_rad_s;
  r->last_link_us=now; r->have_link=1; r->reason=LP_REASON_NONE; return 1;
 }
 return reject(r,LP_REASON_FIELDS);
}
int lp_receiver_enable(lp_receiver *r,uint32_t now) {
 lp_receiver_update(r,now);
 if(!r->have_link || !fresh(r,now) || r->state==LP_REJECTED) return 0;
 r->pending_reenable=0; r->enabled=1; r->state=LP_ESTIMATE_VALID; r->reason=LP_REASON_NONE; return 1;
}
void lp_receiver_disable(lp_receiver *r) {
 withdraw(r); r->enabled=0; r->pending_reenable=1; r->state=LP_RESET_PENDING_REENABLE;
}
