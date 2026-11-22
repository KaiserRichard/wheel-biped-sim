#include "twin.h"
/* ASSUMED text grammar: strict decimal, no exponent, <=6 digits before decimal,
 * <=6 after. Values clip in dm_pack; uint fields reject overflow. */
static int number(const char **p,float *v) {
 const char *s=*p; int neg=0,nd=0,nf=0; float x=0,scale=.1f;
 if(*s=='-' || *s=='+') neg=*s++=='-';
 while(*s>='0' && *s<='9') { if(++nd>6) return 0; x=x*10+(*s++-'0'); }
 if(!nd) return 0;
 if(*s=='.') { s++; while(*s>='0' && *s<='9') { if(++nf>6) return 0; x+=(*s++-'0')*scale; scale*=.1f; } if(!nf) return 0; }
 *v=neg?-x:x; *p=s; return 1;
}
static int natural(const char **p,uint32_t *v) {
 const char *s=*p; uint32_t x=0; int n=0;
 while(*s>='0' && *s<='9') { uint32_t d=*s++-'0'; if(++n>10 || x>(UINT32_MAX-d)/10u) return 0; x=x*10u+d; }
 if(!n) return 0;
 *v=x; *p=s; return 1;
}
int twin_parse(const char *s,twin_command *c) {
 char kind=*s++; uint32_t id;
 *c=(twin_command){0};
 if((kind!='C' && kind!='E' && kind!='D') || *s++!=',') return 0;
 if(!natural(&s,&c->seq) || *s++!=',' || !natural(&s,&id) || dm_index(id)<0) return 0;
 c->id=(uint8_t)id;
 if(kind=='C') {
  float *fields[5]={&c->cmd.p,&c->cmd.v,&c->cmd.kp,&c->cmd.kd,&c->cmd.t};
  for(int i=0;i<5;i++) if(*s++!=',' || !number(&s,fields[i])) return 0;
 } else c->special=kind=='E'?DM_ENABLE:DM_DISABLE;
 if(*s=='\r') s++;
 if(*s=='\n') s++;
 return *s==0;
}
typedef struct { char *p; size_t cap,n; } writer;
static void ch(writer *w,char c) { if(w->n<w->cap) w->p[w->n]=c; w->n++; }
static void uint_(writer *w,uint32_t x) {
 char b[10]; unsigned n=0; do { b[n++]=(char)('0'+x%10); x/=10; } while(x);
 while(n) ch(w,b[--n]);
}
static void fixed(writer *w,float x) {
 if(x<0) { ch(w,'-'); x=-x; }
 uint32_t m=(uint32_t)(x*1000+.5f); uint_(w,m/1000); ch(w,'.');
 ch(w,(char)('0'+m/100%10)); ch(w,(char)('0'+m/10%10)); ch(w,(char)('0'+m%10));
}
size_t twin_format(char *out,size_t cap,uint32_t ms,const dm_feedback *f) {
 writer w={out,cap,0}; ch(&w,'M'); ch(&w,','); uint_(&w,ms); ch(&w,','); uint_(&w,f->id);
 ch(&w,','); fixed(&w,f->p); ch(&w,','); fixed(&w,f->v); ch(&w,','); fixed(&w,f->t);
 ch(&w,','); uint_(&w,f->tmos); ch(&w,','); uint_(&w,f->state); ch(&w,'\n');
 if(w.n>=cap) return 0;
 out[w.n]=0; return w.n;
}
uint8_t twin_leds(const twin_link *s) {
 if(!s->have || (uint32_t)(s->now-s->last)>=TWIN_STALE_MS) return 0;
 /* SOURCE-SUPPORTED mapping: tools/link_bench/firmware/bench.c bench_led_mask. */
 return (uint8_t)(1u<<s->value);
}
static void echo(twin_link *s,uint32_t seq,uint8_t value,lp_reason result) {
 lp_message m={0}; uint8_t out[LP_MAX_WIRE+2];
 m.type=LP_ADVISORY_ECHO; m.session=s->boot_id; m.seq=++s->echo_seq;
 m.data.echo=(lp_advisory_echo){s->uptime_ms*1000000u,seq,value<=3?value:0,result,
  s->accepted,s->rejected,twin_leds(s)==0,s->have?(uint32_t)(s->now-s->last):UINT32_MAX};
 /* ASSUMED mixed UART envelope: 0 + unchanged COBS wire + LF.
  * Keeps every following I-line at the start of a text line. */
 out[0]=0; size_t n=lp_pack(&m,out+1,LP_MAX_WIRE);
 if(n) { out[n+1]='\n'; s->tx(s->ctx,out,n+2); } else s->echo_encode_drop++;
 s->echo_at=s->now;
}
static void packet(void *ctx,const lp_message *m) {
 twin_link *s=ctx; lp_reason why=LP_REASON_NONE;
 if(m->type!=LP_TERRAIN_ADVISORY) { s->other++; return; }
 if(m->session!=TWIN_SESSION) why=LP_REASON_SESSION;
 else if(s->have && !lp_seq_newer(m->seq,s->seq)) why=LP_REASON_SEQUENCE;
 if(why==LP_REASON_NONE) { s->last=s->now; s->seq=m->seq; s->value=m->data.advisory.advisory; s->have=1; s->accepted++; }
 else s->rejected++;
 echo(s,m->seq,m->data.advisory.advisory,why);
}
static void error(void *ctx) {
 twin_link *s=ctx;
 s->parse_errors++;
 if(s->stream.last_type==LP_TERRAIN_ADVISORY) {
  s->rejected++; echo(s,s->stream.last_seq,s->stream.last_advisory,s->stream.last_reason);
 }
}
void twin_init(twin_link *s,uint64_t boot_id,twin_tx tx,twin_on_command command,void *ctx) {
 *s=(twin_link){0}; s->boot_id=boot_id; s->tx=tx; s->command=command; s->ctx=ctx;
}
void twin_rx_gap(twin_link *s) {
 s->rx_gap++; s->parse_errors++;
 s->mode=0; s->used=0; s->overflow=0; s->stream=(lp_stream){0};
}
static void expire(twin_link *s,uint32_t now) {
 if(s->mode && (uint32_t)(now-s->byte_at)>=100u) {
  s->mux_timeout++; s->parse_errors++;
  s->mode=0; s->used=0; s->overflow=0; s->stream=(lp_stream){0};
 }
}
void twin_poll(twin_link *s,uint32_t now) {
 expire(s,now);
 s->uptime_ms+=(uint32_t)(now-s->now); s->now=now;
 if(!twin_leds(s) && (uint32_t)(now-s->echo_at)>=100u) echo(s,s->seq,s->value,LP_REASON_NONE);
}
void twin_feed(twin_link *s,uint8_t b,uint32_t now) {
 expire(s,now); s->byte_at=now;
 s->uptime_ms+=(uint32_t)(now-s->now); s->now=now;
 if(!s->mode) {
  if(b==0 || b=='\n' || b=='\r') return;
  s->mode=(b=='C' || b=='E' || b=='D')?1:2;
 }
 if(s->mode==2) {
  s->binary_bytes++;
  /* Canonical stream only reports overflow when a delimiter arrives. Without
   * zero, it can swallow unlimited C-lines with no error. Count and leave the
   * binary mode immediately at its bounded capacity instead. */
  if(b && s->stream.used==sizeof(s->stream.encoded)) {
   s->binary_overflow++; s->parse_errors++;
   s->stream=(lp_stream){0}; s->mode=0; return;
  }
  lp_stream_feed(&s->stream,&b,1,packet,error,s);
  if(!b) s->mode=0;
 } else if(b=='\n' || b=='\r' || b==0) {
  twin_command c;
  s->line[s->used]=0;
  if(b && !s->overflow && twin_parse(s->line,&c)) { s->text_ok++; s->command(s->ctx,&c); }
  else s->parse_errors++;
  s->used=0; s->overflow=0; s->mode=0;
 } else if(s->used<TWIN_LINE_CAP-1) s->line[s->used++]=(char)b;
 else if(!s->overflow) { s->overflow=1; s->text_overflow++; }
}
