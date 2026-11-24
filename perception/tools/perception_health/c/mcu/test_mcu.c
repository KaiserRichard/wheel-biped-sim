#include "ph_rx.h"
#include "ph_policy.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void feed_frame(struct ph_rx *rx, const uint8_t *f, size_t n)
{ size_t i; for (i=0u;i<n;++i) ph_rx_feed(rx,f[i]); }

static void rx_tests(void)
{
    struct ph_rx rx; struct ph_rx_event ev; struct ph_health h={123u,1000u,3u,2u,1u,40u,3u};
    uint8_t frame[PH_MAX_FRAME], hb[25]={0xa5u,0x5au,3u,1u,0u,0u,16u};
    uint16_t crc; size_t n; unsigned i;
    ph_rx_init(&rx);
    n=ph_encode(9u,&h,frame,sizeof(frame)); assert(n==sizeof(frame));
    frame[10]^=1u; feed_frame(&rx,frame,n); assert(rx.stats.crc_errors==1u); assert(!ph_rx_pop(&rx,&ev));
    frame[10]^=1u; feed_frame(&rx,frame,n); assert(ph_rx_pop(&rx,&ev));
    assert(ev.kind==PH_RX_HEALTH && ev.seq==9u && ev.health.pose_age_us==1000u);
    hb[4]=10u; hb[7]=0x11u; hb[15]=0x78u; hb[16]=0x56u;
    crc=ph_crc16(hb+2u,21u); hb[23]=(uint8_t)crc; hb[24]=(uint8_t)(crc>>8);
    feed_frame(&rx,hb,sizeof(hb)); assert(ph_rx_pop(&rx,&ev));
    assert(ev.kind==PH_RX_HEARTBEAT && ev.seq==10u && ev.session_id==UINT64_C(0x5678));
    for(i=0u;i<4u;++i) feed_frame(&rx,frame,n);
    assert(rx.stats.queue_drops>0u);
}

static void sticky_latch_test(void)
{
    struct ph_policy_config cfg={100000u,500000u,200000u,500000u,500000u,5000u};
    struct ph_policy p;
    struct ph_rx_event ev;
    assert(ph_policy_init(&p,&cfg));
    memset(&ev,0,sizeof(ev)); ev.kind=PH_RX_HEALTH; ev.seq=1u;
    ev.health.tracking_state=PH_TRACKING_LOST;
    assert(ph_policy_update(&p,100u,&ev,100u,1u)==PH_DOWNGRADE_FAILSAFE);
    assert(p.latched);
    memset(&ev,0,sizeof(ev)); ev.kind=PH_RX_HEARTBEAT; ev.seq=2u; ev.session_id=42u;
    assert(ph_policy_update(&p,200u,&ev,200u,1u)==PH_DOWNGRADE_FAILSAFE);
    memset(&ev,0,sizeof(ev)); ev.kind=PH_RX_HEALTH; ev.seq=3u;
    ev.health.pose_age_us=1000u; ev.health.tracking_state=PH_TRACKING_OK;
    ev.health.imu_initialized=1u; ev.health.flags=3u;
    assert(ph_policy_update(&p,300u,&ev,300u,1u)==PH_DOWNGRADE_FAILSAFE);
    assert(p.latched && p.reason==PH_REASON_SOURCE_AGE_TIMEOUT);
}

static enum ph_downgrade action(const char *s)
{
    if(strcmp(s,"NONE")==0)return PH_DOWNGRADE_NONE;
    if(strcmp(s,"HOLD")==0)return PH_DOWNGRADE_HOLD;
    if(strcmp(s,"FAILSAFE")==0)return PH_DOWNGRADE_FAILSAFE;
    return (enum ph_downgrade)-1;
}
static enum ph_reason reason(const char *s)
{
    unsigned i; for(i=0u;i<=PH_REASON_SESSION_CHANGE;++i) if(strcmp(s,ph_reason_name((enum ph_reason)i))==0)return (enum ph_reason)i;
    return (enum ph_reason)-1;
}
static void parity_file(const char *path)
{
    FILE *f=fopen(path,"r"); char line[512], kind[24], expected_action[24], expected_reason[40];
    unsigned a,b,c,d,e,g, primary; struct ph_policy_config cfg; struct ph_policy p;
    assert(f!=NULL);
    while(fgets(line,sizeof(line),f)!=NULL) {
        if(line[0]=='C') { assert(sscanf(line,"C %u %u %u %u %u %u",&a,&b,&c,&d,&e,&g)==6);
            cfg.pose_age_hold_us=a;cfg.pose_age_timeout_us=b;cfg.receipt_silence_hold_us=c;
            cfg.receipt_silence_timeout_us=d;cfg.heartbeat_timeout_us=e;cfg.transit_bound_us=g;
        } else if(line[0]=='S') { assert(ph_policy_init(&p,&cfg));
        } else if(line[0]=='V') {
            uint32_t now, session, received, age; unsigned seq, tracking, flags, imu, reset;
            struct ph_rx_event ev, *ep=NULL; enum ph_downgrade got; enum ph_reason want;
            assert(sscanf(line,"V %u %23s %u %u %u %u %u %u %u %u %u %23s %39s",
                &now,kind,&seq,&session,&received,&age,&tracking,&flags,&imu,&reset,&primary,expected_action,expected_reason)==13);
            memset(&ev,0,sizeof(ev));
            if(strcmp(kind,"heartbeat")==0) {ev.kind=PH_RX_HEARTBEAT;ev.session_id=session;ev.seq=(uint16_t)seq;ep=&ev;}
            else if(strcmp(kind,"health")==0) {ev.kind=PH_RX_HEALTH;ev.seq=(uint16_t)seq;ev.health.pose_age_us=age;
                ev.health.tracking_state=(uint8_t)tracking;ev.health.flags=(uint8_t)flags;ev.health.imu_initialized=(uint8_t)imu;
                ev.health.map_reset_counter=(uint16_t)reset;ep=&ev;}
            got=ph_policy_update(&p,now,ep,received,(uint8_t)primary); want=reason(expected_reason);
            if(got!=action(expected_action) || p.reason!=want) {
                fprintf(stderr,"parity mismatch: %s expected %s/%s got %s/%s\n",line,expected_action,expected_reason,
                    got==PH_DOWNGRADE_NONE?"NONE":got==PH_DOWNGRADE_HOLD?"HOLD":"FAILSAFE",ph_reason_name(p.reason)); abort();
            }
        }
    }
    fclose(f);
}
int main(int argc,char **argv)
{
    assert(argc==2); rx_tests(); sticky_latch_test(); parity_file(argv[1]); puts("C receiver and Python policy parity: PASS"); return 0;
}
