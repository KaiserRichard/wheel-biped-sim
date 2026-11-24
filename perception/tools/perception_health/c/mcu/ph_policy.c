#include "ph_policy.h"
#include "../perception_health.h"
#include <string.h>

#define FLAG_AGE_VALID 1u
#define FLAG_MAPPING_VALID 2u
#define FLAG_DEGRADED 4u
static uint32_t elapsed(uint32_t now, uint32_t then) { return (uint32_t)(now - then); }
static uint32_t sat_add3(uint32_t a, uint32_t b, uint32_t c)
{
    uint64_t v = (uint64_t)a + b + c;
    return v > UINT32_MAX ? UINT32_MAX : (uint32_t)v;
}
static void transition(struct ph_policy *p, enum ph_downgrade state, enum ph_reason why)
{
    if (p->latched && state != PH_DOWNGRADE_FAILSAFE) return;
    p->requested = state; p->reason = why;
}
static int age_flags_valid(const struct ph_health *h)
{ return (h->flags & (FLAG_AGE_VALID | FLAG_MAPPING_VALID)) == (FLAG_AGE_VALID | FLAG_MAPPING_VALID); }

int ph_policy_init(struct ph_policy *p, const struct ph_policy_config *c)
{
    if (p == NULL || c == NULL || c->pose_age_hold_us == 0u ||
        c->pose_age_timeout_us <= c->pose_age_hold_us ||
        c->receipt_silence_timeout_us <= c->receipt_silence_hold_us ||
        c->heartbeat_timeout_us == 0u) return 0;
    memset(p, 0, sizeof(*p)); p->c = *c; p->reason = PH_REASON_NONE; return 1;
}

enum ph_downgrade ph_policy_update(struct ph_policy *p, uint32_t now, const struct ph_rx_event *ev,
                                  uint32_t received, uint8_t primary_fresh)
{
    uint32_t silence, age;
    int valid_age;
    if (p == NULL) return PH_DOWNGRADE_FAILSAFE;
    if (p->have_now && elapsed(now, p->last_now_us) >= UINT32_C(0x80000000)) {
        p->latched=1u; p->have_healthy_since=0u;
        transition(p, PH_DOWNGRADE_FAILSAFE, PH_REASON_CLOCK_REGRESSION); return p->requested;
    }
    p->last_now_us=now; p->have_now=1u;
    if (p->have_heartbeat && elapsed(now,p->last_heartbeat_us) >= p->c.heartbeat_timeout_us && !p->latched) {
        p->latched=1u; p->have_healthy_since=0u; transition(p,PH_DOWNGRADE_FAILSAFE,PH_REASON_HEARTBEAT_TIMEOUT);
    }
    if (p->have_health) {
        silence=elapsed(now,p->last_health_us); valid_age=p->age_valid != 0u;
        age=valid_age ? sat_add3(p->last_age_us,silence,p->c.transit_bound_us) : 0u;
        if (((silence >= p->c.receipt_silence_timeout_us) || (valid_age && age >= p->c.pose_age_timeout_us)) && !p->latched) {
            p->latched=1u; p->have_healthy_since=0u;
            transition(p,PH_DOWNGRADE_FAILSAFE,valid_age && age >= p->c.pose_age_timeout_us ? PH_REASON_SOURCE_AGE_TIMEOUT : PH_REASON_RESULT_TIMEOUT);
        } else if (silence >= p->c.receipt_silence_hold_us) {
            transition(p,PH_DOWNGRADE_HOLD,PH_REASON_RESULT_SILENCE); p->have_healthy_since=0u;
        } else if (valid_age && age >= p->c.pose_age_hold_us) {
            transition(p,PH_DOWNGRADE_HOLD,PH_REASON_SOURCE_AGE_HOLD); p->have_healthy_since=0u;
        }
    }
    if (ev != NULL) {
        uint16_t delta;
        if (elapsed(now,received) >= UINT32_C(0x80000000)) return p->requested;
        if (ev->kind == PH_RX_HEARTBEAT && (!p->have_session || p->session_id != ev->session_id)) {
            p->session_id=ev->session_id; p->have_session=1u; p->have_heartbeat=0u; p->have_health=0u;
            p->age_valid=0u; p->have_reset=0u; p->last_report_fresh=0u; p->have_healthy_since=0u; p->have_seq=0u;
            if (p->requested != PH_DOWNGRADE_NONE && !p->latched) transition(p,PH_DOWNGRADE_NONE,PH_REASON_SESSION_CHANGE);
        }
        if (p->have_seq) { delta=(uint16_t)(ev->seq-p->last_wire_seq); if (delta==0u || delta>=0x8000u) return p->requested; }
        p->last_wire_seq=ev->seq; p->have_seq=1u;
        if (ev->kind == PH_RX_HEARTBEAT) p->last_heartbeat_us=received, p->have_heartbeat=1u;
        else if (ev->kind == PH_RX_HEALTH) {
            uint8_t reset_changed=(uint8_t)(p->have_reset && ev->health.map_reset_counter != p->map_reset_counter);
            p->map_reset_counter=ev->health.map_reset_counter; p->have_reset=1u;
            p->last_health_us=received; p->have_health=1u;
            p->last_age_us=ev->health.pose_age_us; p->age_valid=(uint8_t)age_flags_valid(&ev->health);
            p->last_report_fresh=0u;
            if (ev->health.tracking_state==PH_TRACKING_LOST) { p->latched=1u; p->have_healthy_since=0u; transition(p,PH_DOWNGRADE_FAILSAFE,PH_REASON_SOURCE_AGE_TIMEOUT); return p->requested; }
            if (!p->age_valid) { transition(p,PH_DOWNGRADE_HOLD,PH_REASON_SOURCE_AGE_HOLD); p->have_healthy_since=0u; return p->requested; }
            age=sat_add3(p->last_age_us,0u,p->c.transit_bound_us);
            if (age >= p->c.pose_age_timeout_us) { p->latched=1u; p->have_healthy_since=0u; transition(p,PH_DOWNGRADE_FAILSAFE,PH_REASON_SOURCE_AGE_TIMEOUT); return p->requested; }
            if (age >= p->c.pose_age_hold_us || (ev->health.flags & FLAG_DEGRADED) != 0u ||
                ev->health.imu_initialized != 1u || reset_changed ||
                (ev->health.tracking_state != PH_TRACKING_OK && ev->health.tracking_state != PH_TRACKING_OK_KLT)) {
                transition(p,PH_DOWNGRADE_HOLD,PH_REASON_SOURCE_AGE_HOLD); p->have_healthy_since=0u; return p->requested;
            }
            p->last_report_fresh=1u;
        }
    }
    if (p->have_heartbeat && p->have_health) {
        silence=elapsed(now,p->last_health_us); valid_age=p->age_valid != 0u;
        age=valid_age ? sat_add3(p->last_age_us,silence,p->c.transit_bound_us) : 0u;
        if (primary_fresh && p->last_report_fresh && valid_age && age < p->c.pose_age_hold_us &&
            silence < p->c.receipt_silence_hold_us && elapsed(now,p->last_heartbeat_us) < p->c.heartbeat_timeout_us) {
            if (!p->have_healthy_since) p->healthy_since_us=now, p->have_healthy_since=1u;
            if (!p->latched) transition(p,PH_DOWNGRADE_NONE,PH_REASON_NONE);
        } else {
            p->have_healthy_since=0u;
            if (!p->latched && p->requested==PH_DOWNGRADE_NONE)
                transition(p,PH_DOWNGRADE_HOLD,(!valid_age || age >= p->c.pose_age_hold_us) ? PH_REASON_SOURCE_AGE_HOLD : PH_REASON_RESULT_SILENCE);
        }
    }
    return p->requested;
}

const char *ph_reason_name(enum ph_reason r)
{
    static const char *const names[]={"NONE","SOURCE_AGE_HOLD","SOURCE_AGE_TIMEOUT","RESULT_SILENCE","RESULT_TIMEOUT","HEARTBEAT_TIMEOUT","CLOCK_REGRESSION","SESSION_CHANGE"};
    return (unsigned)r < sizeof(names)/sizeof(names[0]) ? names[(unsigned)r] : "UNKNOWN";
}
