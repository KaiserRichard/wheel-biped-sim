#ifndef PH_POLICY_H
#define PH_POLICY_H
#include <stdint.h>
#include "ph_rx.h"

enum ph_downgrade { PH_DOWNGRADE_NONE = 0, PH_DOWNGRADE_HOLD = 1, PH_DOWNGRADE_FAILSAFE = 2 };
enum ph_reason {
    PH_REASON_NONE = 0, PH_REASON_SOURCE_AGE_HOLD, PH_REASON_SOURCE_AGE_TIMEOUT,
    PH_REASON_RESULT_SILENCE, PH_REASON_RESULT_TIMEOUT, PH_REASON_HEARTBEAT_TIMEOUT,
    PH_REASON_CLOCK_REGRESSION, PH_REASON_SESSION_CHANGE
};
struct ph_policy_config {
    uint32_t pose_age_hold_us, pose_age_timeout_us;
    uint32_t receipt_silence_hold_us, receipt_silence_timeout_us;
    uint32_t heartbeat_timeout_us, transit_bound_us;
};
struct ph_policy {
    struct ph_policy_config c;
    enum ph_downgrade requested;
    enum ph_reason reason;
    uint32_t last_now_us, last_health_us, last_heartbeat_us, healthy_since_us;
    uint32_t last_age_us;
    uint16_t last_wire_seq, map_reset_counter;
    uint64_t session_id;
    uint8_t have_now, have_health, have_heartbeat, have_seq, have_reset;
    uint8_t age_valid, last_report_fresh, latched, have_session, have_healthy_since;
};
int ph_policy_init(struct ph_policy *p, const struct ph_policy_config *config);
/* now_us and receipt times share only the MCU's wrapping u32 microsecond timer.
 * Correct when updates are sampled less than 2^31 us apart. Pi timestamps and
 * Pi send durations are never subtracted from MCU timer values. */
enum ph_downgrade ph_policy_update(struct ph_policy *p, uint32_t now_us,
                                  const struct ph_rx_event *event, uint32_t received_at_us,
                                  uint8_t primary_fresh);
const char *ph_reason_name(enum ph_reason reason);
#endif
