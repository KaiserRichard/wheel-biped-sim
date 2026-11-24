#ifndef PERCEPTION_HEALTH_H
#define PERCEPTION_HEALTH_H
#include <stddef.h>
#include <stdint.h>
#define PH_VERSION 3u
#define PH_TYPE 0x05u
#define PH_PAYLOAD_LEN 19u
#define PH_MAX_FRAME 28u
enum ph_tracking_state {
    PH_TRACKING_SYSTEM_NOT_READY = 255u,
    PH_TRACKING_NO_IMAGES_YET = 0u,
    PH_TRACKING_NOT_INITIALIZED = 1u,
    PH_TRACKING_OK = 2u,
    PH_TRACKING_RECENTLY_LOST = 3u,
    PH_TRACKING_LOST = 4u,
    PH_TRACKING_OK_KLT = 5u
};
struct ph_health {
    uint64_t linux_send_ns;
    uint32_t pose_age_us;
    uint8_t tracking_state;
    uint16_t map_reset_counter;
    uint8_t imu_initialized;
    uint16_t tracked_features;
    uint8_t flags;
};
uint16_t ph_crc16(const uint8_t *data, size_t len);
size_t ph_encode(uint16_t seq, const struct ph_health *health, uint8_t *out, size_t cap);
int ph_decode(const uint8_t *frame, size_t len, uint16_t *seq, struct ph_health *health);
#endif
