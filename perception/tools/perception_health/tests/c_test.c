#include "../c/perception_health.h"
#include <assert.h>
#include <stdint.h>
#include <string.h>
int main(void) {
    static const uint8_t expected[PH_MAX_FRAME] = {
        0xa5,0x5a,0x03,0x05,0x34,0x12,0x13,0x08,0x07,0x06,0x05,0x04,0x03,0x02,0x01,
        0x44,0x33,0x22,0x11,0x03,0x01,0x66,0x55,0x88,0x77,0x03,0x3f,0x33
    };
    const uint8_t check[]="123456789";
    struct ph_health in={UINT64_C(0x0102030405060708),UINT32_C(0x11223344),3u,UINT16_C(0x5566),1u,UINT16_C(0x7788),3u}, out;
    uint8_t frame[PH_MAX_FRAME]; uint16_t seq=0u;
    assert(ph_crc16(check,9u)==UINT16_C(0x29b1));
    assert(ph_encode(UINT16_C(0x1234),&in,frame,sizeof frame)==sizeof frame);
    assert(memcmp(frame,expected,sizeof expected)==0);
    assert(ph_decode(frame,sizeof frame,&seq,&out)==1 && seq==UINT16_C(0x1234));
    assert(in.linux_send_ns==out.linux_send_ns && in.pose_age_us==out.pose_age_us &&
        in.tracking_state==out.tracking_state && in.map_reset_counter==out.map_reset_counter &&
        in.imu_initialized==out.imu_initialized && in.tracked_features==out.tracked_features && in.flags==out.flags);
    return 0;
}
