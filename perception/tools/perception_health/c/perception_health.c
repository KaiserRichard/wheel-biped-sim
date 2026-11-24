#include "perception_health.h"

uint16_t ph_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xffffu;
    size_t i;
    unsigned bit;
    for (i = 0u; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0u; bit < 8u; ++bit)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}
static void put16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void put32(uint8_t *p, uint32_t v) { unsigned i; for(i=0;i<4u;i++) p[i]=(uint8_t)(v>>(8u*i)); }
static void put64(uint8_t *p, uint64_t v) { unsigned i; for(i=0;i<8u;i++) p[i]=(uint8_t)(v>>(8u*i)); }
static uint16_t get16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1]<<8)); }
static uint32_t get32(const uint8_t *p) { uint32_t v=0; unsigned i; for(i=0;i<4u;i++) v|=(uint32_t)p[i]<<(8u*i); return v; }
static uint64_t get64(const uint8_t *p) { uint64_t v=0; unsigned i; for(i=0;i<8u;i++) v|=(uint64_t)p[i]<<(8u*i); return v; }
size_t ph_encode(uint16_t seq, const struct ph_health *h, uint8_t *out, size_t cap)
{
    uint16_t crc;
    if (h == NULL || out == NULL || cap < PH_MAX_FRAME || h->imu_initialized > 1u) return 0u;
    out[0]=0xa5u; out[1]=0x5au; out[2]=PH_VERSION; out[3]=PH_TYPE; put16(out+4,seq); out[6]=PH_PAYLOAD_LEN;
    put64(out+7,h->linux_send_ns); put32(out+15,h->pose_age_us); out[19]=h->tracking_state;
    out[20]=h->imu_initialized; put16(out+21,h->map_reset_counter); put16(out+23,h->tracked_features); out[25]=h->flags;
    crc=ph_crc16(out+2,24u); put16(out+26,crc); return PH_MAX_FRAME;
}
int ph_decode(const uint8_t *f, size_t len, uint16_t *seq, struct ph_health *h)
{
    if (f==NULL || seq==NULL || h==NULL || len!=PH_MAX_FRAME || f[0]!=0xa5u || f[1]!=0x5au ||
        f[2]!=PH_VERSION || f[3]!=PH_TYPE || f[6]!=PH_PAYLOAD_LEN || get16(f+26)!=ph_crc16(f+2,24u) || f[20]>1u) return 0;
    *seq=get16(f+4); h->linux_send_ns=get64(f+7); h->pose_age_us=get32(f+15); h->tracking_state=f[19];
    h->imu_initialized=f[20]; h->map_reset_counter=get16(f+21); h->tracked_features=get16(f+23); h->flags=f[25]; return 1;
}
