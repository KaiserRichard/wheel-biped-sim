#include "ph_rx.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t rng;

static uint32_t next_random(void)
{
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    return rng;
}

static void put64(uint8_t *p, uint64_t value)
{
    unsigned i;
    for (i = 0u; i < 8u; ++i)
        p[i] = (uint8_t)(value >> (8u * i));
}

static size_t heartbeat(uint16_t seq, uint64_t send_ns, uint64_t session, uint8_t *out)
{
    uint16_t crc;
    memset(out, 0, 25u);
    out[0] = 0xa5u;
    out[1] = 0x5au;
    out[2] = PH_VERSION;
    out[3] = PH_HEARTBEAT_TYPE;
    out[4] = (uint8_t)seq;
    out[5] = (uint8_t)(seq >> 8);
    out[6] = 16u;
    put64(out + 7u, send_ns);
    put64(out + 15u, session);
    crc = ph_crc16(out + 2u, 21u);
    out[23] = (uint8_t)crc;
    out[24] = (uint8_t)(crc >> 8);
    return 25u;
}

static void feed(struct ph_rx *rx, const uint8_t *data, size_t len)
{
    size_t i;
    for (i = 0u; i < len; ++i)
        ph_rx_feed_at(rx, data[i], next_random());
}

static int drain(struct ph_rx *rx)
{
    struct ph_rx_event event;
    int count = 0;
    while (ph_rx_pop(rx, &event))
        ++count;
    return count;
}

int main(int argc, char **argv)
{
    unsigned long iterations = argc > 1 ? strtoul(argv[1], NULL, 10) : 1200ul;
    unsigned long i;
    uint8_t valid[PH_MAX_FRAME], mutated[PH_MAX_FRAME], hb[25];
    struct ph_health health = {123456789u, 55u, 3u, 2u, 1u, 40u, 3u};
    rng = argc > 2 ? (uint32_t)strtoul(argv[2], NULL, 10) : 20261018u;
    if (ph_encode(19u, &health, valid, sizeof(valid)) != sizeof(valid))
        return 2;

    for (i = 0ul; i < iterations; ++i) {
        struct ph_rx rx;
        size_t len = sizeof(valid);
        memcpy(mutated, valid, sizeof(valid));
        if (i % 3ul == 0ul)
            mutated[next_random() % len] ^= (uint8_t)(1u << (next_random() % 8u));
        if (i % 5ul == 0ul)
            len = next_random() % (sizeof(valid) + 1u);
        ph_rx_init(&rx);
        feed(&rx, mutated, len);
        (void)drain(&rx);
        if (i % 11ul == 1ul) {
            struct ph_rx_event event;
            ph_rx_init(&rx);
            feed(&rx, valid, sizeof(valid));
            if (!ph_rx_pop(&rx, &event) || event.kind != PH_RX_HEALTH ||
                event.seq != 19u || event.health.pose_age_us != 55u)
                return 3;
            if (ph_rx_pop(&rx, &event))
                return 4;
        }
        if (i % 17ul == 2ul) {
            struct ph_rx_event event;
            size_t n = heartbeat(21u, 123456789u, UINT64_C(0x0102030405060708), hb);
            ph_rx_init(&rx);
            feed(&rx, hb, n);
            if (!ph_rx_pop(&rx, &event) || event.kind != PH_RX_HEARTBEAT ||
                event.seq != 21u || event.session_id != UINT64_C(0x0102030405060708))
                return 5;
        }
        {
            uint8_t noise[16];
            size_t j;
            for (j = 0u; j < sizeof(noise); ++j)
                noise[j] = (uint8_t)next_random();
            feed(&rx, noise, sizeof(noise));
            (void)drain(&rx);
        }
    }
    printf("perception_health ph_rx mutations: %lu seed=%u\n", iterations, rng);
    return 0;
}
