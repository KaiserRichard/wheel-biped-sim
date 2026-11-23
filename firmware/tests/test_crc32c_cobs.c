#ifdef NDEBUG
#undef NDEBUG
#endif
#include "cobs.h"
#include "crc32c.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define S11_CHECK(expr)                                                   \
    do {                                                                  \
        if (!(expr)) {                                                    \
            fprintf(stderr, "%s:%d: %s: Assertion `%s' failed.\n",        \
                    __FILE__, __LINE__, __func__, #expr);                 \
            abort();                                                      \
        }                                                                 \
    } while (0)

#undef assert
#define assert(expr) S11_CHECK(expr)

static uint32_t crc32c_bitwise_ref(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0u; i < len; ++i) {
        crc ^= (uint32_t)data[i];
        for (uint32_t bit = 0u; bit < 8u; ++bit) {
            if ((crc & 1u) != 0u) {
                crc = (crc >> 1u) ^ 0x82F63B78u;
            } else {
                crc >>= 1u;
            }
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

static void test_crc32c_known_vectors(void)
{
    /* Standard Castagnoli check value: "123456789" -> 0xE3069283. */
    const uint8_t check_vec[9] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    assert(crc32c_compute(check_vec, sizeof(check_vec)) == 0xE3069283u);

    /* Incremental update must match one-shot exactly. */
    uint32_t inc = crc32c_init();
    inc = crc32c_update(inc, &check_vec[0], 4u);
    inc = crc32c_update(inc, &check_vec[4], 5u);
    assert(crc32c_final(inc) == 0xE3069283u);

    /* Byte-by-byte incremental update. */
    inc = crc32c_init();
    for (size_t i = 0u; i < sizeof(check_vec); ++i) {
        inc = crc32c_update(inc, &check_vec[i], 1u);
    }
    assert(crc32c_final(inc) == 0xE3069283u);

    /* Empty input & NULL with zero length. */
    assert(crc32c_compute(NULL, 0u) == 0x00000000u);
    assert(crc32c_compute(check_vec, 0u) == 0x00000000u);

    /* RFC 3720 Castagnoli known 32-byte vectors. */
    uint8_t zeros[32];
    memset(zeros, 0x00, sizeof(zeros));
    assert(crc32c_compute(zeros, sizeof(zeros)) == 0x8A9136AAu);

    uint8_t ones[32];
    memset(ones, 0xFF, sizeof(ones));
    assert(crc32c_compute(ones, sizeof(ones)) == 0x62A8AB43u);

    uint8_t asc[32];
    uint8_t desc[32];
    for (uint8_t i = 0u; i < 32u; ++i) {
        asc[i] = i;
        desc[i] = (uint8_t)(31u - i);
    }
    assert(crc32c_compute(asc, sizeof(asc)) == 0x46DD794Eu);
    assert(crc32c_compute(desc, sizeof(desc)) == 0x113FDB5Cu);

    /* Verify every single 1-byte value against bitwise reference. */
    for (uint32_t b = 0u; b < 256u; ++b) {
        const uint8_t v = (uint8_t)b;
        assert(crc32c_compute(&v, 1u) == crc32c_bitwise_ref(&v, 1u));
    }
}

static void assert_cobs_round_trip(const uint8_t *raw, size_t raw_len)
{
    uint8_t encoded[1024];
    uint8_t decoded[1024];
    size_t enc_len = 0u;
    size_t dec_len = 0u;

    assert(cobs_max_encoded_len(raw_len) <= sizeof(encoded));
    assert(raw_len <= sizeof(decoded));

    const CobsStatus es = cobs_encode(raw, raw_len, encoded, sizeof(encoded), &enc_len);
    assert(es == COBS_OK);
    assert(enc_len >= 1u);
    assert(enc_len <= cobs_max_encoded_len(raw_len));

    /* COBS output must NEVER contain 0x00. */
    for (size_t i = 0u; i < enc_len; ++i) {
        assert(encoded[i] != 0x00u);
    }

    const CobsStatus ds = cobs_decode(encoded, enc_len, decoded, sizeof(decoded), &dec_len);
    assert(ds == COBS_OK);
    assert(dec_len == raw_len);
    if (raw_len > 0u) {
        assert(memcmp(raw, decoded, raw_len) == 0);
    }
}

static void test_cobs_canonical_and_edges(void)
{
    uint8_t enc[600];
    size_t enc_len = 0u;

    /* Empty input -> { 0x01 } */
    assert(cobs_encode(NULL, 0u, enc, sizeof(enc), &enc_len) == COBS_OK);
    assert(enc_len == 1u && enc[0] == 0x01u);
    assert_cobs_round_trip(NULL, 0u);

    /* Single zero -> { 0x01, 0x01 } */
    const uint8_t z1[1] = { 0x00u };
    assert(cobs_encode(z1, 1u, enc, sizeof(enc), &enc_len) == COBS_OK);
    assert(enc_len == 2u && enc[0] == 0x01u && enc[1] == 0x01u);
    assert_cobs_round_trip(z1, 1u);

    /* Two zeros -> { 0x01, 0x01, 0x01 } */
    const uint8_t z2[2] = { 0x00u, 0x00u };
    assert(cobs_encode(z2, 2u, enc, sizeof(enc), &enc_len) == COBS_OK);
    assert(enc_len == 3u && enc[0] == 0x01u && enc[1] == 0x01u && enc[2] == 0x01u);
    assert_cobs_round_trip(z2, 2u);

    /* Classic Cheshire & Baker example: { 11, 22, 00, 33 } -> { 03, 11, 22, 02, 33 } */
    const uint8_t ex1[4] = { 0x11u, 0x22u, 0x00u, 0x33u };
    const uint8_t ex1_exp[5] = { 0x03u, 0x11u, 0x22u, 0x02u, 0x33u };
    assert(cobs_encode(ex1, sizeof(ex1), enc, sizeof(enc), &enc_len) == COBS_OK);
    assert(enc_len == sizeof(ex1_exp));
    assert(memcmp(enc, ex1_exp, sizeof(ex1_exp)) == 0);
    assert_cobs_round_trip(ex1, sizeof(ex1));

    /* All zeros of various lengths. */
    uint8_t all_zeros[300];
    memset(all_zeros, 0x00, sizeof(all_zeros));
    assert_cobs_round_trip(all_zeros, 16u);
    assert_cobs_round_trip(all_zeros, 254u);
    assert_cobs_round_trip(all_zeros, 255u);

    /* Non-zero runs around 254-byte boundary: 253, 254, 255, 508, 509. */
    uint8_t nonzero[520];
    for (size_t i = 0u; i < sizeof(nonzero); ++i) {
        nonzero[i] = (uint8_t)((i % 255u) + 1u); /* Strictly in 1..255 */
    }
    assert_cobs_round_trip(nonzero, 253u);
    assert_cobs_round_trip(nonzero, 254u);
    assert_cobs_round_trip(nonzero, 255u);
    assert_cobs_round_trip(nonzero, 508u);
    assert_cobs_round_trip(nonzero, 509u);

    /* Exact 254 non-zero bytes must encode to 255 bytes starting with 0xFF. */
    assert(cobs_encode(nonzero, 254u, enc, sizeof(enc), &enc_len) == COBS_OK);
    assert(enc_len == 255u);
    assert(enc[0] == 0xFFu);
    assert(memcmp(&enc[1], nonzero, 254u) == 0);

    /* 254 non-zero bytes followed by 0x00, and 0x00 followed by 254 non-zero bytes. */
    uint8_t edge_a[255];
    memcpy(edge_a, nonzero, 254u);
    edge_a[254] = 0x00u;
    assert_cobs_round_trip(edge_a, sizeof(edge_a));

    uint8_t edge_b[255];
    edge_b[0] = 0x00u;
    memcpy(&edge_b[1], nonzero, 254u);
    assert_cobs_round_trip(edge_b, sizeof(edge_b));

    /* 254 non-zero + 0x00 + 254 non-zero. */
    uint8_t edge_c[509];
    memcpy(edge_c, nonzero, 254u);
    edge_c[254] = 0x00u;
    memcpy(&edge_c[255], nonzero, 254u);
    assert_cobs_round_trip(edge_c, sizeof(edge_c));
}

static void test_cobs_error_handling(void)
{
    uint8_t dst[16];
    size_t out_len = 99u;

    /* NULL checks */
    assert(cobs_encode(NULL, 1u, dst, sizeof(dst), &out_len) == COBS_ERR_NULL_PTR);
    assert(cobs_encode(dst, 1u, NULL, sizeof(dst), &out_len) == COBS_ERR_NULL_PTR);
    assert(cobs_encode(dst, 1u, dst, sizeof(dst), NULL) == COBS_ERR_NULL_PTR);
    assert(cobs_decode(NULL, 1u, dst, sizeof(dst), &out_len) == COBS_ERR_NULL_PTR);
    assert(cobs_decode(dst, 1u, dst, sizeof(dst), NULL) == COBS_ERR_NULL_PTR);

    /* Empty encoded input */
    assert(cobs_decode(dst, 0u, dst, sizeof(dst), &out_len) == COBS_ERR_EMPTY_INPUT);

    /* Unexpected 0x00 byte in encoded stream */
    const uint8_t bad_zero[3] = { 0x03u, 0x11u, 0x00u };
    assert(cobs_decode(bad_zero, sizeof(bad_zero), dst, sizeof(dst), &out_len)
           == COBS_ERR_ZERO_IN_INPUT);

    const uint8_t bad_code[2] = { 0x00u, 0x11u };
    assert(cobs_decode(bad_code, sizeof(bad_code), dst, sizeof(dst), &out_len)
           == COBS_ERR_ZERO_IN_INPUT);

    /* Truncated block: code=0x05 expects 4 bytes, only 2 follow */
    const uint8_t trunc[3] = { 0x05u, 0xAAu, 0xBBu };
    assert(cobs_decode(trunc, sizeof(trunc), dst, sizeof(dst), &out_len) == COBS_ERR_TRUNCATED);

    /* Output buffer too small on encode and decode */
    const uint8_t raw[4] = { 1u, 2u, 3u, 4u };
    assert(cobs_encode(raw, sizeof(raw), dst, 4u, &out_len) == COBS_ERR_BUFFER_TOO_SMALL);
    assert(cobs_encode(raw, sizeof(raw), dst, sizeof(dst), &out_len) == COBS_OK);
    assert(cobs_decode(dst, out_len, dst, 3u, &out_len) == COBS_ERR_BUFFER_TOO_SMALL);
}

int main(void)
{
    test_crc32c_known_vectors();
    test_cobs_canonical_and_edges();
    test_cobs_error_handling();
    printf("test_crc32c_cobs: ALL PASSED\n");
    return 0;
}
