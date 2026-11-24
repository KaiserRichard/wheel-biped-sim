#ifndef WBR_COBS_H
#define WBR_COBS_H

#include <stddef.h>
#include <stdint.h>

typedef enum {
    COBS_OK = 0,
    COBS_ERR_NULL_PTR = 1,
    COBS_ERR_BUFFER_TOO_SMALL = 2,
    COBS_ERR_EMPTY_INPUT = 3,
    COBS_ERR_ZERO_IN_INPUT = 4,
    COBS_ERR_TRUNCATED = 5
} CobsStatus;

/* Tight upper bound on COBS-encoded length (excluding trailing 0x00 delimiter). */
static inline size_t cobs_max_encoded_len(size_t raw_len)
{
    return 1u + raw_len + (raw_len / 254u);
}

/* Encode raw bytes into COBS form (contains no 0x00 bytes).
 * Caller appends the 0x00 packet delimiter separately if framing on a stream. */
CobsStatus cobs_encode(const uint8_t *src, size_t src_len,
                       uint8_t *dst, size_t dst_cap, size_t *out_len);

/* Decode COBS-encoded bytes (must not contain 0x00 delimiter) into raw bytes. */
CobsStatus cobs_decode(const uint8_t *src, size_t src_len,
                       uint8_t *dst, size_t dst_cap, size_t *out_len);

#endif /* WBR_COBS_H */
