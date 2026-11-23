#include "cobs.h"

CobsStatus cobs_encode(const uint8_t *src, size_t src_len,
                       uint8_t *dst, size_t dst_cap, size_t *out_len)
{
    if (out_len == NULL || dst == NULL || (src == NULL && src_len > 0u)) {
        return COBS_ERR_NULL_PTR;
    }
    *out_len = 0u;
    if (dst_cap == 0u) {
        return COBS_ERR_BUFFER_TOO_SMALL;
    }

    size_t read_idx = 0u;
    size_t write_idx = 1u;
    size_t code_idx = 0u;
    uint8_t code = 1u;

    while (read_idx < src_len) {
        if (src[read_idx] == 0x00u) {
            dst[code_idx] = code;
            code = 1u;
            if (write_idx >= dst_cap) {
                return COBS_ERR_BUFFER_TOO_SMALL;
            }
            code_idx = write_idx;
            write_idx += 1u;
            read_idx += 1u;
        } else {
            if (write_idx >= dst_cap) {
                return COBS_ERR_BUFFER_TOO_SMALL;
            }
            dst[write_idx] = src[read_idx];
            write_idx += 1u;
            read_idx += 1u;
            code = (uint8_t)(code + 1u);
            if (code == 0xFFu) {
                dst[code_idx] = code;
                code = 1u;
                if (read_idx < src_len) {
                    if (write_idx >= dst_cap) {
                        return COBS_ERR_BUFFER_TOO_SMALL;
                    }
                    code_idx = write_idx;
                    write_idx += 1u;
                } else {
                    code_idx = write_idx;
                }
            }
        }
    }

    if (code_idx < write_idx) {
        dst[code_idx] = code;
    }

    *out_len = write_idx;
    return COBS_OK;
}

CobsStatus cobs_decode(const uint8_t *src, size_t src_len,
                       uint8_t *dst, size_t dst_cap, size_t *out_len)
{
    if (out_len == NULL || src == NULL || (dst == NULL && dst_cap > 0u)) {
        return COBS_ERR_NULL_PTR;
    }
    *out_len = 0u;
    if (src_len == 0u) {
        return COBS_ERR_EMPTY_INPUT;
    }

    size_t read_idx = 0u;
    size_t write_idx = 0u;

    while (read_idx < src_len) {
        const uint8_t code = src[read_idx];
        if (code == 0x00u) {
            return COBS_ERR_ZERO_IN_INPUT;
        }
        read_idx += 1u;

        const size_t data_count = (size_t)(code - 1u);
        if (data_count > (src_len - read_idx)) {
            return COBS_ERR_TRUNCATED;
        }
        if (data_count > (dst_cap - write_idx)) {
            return COBS_ERR_BUFFER_TOO_SMALL;
        }
        for (size_t i = 0u; i < data_count; ++i) {
            const uint8_t b = src[read_idx + i];
            if (b == 0x00u) {
                return COBS_ERR_ZERO_IN_INPUT;
            }
            dst[write_idx + i] = b;
        }
        read_idx += data_count;
        write_idx += data_count;

        if (code < 0xFFu && read_idx < src_len) {
            if (write_idx >= dst_cap) {
                return COBS_ERR_BUFFER_TOO_SMALL;
            }
            dst[write_idx] = 0x00u;
            write_idx += 1u;
        }
    }

    *out_len = write_idx;
    return COBS_OK;
}
