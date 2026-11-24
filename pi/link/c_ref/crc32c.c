#include "crc32c.h"

/* 4-bit reflected lookup table for Castagnoli polynomial 0x82F63B78. */
static const uint32_t CRC32C_NIBBLE_LUT[16] = {
    0x00000000u, 0x105EC76Fu, 0x20BD8EDEu, 0x30E349B1u,
    0x417B1DBCu, 0x5125DAD3u, 0x61C69362u, 0x7198540Du,
    0x82F63B78u, 0x92A8FC17u, 0xA24BB5A6u, 0xB21572C9u,
    0xC38D26C4u, 0xD3D3E1ABu, 0xE330A81Au, 0xF36E6F75u
};

uint32_t crc32c_init(void)
{
    return 0xFFFFFFFFu;
}

uint32_t crc32c_update(uint32_t crc, const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0u) {
        return crc;
    }
    uint32_t acc = crc;
    for (size_t i = 0u; i < len; ++i) {
        acc ^= (uint32_t)data[i];
        acc = (acc >> 4u) ^ CRC32C_NIBBLE_LUT[acc & 0x0Fu];
        acc = (acc >> 4u) ^ CRC32C_NIBBLE_LUT[acc & 0x0Fu];
    }
    return acc;
}

uint32_t crc32c_final(uint32_t crc)
{
    return crc ^ 0xFFFFFFFFu;
}

uint32_t crc32c_compute(const uint8_t *data, size_t len)
{
    return crc32c_final(crc32c_update(crc32c_init(), data, len));
}
