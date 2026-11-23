#ifndef WBR_CRC32C_H
#define WBR_CRC32C_H

#include <stddef.h>
#include <stdint.h>

/* Castagnoli CRC-32C:
 *   Normal polynomial:    0x1EDC6F41
 *   Reflected polynomial: 0x82F63B78
 *   Init:                 0xFFFFFFFF
 *   Final XOR:            0xFFFFFFFF
 *   Check("123456789"):   0xE3069283
 */

uint32_t crc32c_init(void);
uint32_t crc32c_update(uint32_t crc, const uint8_t *data, size_t len);
uint32_t crc32c_final(uint32_t crc);
uint32_t crc32c_compute(const uint8_t *data, size_t len);

#endif /* WBR_CRC32C_H */
