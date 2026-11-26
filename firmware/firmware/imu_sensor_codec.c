/*
 * FW10 IMU sensor codec. See imu_sensor_codec.h. Register values [ASSUMED].
 */

#include "imu_sensor_codec.h"

#include <stddef.h>
#include <string.h>

#define IMU_PI 3.14159265358979323846

static ImuInitStep step(uint8_t kind, uint8_t die, uint8_t reg, uint8_t value,
                        uint8_t mask, uint16_t delay_ms)
{
    ImuInitStep s;
    s.kind = kind;
    s.die = die;
    s.reg = reg;
    s.value = value;
    s.mask = mask;
    s.delay_ms = delay_ms;
    return s;
}

uint32_t imu_codec_init_sequence(ImuInitStep out[IMU_INIT_MAX_STEPS])
{
    if (out == NULL) {
        return 0u;
    }
    uint32_t n = 0u;
    /* Accelerometer: a CS rising edge after power-up selects SPI; soft reset
     * returns it to I2C mode, so the dummy read is repeated after the reset. */
    out[n++] = step(IMU_STEP_DUMMY_READ, IMU_DIE_ACCEL, IMU_ACC_REG_CHIP_ID, 0u, 0u, 1u);
    out[n++] = step(IMU_STEP_WRITE, IMU_DIE_ACCEL, IMU_ACC_REG_SOFTRESET, IMU_SOFTRESET_CMD, 0u, 2u);
    out[n++] = step(IMU_STEP_DUMMY_READ, IMU_DIE_ACCEL, IMU_ACC_REG_CHIP_ID, 0u, 0u, 1u);
    out[n++] = step(IMU_STEP_CHECK, IMU_DIE_ACCEL, IMU_ACC_REG_CHIP_ID, IMU_ACC_CHIP_ID_VALUE, 0xFFu, 0u);
    out[n++] = step(IMU_STEP_WRITE, IMU_DIE_ACCEL, IMU_ACC_REG_PWR_CONF, 0x00u, 0u, 2u); /* active */
    out[n++] = step(IMU_STEP_WRITE, IMU_DIE_ACCEL, IMU_ACC_REG_PWR_CTRL, 0x04u, 0u, 5u); /* accel on */
    out[n++] = step(IMU_STEP_WRITE, IMU_DIE_ACCEL, IMU_ACC_REG_CONF, IMU_ACC_CONF_VALUE, 0u, 1u);
    out[n++] = step(IMU_STEP_WRITE, IMU_DIE_ACCEL, IMU_ACC_REG_RANGE, IMU_ACC_RANGE_VALUE, 0u, 1u);
    out[n++] = step(IMU_STEP_CHECK, IMU_DIE_ACCEL, IMU_ACC_REG_CONF, IMU_ACC_CONF_VALUE, 0xFFu, 0u);
    out[n++] = step(IMU_STEP_CHECK, IMU_DIE_ACCEL, IMU_ACC_REG_RANGE, IMU_ACC_RANGE_VALUE, 0x03u, 0u);
    /* Gyroscope */
    out[n++] = step(IMU_STEP_WRITE, IMU_DIE_GYRO, IMU_GYR_REG_SOFTRESET, IMU_SOFTRESET_CMD, 0u, 30u);
    out[n++] = step(IMU_STEP_CHECK, IMU_DIE_GYRO, IMU_GYR_REG_CHIP_ID, IMU_GYR_CHIP_ID_VALUE, 0xFFu, 0u);
    out[n++] = step(IMU_STEP_WRITE, IMU_DIE_GYRO, IMU_GYR_REG_RANGE, IMU_GYR_RANGE_VALUE, 0u, 1u);
    out[n++] = step(IMU_STEP_WRITE, IMU_DIE_GYRO, IMU_GYR_REG_BANDWIDTH, IMU_GYR_BANDWIDTH_VALUE, 0u, 1u);
    out[n++] = step(IMU_STEP_WRITE, IMU_DIE_GYRO, IMU_GYR_REG_LPM1, 0x00u, 0u, 1u); /* normal */
    out[n++] = step(IMU_STEP_CHECK, IMU_DIE_GYRO, IMU_GYR_REG_BANDWIDTH, IMU_GYR_BANDWIDTH_VALUE, 0x0Fu, 0u);
    return n;
}

uint8_t imu_codec_read_dummy_bytes(uint8_t die)
{
    return (die == IMU_DIE_ACCEL) ? 1u : 0u;
}

uint32_t imu_codec_build_read(uint8_t die, uint8_t reg, uint32_t n, uint8_t *tx, uint32_t cap)
{
    const uint32_t total = 1u + (uint32_t)imu_codec_read_dummy_bytes(die) + n;
    if (tx == NULL || total > cap) {
        return 0u;
    }
    memset(tx, 0, total);
    tx[0] = (uint8_t)(reg | IMU_SPI_READ_FLAG);
    return total;
}

int16_t imu_codec_le16(const uint8_t *p)
{
    const uint16_t u = (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
    return (int16_t)u;
}

double imu_codec_accel_lsb_to_m_s2(int16_t raw)
{
    return (double)raw / 32768.0 * IMU_ACC_RANGE_G * IMU_STANDARD_GRAVITY;
}

double imu_codec_gyro_lsb_to_rad_s(int16_t raw)
{
    return (double)raw / 32768.0 * IMU_GYR_RANGE_DPS * (IMU_PI / 180.0);
}

float imu_codec_temp_c(uint8_t msb, uint8_t lsb)
{
    int32_t t = (int32_t)(((uint32_t)msb << 3) | ((uint32_t)lsb >> 5));
    if (t > 1023) {
        t -= 2048;
    }
    return (float)t * 0.125f + 23.0f;
}

ImuAxisMap imu_codec_default_axis_map(void)
{
    ImuAxisMap m;
    for (uint8_t i = 0u; i < 3u; ++i) {
        m.src[i] = i;
        m.sign[i] = 1;
    }
    return m;
}

uint8_t imu_codec_axis_map_valid(const ImuAxisMap *map)
{
    if (map == NULL) {
        return 0u;
    }
    uint8_t seen = 0u;
    for (uint8_t i = 0u; i < 3u; ++i) {
        if (map->src[i] > 2u || (map->sign[i] != 1 && map->sign[i] != -1)) {
            return 0u;
        }
        seen = (uint8_t)(seen | (1u << map->src[i]));
    }
    return (seen == 0x07u) ? 1u : 0u; /* a permutation */
}

void imu_codec_decode(const ImuAxisMap *map, const uint8_t acc6[6], const uint8_t gyr6[6],
                      uint8_t temp_msb, uint8_t temp_lsb, ImuSample *out)
{
    if (out == NULL || acc6 == NULL || gyr6 == NULL) {
        return;
    }
    const ImuAxisMap m = (map != NULL && imu_codec_axis_map_valid(map) != 0u)
                       ? *map : imu_codec_default_axis_map();
    double a_s[3];
    double g_s[3];
    for (uint8_t i = 0u; i < 3u; ++i) {
        out->accel_raw[i] = imu_codec_le16(&acc6[2u * i]);
        out->gyro_raw[i] = imu_codec_le16(&gyr6[2u * i]);
        a_s[i] = imu_codec_accel_lsb_to_m_s2(out->accel_raw[i]);
        g_s[i] = imu_codec_gyro_lsb_to_rad_s(out->gyro_raw[i]);
    }
    for (uint8_t i = 0u; i < 3u; ++i) {
        out->accel_m_s2[i] = (double)m.sign[i] * a_s[m.src[i]];
        out->gyro_rad_s[i] = (double)m.sign[i] * g_s[m.src[i]];
    }
    out->temp_c = imu_codec_temp_c(temp_msb, temp_lsb);
}
