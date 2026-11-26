/*
 * FW10 IMU sensor codec (portable, host-tested).
 *
 * The CTRL rev4 IMU sensor (U8) is a two-die part: an accelerometer die and a
 * gyroscope die on one SPI bus (SPI4: PE12 SCK, PE13 MISO, PE14 MOSI) with
 * separate chip selects (PE3 ACC_CS, PE4 GYRO_CS) [FROM FILE ctrl_rev4.net].
 * This module holds the register map, the boot configuration sequence, the SPI
 * frame layout (the accel die returns one dummy byte after the address) and the
 * raw -> SI conversion and sensor -> body axis map. The register values are
 * [ASSUMED] from the sensor family datasheet and must be confirmed in bench
 * stage 2.1 (chip-ID read-back is checked at boot and reported in telemetry).
 *
 * Pure C11, zero HAL dependencies. Simulation/design only.
 */

#ifndef WBR_IMU_SENSOR_CODEC_H
#define WBR_IMU_SENSOR_CODEC_H

#include <stdint.h>

#define IMU_DIE_ACCEL               0u
#define IMU_DIE_GYRO                1u

#define IMU_SPI_READ_FLAG           0x80u

/* Accelerometer die */
#define IMU_ACC_REG_CHIP_ID         0x00u
#define IMU_ACC_CHIP_ID_VALUE       0x1Eu
#define IMU_ACC_REG_ERR             0x02u
#define IMU_ACC_REG_DATA            0x12u  /* X LSB .. Z MSB, 6 bytes */
#define IMU_ACC_REG_TEMP_MSB        0x22u  /* MSB, LSB */
#define IMU_ACC_REG_CONF            0x40u
#define IMU_ACC_REG_RANGE           0x41u
#define IMU_ACC_REG_PWR_CONF        0x7Cu
#define IMU_ACC_REG_PWR_CTRL        0x7Du
#define IMU_ACC_REG_SOFTRESET       0x7Eu
#define IMU_ACC_CONF_VALUE          0xABu  /* normal filter, ODR 800 Hz */
#define IMU_ACC_RANGE_VALUE         0x01u  /* +/-6 g */
#define IMU_ACC_RANGE_G             6.0

/* Gyroscope die */
#define IMU_GYR_REG_CHIP_ID         0x00u
#define IMU_GYR_CHIP_ID_VALUE       0x0Fu
#define IMU_GYR_REG_DATA            0x02u  /* X LSB .. Z MSB, 6 bytes */
#define IMU_GYR_REG_RANGE           0x0Fu
#define IMU_GYR_REG_BANDWIDTH       0x10u
#define IMU_GYR_REG_LPM1            0x11u
#define IMU_GYR_REG_SOFTRESET       0x14u
#define IMU_GYR_RANGE_VALUE         0x00u  /* +/-2000 deg/s */
#define IMU_GYR_BANDWIDTH_VALUE     0x02u  /* ODR 1000 Hz, 116 Hz filter */
#define IMU_GYR_RANGE_DPS           2000.0

#define IMU_SOFTRESET_CMD           0xB6u

#define IMU_STANDARD_GRAVITY        9.80665
#define IMU_INIT_MAX_STEPS          16u

typedef enum {
    IMU_STEP_DUMMY_READ = 0,   /* read CHIP_ID, ignore value (accel SPI-mode switch) */
    IMU_STEP_WRITE      = 1,
    IMU_STEP_CHECK      = 2    /* read reg, (value & mask) must equal expect */
} ImuStepKind;

typedef struct {
    uint8_t  kind;      /* ImuStepKind */
    uint8_t  die;       /* IMU_DIE_ACCEL / IMU_DIE_GYRO */
    uint8_t  reg;
    uint8_t  value;     /* WRITE value or CHECK expected value */
    uint8_t  mask;      /* CHECK mask */
    uint16_t delay_ms;  /* wait after the step */
} ImuInitStep;

/* Sensor-frame -> body-frame axis map: body[i] = sign[i] * sensor[src[i]].
 * Default identity is ASSUMED; bench stage 2.2 (hand tilt) fixes it. */
typedef struct {
    uint8_t src[3];
    int8_t  sign[3];
} ImuAxisMap;

typedef struct {
    double  accel_m_s2[3];
    double  gyro_rad_s[3];
    float   temp_c;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
} ImuSample;

/* Boot configuration sequence; returns number of steps written to out[]. */
uint32_t imu_codec_init_sequence(ImuInitStep out[IMU_INIT_MAX_STEPS]);

/* Bytes of dummy data between the address byte and the first data byte. */
uint8_t imu_codec_read_dummy_bytes(uint8_t die);

/* Build an SPI read frame: tx[0] = reg | 0x80, rest 0. Returns total length
 * (1 + dummy + n) or 0 if it does not fit in cap. */
uint32_t imu_codec_build_read(uint8_t die, uint8_t reg, uint32_t n, uint8_t *tx, uint32_t cap);

/* Conversions */
double imu_codec_accel_lsb_to_m_s2(int16_t raw);
double imu_codec_gyro_lsb_to_rad_s(int16_t raw);
float  imu_codec_temp_c(uint8_t msb, uint8_t lsb);
int16_t imu_codec_le16(const uint8_t *p);

/* Decode the 6-byte data payloads (after the dummy bytes) into SI + body frame. */
void imu_codec_decode(const ImuAxisMap *map, const uint8_t acc6[6], const uint8_t gyr6[6],
                      uint8_t temp_msb, uint8_t temp_lsb, ImuSample *out);

ImuAxisMap imu_codec_default_axis_map(void);
uint8_t    imu_codec_axis_map_valid(const ImuAxisMap *map);

#endif /* WBR_IMU_SENSOR_CODEC_H */
