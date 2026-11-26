/*
 * FW10 SPI4 IMU sensor driver + board outputs. See target_imu.h.
 */

#include "target_imu.h"

#include <string.h>

static SPI_HandleTypeDef s_hspi_imu;
static TargetImuStatus   s_status;
static ImuAxisMap        s_axis_map;
static uint32_t          s_read_count;
static uint8_t           s_temp_msb;
static uint8_t           s_temp_lsb;

void target_board_outputs_init(void)
{
    GPIO_InitTypeDef gpio;
    memset(&gpio, 0, sizeof(gpio));
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* Heater gate and buzzer: LOW before the pin becomes an output. */
    HAL_GPIO_WritePin(TARGET_IMU_HEAT_GPIO_PORT, TARGET_IMU_HEAT_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TARGET_BUZZER_GPIO_PORT, TARGET_BUZZER_PIN, GPIO_PIN_RESET);
    gpio.Pin = TARGET_IMU_HEAT_PIN | TARGET_BUZZER_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(TARGET_LED_GPIO_PORT, TARGET_LED_RUN_PIN | TARGET_LED_FAULT_PIN,
                      GPIO_PIN_RESET);
    gpio.Pin = TARGET_LED_RUN_PIN | TARGET_LED_FAULT_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TARGET_LED_GPIO_PORT, &gpio);

    /* Chip selects idle HIGH. */
    HAL_GPIO_WritePin(TARGET_IMU_CS_GPIO_PORT, TARGET_IMU_ACC_CS_PIN | TARGET_IMU_GYRO_CS_PIN,
                      GPIO_PIN_SET);
    gpio.Pin = TARGET_IMU_ACC_CS_PIN | TARGET_IMU_GYRO_CS_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TARGET_IMU_CS_GPIO_PORT, &gpio);

    /* Data-ready lines: inputs only, no EXTI. */
    gpio.Pin = TARGET_IMU_ACC_INT_PIN | TARGET_IMU_GYRO_INT_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TARGET_IMU_INT_GPIO_PORT, &gpio);
}

void target_led_set(uint8_t run_on, uint8_t fault_on)
{
    HAL_GPIO_WritePin(TARGET_LED_GPIO_PORT, TARGET_LED_RUN_PIN,
                      run_on != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TARGET_LED_GPIO_PORT, TARGET_LED_FAULT_PIN,
                      fault_on != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != TARGET_IMU_SPI_INSTANCE) {
        return;
    }
    RCC_PeriphCLKInitTypeDef periph_clk;
    memset(&periph_clk, 0, sizeof(periph_clk));
    periph_clk.PeriphClockSelection = RCC_PERIPHCLK_SPI45;
    periph_clk.Spi45ClockSelection = RCC_SPI45CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&periph_clk) != HAL_OK) {
        return; /* init then fails on the chip-ID check; reported, not fatal */
    }
    __HAL_RCC_SPI4_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef gpio;
    memset(&gpio, 0, sizeof(gpio));
    gpio.Pin = TARGET_IMU_SPI_SCK_PIN | TARGET_IMU_SPI_MISO_PIN | TARGET_IMU_SPI_MOSI_PIN;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = TARGET_IMU_SPI_GPIO_AF;
    HAL_GPIO_Init(TARGET_IMU_SPI_GPIO_PORT, &gpio);
}

static uint16_t cs_pin(uint8_t die)
{
    return (die == IMU_DIE_ACCEL) ? (uint16_t)TARGET_IMU_ACC_CS_PIN
                                  : (uint16_t)TARGET_IMU_GYRO_CS_PIN;
}

static HAL_StatusTypeDef spi_xfer(uint8_t die, const uint8_t *tx, uint8_t *rx, uint16_t n)
{
    HAL_GPIO_WritePin(TARGET_IMU_CS_GPIO_PORT, cs_pin(die), GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&s_hspi_imu, (uint8_t *)tx, rx, n,
                                                   TARGET_IMU_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(TARGET_IMU_CS_GPIO_PORT, cs_pin(die), GPIO_PIN_SET);
    return st;
}

static HAL_StatusTypeDef reg_read(uint8_t die, uint8_t reg, uint8_t *data, uint32_t n)
{
    uint8_t tx[16];
    uint8_t rx[16];
    const uint32_t len = imu_codec_build_read(die, reg, n, tx, sizeof(tx));
    if (len == 0u) {
        return HAL_ERROR;
    }
    memset(rx, 0, sizeof(rx));
    if (spi_xfer(die, tx, rx, (uint16_t)len) != HAL_OK) {
        return HAL_ERROR;
    }
    memcpy(data, &rx[1u + imu_codec_read_dummy_bytes(die)], n);
    return HAL_OK;
}

static HAL_StatusTypeDef reg_write(uint8_t die, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), value };
    uint8_t rx[2] = { 0u, 0u };
    return spi_xfer(die, tx, rx, 2u);
}

HAL_StatusTypeDef target_imu_init(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.fail_step = 0xFFu;
    s_axis_map = imu_codec_default_axis_map(); /* ASSUMED until bench stage 2.2 */
    s_read_count = 0u;

    memset(&s_hspi_imu, 0, sizeof(s_hspi_imu));
    s_hspi_imu.Instance = TARGET_IMU_SPI_INSTANCE;
    s_hspi_imu.Init.Mode = SPI_MODE_MASTER;
    s_hspi_imu.Init.Direction = SPI_DIRECTION_2LINES;
    s_hspi_imu.Init.DataSize = SPI_DATASIZE_8BIT;
    s_hspi_imu.Init.CLKPolarity = SPI_POLARITY_HIGH;   /* SPI mode 3 */
    s_hspi_imu.Init.CLKPhase = SPI_PHASE_2EDGE;
    s_hspi_imu.Init.NSS = SPI_NSS_SOFT;
    s_hspi_imu.Init.BaudRatePrescaler = TARGET_IMU_SPI_PRESCALER;
    s_hspi_imu.Init.FirstBit = SPI_FIRSTBIT_MSB;
    s_hspi_imu.Init.TIMode = SPI_TIMODE_DISABLE;
    s_hspi_imu.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    s_hspi_imu.Init.CRCPolynomial = 7u;
    s_hspi_imu.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    s_hspi_imu.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    s_hspi_imu.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    s_hspi_imu.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    s_hspi_imu.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    s_hspi_imu.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    s_hspi_imu.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE; /* SCK idles HIGH */
    s_hspi_imu.Init.IOSwap = SPI_IO_SWAP_DISABLE;
    if (HAL_SPI_Init(&s_hspi_imu) != HAL_OK) {
        s_status.fail_step = 0xFEu;
        return HAL_ERROR;
    }

    ImuInitStep steps[IMU_INIT_MAX_STEPS];
    const uint32_t n = imu_codec_init_sequence(steps);
    for (uint32_t i = 0u; i < n; ++i) {
        const ImuInitStep *s = &steps[i];
        uint8_t value = 0u;
        HAL_StatusTypeDef st = HAL_OK;
        if (s->kind == (uint8_t)IMU_STEP_WRITE) {
            st = reg_write(s->die, s->reg, s->value);
        } else {
            st = reg_read(s->die, s->reg, &value, 1u);
            if (s->kind == (uint8_t)IMU_STEP_CHECK) {
                if (s->reg == IMU_ACC_REG_CHIP_ID && s->die == IMU_DIE_ACCEL) {
                    s_status.acc_chip_id = value;
                } else if (s->reg == IMU_GYR_REG_CHIP_ID && s->die == IMU_DIE_GYRO) {
                    s_status.gyr_chip_id = value;
                }
                if (st == HAL_OK && (uint8_t)(value & s->mask) != s->value) {
                    st = HAL_ERROR;
                }
            }
        }
        if (st != HAL_OK) {
            s_status.fail_step = (uint8_t)i;
            return HAL_ERROR;
        }
        if (s->delay_ms != 0u) {
            HAL_Delay(s->delay_ms);
        }
    }
    s_status.init_ok = 1u;
    return HAL_OK;
}

uint8_t target_imu_read(int32_t tick_ms, BalanceImuSample *out, ImuSample *raw)
{
    if (s_status.init_ok == 0u || out == NULL || raw == NULL) {
        return 0u;
    }
    uint8_t acc[6];
    uint8_t gyr[6];
    if (reg_read(IMU_DIE_ACCEL, IMU_ACC_REG_DATA, acc, 6u) != HAL_OK
        || reg_read(IMU_DIE_GYRO, IMU_GYR_REG_DATA, gyr, 6u) != HAL_OK) {
        s_status.read_errors++;
        return 0u;
    }
    if ((s_read_count % 50u) == 0u) {
        uint8_t t[2];
        if (reg_read(IMU_DIE_ACCEL, IMU_ACC_REG_TEMP_MSB, t, 2u) == HAL_OK) {
            s_temp_msb = t[0];
            s_temp_lsb = t[1];
        }
    }
    s_read_count++;
    imu_codec_decode(&s_axis_map, acc, gyr, s_temp_msb, s_temp_lsb, raw);
    s_status.temp_c = raw->temp_c;
    s_status.reads_ok++;
    out->tick_ms = tick_ms;
    for (uint32_t i = 0u; i < 3u; ++i) {
        out->gyro_rad_s[i] = raw->gyro_rad_s[i];
        out->accel_m_s2[i] = raw->accel_m_s2[i];
    }
    return 1u;
}

void target_imu_get_status(TargetImuStatus *out)
{
    if (out != NULL) {
        *out = s_status;
    }
}
