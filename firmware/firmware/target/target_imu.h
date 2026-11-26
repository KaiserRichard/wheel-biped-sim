/*
 * FW10 STM32H723 SPI4 driver for the CTRL rev4 IMU sensor (U8) and the
 * board-level outputs the bring-up image must hold in a known state
 * (heater gate, buzzer, RUN/FAULT LEDs).
 *
 * The sensor is sampled synchronously once per 500 Hz control tick (two short
 * blocking SPI bursts, ~15 bytes at 8.125 MHz). No EXTI is used (see
 * target_board_rev4.h). Simulation/design only: not run on hardware.
 */

#ifndef WBR_TARGET_IMU_H
#define WBR_TARGET_IMU_H

#include "target_board_rev4.h"
#include "imu_sensor_codec.h"
#include "control/balance_ctrl.h"

#include <stdint.h>

typedef struct {
    uint8_t  init_ok;        /* 1 = both chip IDs and config read-backs matched */
    uint8_t  acc_chip_id;    /* last read accel CHIP_ID (expected 0x1E) */
    uint8_t  gyr_chip_id;    /* last read gyro CHIP_ID (expected 0x0F) */
    uint8_t  fail_step;      /* index of the first failing init step, 0xFF = none */
    uint32_t reads_ok;
    uint32_t read_errors;
    float    temp_c;
} TargetImuStatus;

/* Heater/buzzer LOW, LEDs off, CS lines HIGH. Call before any other init. */
void target_board_outputs_init(void);
void target_led_set(uint8_t run_on, uint8_t fault_on);

/* SPI4 + sensor configuration. HAL_ERROR if the sensor did not answer as
 * expected; never fatal (bench stage 2.1 needs the telemetry to diagnose). */
HAL_StatusTypeDef target_imu_init(void);

/* Read one sample. Returns 1 and fills both outputs on success. */
uint8_t target_imu_read(int32_t tick_ms, BalanceImuSample *out, ImuSample *raw);

void target_imu_get_status(TargetImuStatus *out);

#endif /* WBR_TARGET_IMU_H */
