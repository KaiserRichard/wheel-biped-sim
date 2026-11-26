/*
 * FW10: CTRL rev4 pins used by the bring-up image that are not in target_hw.h.
 * Kept in a separate header so the original pin audit (which only knows the
 * target_hw.h macro families) still runs unchanged; an extended audit checks
 * both headers and board/ctrl_rev4.ioc against tests/contract/ctrl_rev4.net.
 *
 * Net names [FROM FILE ctrl_rev4.net, U3 pins 2-3, 34-35, 40-44, 85-86].
 */

#ifndef WBR_TARGET_BOARD_REV4_H
#define WBR_TARGET_BOARD_REV4_H

#include "target_hw.h"

/* IMU sensor on SPI4 (AF5), PE12 SPI4_SCK_MCU, PE13 SPI4_MISO, PE14 SPI4_MOSI_MCU. */
#define TARGET_IMU_SPI_INSTANCE         SPI4
#define TARGET_IMU_SPI_GPIO_PORT        GPIOE
#define TARGET_IMU_SPI_SCK_PIN          GPIO_PIN_12
#define TARGET_IMU_SPI_MISO_PIN         GPIO_PIN_13
#define TARGET_IMU_SPI_MOSI_PIN         GPIO_PIN_14
#define TARGET_IMU_SPI_GPIO_AF          GPIO_AF5_SPI4
/* 130 MHz D2PCLK2 / 16 = 8.125 MHz SCK (sensor limit 10 MHz, ASSUMED). */
#define TARGET_IMU_SPI_PRESCALER        SPI_BAUDRATEPRESCALER_16
#define TARGET_IMU_SPI_TIMEOUT_MS       2u

/* Chip selects (active LOW, idle HIGH): PE3 ACC_CS_MCU, PE4 GYRO_CS_MCU. */
#define TARGET_IMU_CS_GPIO_PORT         GPIOE
#define TARGET_IMU_ACC_CS_PIN           GPIO_PIN_3
#define TARGET_IMU_GYRO_CS_PIN          GPIO_PIN_4

/* Data-ready lines PE10 ACC_INT, PE11 GYRO_INT. Read as plain inputs only:
 * EXTI10/11 share EXTI15_10_IRQn with the PB12 wired stop (priority 0), so the
 * firmware samples the sensor synchronously in the 500 Hz loop instead. */
#define TARGET_IMU_INT_GPIO_PORT        GPIOE
#define TARGET_IMU_ACC_INT_PIN          GPIO_PIN_10
#define TARGET_IMU_GYRO_INT_PIN         GPIO_PIN_11

/* IMU heater gate PB1 HEAT_PWM: driven LOW (heater off) in FW10. */
#define TARGET_IMU_HEAT_GPIO_PORT       GPIOB
#define TARGET_IMU_HEAT_PIN             GPIO_PIN_1

/* Indicators: PD4 RUN_LED, PD5 FAULT_LED, PB0 BUZZER (held LOW). */
#define TARGET_LED_GPIO_PORT            GPIOD
#define TARGET_LED_RUN_PIN              GPIO_PIN_4
#define TARGET_LED_FAULT_PIN            GPIO_PIN_5
#define TARGET_BUZZER_GPIO_PORT         GPIOB
#define TARGET_BUZZER_PIN               GPIO_PIN_0

#endif /* WBR_TARGET_BOARD_REV4_H */
