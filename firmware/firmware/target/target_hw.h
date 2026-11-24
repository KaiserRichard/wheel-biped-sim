/*
 * STM32H723VGT6 Target Hardware Pinout, Clock & Peripheral Assumptions
 *
 * All pin, clock, and peripheral assignments in this file are ASSUMED
 * pending verification of the physical CTRL board pinout against HW1.
 *
 * Adapted from DM-jump (Core/Src/fdcan.c, usart.c, main.c, User/Bsp/bsp_dwt.c)
 * and FIRMWARE_DESIGN_v3.md / can_map/CAN_ID_MAP.md:
 *   Copyright (c) 2024-2026 Tran Nguyen Binh
 *   Project: DM-jump (DM-jump), MIT License
 *   Reference: dmBots/wheel-legged (https://github.com/dmBots/wheel-legged)
 *   See firmware/DM_JUMP_LICENSE.txt for the full MIT license notice.
 */

#ifndef WBR_TARGET_HW_H
#define WBR_TARGET_HW_H

#ifdef TARGET_HOST_TEST
#include "hal_stub.h"
#else
#include "stm32h7xx_hal.h"
#endif
#include <stdint.h>

/* ============================================================================
 * 1. Clock Tree & DWT Cycle Counter Assumptions (ASSUMED pending HW1)
 * ============================================================================
 * - HSE crystal: 24 MHz (ASSUMED from DM-jump stm32h7xx_hal_conf.h)
 * - PLL1: PLLM = 12, PLLN = 260, PLLP = 1 -> SYSCLK = 520 MHz (VOS0)
 * - HCLK = 260 MHz (AHB /2), APB1/2/3/4 = 130 MHz (/2), Timer clocks = 260 MHz
 * - PLL2: PLL2M = 3, PLL2N = 40, PLL2Q = 4 -> FDCAN kernel clock = 80 MHz
 * - LSI: 32 kHz internal oscillator for IWDG1
 */
#define TARGET_SYSCLK_MHZ               520u
#define TARGET_FDCAN_KER_CLK_HZ         80000000u

/* ============================================================================
 * 2. NVIC Interrupt Priority Plan (4-bit priority, 0 = highest, 15 = lowest)
 * ============================================================================ */
#define TARGET_NVIC_PRIO_ESTOP_EXTI     0u  /* Highest priority: wired e-stop torque-off */
#define TARGET_NVIC_PRIO_CTRL_TIM       4u  /* 500 Hz deterministic control tick timer */
#define TARGET_NVIC_PRIO_FDCAN          5u  /* FDCAN1/2/3 RX FIFO0 & error status ISRs */
#define TARGET_NVIC_PRIO_CRSF_UART      6u  /* ExpressLRS CRSF receiver UART ISR */
#define TARGET_NVIC_PRIO_USB_CDC        7u  /* USB OTG FS CDC ACM ISR */

/* ============================================================================
 * 3. FDCAN1 / FDCAN2 / FDCAN3 Pinout & 1 Mbit/s Bit Timing (ASSUMED pending HW1)
 * ============================================================================
 * Bit timing at 80 MHz PLL2Q kernel clock:
 *   Prescaler = 4 -> 20 MHz TQ clock (20 TQ per 1 us bit)
 *   Sync = 1 TQ, TimeSeg1 = 15 TQ, TimeSeg2 = 4 TQ, SJW = 1 TQ
 *   Bitrate = 80 MHz / (4 * (1 + 15 + 4)) = 1.000 Mbit/s, 80.0% sample point
 */
#define TARGET_FDCAN_PRESCALER          4u
#define TARGET_FDCAN_SJW                1u
#define TARGET_FDCAN_TSEG1              15u
#define TARGET_FDCAN_TSEG2              4u
#define TARGET_FDCAN_TX_QUEUE_ELMTS     8u
#define TARGET_FDCAN_RX_FIFO0_ELMTS     8u

/* J5 RIGHT: FDCAN1 (bus_idx = 0), axes 3/4/5, cmd 0x004..006, reply 0x014..016. */
#define TARGET_FDCAN1_GPIO_PORT         GPIOD
#define TARGET_FDCAN1_RX_PIN            GPIO_PIN_0
#define TARGET_FDCAN1_TX_PIN            GPIO_PIN_1
#define TARGET_FDCAN1_GPIO_AF           GPIO_AF9_FDCAN1
#define TARGET_FDCAN1_RAM_OFFSET        0u

/* J6 LEFT: FDCAN2 (bus_idx = 1), axes 0/1/2, cmd 0x001..003, reply 0x011..013. */
#define TARGET_FDCAN2_GPIO_PORT         GPIOB
#define TARGET_FDCAN2_RX_PIN            GPIO_PIN_5
#define TARGET_FDCAN2_TX_PIN            GPIO_PIN_6
#define TARGET_FDCAN2_GPIO_AF           GPIO_AF9_FDCAN2
#define TARGET_FDCAN2_RAM_OFFSET        256u

/* J7 SPARE: FDCAN3 (bus_idx = 2), no motor TX; all RX rejected. */
#define TARGET_FDCAN3_GPIO_PORT         GPIOD
#define TARGET_FDCAN3_RX_PIN            GPIO_PIN_12
#define TARGET_FDCAN3_TX_PIN            GPIO_PIN_13
#define TARGET_FDCAN3_GPIO_AF           GPIO_AF5_FDCAN3
#define TARGET_FDCAN3_RAM_OFFSET        512u

/* ============================================================================
 * 4. CRSF (ExpressLRS) Receiver UART Pinout (ASSUMED pending HW1)
 * ============================================================================
 * USART6 on PC6 (TX) / PC7 (RX), AF7, 420000 baud 8N1.
 * PA9 is VBUS_SENSE and must only be configured as an input.
 */
#define TARGET_CRSF_UART_INSTANCE       USART6
#define TARGET_CRSF_GPIO_PORT           GPIOC
#define TARGET_CRSF_TX_PIN              GPIO_PIN_6
#define TARGET_CRSF_RX_PIN              GPIO_PIN_7
#define TARGET_CRSF_GPIO_AF             GPIO_AF7_USART6
#define TARGET_CRSF_UART_IRQn           USART6_IRQn
#define TARGET_CRSF_UART_IRQHandler     USART6_IRQHandler
#define TARGET_CRSF_BAUDRATE            420000u
#define TARGET_CRSF_TIMEOUT_MS          150u

/* ============================================================================
 * 5. Wired E-Stop inputs & external watchdog heartbeat (CTRL rev4)
 * ============================================================================
 * - PB12 WIRED_STOP_OPEN and PD7 ESTOP_STATUS are both LOW when healthy and
 *   HIGH when open/tripped. Either input trips the latched stop.
 * - PD11 MCU_HEARTBEAT drives the external watchdog permit gate; it requires
 *   recurring edges while enabled, and low is the safe state.
 */
#define TARGET_ESTOP_GPIO_PORT          GPIOB
#define TARGET_ESTOP_PIN                GPIO_PIN_12
#define TARGET_ESTOP_OPEN_PIN_STATE     GPIO_PIN_SET
#define TARGET_ESTOP_IRQn               EXTI15_10_IRQn
#define TARGET_ESTOP_OPTO_GPIO_PORT     GPIOD
#define TARGET_ESTOP_OPTO_PIN           GPIO_PIN_7
#define TARGET_ESTOP_OPTO_OPEN_PIN_STATE GPIO_PIN_SET
#define TARGET_ESTOP_OPTO_IRQn          EXTI9_5_IRQn

#define TARGET_PERMIT_GPIO_PORT         GPIOD
#define TARGET_PERMIT_PIN               GPIO_PIN_11

/* ============================================================================
 * 6. USB OTG Full-Speed CDC ACM Pinout (ASSUMED pending HW1)
 * ============================================================================
 * STM32H723 USB1_OTG_HS peripheral operating with internal Full-Speed PHY
 * on PA11 (DM) / PA12 (DP), AF10, OTG_HS_IRQn
 */
#define TARGET_USB_INSTANCE             USB1_OTG_HS
#define TARGET_USB_GPIO_PORT            GPIOA
#define TARGET_USB_DM_PIN               GPIO_PIN_11
#define TARGET_USB_DP_PIN               GPIO_PIN_12
#define TARGET_USB_VBUS_SENSE_GPIO_PORT GPIOA
#define TARGET_USB_VBUS_SENSE_PIN       GPIO_PIN_9
#define TARGET_USB_GPIO_AF              GPIO_AF10_OTG1_FS
#define TARGET_USB_IRQn                 OTG_HS_IRQn

/* ============================================================================
 * 7. 500 Hz Control Timer & Independent Watchdog (ASSUMED pending HW1)
 * ============================================================================
 * - TIM6: 500 Hz (2000 us period) basic timer driving balance_ctrl_tick()
 * - IWDG1: 32 kHz LSI / 32 = 1 kHz counter, reload = 52 (53 ms nominal).
 *   50.476 ms minimum at 33.6 kHz LSI; hardware frequency remains unmeasured.
 */
#define TARGET_CTRL_TIM_INSTANCE        TIM6
#define TARGET_CTRL_TIM_IRQn            TIM6_DAC_IRQn
#define TARGET_CTRL_PERIOD_US           2000u

#define TARGET_IWDG_INSTANCE            IWDG1
#define TARGET_IWDG_TIMEOUT_MS          53u /* Nominal; >=50 ms at 33.6 kHz LSI. */
#define TARGET_IWDG_MAX_STALL_US        30000u

/* ============================================================================
 * DWT Microsecond Timebase Helpers (defined in main.c, adapted from DM-jump)
 * ============================================================================ */
void     target_dwt_init(uint32_t cpu_freq_mhz);
uint32_t target_micros(void);
uint64_t target_micros64(void);
void     Error_Handler(void);

#endif /* WBR_TARGET_HW_H */
