/*
 * STM32H723 500 Hz Deterministic Control Loop Timer (TIM6)
 *
 * Configures TIM6 (`TIM6_DAC_IRQn`, NVIC priority 4, ASSUMED pending HW1) to
 * generate a deterministic 500.0 Hz (2000 us) update interrupt that wakes and
 * triggers the 500 Hz `balance_ctrl_tick()` execution in `main.c`.
 *
 * Adapted from DM-jump (Core/Src/tim.c, MIT License).
 */

#ifndef WBR_TARGET_TIMER_H
#define WBR_TARGET_TIMER_H

#include "target_hw.h"
#include <stdint.h>

extern TIM_HandleTypeDef g_htim_ctrl;

/* Initialize and start TIM6 for 500.0 Hz (2000 us period) update interrupts. */
HAL_StatusTypeDef target_timer_500hz_init(void);

/* Atomically consume a pending 500 Hz tick notification.
 * Returns 1 and writes the tick's ISR microsecond timestamp and sequence number
 * if a 500 Hz tick is ready; returns 0 if no tick has elapsed yet. */
uint8_t target_timer_500hz_consume_tick(uint32_t *out_tick_us,
                                        uint32_t *out_tick_seq,
                                        uint32_t *out_overrun_count);

#endif /* WBR_TARGET_TIMER_H */
