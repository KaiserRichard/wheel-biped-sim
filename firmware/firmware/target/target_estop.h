/*
 * STM32H723 Wired E-Stop EXTI Interrupt & Hardware Permit Output Driver
 *
 * CTRL rev4 stop inputs are PB12 direct NC and PD7 opto status. Both use
 * LOW=healthy and HIGH=open/tripped; either rising edge latches the stop.
 * PD11 drives the external watchdog heartbeat gate.
 * Latching torque-off: when either input opens, its EXTI ISR
 *     immediately:
 *       1. Latches `estop_latched = 1`
 *       2. Stops the PD11 heartbeat LOW
 *       3. Calls `target_fdcan_emergency_disable_isr()` to abort all pending CAN
 *          TX frames and broadcast Damiao `0xFF..FD` motor disable across
 *          FDCAN1 (right) and FDCAN2 (left); FDCAN3 is spare.
 *   - Latch remains set even if either input recloses, until explicitly cleared by the
 *     supervisor via `target_estop_try_clear_latch()` when disarmed and healthy.
 */

#ifndef WBR_TARGET_ESTOP_H
#define WBR_TARGET_ESTOP_H

#include "target_hw.h"
#include <stdint.h>

typedef struct {
    uint8_t  pin_open;          /* 1 = either PB12 or PD7 currently HIGH (open/tripped) */
    uint8_t  latched;           /* 1 = e-stop latch active (stays 1 until explicit clear) */
    uint8_t  permit_asserted;   /* 1 = PD11 watchdog heartbeat enabled */
    uint32_t trip_count;        /* Total EXTI trip events latched since boot */
    uint32_t last_trip_us;      /* DWT timestamp (us) of most recent trip */
} TargetEstopState;

/* Initialize PB12 and PD7 stop inputs plus PD11 heartbeat (default LOW). */
void target_estop_init(void);

/* Called from either stop input's rising-edge EXTI at NVIC priority 0. */
void target_estop_exti_isr(void);

/* Software-triggered emergency latch & immediate 3-bus torque-off. */
void target_estop_force_trip(void);

/* Enable/disable PD11 watchdog edges (only enabled while latch and inputs are healthy). */
void target_estop_set_permit(uint8_t enable);

/* Clear only on a deliberate reset edge, both PB12 and PD7 LOW (healthy),
 * and no pending rising edge on either input. */
uint8_t target_estop_try_clear_latch(uint8_t operator_reset_edge);

/* Call under target_irq_save() when used as a check-before-write gate. */
uint8_t target_estop_is_tripped(void);

/* Snapshot current e-stop pin, latch, and permit state for BalanceTickInput. */
void target_estop_get_state(TargetEstopState *out_state);

#endif /* WBR_TARGET_ESTOP_H */
