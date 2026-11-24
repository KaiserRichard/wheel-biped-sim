/*
 * STM32H723 Independent Watchdog (IWDG1) & Boot Reset-Cause Driver
 *
 * Implements:
 *   - Boot reset-cause capture from RCC->RSR (`reset_cause` byte for TELEM_HEALTH,
 *     plus `bor3_fault` and `wdg_fault` flags for `BalanceTickInput` / `SafetyInput`)
 *   - IWDG1 initialization at 53 ms nominal (32 kHz LSI / 32 = 1 kHz counter,
 *     reload = 52; 50.476 ms at 33.6 kHz, FROM FILE clock_check.py LSI bounds)
 *   - Heartbeat-gated watchdog refresh: only kicks IWDG1 when `balance_heartbeat`
 *     from the 500 Hz control tick advances within TARGET_IWDG_MAX_STALL_US (30 ms)
 */

#ifndef WBR_TARGET_IWDG_H
#define WBR_TARGET_IWDG_H

#include "target_hw.h"
#include <stdint.h>

/* Bitmask definitions for TELEM_HEALTH `reset_cause` byte */
#define TARGET_RESET_CAUSE_POR      (1u << 0u)
#define TARGET_RESET_CAUSE_PIN      (1u << 1u)
#define TARGET_RESET_CAUSE_BOR      (1u << 2u)
#define TARGET_RESET_CAUSE_SFT      (1u << 3u)
#define TARGET_RESET_CAUSE_IWDG     (1u << 4u)
#define TARGET_RESET_CAUSE_WWDG     (1u << 5u)
#define TARGET_RESET_CAUSE_LPWR     (1u << 6u)

typedef struct {
    uint8_t  reset_cause_bits;
    uint8_t  bor3_fault;
    uint8_t  wdg_fault;
    uint32_t last_heartbeat;
    uint32_t last_advance_us;
    uint32_t refresh_count;
} TargetIwdgStatus;

/* Capture RCC->RSR reset flags at startup and clear RMVF. */
void target_iwdg_capture_reset_cause(void);

/* Configure IWDG1 (53 ms nominal, >=50 ms for the stated LSI bounds). */
HAL_StatusTypeDef target_iwdg_init(uint32_t now_us);

/* Service IWDG1 if and only if the 500 Hz balance_heartbeat is advancing. */
void target_iwdg_service(uint32_t now_us, uint32_t balance_heartbeat);

/* Only in DISARMED/FAULT after a local reset; never clears reset provenance. */
void target_iwdg_clear_fault(uint8_t operator_reset_edge);

/* Query current watchdog and boot reset-cause diagnostics. */
void target_iwdg_get_status(TargetIwdgStatus *out_status);

#endif /* WBR_TARGET_IWDG_H */
