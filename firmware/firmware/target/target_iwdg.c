/*
 * STM32H723 Independent Watchdog (IWDG1) & Boot Reset-Cause Driver
 */

#include "target_iwdg.h"
#include <string.h>

static IWDG_HandleTypeDef s_hiwdg;
static TargetIwdgStatus   s_iwdg_status;
static uint8_t            s_iwdg_started = 0u;

void target_iwdg_capture_reset_cause(void)
{
    uint8_t bits = 0u;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != 0u) {
        bits |= TARGET_RESET_CAUSE_POR;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != 0u) {
        bits |= TARGET_RESET_CAUSE_PIN;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) != 0u) {
        bits |= TARGET_RESET_CAUSE_BOR;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != 0u) {
        bits |= TARGET_RESET_CAUSE_SFT;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST) != 0u) {
        bits |= TARGET_RESET_CAUSE_IWDG;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDG1RST) != 0u) {
        bits |= TARGET_RESET_CAUSE_WWDG;
    }
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWR1RST) != 0u) {
        bits |= TARGET_RESET_CAUSE_LPWR;
    }

    /* Clear reset flags in RCC->RSR */
    __HAL_RCC_CLEAR_RESET_FLAGS();

    s_iwdg_status.reset_cause_bits = bits;

    /* Flag abnormal watchdog reset so SafetyFsm requires an explicit operator reset */
    if ((bits & (TARGET_RESET_CAUSE_IWDG | TARGET_RESET_CAUSE_WWDG)) != 0u) {
        s_iwdg_status.wdg_fault = 1u;
    } else {
        s_iwdg_status.wdg_fault = 0u;
    }

    /* Brownout without full Power-On Reset latches bor3_fault */
    if (((bits & TARGET_RESET_CAUSE_BOR) != 0u) &&
        ((bits & TARGET_RESET_CAUSE_POR) == 0u)) {
        s_iwdg_status.bor3_fault = 1u;
    } else {
        s_iwdg_status.bor3_fault = 0u;
    }
}

HAL_StatusTypeDef target_iwdg_init(uint32_t now_us)
{
    s_iwdg_status.last_heartbeat  = 0u;
    s_iwdg_status.last_advance_us = now_us;
    s_iwdg_status.refresh_count   = 0u;

    /*
     * LSI = 32 kHz (typical on STM32H723).
     * Prescaler = 32 -> 1000 Hz (1.0 ms per count).
     * Reload = 52 -> 53 counts, 53 ms nominal; 50.476 ms at 33.6 kHz max LSI.
     * Hardware starvation, not the software branch below, covers a hung loop.
     */
    memset(&s_hiwdg, 0, sizeof(s_hiwdg));
    s_hiwdg.Instance       = TARGET_IWDG_INSTANCE;
    s_hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    s_hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;
    s_hiwdg.Init.Reload    = TARGET_IWDG_TIMEOUT_MS - 1u;

    if (HAL_IWDG_Init(&s_hiwdg) != HAL_OK) {
        return HAL_ERROR;
    }

    s_iwdg_started = 1u;
    return HAL_OK;
}

void target_iwdg_service(uint32_t now_us, uint32_t balance_heartbeat)
{
    if (balance_heartbeat != s_iwdg_status.last_heartbeat) {
        s_iwdg_status.last_heartbeat  = balance_heartbeat;
        s_iwdg_status.last_advance_us = now_us;
    }

    uint32_t stall_us = now_us - s_iwdg_status.last_advance_us;
    if (stall_us <= TARGET_IWDG_MAX_STALL_US) {
        if (s_iwdg_started != 0u) {
            (void)HAL_IWDG_Refresh(&s_hiwdg);
        }
        s_iwdg_status.refresh_count++;
    } else {
        /* 500 Hz control loop has stalled > 30 ms: latch wdg_fault and withhold refresh */
        s_iwdg_status.wdg_fault = 1u;
    }
}

void target_iwdg_get_status(TargetIwdgStatus *out_status)
{
    if (out_status != NULL) {
        *out_status = s_iwdg_status;
    }
}

void target_iwdg_clear_fault(uint8_t operator_reset_edge)
{
    if (operator_reset_edge != 0u) {
        s_iwdg_status.wdg_fault = 0u;
        /* Preserve reset_cause_bits for telemetry throughout this boot. */
    }
}
