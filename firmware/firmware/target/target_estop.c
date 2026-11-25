/*
 * STM32H723 Wired E-Stop EXTI Interrupt & Hardware Permit Output Driver
 */

#include "target_estop.h"
#include "target_fdcan.h"
#include "target_sync.h"
#include <string.h>

static volatile uint8_t  s_estop_latched   = 0u;
static volatile uint8_t  s_permit_asserted = 0u;
static volatile uint32_t s_estop_trip_cnt  = 0u;
static volatile uint32_t s_last_trip_us    = 0u;

static uint8_t target_estop_read_pin_open(void)
{
    /* Rev4 direct NC and opto status both use LOW=healthy, HIGH=open/tripped. */
    GPIO_PinState direct = HAL_GPIO_ReadPin(TARGET_ESTOP_GPIO_PORT, TARGET_ESTOP_PIN);
    GPIO_PinState opto = HAL_GPIO_ReadPin(TARGET_ESTOP_OPTO_GPIO_PORT,
                                          TARGET_ESTOP_OPTO_PIN);
    return (direct == TARGET_ESTOP_OPEN_PIN_STATE
            || opto == TARGET_ESTOP_OPTO_OPEN_PIN_STATE) ? 1u : 0u;
}

static void target_estop_latch_and_torque_off_isr(void)
{
    uint32_t now_us = target_micros();

    /* 1. Immediately stop heartbeat on the external watchdog permit input. */
    HAL_GPIO_WritePin(TARGET_PERMIT_GPIO_PORT, TARGET_PERMIT_PIN, GPIO_PIN_RESET);
    s_permit_asserted = 0u;

    /* 2. Latch e-stop and stamp DWT microsecond clock */
    if (s_estop_latched == 0u) {
        s_estop_latched = 1u;
        s_estop_trip_cnt++;
        s_last_trip_us = now_us;
    }

    /* 3. Abort all pending CAN TX buffers and send 0xFF..FD torque-off on all 6 axes */
    target_fdcan_emergency_disable_isr(now_us);
}

void target_estop_init(void)
{
    GPIO_InitTypeDef gpio;
    memset(&gpio, 0, sizeof(gpio));

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Configure PD11 heartbeat output, default LOW (safe state). */
    HAL_GPIO_WritePin(TARGET_PERMIT_GPIO_PORT, TARGET_PERMIT_PIN, GPIO_PIN_RESET);
    gpio.Pin   = TARGET_PERMIT_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TARGET_PERMIT_GPIO_PORT, &gpio);

    /* External pull-ups make a disconnected/open stop fail HIGH. */
    gpio.Pin   = TARGET_ESTOP_PIN;
    gpio.Mode  = GPIO_MODE_IT_RISING;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TARGET_ESTOP_GPIO_PORT, &gpio);

    gpio.Pin   = TARGET_ESTOP_OPTO_PIN;
    gpio.Mode  = GPIO_MODE_IT_RISING;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TARGET_ESTOP_OPTO_GPIO_PORT, &gpio);

    s_permit_asserted = 0u;
    s_estop_trip_cnt  = 0u;
    s_last_trip_us    = 0u;

    /* Check initial physical state at boot: latch immediately if chain is open */
    if (target_estop_read_pin_open() != 0u) {
        s_estop_latched = 1u;
        s_estop_trip_cnt = 1u;
    } else {
        s_estop_latched = 0u;
    }

    HAL_NVIC_SetPriority(TARGET_ESTOP_IRQn, TARGET_NVIC_PRIO_ESTOP_EXTI, 0u);
    HAL_NVIC_EnableIRQ(TARGET_ESTOP_IRQn);
    HAL_NVIC_SetPriority(TARGET_ESTOP_OPTO_IRQn, TARGET_NVIC_PRIO_ESTOP_EXTI, 0u);
    HAL_NVIC_EnableIRQ(TARGET_ESTOP_OPTO_IRQn);
}

void target_estop_exti_isr(void)
{
    /* Rising edge is evidence even when the pin bounced closed before entry. */
    target_estop_latch_and_torque_off_isr();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == TARGET_ESTOP_PIN || GPIO_Pin == TARGET_ESTOP_OPTO_PIN) {
        target_estop_exti_isr();
    }
}

void target_estop_force_trip(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    target_estop_latch_and_torque_off_isr();
    if (primask == 0u) {
        __enable_irq();
    }
}

void target_estop_fatal_halt(void)
{
    /* Latch emergency stop & transmit 3-bus torque-off before halting */
    target_estop_force_trip();
    __disable_irq();
    for (;;) {
        /* FW8 F-05: with IRQs masked, SysTick/FDCAN callbacks can no longer emit
         * the +1 ms 0xFF..FD repeat or frames that did not fit the TX FIFO, so
         * poll the service here (DWT time runs under PRIMASK). It only ever
         * sends disable frames; the IWDG still resets the core. */
        target_fdcan_emergency_service();
#ifdef TARGET_HOST_TEST
        if (stub_halt_poll() != 0u) {
            return;
        }
#endif
    }
}

void target_estop_set_permit(uint8_t enable)
{
    uint32_t saved = target_irq_save();
    if (enable != 0u && target_estop_is_tripped() == 0u) {
        /* Called once per 500 Hz control iteration. Toggle edges keep the
         * external watchdog alive; a static high level is not a valid permit. */
        GPIO_PinState state = HAL_GPIO_ReadPin(TARGET_PERMIT_GPIO_PORT,
                                               TARGET_PERMIT_PIN);
        HAL_GPIO_WritePin(TARGET_PERMIT_GPIO_PORT, TARGET_PERMIT_PIN,
                          state == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET);
        s_permit_asserted = 1u;
    } else {
        HAL_GPIO_WritePin(TARGET_PERMIT_GPIO_PORT, TARGET_PERMIT_PIN, GPIO_PIN_RESET);
        s_permit_asserted = 0u;
    }
    target_irq_restore(saved);
}

uint8_t target_estop_is_tripped(void)
{
    return (s_estop_latched != 0u || target_estop_read_pin_open() != 0u
            || __HAL_GPIO_EXTI_GET_IT(TARGET_ESTOP_PIN) != 0u
            || __HAL_GPIO_EXTI_GET_IT(TARGET_ESTOP_OPTO_PIN) != 0u) ? 1u : 0u;
}

uint8_t target_estop_try_clear_latch(uint8_t operator_reset_edge)
{
    uint32_t saved = target_irq_save();
    uint8_t cleared = 0u;
    if (operator_reset_edge != 0u && target_estop_read_pin_open() == 0u
        && __HAL_GPIO_EXTI_GET_IT(TARGET_ESTOP_PIN) == 0u
        && __HAL_GPIO_EXTI_GET_IT(TARGET_ESTOP_OPTO_PIN) == 0u) {
        s_estop_latched = 0u;
        target_fdcan_clear_emergency();
        cleared = 1u;
    }
    target_irq_restore(saved);
    return cleared;
}

void target_estop_get_state(TargetEstopState *out_state)
{
    uint32_t saved = target_irq_save();
    uint8_t pin_open = target_estop_read_pin_open();
    if (pin_open != 0u && s_estop_latched == 0u) {
        target_estop_force_trip();
    }

    if (out_state != NULL) {
        out_state->pin_open        = pin_open;
        out_state->latched         = s_estop_latched;
        out_state->permit_asserted = s_permit_asserted;
        out_state->trip_count      = s_estop_trip_cnt;
        out_state->last_trip_us    = s_last_trip_us;
    }
    target_irq_restore(saved);
}
