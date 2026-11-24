/*
 * STM32H723 500 Hz Deterministic Control Loop Timer (TIM6)
 *
 * Adapted from DM-jump (Core/Src/tim.c, MIT License).
 */

#include "target_timer.h"
#include <string.h>

TIM_HandleTypeDef g_htim_ctrl;

static volatile uint8_t  s_tick_pending      = 0u;
static volatile uint32_t s_tick_isr_us       = 0u;
static volatile uint32_t s_tick_seq          = 0u;
static volatile uint32_t s_tick_overrun_cnt  = 0u;

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TARGET_CTRL_TIM_INSTANCE) {
        __HAL_RCC_TIM6_CLK_ENABLE();
        HAL_NVIC_SetPriority(TARGET_CTRL_TIM_IRQn, TARGET_NVIC_PRIO_CTRL_TIM, 0u);
        HAL_NVIC_EnableIRQ(TARGET_CTRL_TIM_IRQn);
    }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TARGET_CTRL_TIM_INSTANCE) {
        __HAL_RCC_TIM6_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(TARGET_CTRL_TIM_IRQn);
    }
}

HAL_StatusTypeDef target_timer_500hz_init(void)
{
    TIM_MasterConfigTypeDef master_cfg;
    memset(&master_cfg, 0, sizeof(master_cfg));

    s_tick_pending     = 0u;
    s_tick_isr_us      = 0u;
    s_tick_seq         = 0u;
    s_tick_overrun_cnt = 0u;

    /* Compute APB1 timer input clock dynamically from HAL RCC */
    uint32_t pclk1_hz = HAL_RCC_GetPCLK1Freq();
    uint32_t tim_clk_hz = pclk1_hz;
    if ((RCC->D2CFGR & RCC_D2CFGR_D2PPRE1) != RCC_APB1_DIV1) {
        tim_clk_hz = pclk1_hz * 2u;
    }
    if (tim_clk_hz < 1000000u) {
        tim_clk_hz = 260000000u; /* Rev4 APB1 timer clock at 520 MHz SYSCLK. */
    }

    /* Prescale timer counter to 1.0 MHz (1 us resolution), period = 2000 us (500.0 Hz) */
    uint32_t prescaler = (tim_clk_hz / 1000000u) - 1u;

    memset(&g_htim_ctrl, 0, sizeof(g_htim_ctrl));
    g_htim_ctrl.Instance               = TARGET_CTRL_TIM_INSTANCE;
    g_htim_ctrl.Init.Prescaler         = prescaler;
    g_htim_ctrl.Init.CounterMode       = TIM_COUNTERMODE_UP;
    g_htim_ctrl.Init.Period            = TARGET_CTRL_PERIOD_US - 1u;
    g_htim_ctrl.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&g_htim_ctrl) != HAL_OK) {
        return HAL_ERROR;
    }

    master_cfg.MasterOutputTrigger = TIM_TRGO_RESET;
    master_cfg.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&g_htim_ctrl, &master_cfg) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_TIM_Base_Start_IT(&g_htim_ctrl);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TARGET_CTRL_TIM_INSTANCE) {
        uint32_t now_us = target_micros();
        if (s_tick_pending != 0u) {
            s_tick_overrun_cnt++;
        }
        s_tick_isr_us  = now_us;
        s_tick_seq++;
        s_tick_pending = 1u;
    }
}

uint8_t target_timer_500hz_consume_tick(uint32_t *out_tick_us,
                                        uint32_t *out_tick_seq,
                                        uint32_t *out_overrun_count)
{
    uint8_t  ready = 0u;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (s_tick_pending != 0u) {
        s_tick_pending = 0u;
        ready = 1u;
        if (out_tick_us != NULL) {
            *out_tick_us = s_tick_isr_us;
        }
        if (out_tick_seq != NULL) {
            *out_tick_seq = s_tick_seq;
        }
        if (out_overrun_count != NULL) {
            *out_overrun_count = s_tick_overrun_cnt;
        }
    }

    if (primask == 0u) {
        __enable_irq();
    }
    return ready;
}
