#include "target_safety_io.h"
#include "target_sync.h"
#include <string.h>

static BalanceImuSample s_imu;
static uint8_t s_imu_valid;

uint8_t target_operator_reset_edge(TargetOperator *op, uint8_t raw, uint32_t now_us)
{
    if (raw != op->raw_last) {
        op->raw_last = raw;
        op->changed_us = now_us;
    }
    if ((uint32_t)(now_us - op->changed_us) < 30000u) {
        return 0u;
    }
    if (raw == 0u) {
        op->released = 1u;
    }
    if (raw != op->stable) {
        op->stable = raw;
        if (raw != 0u && op->released != 0u) {
            op->released = 0u;
            return 1u;
        }
    }
    return 0u;
}

uint8_t target_operator_arm_edge(TargetOperator *op, uint8_t fresh_radio,
                                 uint8_t arm_sw, uint8_t reset_edge,
                                 uint8_t disarmed, uint32_t now_us)
{
    if (disarmed == 0u || fresh_radio == 0u) {
        op->radio_low_seen = 0u;
        op->reset_authorized = 0u;
        return 0u;
    }
    if (reset_edge != 0u && arm_sw == 0u) {
        op->reset_authorized = 1u;
        op->reset_us = now_us;
    }
    if ((uint32_t)(now_us - op->reset_us) > 5000000u) {
        op->reset_authorized = 0u;
    }
    if (arm_sw == 0u) {
        op->radio_low_seen = 1u;
        return 0u;
    }
    uint8_t edge = op->radio_low_seen;
    op->radio_low_seen = 0u;
    if (edge != 0u && op->reset_authorized != 0u) {
        op->reset_authorized = 0u;
        return 1u;
    }
    return 0u;
}

void target_safety_io_init(void)
{
    s_imu_valid = 0u;
    memset(&s_imu, 0, sizeof(s_imu));
    /* No guessed GPIO. Board integration must define all three macros together. */
#if defined(TARGET_RESET_BTN_PORT) && defined(TARGET_RESET_BTN_PIN) && defined(TARGET_RESET_BTN_CLK_ENABLE)
    GPIO_InitTypeDef gpio;
    memset(&gpio, 0, sizeof(gpio));
    TARGET_RESET_BTN_CLK_ENABLE();
    gpio.Pin = TARGET_RESET_BTN_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN; /* Active high momentary local reset button. */
    HAL_GPIO_Init(TARGET_RESET_BTN_PORT, &gpio);
#endif
}

uint8_t target_reset_button_raw(void)
{
#if defined(TARGET_RESET_BTN_PORT) && defined(TARGET_RESET_BTN_PIN) && defined(TARGET_RESET_BTN_CLK_ENABLE)
    return HAL_GPIO_ReadPin(TARGET_RESET_BTN_PORT, TARGET_RESET_BTN_PIN) == GPIO_PIN_SET ? 1u : 0u;
#else
    return 0u; /* Unwired: no reset authorization and therefore no arm authorization. */
#endif
}

void target_power_unwired_inputs(BalanceTickInput *in)
{
    /* UNKNOWN measurements are never represented as healthy contacts/voltages.
     * Replace this entire adapter only after board ADC/GPIO validation. */
    in->boot_self_test_ok = 0u;
    in->dump_continuity_ok = 0u;
    in->k1_closed = 0u;
    in->k2_closed = 0u;
    in->kpre_closed = 0u;
    in->precharge_dv_ok = 0u;
    in->mirror_mismatch = 1u;
    in->drive_enable_fail = 1u;
    in->v_bus_v = 0.0f;
    in->v_batt_ocv_v = 0.0f;
    in->i_bus_ma = 0;
}

void target_imu_publish_isr(const BalanceImuSample *sample)
{
    if (sample != NULL) {
        uint32_t saved = target_irq_save();
        s_imu = *sample;
        s_imu_valid = 1u;
        target_irq_restore(saved);
    }
}

uint8_t target_imu_snapshot(BalanceImuSample *sample)
{
    uint32_t saved = target_irq_save();
    uint8_t valid = s_imu_valid;
    if (sample != NULL) {
        *sample = s_imu; /* Keep acquisition timestamp; do not invent a fresh sample. */
        s_imu_valid = 0u;
    }
    target_irq_restore(saved);
    return valid;
}
