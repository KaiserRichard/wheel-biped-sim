#ifndef WBR_TARGET_SAFETY_IO_H
#define WBR_TARGET_SAFETY_IO_H

#include "target_hw.h"
#include "control/balance_ctrl.h"

typedef struct {
    uint8_t raw_last;
    uint8_t stable;
    uint8_t released;
    uint8_t radio_low_seen;
    uint8_t reset_authorized;
    uint32_t changed_us;
    uint32_t reset_us;
} TargetOperator;

/* Call only in thread mode. Boot-high never counts as a reset or arm edge. */
uint8_t target_operator_reset_edge(TargetOperator *op, uint8_t raw, uint32_t now_us);
uint8_t target_operator_arm_edge(TargetOperator *op, uint8_t fresh_radio,
                                 uint8_t arm_sw, uint8_t reset_edge,
                                 uint8_t disarmed, uint32_t now_us);
void target_safety_io_init(void);
uint8_t target_reset_button_raw(void);
void target_power_unwired_inputs(BalanceTickInput *in);
void target_imu_publish_isr(const BalanceImuSample *sample);
uint8_t target_imu_snapshot(BalanceImuSample *sample);

#endif
