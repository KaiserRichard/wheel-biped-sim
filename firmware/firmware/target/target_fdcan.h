/*
 * STM32H723 3-Bus FDCAN1/2/3 Hardware Driver & ISR Glue into can_sched / S11IsrMailbox
 *
 * Configures FDCAN1, FDCAN2, and FDCAN3 at identical 1.000 Mbit/s bit timing
 * (80% sample point at 80 MHz PLL2Q), priority TX queue mode (FDCAN_TX_QUEUE_OPERATION),
 * AutoRetransmission = DISABLE for periodic 500 Hz MIT commands, and strict per-bus
 * hardware dual-ID RX filters from the N1 CAN ID map (can_map/CAN_ID_MAP.md):
 *   - J5 RIGHT, FDCAN1 (bus_idx = 0, PD0/PD1 AF9): cmd 0x004..006 | reply 0x014..016
 *   - J6 LEFT, FDCAN2 (bus_idx = 1, PB5/PB6 AF9): cmd 0x001..003 | reply 0x011..013
 *   - J7 SPARE, FDCAN3 (bus_idx = 2, PD12/PD13 AF5): no motor TX or RX filters
 * See can_map/REV4_CAN_MAPPING.md; the older N1 bus split is superseded.
 *
 * Adapted from DM-jump (Core/Src/fdcan.c, User/Bsp/can_bsp.c):
 *   Copyright (c) 2024-2026 Tran Nguyen Binh (DM-jump, MIT License)
 *   See firmware/DM_JUMP_LICENSE.txt for the full MIT license notice.
 */

#ifndef WBR_TARGET_FDCAN_H
#define WBR_TARGET_FDCAN_H

#include "target_hw.h"
#include "can_sched.h"
#include "bringup_mode.h"

#include <stdint.h>

typedef struct {
    uint8_t  bus_idx;
    uint16_t cmd_id[CAN_SCHED_MOTORS_PER_BUS];
    uint16_t reply_id[CAN_SCHED_MOTORS_PER_BUS];
} TargetFdcanBusFilter;

extern FDCAN_HandleTypeDef g_hfdcan[CAN_SCHED_NUM_BUSES];

/* Retrieve rev4 filters. Only axis_count_for_bus() slots are valid. */
CanSchedStatus target_fdcan_get_bus_filter(uint8_t bus_idx, TargetFdcanBusFilter *out_filter);

/* Initialize FDCAN1/2/3 at 1 Mbit/s with N1 RX filters, priority TX queues, and IRQs. */
HAL_StatusTypeDef target_fdcan_init(CanSched *sched);

/* Poll FDCAN1/2/3 hardware error counters (TEC/REC) & protocol status (BusOff/ErrorPassive)
 * into can_sched_update_bus_status(). If BusOff latches, flushes emergency disable frames. */
void target_fdcan_poll_status(uint32_t now_us);

/* Flush scheduled TX frames from CanSched priority queues into FDCAN1/2/3 hardware TX queues.
 * If abort_pending != 0, first aborts all 8 hardware TX buffers on all 3 buses. */
void target_fdcan_flush_tx(CanSched *sched, uint8_t abort_pending);

/* Async ISR-safe emergency torque-off: abort all pending hardware TX buffers on FDCAN1/2/3,
 * use an independent ISR-owned disable mask (+1 ms repeat), and push
 * immediate 0xFF..FD disable frames for all 6 axes into hardware TX queues. */
void target_fdcan_emergency_disable_isr(uint32_t now_us);

/* Called from the 1 kHz SysTick and TX abort/empty completion callbacks. */
void target_fdcan_emergency_service(void);

/* Thread-only: after deliberate local reset, both PB12 and PD7 healthy, no pending edge. */
void target_fdcan_clear_emergency(void);

/* DISARMED-only manual bus-off recovery (HAL_FDCAN_Stop -> HAL_FDCAN_Start). */
CanSchedStatus target_fdcan_recover_bus_off(CanSched *sched, uint8_t is_disarmed);

/* FW10: bind the bring-up TX gate consulted by the single hardware choke point
 * (send_hw) for every frame, including the emergency path. On the target,
 * target_fdcan_init() refuses to start without a bound gate. */
void target_fdcan_bind_tx_gate(const BringupCtx *gate);
uint32_t target_fdcan_tx_gate_blocked(void);

/* FW10: thread-context single-frame send through the same gates (bring-up
 * encoder poll and MOTOR_ID frames). HAL_ERROR if blocked or FIFO full. */
HAL_StatusTypeDef target_fdcan_send_direct(uint8_t bus_idx, uint16_t std_id, uint8_t dlc,
                                           const uint8_t data[8]);

#endif /* WBR_TARGET_FDCAN_H */
