/*
 * STM32H723 3-Bus FDCAN1/2/3 Hardware Driver & ISR Glue into can_sched / S11IsrMailbox
 *
 * Adapted from DM-jump (Core/Src/fdcan.c, User/Bsp/can_bsp.c):
 *   Copyright (c) 2024-2026 Tran Nguyen Binh
 *   Project: DM-jump (DM-jump), MIT License
 *   See firmware/DM_JUMP_LICENSE.txt for the full MIT license notice.
 */

#include "target_fdcan.h"
#include "target_estop.h"
#include "target_sync.h"

#include <string.h>

FDCAN_HandleTypeDef g_hfdcan[CAN_SCHED_NUM_BUSES];

static CanSched *s_bound_sched = NULL;
static uint32_t  s_fdcan_clk_refcount = 0u;

/* Scheduler is exclusively thread-owned. ISRs stage complete RX records here. */
#define TARGET_CAN_RX_CAP 16u
typedef struct {
    CanFrame frame[TARGET_CAN_RX_CAP];
    uint32_t rx_us[TARGET_CAN_RX_CAP];
    uint32_t head;
    uint32_t tail;
    uint16_t drops;
} TargetCanRxRing;
static TargetCanRxRing s_rx[CAN_SCHED_NUM_BUSES];
static volatile uint8_t s_emergency_active;
static uint8_t s_disable_pending[CAN_SCHED_NUM_BUSES];
static uint8_t s_repeat_sent;
static uint32_t s_emergency_us;


/* Rev4 connector roles; N1 globally unique IDs retained.
 * FROM FILE: ctrl_rev4.net; see can_map/REV4_CAN_MAPPING.md. */
static const TargetFdcanBusFilter s_rev4_bus_filters[CAN_SCHED_NUM_BUSES] = {
    { 0u, { 0x004u, 0x005u, 0x006u }, { 0x014u, 0x015u, 0x016u } },
    { 1u, { 0x001u, 0x002u, 0x003u }, { 0x011u, 0x012u, 0x013u } },
    { 2u, { 0u, 0u, 0u }, { 0u, 0u, 0u } }
};

static const uint32_t s_ram_offsets[CAN_SCHED_NUM_BUSES] = {
    TARGET_FDCAN1_RAM_OFFSET,
    TARGET_FDCAN2_RAM_OFFSET,
    TARGET_FDCAN3_RAM_OFFSET
};

static uint8_t bus_idx_from_instance(const FDCAN_GlobalTypeDef *inst)
{
    if (inst == FDCAN1) {
        return 0u;
    }
    if (inst == FDCAN2) {
        return 1u;
    }
    if (inst == FDCAN3) {
        return 2u;
    }
    return CAN_SCHED_NUM_BUSES;
}

static uint32_t dlc_to_hal_code(uint8_t dlc)
{
    if (dlc >= 8u) {
        return FDCAN_DLC_BYTES_8;
    }
    return ((uint32_t)dlc) << 16u;
}

static uint8_t hal_code_to_dlc(uint32_t hal_dlc)
{
    const uint32_t code = (hal_dlc >> 16u) & 0x0Fu;
    if (code <= 8u) {
        return (uint8_t)code;
    }
    return 8u;
}

/* Verify N1 low-nibble motor_id rule: (rx_data[0] & 0x0F) == (cmd_can_id & 0x0F)
 * per can_map/CAN_ID_MAP.md Section (d). */
static uint8_t verify_n1_payload_motor_id(uint8_t bus_idx, uint16_t std_id, uint8_t byte0)
{
    if (bus_idx >= CAN_SCHED_NUM_BUSES) {
        return 0u;
    }
    const TargetFdcanBusFilter *f = &s_rev4_bus_filters[bus_idx];
    for (uint8_t slot = 0u; slot < can_sched_axis_count_for_bus(bus_idx); ++slot) {
        if (std_id == f->reply_id[slot]) {
            const uint8_t expected_id = (uint8_t)(f->cmd_id[slot] & 0x0Fu);
            const uint8_t actual_id   = (uint8_t)(byte0 & 0x0Fu);
            return (actual_id == expected_id) ? 1u : 0u;
        }
    }
    return 0u;
}

static void abort_all_hw_tx_buffers(void)
{
    const uint32_t all_8_buffers =
        FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2 | FDCAN_TX_BUFFER3 |
        FDCAN_TX_BUFFER4 | FDCAN_TX_BUFFER5 | FDCAN_TX_BUFFER6 | FDCAN_TX_BUFFER7;

    for (uint8_t b = 0u; b < CAN_SCHED_NUM_BUSES; ++b) {
        if (g_hfdcan[b].Instance != NULL) {
            (void)HAL_FDCAN_AbortTxRequest(&g_hfdcan[b], all_8_buffers);
        }
    }
}

static uint8_t frame_is_disable(const CanFrame *frame)
{
    if (frame->dlc != 8u || frame->data[7] != 0xFDu) {
        return 0u;
    }
    for (uint8_t i = 0u; i < 7u; ++i) {
        if (frame->data[i] != 0xFFu) {
            return 0u;
        }
    }
    return 1u;
}

/* Caller holds PRIMASK across the final stop gate and HW enqueue. */
static HAL_StatusTypeDef send_hw(uint8_t bus_idx, const CanFrame *frame)
{
    if ((s_emergency_active != 0u || target_estop_is_tripped() != 0u)
        && frame_is_disable(frame) == 0u) {
        return HAL_ERROR;
    }
    FDCAN_TxHeaderTypeDef hdr;
    memset(&hdr, 0, sizeof(hdr));
    hdr.Identifier = frame->std_id;
    hdr.IdType = FDCAN_STANDARD_ID;
    hdr.TxFrameType = FDCAN_DATA_FRAME;
    hdr.DataLength = dlc_to_hal_code(frame->dlc);
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch = FDCAN_BRS_OFF;
    hdr.FDFormat = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    return HAL_FDCAN_AddMessageToTxFifoQ(&g_hfdcan[bus_idx], &hdr, (uint8_t *)frame->data);
}

static void drain_bus_tx_to_hw(CanSched *sched, uint8_t bus_idx)
{
    if (sched == NULL || bus_idx >= CAN_SCHED_NUM_BUSES
        || g_hfdcan[bus_idx].Instance == NULL) {
        return;
    }
    while (can_sched_tx_pending(sched, bus_idx) > 0u) {
        uint32_t saved = target_irq_save();
        if (HAL_FDCAN_GetTxFifoFreeLevel(&g_hfdcan[bus_idx]) == 0u) {
            target_irq_restore(saved);
            break;
        }
        CanFrame frame;
        CanSchedStatus st = can_sched_pop_tx(sched, bus_idx, &frame);
        if (st == CAN_SCHED_OK && send_hw(bus_idx, &frame) != HAL_OK) {
            sched->bus[bus_idx].tx_drop_count++;
        }
        target_irq_restore(saved);
        if (st != CAN_SCHED_OK) {
            break;
        }
    }
}

/* Never touches CanSched. Abort may finish later; keep unsent disable bits. */
void target_fdcan_emergency_service(void)
{
    uint32_t saved = target_irq_save();
    /* Sample after masking: an EXTI trip cannot publish a newer timestamp
     * between this sample and subtraction (HFX2-03). u32 wrap remains valid. */
    uint32_t now_us = target_micros();
    if (s_emergency_active != 0u) {
        if (s_repeat_sent == 0u && (uint32_t)(now_us - s_emergency_us) >= 1000u) {
            for (uint8_t b = 0u; b < CAN_SCHED_NUM_BUSES; ++b) {
                s_disable_pending[b] |= (uint8_t)((1u << can_sched_axis_count_for_bus(b)) - 1u);
            }
            s_repeat_sent = 1u;
        }
        for (uint8_t b = 0u; b < CAN_SCHED_NUM_BUSES; ++b) {
            if (g_hfdcan[b].Instance == NULL) {
                continue;
            }
            for (uint8_t slot = 0u; slot < can_sched_axis_count_for_bus(b); ++slot) {
                uint8_t bit = (uint8_t)(1u << slot);
                if ((s_disable_pending[b] & bit) != 0u
                    && HAL_FDCAN_GetTxFifoFreeLevel(&g_hfdcan[b]) > 0u) {
                    CanFrame frame;
                    memset(&frame, 0, sizeof(frame));
                    frame.std_id = s_rev4_bus_filters[b].cmd_id[slot];
                    frame.dlc = 8u;
                    memset(frame.data, 0xFF, sizeof(frame.data));
                    frame.data[7] = 0xFDu;
                    if (send_hw(b, &frame) == HAL_OK) {
                        s_disable_pending[b] &= (uint8_t)~bit;
                    }
                }
            }
        }
    }
    target_irq_restore(saved);
}

void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t indexes)
{
    (void)hfdcan;
    (void)indexes;
    target_fdcan_emergency_service();
}

void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
    (void)hfdcan;
    if (s_emergency_active != 0u) {
        target_fdcan_emergency_service();
    }
}

CanSchedStatus target_fdcan_get_bus_filter(uint8_t bus_idx, TargetFdcanBusFilter *out_filter)
{
    if (out_filter == NULL) {
        return CAN_SCHED_ERR_NULL_PTR;
    }
    if (bus_idx >= CAN_SCHED_NUM_BUSES) {
        return CAN_SCHED_ERR_INVALID_BUS;
    }
    *out_filter = s_rev4_bus_filters[bus_idx];
    return CAN_SCHED_OK;
}

HAL_StatusTypeDef target_fdcan_init(CanSched *sched)
{
    if (sched == NULL) {
        return HAL_ERROR;
    }
    s_bound_sched = sched;
    memset(s_rx, 0, sizeof(s_rx));

    FDCAN_GlobalTypeDef * const instances[CAN_SCHED_NUM_BUSES] = {
        FDCAN1, FDCAN2, FDCAN3
    };

    for (uint8_t b = 0u; b < CAN_SCHED_NUM_BUSES; ++b) {
        FDCAN_HandleTypeDef *h = &g_hfdcan[b];
        memset(h, 0, sizeof(*h));

        h->Instance                  = instances[b];
        h->Init.FrameFormat          = FDCAN_FRAME_CLASSIC;
        h->Init.Mode                 = FDCAN_MODE_NORMAL;
        h->Init.AutoRetransmission   = DISABLE;
        h->Init.TransmitPause        = DISABLE;
        h->Init.ProtocolException    = DISABLE;
        /* Identical 1.000 Mbit/s @ 80 MHz PLL2Q (20 TQ/bit, 80.0% sample point) */
        h->Init.NominalPrescaler     = TARGET_FDCAN_PRESCALER;
        h->Init.NominalSyncJumpWidth = TARGET_FDCAN_SJW;
        h->Init.NominalTimeSeg1      = TARGET_FDCAN_TSEG1;
        h->Init.NominalTimeSeg2      = TARGET_FDCAN_TSEG2;
        h->Init.DataPrescaler        = TARGET_FDCAN_PRESCALER;
        h->Init.DataSyncJumpWidth    = TARGET_FDCAN_SJW;
        h->Init.DataTimeSeg1         = TARGET_FDCAN_TSEG1;
        h->Init.DataTimeSeg2         = TARGET_FDCAN_TSEG2;
        h->Init.MessageRAMOffset     = s_ram_offsets[b];
        h->Init.StdFiltersNbr        = (can_sched_axis_count_for_bus(b) + 1u) / 2u;
        h->Init.ExtFiltersNbr        = 0u;
        h->Init.RxFifo0ElmtsNbr      = TARGET_FDCAN_RX_FIFO0_ELMTS;
        h->Init.RxFifo0ElmtSize      = FDCAN_DATA_BYTES_8;
        h->Init.RxFifo1ElmtsNbr      = 0u;
        h->Init.RxFifo1ElmtSize      = FDCAN_DATA_BYTES_8;
        h->Init.RxBuffersNbr         = 0u;
        h->Init.RxBufferSize         = FDCAN_DATA_BYTES_8;
        h->Init.TxEventsNbr          = 0u;
        h->Init.TxBuffersNbr         = 0u;
        h->Init.TxFifoQueueElmtsNbr  = TARGET_FDCAN_TX_QUEUE_ELMTS;
        h->Init.TxFifoQueueMode      = FDCAN_TX_QUEUE_OPERATION;
        h->Init.TxElmtSize           = FDCAN_DATA_BYTES_8;

        if (HAL_FDCAN_Init(h) != HAL_OK) {
            return HAL_ERROR;
        }

        /* Two exact dual-ID filters cover three leg motors. Duplicate the last
         * ID rather than admitting a fourth. Spare has zero filters: reject all. */
        for (uint8_t i = 0u; i < h->Init.StdFiltersNbr; ++i) {
            uint8_t first = (uint8_t)(2u * i);
            uint8_t second = (uint8_t)(first + 1u);
            if (second >= can_sched_axis_count_for_bus(b)) {
                second = first;
            }
            FDCAN_FilterTypeDef filter;
            memset(&filter, 0, sizeof(filter));
            filter.IdType       = FDCAN_STANDARD_ID;
            filter.FilterIndex  = i;
            filter.FilterType   = FDCAN_FILTER_DUAL;
            filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
            filter.FilterID1    = s_rev4_bus_filters[b].reply_id[first];
            filter.FilterID2    = s_rev4_bus_filters[b].reply_id[second];
            if (HAL_FDCAN_ConfigFilter(h, &filter) != HAL_OK) {
                return HAL_ERROR;
            }
        }

        /* Reject all non-matching standard/extended frames and all remote frames. */
        if (HAL_FDCAN_ConfigGlobalFilter(h,
                                         FDCAN_REJECT,
                                         FDCAN_REJECT,
                                         FDCAN_REJECT_REMOTE,
                                         FDCAN_REJECT_REMOTE) != HAL_OK) {
            return HAL_ERROR;
        }

        /* Route RX FIFO0 new message + BusOff/ErrorPassive/ErrorWarning to Line 0. */
        const uint32_t active_its =
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
            FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
            FDCAN_IT_BUS_OFF |
            FDCAN_IT_ERROR_PASSIVE |
            FDCAN_IT_ERROR_WARNING |
            FDCAN_IT_TX_ABORT_COMPLETE;

        if (HAL_FDCAN_ConfigInterruptLines(h, active_its, FDCAN_INTERRUPT_LINE0) != HAL_OK) {
            return HAL_ERROR;
        }
        if (HAL_FDCAN_ActivateNotification(h, active_its, 0xFFu) != HAL_OK) {
            return HAL_ERROR;
        }
        if (HAL_FDCAN_Start(h) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle)
{
    if (fdcanHandle == NULL) {
        return;
    }

    GPIO_InitTypeDef gpio;
    memset(&gpio, 0, sizeof(gpio));

    if (s_fdcan_clk_refcount == 0u) {
        __HAL_RCC_FDCAN_CLK_ENABLE();
    }
    s_fdcan_clk_refcount++;

    if (fdcanHandle->Instance == FDCAN1) {
        /* ASSUMED: PD0 (FDCAN1_RX), PD1 (FDCAN1_TX), AF9 */
        __HAL_RCC_GPIOD_CLK_ENABLE();
        gpio.Pin       = TARGET_FDCAN1_RX_PIN | TARGET_FDCAN1_TX_PIN;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = TARGET_FDCAN1_GPIO_AF;
        HAL_GPIO_Init(TARGET_FDCAN1_GPIO_PORT, &gpio);

        HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, TARGET_NVIC_PRIO_FDCAN, 0u);
        HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
    } else if (fdcanHandle->Instance == FDCAN2) {
        /* ASSUMED: PB5 (FDCAN2_RX), PB6 (FDCAN2_TX), AF9 */
        __HAL_RCC_GPIOB_CLK_ENABLE();
        gpio.Pin       = TARGET_FDCAN2_RX_PIN | TARGET_FDCAN2_TX_PIN;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = TARGET_FDCAN2_GPIO_AF;
        HAL_GPIO_Init(TARGET_FDCAN2_GPIO_PORT, &gpio);

        HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, TARGET_NVIC_PRIO_FDCAN, 0u);
        HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
    } else if (fdcanHandle->Instance == FDCAN3) {
        /* ASSUMED: PD12 (FDCAN3_RX), PD13 (FDCAN3_TX), AF5 */
        __HAL_RCC_GPIOD_CLK_ENABLE();
        gpio.Pin       = TARGET_FDCAN3_RX_PIN | TARGET_FDCAN3_TX_PIN;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = TARGET_FDCAN3_GPIO_AF;
        HAL_GPIO_Init(TARGET_FDCAN3_GPIO_PORT, &gpio);

        HAL_NVIC_SetPriority(FDCAN3_IT0_IRQn, TARGET_NVIC_PRIO_FDCAN, 0u);
        HAL_NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan == NULL || s_bound_sched == NULL) {
        return;
    }

    const uint8_t bus_idx = bus_idx_from_instance(hfdcan->Instance);
    if (bus_idx >= CAN_SCHED_NUM_BUSES) {
        return;
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0u) {
        uint32_t saved = target_irq_save();
        s_rx[bus_idx].drops++;
        target_irq_restore(saved);
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0u) {
        /* Stamp DWT microsecond clock immediately on ISR entry. */
        const uint32_t rx_us = target_micros();

        uint32_t drained = 0u;
        while (drained < TARGET_FDCAN_RX_FIFO0_ELMTS
               && HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u) {
            FDCAN_RxHeaderTypeDef rx_hdr;
            uint8_t rx_data[64];
            memset(&rx_hdr, 0, sizeof(rx_hdr));
            memset(rx_data, 0, sizeof(rx_data));

            uint32_t saved = target_irq_save();
            HAL_StatusTypeDef rx_st = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_hdr, rx_data);
            target_irq_restore(saved);
            if (rx_st != HAL_OK) {
                break;
            }
            drained++;

            CanFrame frame;
            memset(&frame, 0, sizeof(frame));
            frame.std_id      = (uint16_t)(rx_hdr.Identifier & 0x7FFu);
            frame.dlc         = hal_code_to_dlc(rx_hdr.DataLength);
            frame.is_extended = (rx_hdr.IdType == FDCAN_EXTENDED_ID) ? 1u : 0u;
            frame.is_rtr      = (rx_hdr.RxFrameType == FDCAN_REMOTE_FRAME) ? 1u : 0u;
            memcpy(frame.data, rx_data, DAMIAO_CAN_DLC);

            /* Verify low-nibble payload motor_id against N1 CAN map before dispatching
             * into can_sched_rx_frame() -> S11IsrMailbox. */
            if (frame.dlc == DAMIAO_CAN_DLC
                && verify_n1_payload_motor_id(bus_idx, frame.std_id, frame.data[0]) == 0u) {
                /* Corrupt byte 0 / mismatched motor_id: force DLC mismatch so can_sched
                 * increments rx_drop_count and marks the S11 mailbox slot invalid. */
                frame.dlc = 0u;
            }

            saved = target_irq_save();
            TargetCanRxRing *ring = &s_rx[bus_idx];
            uint32_t next = (ring->head + 1u) % TARGET_CAN_RX_CAP;
            if (next != ring->tail) {
                ring->frame[ring->head] = frame;
                ring->rx_us[ring->head] = rx_us;
                ring->head = next;
            } else {
                ring->drops++;
            }
            target_irq_restore(saved);
        }
    }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t its)
{
    (void)hfdcan;
    /* Drive communication fault: immediate independent torque-off; main records
     * counters/updates scheduler. No scheduler mutation at interrupt priority. */
    if ((its & FDCAN_IT_BUS_OFF) != 0u) {
        target_fdcan_emergency_disable_isr(target_micros());
    }
}

static void consume_rx(void)
{
    for (uint8_t b = 0u; b < CAN_SCHED_NUM_BUSES; ++b) {
        uint32_t consumed = 0u;
        while (consumed < TARGET_CAN_RX_CAP) {
            CanFrame frame;
            uint32_t rx_us = 0u;
            uint32_t saved = target_irq_save();
            TargetCanRxRing *ring = &s_rx[b];
            s_bound_sched->bus[b].rx_drop_count += ring->drops;
            ring->drops = 0u;
            uint8_t ready = ring->tail != ring->head ? 1u : 0u;
            if (ready != 0u) {
                frame = ring->frame[ring->tail];
                rx_us = ring->rx_us[ring->tail];
                ring->tail = (ring->tail + 1u) % TARGET_CAN_RX_CAP;
            }
            target_irq_restore(saved);
            if (ready == 0u) {
                break;
            }
            consumed++;
            (void)can_sched_rx_frame(s_bound_sched, b, &frame, rx_us);
        }
    }
}

void target_fdcan_poll_status(uint32_t now_us)
{
    if (s_bound_sched == NULL) {
        return;
    }

    consume_rx();
    const uint8_t was_bus_off = s_bound_sched->any_bus_off_latched;
    for (uint8_t b = 0u; b < CAN_SCHED_NUM_BUSES; ++b) {
        FDCAN_HandleTypeDef *h = &g_hfdcan[b];
        if (h->Instance == NULL) {
            continue;
        }
        FDCAN_ErrorCountersTypeDef cnt;
        FDCAN_ProtocolStatusTypeDef proto;
        memset(&cnt, 0, sizeof(cnt));
        memset(&proto, 0, sizeof(proto));

        uint32_t saved = target_irq_save();
        uint8_t status_ok = (HAL_FDCAN_GetErrorCounters(h, &cnt) == HAL_OK
                             && HAL_FDCAN_GetProtocolStatus(h, &proto) == HAL_OK) ? 1u : 0u;
        target_irq_restore(saved);
        if (status_ok != 0u) {
            (void)can_sched_update_bus_status(s_bound_sched,
                                              b,
                                              (uint16_t)cnt.TxErrorCnt,
                                              (uint16_t)cnt.RxErrorCnt,
                                              (proto.BusOff != 0u) ? 1u : 0u,
                                              (proto.ErrorPassive != 0u) ? 1u : 0u,
                                              (proto.Warning != 0u) ? 1u : 0u,
                                              now_us);
        }
    }

    if (was_bus_off == 0u && s_bound_sched->any_bus_off_latched != 0u) {
        target_fdcan_flush_tx(s_bound_sched, 1u);
    }
}

void target_fdcan_flush_tx(CanSched *sched, uint8_t abort_pending)
{
    if (sched == NULL) {
        return;
    }
    if (abort_pending != 0u && s_emergency_active == 0u) {
        target_fdcan_emergency_disable_isr(target_micros());
    }
    target_fdcan_emergency_service();
    for (uint8_t b = 0u; b < CAN_SCHED_NUM_BUSES; ++b) {
        drain_bus_tx_to_hw(sched, b);
    }
}

void target_fdcan_emergency_disable_isr(uint32_t now_us)
{
    uint32_t saved = target_irq_save();
    if (s_emergency_active == 0u) {
        s_emergency_active = 1u;
        s_emergency_us = now_us;
        s_repeat_sent = 0u;
        abort_all_hw_tx_buffers();
        for (uint8_t b = 0u; b < CAN_SCHED_NUM_BUSES; ++b) {
            s_disable_pending[b] = (uint8_t)((1u << can_sched_axis_count_for_bus(b)) - 1u);
        }
    }
    target_fdcan_emergency_service();
    target_irq_restore(saved);
}

void target_fdcan_clear_emergency(void)
{
    uint32_t saved = target_irq_save();
    s_emergency_active = 0u;
    memset(s_disable_pending, 0, sizeof(s_disable_pending));
    /* Thread's old queued commands are also discarded before any future arm. */
    if (s_bound_sched != NULL) {
        for (uint8_t b = 0u; b < CAN_SCHED_NUM_BUSES; ++b) {
            memset(s_bound_sched->bus[b].tx_queue, 0, sizeof(s_bound_sched->bus[b].tx_queue));
            s_bound_sched->bus[b].tx_count = 0u;
        }
        s_bound_sched->disable_repeat_armed = 0u;
    }
    target_irq_restore(saved);
}

CanSchedStatus target_fdcan_recover_bus_off(CanSched *sched, uint8_t is_disarmed)
{
    if (sched == NULL) {
        return CAN_SCHED_ERR_NULL_PTR;
    }
    const CanSchedStatus st = can_sched_request_bus_recovery(sched, is_disarmed);
    if (st != CAN_SCHED_OK) {
        return st;
    }

    for (uint8_t b = 0u; b < CAN_SCHED_NUM_BUSES; ++b) {
        if (sched->bus[b].recovery_requested != 0u && g_hfdcan[b].Instance != NULL) {
            uint32_t saved = target_irq_save();
            (void)HAL_FDCAN_Stop(&g_hfdcan[b]);
            (void)HAL_FDCAN_Start(&g_hfdcan[b]);
            target_irq_restore(saved);
        }
    }
    return CAN_SCHED_OK;
}
