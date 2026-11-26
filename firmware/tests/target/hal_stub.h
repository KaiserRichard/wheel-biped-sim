#ifndef WBR_HAL_STUB_H
#define WBR_HAL_STUB_H
#include <stdint.h>
#include <stddef.h>
#include "hal_stub_constants.h"
typedef enum { HAL_OK = 0, HAL_ERROR = 1 } HAL_StatusTypeDef;
typedef uint32_t GPIO_PinState;
typedef struct { uint32_t tag; } GPIO_TypeDef;
typedef struct { uint32_t tag; } FDCAN_GlobalTypeDef;
extern GPIO_TypeDef stub_gpio[5];
extern FDCAN_GlobalTypeDef stub_can[3];
#define GPIOA (&stub_gpio[0])
#define GPIOB (&stub_gpio[1])
#define GPIOD (&stub_gpio[2])
#define GPIOE (&stub_gpio[3])
#define GPIOC (&stub_gpio[4])
#define FDCAN1 (&stub_can[0])
#define FDCAN2 (&stub_can[1])
#define FDCAN3 (&stub_can[2])
#define USART1 ((void *)1)
#define USART6 ((void *)6)
#define USB1_OTG_HS ((void *)2)
#define IWDG1 ((void *)3)
#define TIM6 ((void *)4)
#define EXTI9_5_IRQn 0u
#define EXTI15_10_IRQn 7u
#define FDCAN1_IT0_IRQn 1u
#define FDCAN2_IT0_IRQn 2u
#define FDCAN3_IT0_IRQn 3u
#define USART1_IRQn 4u
#define USART6_IRQn 8u
#define OTG_HS_IRQn 5u
#define TIM6_DAC_IRQn 6u
#define DISABLE 0u

typedef struct { uint32_t Pin, Mode, Pull, Speed, Alternate; } GPIO_InitTypeDef;
typedef struct { uint32_t PeriphClockSelection, Usart16ClockSelection; } RCC_PeriphCLKInitTypeDef;
typedef struct {
 uint32_t FrameFormat, Mode, AutoRetransmission, TransmitPause, ProtocolException;
 uint32_t NominalPrescaler, NominalSyncJumpWidth, NominalTimeSeg1, NominalTimeSeg2;
 uint32_t DataPrescaler, DataSyncJumpWidth, DataTimeSeg1, DataTimeSeg2;
 uint32_t MessageRAMOffset, StdFiltersNbr, ExtFiltersNbr;
 uint32_t RxFifo0ElmtsNbr, RxFifo0ElmtSize, RxFifo1ElmtsNbr, RxFifo1ElmtSize;
 uint32_t RxBuffersNbr, RxBufferSize, TxEventsNbr, TxBuffersNbr;
 uint32_t TxFifoQueueElmtsNbr, TxFifoQueueMode, TxElmtSize;
} FDCAN_InitTypeDef;
typedef struct { FDCAN_GlobalTypeDef *Instance; FDCAN_InitTypeDef Init; } FDCAN_HandleTypeDef;
typedef struct { uint32_t IdType, FilterIndex, FilterType, FilterConfig, FilterID1, FilterID2; } FDCAN_FilterTypeDef;
typedef struct {
 uint32_t Identifier, IdType, TxFrameType, DataLength, ErrorStateIndicator;
 uint32_t BitRateSwitch, FDFormat, TxEventFifoControl, MessageMarker;
} FDCAN_TxHeaderTypeDef;
typedef struct { uint32_t Identifier, IdType, RxFrameType, DataLength; } FDCAN_RxHeaderTypeDef;
typedef struct { uint32_t TxErrorCnt, RxErrorCnt; } FDCAN_ErrorCountersTypeDef;
typedef struct { uint32_t BusOff, ErrorPassive, Warning; } FDCAN_ProtocolStatusTypeDef;
typedef struct {
 uint32_t BaudRate, WordLength, StopBits, Parity, Mode, HwFlowCtl, OverSampling;
 uint32_t OneBitSampling, ClockPrescaler;
} UART_InitTypeDef;
typedef struct { void *Instance; UART_InitTypeDef Init; struct { uint32_t AdvFeatureInit; } AdvancedInit; } UART_HandleTypeDef;
typedef struct {
 uint32_t dev_endpoints, speed, dma_enable, ep0_mps, phy_itface, Sof_enable;
 uint32_t low_power_enable, lpm_enable, battery_charging_enable, vbus_sensing_enable, use_dedicated_ep1;
} PCD_InitTypeDef;
typedef struct { void *Instance; PCD_InitTypeDef Init; uint32_t Setup[3]; } PCD_HandleTypeDef;
typedef struct { void *Instance; struct { uint32_t Prescaler, Window, Reload; } Init; } IWDG_HandleTypeDef;

extern uint32_t stub_primask, stub_exti_pending, stub_reset_flags, stub_now;
extern uint8_t stub_stop_closed, stub_opto_closed, stub_permit;
extern void (*stub_irq_on_enable)(void);
extern void (*stub_before_irq_disable)(void);
/* FW8: host exit for target_estop_fatal_halt(); each poll advances stub_now by
 * 100 us and returns 1 after stub_halt_polls_max polls. */
extern uint32_t stub_halt_polls, stub_halt_polls_max;
uint8_t stub_halt_poll(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __enable_irq(void);
void __DMB(void);
#define __HAL_GPIO_EXTI_GET_IT(pin) (stub_exti_pending & (pin))
#define __HAL_RCC_GET_FLAG(flag) (stub_reset_flags & (flag))
#define __HAL_RCC_CLEAR_RESET_FLAGS() (stub_reset_flags = 0u)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __HAL_RCC_FDCAN_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART1_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USART6_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USB1_OTG_HS_CLK_ENABLE() ((void)0)
#define __HAL_RCC_USB1_OTG_HS_CLK_DISABLE() ((void)0)
void HAL_GPIO_Init(GPIO_TypeDef *, GPIO_InitTypeDef *);
void HAL_GPIO_DeInit(GPIO_TypeDef *, uint32_t);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint32_t);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint32_t, GPIO_PinState);
void HAL_NVIC_SetPriority(uint32_t, uint32_t, uint32_t);
void HAL_NVIC_EnableIRQ(uint32_t);
void HAL_NVIC_DisableIRQ(uint32_t);
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_Init(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *, FDCAN_FilterTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *, uint32_t, uint32_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ConfigInterruptLines(FDCAN_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_Start(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_Stop(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *, uint32_t);
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *, FDCAN_TxHeaderTypeDef *, uint8_t *);
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *, uint32_t, FDCAN_RxHeaderTypeDef *, uint8_t *);
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *, FDCAN_ErrorCountersTypeDef *);
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *, FDCAN_ProtocolStatusTypeDef *);
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UARTEx_SetTxFifoThreshold(UART_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_UARTEx_SetRxFifoThreshold(UART_HandleTypeDef *, uint32_t);
HAL_StatusTypeDef HAL_UARTEx_DisableFifoMode(UART_HandleTypeDef *);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint32_t);
void HAL_PWREx_EnableUSBVoltageDetector(void);
HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *);
void HAL_PCDEx_SetRxFiFo(PCD_HandleTypeDef *, uint32_t);
void HAL_PCDEx_SetTxFiFo(PCD_HandleTypeDef *, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *);
HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *, uint8_t, uint32_t, uint32_t);
HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *, uint8_t, uint8_t *, uint32_t);
HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *, uint8_t, uint8_t *, uint32_t);
HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *, uint8_t);
uint32_t HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef *, uint8_t);
/* FW10 EP0 / enumeration stubs and observation points. */
HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *, uint8_t);
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *, uint8_t);
HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *, uint8_t);
uint32_t HAL_GetUIDw0(void);
uint32_t HAL_GetUIDw1(void);
uint32_t HAL_GetUIDw2(void);
extern uint8_t  stub_ep0_in[512];       /* concatenated EP0 IN data packets */
extern uint32_t stub_ep0_in_len, stub_ep0_in_pkts, stub_ep0_zlp, stub_ep0_stalls;
extern uint32_t stub_ep0_status_out_armed, stub_ep_open_mask, stub_usb_address;
extern uint8_t *stub_ep0_out_buf;       /* buffer armed by EP0 OUT data stage */
extern uint32_t stub_ep0_out_len, stub_rx_count;
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *);
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *, uint8_t);
HAL_StatusTypeDef HAL_IWDG_Init(IWDG_HandleTypeDef *);
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *);

/* Deterministic fixture observation/injection points. */
typedef struct { uint8_t bus; uint32_t id; uint8_t data[8]; } StubTx;
extern StubTx stub_tx[256];
extern uint32_t stub_tx_count, stub_free[3], stub_abort_count;
extern FDCAN_FilterTypeDef stub_filters[3][2];
extern uint8_t stub_filter_count[3];
extern IWDG_HandleTypeDef stub_iwdg_config;
extern uint8_t stub_rx_ready[3], stub_rx_data[3][8];
extern FDCAN_RxHeaderTypeDef stub_rx_header[3];
extern uint8_t *stub_usb_active;
extern uint32_t stub_usb_len, stub_usb_calls, stub_usb_fail, stub_iwdg_refreshes;
extern void (*stub_on_gpio_read)(void);
void stub_reset(void);
void HAL_GPIO_EXTI_Callback(uint16_t);
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *);
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *, uint8_t);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *, uint32_t);
void HAL_FDCAN_TxBufferAbortCallback(FDCAN_HandleTypeDef *, uint32_t);
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
#endif
