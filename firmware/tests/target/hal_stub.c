#include "hal_stub.h"
#include "target_hw.h"
#include <assert.h>
#include <string.h>
GPIO_TypeDef stub_gpio[5];
FDCAN_GlobalTypeDef stub_can[3];
uint32_t stub_primask, stub_exti_pending, stub_reset_flags, stub_now;
uint8_t stub_stop_closed, stub_opto_closed, stub_permit;
void (*stub_irq_on_enable)(void);
void (*stub_before_irq_disable)(void);
void (*stub_on_gpio_read)(void);
StubTx stub_tx[256];
uint32_t stub_tx_count, stub_free[3], stub_abort_count;
FDCAN_FilterTypeDef stub_filters[3][2];
uint8_t stub_filter_count[3];
IWDG_HandleTypeDef stub_iwdg_config;
uint8_t stub_rx_ready[3], stub_rx_data[3][8];
FDCAN_RxHeaderTypeDef stub_rx_header[3];
uint8_t *stub_usb_active;
uint32_t stub_usb_len, stub_usb_calls, stub_usb_fail, stub_iwdg_refreshes;
void stub_reset(void) {
 stub_primask = stub_exti_pending = stub_reset_flags = stub_now = 0u;
 stub_stop_closed = 1u; stub_opto_closed = 1u; stub_permit = 0u;
 stub_irq_on_enable = NULL; stub_on_gpio_read = NULL;
 stub_before_irq_disable = NULL;
 memset(stub_filter_count, 0, sizeof(stub_filter_count));
 memset(stub_filters, 0, sizeof(stub_filters));
 stub_tx_count = stub_abort_count = 0u;
 memset(stub_rx_ready, 0, sizeof(stub_rx_ready));
 for (unsigned i = 0; i < 3; ++i) { stub_free[i] = 8u; }
 stub_usb_active = NULL; stub_usb_len = stub_usb_calls = stub_usb_fail = 0u;
 stub_iwdg_refreshes = 0u;
}
uint32_t __get_PRIMASK(void) { return stub_primask; }
void __disable_irq(void) {
 if (stub_before_irq_disable != NULL) {
  void (*hook)(void) = stub_before_irq_disable; stub_before_irq_disable = NULL; hook();
 }
 stub_primask = 1u;
}
void __enable_irq(void) {
 stub_primask = 0u;
 if (stub_irq_on_enable != NULL) {
  void (*irq)(void) = stub_irq_on_enable; stub_irq_on_enable = NULL; irq();
 }
}
void __DMB(void) { __asm__ volatile ("" ::: "memory"); }
uint32_t target_micros(void) { return stub_now; }
uint64_t target_micros64(void) { return stub_now; }
void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g) { (void)p; (void)g; }
void HAL_GPIO_DeInit(GPIO_TypeDef *p, uint32_t pin) { (void)p; (void)pin; }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint32_t pin) {
 uint32_t value = GPIO_PIN_RESET;
 if (p == GPIOB && pin == TARGET_ESTOP_PIN) {
  value = stub_stop_closed != 0u ? GPIO_PIN_RESET : GPIO_PIN_SET;
 } else if (p == GPIOD && pin == TARGET_ESTOP_OPTO_PIN) {
  value = stub_opto_closed != 0u ? GPIO_PIN_RESET : GPIO_PIN_SET;
 } else if (p == GPIOD && pin == TARGET_PERMIT_PIN) {
  value = stub_permit != 0u ? GPIO_PIN_SET : GPIO_PIN_RESET;
 }
 if (stub_on_gpio_read != NULL) {
  void (*hook)(void) = stub_on_gpio_read; stub_on_gpio_read = NULL; hook();
 }
 return value;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint32_t pin, GPIO_PinState v) {
 if (p == GPIOD && pin == TARGET_PERMIT_PIN) {
  stub_permit = v == GPIO_PIN_SET ? 1u : 0u;
 }
}
void HAL_NVIC_SetPriority(uint32_t a, uint32_t b, uint32_t c) { (void)a; (void)b; (void)c; }
void HAL_NVIC_EnableIRQ(uint32_t a) { (void)a; }
void HAL_NVIC_DisableIRQ(uint32_t a) { (void)a; }
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *c) { (void)c; return HAL_OK; }
#define SIMPLE_HAL(name, type) HAL_StatusTypeDef name(type *h) { (void)h; return HAL_OK; }
SIMPLE_HAL(HAL_FDCAN_Init, FDCAN_HandleTypeDef)
SIMPLE_HAL(HAL_FDCAN_Start, FDCAN_HandleTypeDef)
SIMPLE_HAL(HAL_FDCAN_Stop, FDCAN_HandleTypeDef)
SIMPLE_HAL(HAL_UART_Init, UART_HandleTypeDef)
SIMPLE_HAL(HAL_UARTEx_DisableFifoMode, UART_HandleTypeDef)
SIMPLE_HAL(HAL_PCD_Init, PCD_HandleTypeDef)
SIMPLE_HAL(HAL_PCD_Start, PCD_HandleTypeDef)
HAL_StatusTypeDef HAL_IWDG_Init(IWDG_HandleTypeDef *h) { stub_iwdg_config = *h; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ConfigFilter(FDCAN_HandleTypeDef *h, FDCAN_FilterTypeDef *f) {
 unsigned b = (unsigned)(h->Instance - stub_can);
 assert(b < 3u && f->FilterIndex < 2u);
 stub_filters[b][f->FilterIndex] = *f; stub_filter_count[b]++;
 return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *h, uint32_t a, uint32_t b, uint32_t c, uint32_t d) { (void)h; (void)a; (void)b; (void)c; (void)d; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ConfigInterruptLines(FDCAN_HandleTypeDef *h, uint32_t a, uint32_t b) { (void)h; (void)a; (void)b; return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *h, uint32_t a, uint32_t b) { (void)h; (void)a; (void)b; return HAL_OK; }
static unsigned bus(FDCAN_HandleTypeDef *h) { return (unsigned)(h->Instance - stub_can); }
HAL_StatusTypeDef HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *h, uint32_t mask) {
 (void)h; (void)mask; assert(stub_primask == 1u); stub_abort_count++;
 /* Deliberately do not free FIFO here: real hardware abort is asynchronous. */
 return HAL_OK;
}
uint32_t HAL_FDCAN_GetTxFifoFreeLevel(FDCAN_HandleTypeDef *h) { return stub_free[bus(h)]; }
HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *h, FDCAN_TxHeaderTypeDef *hdr, uint8_t *data) {
 assert(stub_primask == 1u);
 unsigned b = bus(h);
 assert(stub_tx_count < 256u); assert(stub_free[b] > 0u);
 StubTx *r = &stub_tx[stub_tx_count++]; r->bus = (uint8_t)b; r->id = hdr->Identifier;
 memcpy(r->data, data, 8u); stub_free[b]--; return HAL_OK;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h, uint32_t fifo) { (void)fifo; return stub_rx_ready[bus(h)]; }
HAL_StatusTypeDef HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *h, uint32_t fifo, FDCAN_RxHeaderTypeDef *hdr, uint8_t *data) {
 (void)fifo; unsigned b = bus(h); *hdr = stub_rx_header[b]; memcpy(data, stub_rx_data[b], 8u); stub_rx_ready[b] = 0u; return HAL_OK;
}
HAL_StatusTypeDef HAL_FDCAN_GetErrorCounters(FDCAN_HandleTypeDef *h, FDCAN_ErrorCountersTypeDef *c) { (void)h; memset(c, 0, sizeof(*c)); return HAL_OK; }
HAL_StatusTypeDef HAL_FDCAN_GetProtocolStatus(FDCAN_HandleTypeDef *h, FDCAN_ProtocolStatusTypeDef *c) { (void)h; memset(c, 0, sizeof(*c)); return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_SetTxFifoThreshold(UART_HandleTypeDef *h, uint32_t n) { (void)h; (void)n; return HAL_OK; }
HAL_StatusTypeDef HAL_UARTEx_SetRxFifoThreshold(UART_HandleTypeDef *h, uint32_t n) { (void)h; (void)n; return HAL_OK; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *p, uint32_t n) { (void)h; (void)p; (void)n; return HAL_OK; }
void HAL_PWREx_EnableUSBVoltageDetector(void) { }
void HAL_PCDEx_SetRxFiFo(PCD_HandleTypeDef *h, uint32_t n) { (void)h; (void)n; }
void HAL_PCDEx_SetTxFiFo(PCD_HandleTypeDef *h, uint32_t n, uint32_t m) { (void)h; (void)n; (void)m; }
HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *h, uint8_t ep, uint32_t n, uint32_t type) { (void)h; (void)ep; (void)n; (void)type; return HAL_OK; }
HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *h, uint8_t ep, uint8_t *p, uint32_t n) { (void)h; (void)ep; (void)p; (void)n; return HAL_OK; }
HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *h, uint8_t ep, uint8_t *p, uint32_t n) {
 (void)h;
 if (ep == 0x81u) {
  assert(stub_primask == 1u);
  if (stub_usb_fail != 0u) { return HAL_ERROR; }
  stub_usb_active = p; stub_usb_len = n; stub_usb_calls++;
 }
 return HAL_OK;
}
HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *h, uint8_t addr) { (void)h; (void)addr; return HAL_OK; }
uint32_t HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef *h, uint8_t ep) { (void)h; (void)ep; return 0u; }
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *h) { (void)h; stub_iwdg_refreshes++; return HAL_OK; }
