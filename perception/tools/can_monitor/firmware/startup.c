#include <stddef.h>
#include <stdint.h>
#include "f446.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
extern void __libc_init_array(void);
int main(void);
void Reset_Handler(void);
void USART2_IRQHandler(void);
void _init(void) {}
static void default_handler(void) { for (;;) { __asm volatile("wfi"); } }

__attribute__((section(".isr_vector"), used))
static void (*const vectors[16 + 97])(void) = {
  [0] = (void (*)(void))&_estack,
  [1] = Reset_Handler,
  [2 ... 15] = default_handler,
  [16 ... 16 + 36] = default_handler,
#ifdef UROS_USART1
  [16 + 37] = USART2_IRQHandler, /* same RX handler, USART1 vector */
  [16 + 38] = default_handler,
#else
  [16 + 37] = default_handler,
  [16 + 38] = USART2_IRQHandler,
#endif
  [16 + 39 ... 16 + 96] = default_handler,
};

void Reset_Handler(void) {
  *(volatile uint32_t *)0xE000ED08u = 0x08000000u; /* VTOR before any IRQ */
  __asm volatile("dsb" ::: "memory");
  __asm volatile("isb" ::: "memory");
  REG(0xE000ED88u) |= (0xFu << 20); /* CP10/CP11: Cortex-M4F FPU */
  __asm volatile("dsb" ::: "memory");
  __asm volatile("isb" ::: "memory");
  uint32_t *src = &_sidata, *dst = &_sdata;
  while (dst < &_edata) *dst++ = *src++;
  for (dst = &_sbss; dst < &_ebss;) *dst++ = 0;
  __libc_init_array();
  (void)main();
  for (;;) {}
}
