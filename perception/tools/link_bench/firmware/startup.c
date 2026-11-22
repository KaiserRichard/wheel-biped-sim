/* Minimal Cortex-M4 startup for the F446 bench firmware: vector table,
 * .data/.bss init, then main(). Unused IRQs fall into default_handler. */
#include <stddef.h>
#include <stdint.h>
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
int main(void);
void Reset_Handler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
static void default_handler(void) { for (;;) {} }
#define IRQS 97
__attribute__((section(".isr_vector"), used))
static void (*const vectors[16 + IRQS])(void) = {
 [0] = (void (*)(void))&_estack,
 [1] = Reset_Handler,
 [2 ... 15] = default_handler,
 [16 ... 16 + 36] = default_handler,
 [16 + 37] = USART1_IRQHandler,
 [16 + 38] = USART2_IRQHandler,
 [16 + 39 ... 16 + IRQS - 1] = default_handler,
};
void Reset_Handler(void) {
#ifndef BENCH_EMU
 /* The ROM bootloader's GO command does not relocate the vector table. Set
  * VTOR before C initialization or any peripheral can enable an interrupt. */
 *(volatile uint32_t *)0xE000ED08u = 0x08000000u;
 __asm volatile ("dsb" ::: "memory");
 __asm volatile ("isb" ::: "memory");
#endif
 uint32_t *s = &_sidata, *d = &_sdata;
 while (d < &_edata) *d++ = *s++;
 for (d = &_sbss; d < &_ebss; ) *d++ = 0;
 main();
 for (;;) {}
}
/* The codec and bench code use memset/memcpy; no C library is linked. */
void *memset(void *p, int v, size_t n) { uint8_t *b = p; while (n--) *b++ = (uint8_t)v; return p; }
void *memcpy(void *d, const void *s, size_t n) { uint8_t *o = d; const uint8_t *i = s; while (n--) *o++ = *i++; return d; }
