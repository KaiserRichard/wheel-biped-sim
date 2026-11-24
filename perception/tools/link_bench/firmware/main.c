/* Nucleo-F446RE bench firmware for the CANDIDATE Pi<->MCU link (tools/link_proto).
 * DIAGNOSTIC stand-in for the STM32H7: link/protocol test only, no balance,
 * actuator or controller code.
 *
 * Ports (both 460800 8N1, both run the same bench logic independently):
 *   USART1 PA9 TX / PA10 RX  -> Pi GPIO15 RXD / GPIO14 TXD (primary path)
 *   USART2 PA2 TX / PA3 RX   -> ST-Link virtual COM port over USB (fallback path)
 * Timing marker: PA8 (Arduino D7) toggles once per USART1 packet that decodes
 *   (COBS + CRC + layout OK), for a logic analyzer. USART2 packets do not toggle it.
 * LD2 (PA5): on while USART1 or USART2 holds an accepted link, off in NO_LINK.
 * Advisory LEDs: PB5 green, PB4 yellow, PB10 red, PC7 blue (active high).
 * Clock: 84 MHz from the ST-Link 8 MHz MCO (HSE bypass), HSI 16 MHz fallback.
 * Time base: TIM2 32-bit free-running at 1 MHz, extended to 64 bit. */
#include "f446.h"
#include "bench.h"

#define BAUD 460800u
#define RING 1024u /* power of two */

typedef struct {
 uint32_t base;
 volatile uint8_t rx[RING]; volatile uint16_t rx_head, rx_tail;
 volatile uint8_t tx[RING]; volatile uint16_t tx_head, tx_tail;
 volatile uint32_t rx_overflow, ore, fe, ne;
 bench_port bench;
} uart;

static uart u1, u2;
static uint32_t hse_ok;

static void clock_init(void) {
 uint32_t i, src;
#ifdef BENCH_EMU
 return; /* QEMU has no RCC model: the PLL never reports ready there */
#endif
 RCC_CR |= 1u << 18; /* HSEBYP first: the ST-Link drives an 8 MHz MCO clock, not a crystal */
 RCC_CR |= 1u << 16; /* HSEON */
 for (i = 0; i < 200000u && !(RCC_CR & (1u << 17)); i++) {}
 hse_ok = (RCC_CR & (1u << 17)) != 0;
 if (!hse_ok) { RCC_CR &= ~(1u << 16); RCC_CR &= ~(1u << 18); }
 src = hse_ok ? 1u : 0u;
 /* VCO in = 1 MHz (M=8 HSE / M=16 HSI), VCO = 336 MHz, SYSCLK = /4 = 84 MHz. */
 RCC_PLLCFGR = (hse_ok ? 8u : 16u) | (336u << 6) | (1u << 16) | (src << 22) | (7u << 24) | (2u << 28);
 RCC_CR |= 1u << 24;
 while (!(RCC_CR & (1u << 25))) {}
 FLASH_ACR = 2u | (1u << 8) | (1u << 9) | (1u << 10); /* 2 WS, prefetch, I/D cache */
 RCC_CFGR = 4u << 10; /* AHB /1, APB1 /2 = 42 MHz (max 45), APB2 /1 = 84 MHz */
 RCC_CFGR |= 2u;      /* then switch SYSCLK to the PLL */
 while ((RCC_CFGR & (3u << 2)) != (2u << 2)) {}
}

static void gpio_af7(uint32_t pin) {
 GPIOA_MODER = (GPIOA_MODER & ~(3u << (2 * pin))) | (2u << (2 * pin));
 GPIOA_OSPEEDR |= 2u << (2 * pin);
 if (pin < 8) GPIOA_AFRL = (GPIOA_AFRL & ~(15u << (4 * pin))) | (7u << (4 * pin));
 else GPIOA_AFRH = (GPIOA_AFRH & ~(15u << (4 * (pin - 8)))) | (7u << (4 * (pin - 8)));
}

static void periph_init(void) {
 RCC_AHB1ENR |= 7u;                       /* GPIOA/B/C */
 RCC_APB1ENR |= (1u << 0) | (1u << 17) | (1u << 28); /* TIM2, USART2, PWR */
 RCC_APB2ENR |= 1u << 4;                  /* USART1 */
 (void)RCC_APB2ENR;
 gpio_af7(2); gpio_af7(3); gpio_af7(9); gpio_af7(10);
 GPIOA_PUPDR = (GPIOA_PUPDR & ~((3u << 6) | (3u << 20))) | (1u << 6) | (1u << 20); /* RX pull-ups: an unwired RX idles high */
 GPIOA_MODER = (GPIOA_MODER & ~(3u << 10)) | (1u << 10); /* PA5 LD2 output */
 GPIOA_MODER = (GPIOA_MODER & ~(3u << 16)) | (1u << 16); /* PA8 timing marker output */
 GPIOA_OSPEEDR |= 3u << 16;
 /* Drive LED latches low before enabling push-pull, low-speed outputs. */
 GPIOB_BSRR = (1u << (5 + 16)) | (1u << (4 + 16)) | (1u << (10 + 16));
 GPIOC_BSRR = 1u << (7 + 16);
 GPIOB_OTYPER &= ~((1u << 5) | (1u << 4) | (1u << 10));
 GPIOC_OTYPER &= ~(1u << 7);
 GPIOB_OSPEEDR &= ~((3u << 10) | (3u << 8) | (3u << 20));
 GPIOC_OSPEEDR &= ~(3u << 14);
 GPIOB_MODER = (GPIOB_MODER & ~((3u << 10) | (3u << 8) | (3u << 20))) |
               (1u << 10) | (1u << 8) | (1u << 20);
 GPIOC_MODER = (GPIOC_MODER & ~(3u << 14)) | (1u << 14);
 TIM2_PSC = 84u - 1u; TIM2_ARR = 0xFFFFFFFFu; TIM2_EGR = 1u; TIM2_CR1 = 1u; /* 84 MHz timer clock -> 1 MHz */
 USART_BRR(USART1_BASE) = (84000000u + BAUD / 2) / BAUD; /* 182 -> 461538 baud, +0.16 % */
 USART_BRR(USART2_BASE) = (42000000u + BAUD / 2) / BAUD; /* 91 -> 461538 baud, +0.16 % */
 USART_CR1(USART1_BASE) = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
 USART_CR1(USART2_BASE) = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
 NVIC_ISER1 = (1u << (37 - 32)) | (1u << (38 - 32));
}

/* MCU boot ID for the McuStatus header: chip UID mixed with a boot counter kept
 * in RTC backup register 0 (survives NRST while powered), so a reset is visible
 * to the Pi as a session change. */
static uint64_t boot_id(void) {
#ifdef BENCH_EMU
 return 0xE0u; /* QEMU netduinoplus2 (F405) has no UID/backup-register model */
#endif
 uint32_t uid = REG(UID_BASE) ^ REG(UID_BASE + 4) ^ REG(UID_BASE + 8), count;
 PWR_CR |= 1u << 8; /* DBP: backup domain write access */
 count = RTC_BKP0R + 1u; RTC_BKP0R = count;
 return ((uint64_t)uid << 32) | count;
}

static uint64_t micros(void) {
 static uint32_t last; static uint64_t high;
 uint32_t now = TIM2_CNT;
 if (now < last) high += 1ull << 32;
 last = now;
 return high | now;
}

static void uart_isr(uart *u) {
 uint32_t sr = USART_SR(u->base);
 if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NF | USART_SR_PE)) {
  uint8_t b = (uint8_t)USART_DR(u->base); /* SR then DR read clears ORE/FE/NF */
  uint16_t next = (uint16_t)((u->rx_head + 1u) & (RING - 1u));
  if (sr & USART_SR_ORE) u->ore++;
  if (sr & USART_SR_FE) u->fe++;
  if (sr & USART_SR_NF) u->ne++;
  if (next == u->rx_tail) u->rx_overflow++;
  else { u->rx[u->rx_head] = b; u->rx_head = next; }
 }
 if ((USART_CR1(u->base) & USART_CR1_TXEIE) && (sr & USART_SR_TXE)) {
  if (u->tx_tail == u->tx_head) USART_CR1(u->base) &= ~USART_CR1_TXEIE;
  else { USART_DR(u->base) = u->tx[u->tx_tail]; u->tx_tail = (uint16_t)((u->tx_tail + 1u) & (RING - 1u)); }
 }
}
void USART1_IRQHandler(void) { uart_isr(&u1); }
void USART2_IRQHandler(void) { uart_isr(&u2); }

/* Whole status records only: a record that does not fit is dropped and counted. */
static void uart_tx(void *ctx, const uint8_t *data, size_t n) {
 uart *u = ctx; size_t i;
 uint16_t free_bytes = (uint16_t)((u->tx_tail - u->tx_head - 1u) & (RING - 1u));
 if (n > free_bytes) { u->bench.tx_dropped++; return; }
 for (i = 0; i < n; i++) { u->tx[u->tx_head] = data[i]; u->tx_head = (uint16_t)((u->tx_head + 1u) & (RING - 1u)); }
 irq_off(); USART_CR1(u->base) |= USART_CR1_TXEIE; irq_on();
}

/* Single-cycle-ish edge on PA8; only the main loop writes GPIOA_ODR bit 8. */
static void mark(void *ctx) { (void)ctx; GPIOA_ODR ^= 1u << 8; }

static void service(uart *u, uint64_t now) {
 uint8_t chunk[64]; size_t n = 0;
 while (u->rx_tail != u->rx_head && n < sizeof(chunk)) {
  chunk[n++] = u->rx[u->rx_tail];
  u->rx_tail = (uint16_t)((u->rx_tail + 1u) & (RING - 1u));
 }
 if (n) bench_rx(&u->bench, chunk, n, now);
 bench_poll(&u->bench, now);
}

static void advisory_leds(uint8_t mask) {
 uint32_t bset = 0, breset = (1u << 5) | (1u << 4) | (1u << 10);
 uint32_t cset = 0, creset = 1u << 7;
 if (mask & (1u << 0)) bset |= 1u << 5, breset &= ~(1u << 5);
 if (mask & (1u << 1)) bset |= 1u << 4, breset &= ~(1u << 4);
 if (mask & (1u << 2)) bset |= 1u << 10, breset &= ~(1u << 10);
 if (mask & (1u << 3)) cset |= 1u << 7, creset = 0;
 GPIOB_BSRR = bset | (breset << 16);
 GPIOC_BSRR = cset | (creset << 16);
}

int main(void) {
 uint64_t id;
 u1.base = USART1_BASE; u2.base = USART2_BASE;
 clock_init();
 periph_init();
 id = boot_id();
 bench_init(&u1.bench, id, uart_tx, &u1);
 bench_init(&u2.bench, id, uart_tx, &u2);
 u1.bench.mark = mark;
 for (;;) {
  uint64_t now = micros();
  uint8_t led_mask;
  service(&u1, now);
  service(&u2, now);
  GPIOA_BSRR = (bench_linked(&u1.bench) || bench_linked(&u2.bench)) ? (1u << 5) : (1u << 21);
  led_mask = bench_led_mask(&u1.bench);
  if (!led_mask && ((now / 250000u) & 1u)) led_mask = 1u << 2;
  /* UNCERTAIN also blinks yellow at 4 Hz, for rigs without a blue LED on D9. */
  if (led_mask == (1u << 3) && ((now / 125000u) & 1u)) led_mask |= 1u << 1;
  advisory_leds(led_mask);
 }
}
