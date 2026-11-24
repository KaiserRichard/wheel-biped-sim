/* IMU + dual-CAN bench test for the Nucleo-F446RE (no micro-ROS, no motors).
 *
 * Output: text lines on USART1 PA9/PA10, 115200 8N1 (Pi /dev/ttyAMA0).
 *   BOOT/PIN/IMUID/LOOP/NORM  boot results, repeated every 5 s
 *   I,ms,fresh,ax,ay,az,temp,gx,gy,gz  raw imu_sensor sample (+-4 g, +-500 dps)
 *   STAT k=v ...               cumulative counters, once per second
 * Send "BOOTLOADER" on USART1 to system-reset; with BOOT0 fitted that lands in the ROM
 * bootloader, so several images can be flashed with the jumper left in place.
 *
 * Clock: HSI 16 MHz only (no HSE/PLL dependency). CAN_KBPS 500 (default) or 1000, 16 tq, SP 87.5 %.
 * CAN1 PB8 RX / PB9 TX, CAN2 PB12 RX / PB13 TX, I2C1 PB6 SCL / PB7 SDA.
 * LEDs: D4 PB5 green = IMU ok, D9 PC7 blue = CAN round trip ok,
 *       D6 PB10 red = any fault, D5 PB4 yellow = 1 Hz heartbeat. */
#include <stdint.h>

#define REG(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define RCC 0x40023800u
#define RCC_CR REG(RCC + 0x00u)
#define RCC_PLLCFGR REG(RCC + 0x04u)
#define RCC_CFGR REG(RCC + 0x08u)
#define RCC_AHB1RSTR REG(RCC + 0x10u)
#define RCC_APB1RSTR REG(RCC + 0x20u)
#define RCC_APB2RSTR REG(RCC + 0x24u)
#define RCC_AHB1ENR REG(RCC + 0x30u)
#define RCC_APB1ENR REG(RCC + 0x40u)
#define RCC_APB2ENR REG(RCC + 0x44u)
#define GPIOA 0x40020000u
#define GPIOB 0x40020400u
#define GPIOC 0x40020800u
#define MODER 0x00u
#define OTYPER 0x04u
#define OSPEEDR 0x08u
#define PUPDR 0x0Cu
#define IDR 0x10u
#define BSRR 0x18u
#define AFRL 0x20u
#define AFRH 0x24u
#define USART1 0x40011000u
#define U_SR REG(USART1 + 0x00u)
#define U_DR REG(USART1 + 0x04u)
#define U_BRR REG(USART1 + 0x08u)
#define U_CR1 REG(USART1 + 0x0Cu)
#define I2C1 0x40005400u
#define I_CR1 REG(I2C1 + 0x00u)
#define I_CR2 REG(I2C1 + 0x04u)
#define I_DR REG(I2C1 + 0x10u)
#define I_SR1 REG(I2C1 + 0x14u)
#define I_SR2 REG(I2C1 + 0x18u)
#define I_CCR REG(I2C1 + 0x1Cu)
#define I_TRISE REG(I2C1 + 0x20u)
#define CAN1 0x40006400u
#define CAN2 0x40006800u
#define C_MCR 0x00u
#define C_MSR 0x04u
#define C_TSR 0x08u
#define C_RF0R 0x0Cu
#define C_ESR 0x18u
#define C_BTR 0x1Cu
#define C_TIR 0x180u
#define C_TDTR 0x184u
#define C_TDLR 0x188u
#define C_TDHR 0x18Cu
#define C_RIR 0x1B0u
#define C_RDTR 0x1B4u
#define C_RDLR 0x1B8u
#define C_RDHR 0x1BCu
#define CF (CAN1 + 0x200u) /* shared filter block lives in CAN1 */
#define SYST_CSR REG(0xE000E010u)
#define SYST_RVR REG(0xE000E014u)
#define SYST_CVR REG(0xE000E018u)
#define DEMCR REG(0xE000EDFCu)
#define DWT_CTRL REG(0xE0001000u)
#define DWT_CYCCNT REG(0xE0001004u)
#define VTOR REG(0xE000ED08u)

#define SYSCLK 16000000u
#ifndef CAN_KBPS
#define CAN_KBPS 500u
#endif
/* 16 tq per bit (1+13+2): prescaler 16 MHz / (16 * kbit/s) */
#define CAN_BTR ((1u << 20) | (12u << 16) | (SYSCLK / (16000u * CAN_KBPS) - 1u))
#define ID_PING 0x101u
#define ID_ECHO 0x202u
#define ID_LOOP 0x111u

static volatile uint32_t ms;
void SysTick_Handler(void) { ms++; }

static void delay_us(uint32_t us) {
  uint32_t start = DWT_CYCCNT, n = us * (SYSCLK / 1000000u);
  while (DWT_CYCCNT - start < n) {}
}

/* ---- GPIO helpers ---- */
static void pin_mode(uint32_t port, unsigned pin, unsigned mode) {
  REG(port + MODER) = (REG(port + MODER) & ~(3u << (2u * pin))) | (mode << (2u * pin));
}
static void pin_pull(uint32_t port, unsigned pin, unsigned pull) {
  REG(port + PUPDR) = (REG(port + PUPDR) & ~(3u << (2u * pin))) | (pull << (2u * pin));
}
static void pin_af(uint32_t port, unsigned pin, unsigned af) {
  uint32_t r = port + (pin < 8u ? AFRL : AFRH), sh = 4u * (pin & 7u);
  REG(r) = (REG(r) & ~(15u << sh)) | (af << sh);
  pin_mode(port, pin, 2u);
}
static void pin_set(uint32_t port, unsigned pin, int high) {
  REG(port + BSRR) = high ? (1u << pin) : (1u << (pin + 16u));
}
static int pin_get(uint32_t port, unsigned pin) { return (int)((REG(port + IDR) >> pin) & 1u); }

/* ---- UART: ring buffer pumped from the main loop, never blocks the CAN service ---- */
#define TXN 4096u
static char txb[TXN];
static uint32_t txh, txt, tx_drop;
static void uart_pump(void) {
  while (txt != txh && (U_SR & (1u << 7))) { U_DR = (uint8_t)txb[txt]; txt = (txt + 1u) % TXN; }
}
static void putc_(char c) {
  uint32_t n = (txh + 1u) % TXN;
  if (n == txt) { tx_drop++; return; }
  txb[txh] = c; txh = n;
}
static void puts_(const char *s) { while (*s) putc_(*s++); }
static void putu(uint32_t v) {
  char b[11]; int i = 0;
  do { b[i++] = (char)('0' + v % 10u); v /= 10u; } while (v);
  while (i) putc_(b[--i]);
}
static void puti(int32_t v) { if (v < 0) { putc_('-'); putu((uint32_t)(-v)); } else putu((uint32_t)v); }
static void puthex(uint32_t v, int digits) {
  puts_("0x");
  for (int i = digits - 1; i >= 0; i--) putc_("0123456789abcdef"[(v >> (4 * i)) & 15u]);
}
static void kv(const char *k, uint32_t v) { putc_(' '); puts_(k); putc_('='); putu(v); }
static void kvh(const char *k, uint32_t v, int d) { putc_(' '); puts_(k); putc_('='); puthex(v, d); }
static void uart_flush(void) { while (txt != txh) uart_pump(); while (!(U_SR & (1u << 6))) {} }

/* ---- clocks ---- */
static void clock_init(void) {
  RCC_CR |= 1u; while (!(RCC_CR & 2u)) {}         /* HSI on */
  RCC_CFGR = 0u; while (RCC_CFGR & (3u << 2)) {}  /* SW = HSI, all prescalers /1 */
  RCC_CR &= ~((1u << 24) | (1u << 16));           /* PLL and HSE off (a ROM bootloader GO may leave them on) */
  SYST_RVR = SYSCLK / 1000u - 1u; SYST_CVR = 0; SYST_CSR = 7u;
  DEMCR |= 1u << 24; DWT_CYCCNT = 0; DWT_CTRL |= 1u;
}

/* ---- I2C1 / imu_sensor ---- */
static uint8_t imu_addr; /* 7-bit, 0 = none found */
static int i2c_wait(uint32_t mask) {
  uint32_t start = DWT_CYCCNT;
  while (!(I_SR1 & mask)) {
    if (I_SR1 & (1u << 10)) { I_SR1 = ~(1u << 10); return 0; } /* AF: NACK */
    if (DWT_CYCCNT - start > 2u * (SYSCLK / 1000u)) return 0;
  }
  return 1;
}
static void i2c_stop(void) { I_CR1 |= 1u << 9; }
/* Clock out a slave that holds SDA low (interrupted transfer), then send STOP. */
static void i2c_recover(void) {
  I_CR1 = 0;
  REG(GPIOB + OTYPER) |= (1u << 6) | (1u << 7);
  pin_set(GPIOB, 6, 1); pin_set(GPIOB, 7, 1);
  pin_mode(GPIOB, 6, 1u); pin_mode(GPIOB, 7, 1u);
  for (int i = 0; i < 9 && !pin_get(GPIOB, 7); i++) {
    pin_set(GPIOB, 6, 0); delay_us(10); pin_set(GPIOB, 6, 1); delay_us(10);
  }
  pin_set(GPIOB, 7, 0); delay_us(10); pin_set(GPIOB, 6, 1); delay_us(10); pin_set(GPIOB, 7, 1); delay_us(10);
}
static void i2c_init(void) {
  RCC_APB1ENR |= 1u << 21;
  i2c_recover();
  pin_pull(GPIOB, 6, 1u); pin_pull(GPIOB, 7, 1u); /* weak pull-up backs up the imu_sensor resistors */
  REG(GPIOB + OSPEEDR) |= (2u << 12) | (2u << 14);
  pin_af(GPIOB, 6, 4u); pin_af(GPIOB, 7, 4u);
  I_CR1 = 1u << 15; I_CR1 = 0;                    /* SWRST clears a stuck BUSY flag */
  I_CR2 = SYSCLK / 1000000u;
  I_CCR = SYSCLK / (2u * 100000u);                /* 100 kHz standard mode: forgiving on breadboard leads */
  I_TRISE = SYSCLK / 1000000u + 1u;
  I_CR1 = 1u;
}
static int i2c_start(uint8_t addr_rw) {
  I_CR1 |= 1u << 8;
  if (!i2c_wait(1u)) return 0;
  I_DR = addr_rw;
  if (!i2c_wait(1u << 1)) return 0;
  return 1;
}
static int i2c_write(uint8_t a, uint8_t reg, uint8_t val) {
  int ok = i2c_start((uint8_t)(a << 1)) && ((void)I_SR2, i2c_wait(1u << 7));
  if (ok) { I_DR = reg; ok = i2c_wait(1u << 7); }
  if (ok) { I_DR = val; ok = i2c_wait(1u << 2); }
  i2c_stop();
  return ok;
}
static int i2c_read(uint8_t a, uint8_t reg, uint8_t *d, unsigned n) {
  int ok = i2c_start((uint8_t)(a << 1)) && ((void)I_SR2, i2c_wait(1u << 7));
  if (ok) { I_DR = reg; ok = i2c_wait(1u << 2); }
  if (ok) { I_CR1 |= 1u << 10; ok = i2c_start((uint8_t)((a << 1) | 1u)); }
  if (!ok) { i2c_stop(); return 0; }
  if (n == 1u) I_CR1 &= ~(1u << 10);
  (void)I_SR2;
  for (unsigned i = 0; i < n; i++) {
    if (i == n - 1u) { I_CR1 &= ~(1u << 10); i2c_stop(); }
    if (!i2c_wait(1u << 6)) { i2c_stop(); return 0; }
    d[i] = (uint8_t)I_DR;
  }
  return 1;
}
static uint8_t imu_who;
static int imu_init(void) {
  static const uint8_t cand[2] = {0x68u, 0x69u};
  imu_addr = 0; imu_who = 0;
  for (unsigned k = 0; k < 2u; k++) {
    uint8_t w = 0;
    if (i2c_read(cand[k], 0x75u, &w, 1u)) { imu_addr = cand[k]; imu_who = w; break; }
  }
  if (!imu_addr || imu_who != 0x68u) return 0;
  uint8_t g = 0;
  int ok = i2c_write(imu_addr, 0x6Bu, 0x01u)    /* wake, PLL on gyro X */
        && i2c_write(imu_addr, 0x19u, 9u)       /* 1 kHz / (1+9) = 100 Hz */
        && i2c_write(imu_addr, 0x1Au, 0x03u)    /* DLPF 44 Hz */
        && i2c_write(imu_addr, 0x1Bu, 0x08u)    /* +-500 dps, 65.5 LSB/dps */
        && i2c_write(imu_addr, 0x1Cu, 0x08u)    /* +-4 g, 8192 LSB/g */
        && i2c_write(imu_addr, 0x38u, 0x01u)    /* DATA_RDY flag in INT_STATUS */
        && i2c_read(imu_addr, 0x1Bu, &g, 1u);
  return ok && g == 0x08u;
}

/* ---- pin-level checks before any peripheral owns the pins ---- */
static int pin_res[8];
static void pin_checks(void) {
  /* RX pins with pull-down: reading 1 means the transceiver RO actively drives recessive. */
  pin_mode(GPIOB, 8, 0u); pin_mode(GPIOB, 12, 0u); pin_pull(GPIOB, 8, 2u); pin_pull(GPIOB, 12, 2u);
  pin_set(GPIOB, 9, 1); pin_set(GPIOB, 13, 1); pin_mode(GPIOB, 9, 1u); pin_mode(GPIOB, 13, 1u);
  pin_mode(GPIOB, 6, 0u); pin_mode(GPIOB, 7, 0u); pin_pull(GPIOB, 6, 2u); pin_pull(GPIOB, 7, 2u);
  delay_us(200);
  pin_res[0] = pin_get(GPIOB, 8); pin_res[1] = pin_get(GPIOB, 12);
  /* Short dominant pulse on each TXD: own RX and the other RX must follow if the buses are joined. */
  pin_set(GPIOB, 9, 0); delay_us(20); pin_res[2] = pin_get(GPIOB, 8); pin_res[3] = pin_get(GPIOB, 12);
  pin_set(GPIOB, 9, 1); delay_us(50);
  pin_set(GPIOB, 13, 0); delay_us(20); pin_res[4] = pin_get(GPIOB, 12); pin_res[5] = pin_get(GPIOB, 8);
  pin_set(GPIOB, 13, 1); delay_us(50);
  pin_res[6] = pin_get(GPIOB, 6); pin_res[7] = pin_get(GPIOB, 7); /* imu_sensor pull-ups beat the 40k pull-down */
  pin_pull(GPIOB, 8, 0u); pin_pull(GPIOB, 12, 0u); pin_pull(GPIOB, 6, 0u); pin_pull(GPIOB, 7, 0u);
}

/* ---- bxCAN ---- */
static int can_wait_msr(uint32_t b, uint32_t mask, uint32_t want) {
  uint32_t t0 = ms;
  while ((REG(b + C_MSR) & mask) != want) if (ms - t0 > 50u) return 0;
  return 1;
}
/* Returns 1 when the controller left init mode; leaving needs 11 recessive bits on RX. */
static int can_config(uint32_t b, uint32_t btr) {
  REG(b + C_MCR) = 1u;                              /* INRQ, clears SLEEP */
  if (!can_wait_msr(b, 3u, 1u)) return 0;
  REG(b + C_MCR) = 1u | (1u << 6) | (1u << 2);      /* INRQ + ABOM + TXFP (FIFO order) */
  REG(b + C_BTR) = btr;
  REG(b + C_MCR) &= ~1u;
  return can_wait_msr(b, 1u, 0u);
}
static void can_filters(void) {
  REG(CF + 0x00u) = (REG(CF + 0x00u) & ~(0x3fu << 8)) | (14u << 8) | 1u; /* FINIT, CAN2SB = 14 */
  static const unsigned banks[2] = {0u, 14u};
  for (unsigned i = 0; i < 2u; i++) {
    unsigned k = banks[i];
    REG(CF + 0x1Cu) &= ~(1u << k);
    REG(CF + 0x04u) &= ~(1u << k);  /* mask mode */
    REG(CF + 0x0Cu) |= 1u << k;     /* 32-bit */
    REG(CF + 0x14u) &= ~(1u << k);  /* FIFO0 */
    REG(CF + 0x40u + 8u * k) = 0; REG(CF + 0x44u + 8u * k) = 0; /* accept all */
    REG(CF + 0x1Cu) |= 1u << k;
  }
  REG(CF + 0x00u) &= ~1u;
}
static void can_pins(void) {
  static const unsigned p[4] = {8u, 9u, 12u, 13u};
  for (unsigned i = 0; i < 4u; i++) {
    REG(GPIOB + OSPEEDR) |= 3u << (2u * p[i]);
    pin_pull(GPIOB, p[i], 0u);
    pin_af(GPIOB, p[i], 9u);
  }
}
static int can_send(uint32_t b, uint32_t id, const uint8_t d[8]) {
  uint32_t tsr = REG(b + C_TSR); unsigned m;
  for (m = 0; m < 3u; m++) if (tsr & (1u << (26u + m))) break;
  if (m == 3u) return 0;
  uint32_t o = 0x10u * m;
  REG(b + C_TIR + o) = id << 21;
  REG(b + C_TDTR + o) = 8u;
  REG(b + C_TDLR + o) = d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
  REG(b + C_TDHR + o) = d[4] | ((uint32_t)d[5] << 8) | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
  REG(b + C_TIR + o) |= 1u;
  return 1;
}
static int can_recv(uint32_t b, uint32_t *id, uint8_t d[8], uint32_t *ovr) {
  uint32_t rf = REG(b + C_RF0R);
  if (rf & (1u << 4)) { (*ovr)++; REG(b + C_RF0R) = 1u << 4; }
  if (!(rf & 3u)) return 0;
  uint32_t rir = REG(b + C_RIR), lo = REG(b + C_RDLR), hi = REG(b + C_RDHR);
  *id = (rir & 4u) ? (rir >> 3) | 0x80000000u : rir >> 21; /* bit31 marks an extended frame */
  for (unsigned i = 0; i < 4u; i++) { d[i] = (uint8_t)(lo >> (8u * i)); d[i + 4u] = (uint8_t)(hi >> (8u * i)); }
  REG(b + C_RF0R) = 1u << 5;
  return 1;
}

typedef struct { uint32_t req, busy, txok, txerr, rx, ovr, boff_n, other; int boff_prev; } canstat_t;
static canstat_t c1, c2;
static void can_completions(uint32_t b, canstat_t *s) {
  uint32_t tsr = REG(b + C_TSR);
  for (unsigned m = 0; m < 3u; m++) {
    uint32_t sh = 8u * m;
    if (tsr & (1u << sh)) {
      if (tsr & (2u << sh)) s->txok++; else s->txerr++;
      REG(b + C_TSR) = 1u << sh;
    }
  }
  int boff = (REG(b + C_ESR) >> 2) & 1u;
  if (boff && !s->boff_prev) s->boff_n++;
  s->boff_prev = boff;
}

/* Internal loopback + silent: proves controller, filters and timing without the wiring. */
static uint32_t loop_tx[2], loop_rx[2];
static void can_loopback_test(void) {
  static const uint32_t bases[2] = {CAN1, CAN2};
  for (unsigned k = 0; k < 2u; k++) {
    uint32_t b = bases[k], ovr = 0;
    if (!can_config(b, CAN_BTR | (1u << 30) | (1u << 31))) continue;
    for (uint32_t n = 0; n < 10u; n++) {
      uint8_t d[8] = {(uint8_t)n, (uint8_t)k, 0x55, 0xAA, 0, 0, 0, 0};
      if (!can_send(b, ID_LOOP, d)) continue;
      loop_tx[k]++;
      uint32_t t0 = ms, id; uint8_t r[8];
      while (ms - t0 < 5u) {
        if (can_recv(b, &id, r, &ovr)) { if (id == ID_LOOP && r[0] == (uint8_t)n && r[1] == (uint8_t)k) loop_rx[k]++; break; }
      }
    }
    REG(b + C_TSR) = 0x00010101u; /* clear completion flags */
  }
}

/* ---- reset for reflashing ----
 * A direct jump into the ROM bootloader stopped the app but the bootloader never answered
 * on USART1 (29 Oct), so reset instead: with BOOT0 fitted the ROM bootloader starts from a
 * clean reset state. */
static void reset_to_bootloader(void) {
  puts_("RESET for bootloader (needs BOOT0 fitted)\n"); uart_flush();
  REG(0xE000ED0Cu) = 0x05FA0004u; /* SCB AIRCR: SYSRESETREQ */
  for (;;) {}
}
static void rx_poll(void) {
  static const char magic[] = "BOOTLOADER";
  static unsigned pos;
  uint32_t sr = U_SR;
  if (!(sr & ((1u << 5) | (1u << 3)))) return;
  char c = (char)U_DR;
  if (c == magic[pos]) { if (magic[++pos] == '\0') reset_to_bootloader(); }
  else pos = (c == magic[0]) ? 1u : 0u;
}

static void leds(int imu_ok, int can_ok, int fault) {
  pin_set(GPIOB, 5, imu_ok); pin_set(GPIOC, 7, can_ok); pin_set(GPIOB, 10, fault);
  pin_set(GPIOB, 4, (ms / 500u) & 1u);
}

static int c1_init_ok, c2_init_ok, imu_up;
static void print_boot(void) {
  puts_("BOOT fw=imu_can_bench ver=3"); kv("sysclk", SYSCLK); kv("can_bps", CAN_KBPS * 1000u); kv("imu_hz", 100u); putc_('\n');
  puts_("PIN"); kv("c1rx_idle", (uint32_t)pin_res[0]); kv("c2rx_idle", (uint32_t)pin_res[1]);
  kv("c1tx_c1rx", (uint32_t)pin_res[2]); kv("c1tx_c2rx", (uint32_t)pin_res[3]);
  kv("c2tx_c2rx", (uint32_t)pin_res[4]); kv("c2tx_c1rx", (uint32_t)pin_res[5]);
  kv("scl", (uint32_t)pin_res[6]); kv("sda", (uint32_t)pin_res[7]); putc_('\n');
  puts_("IMUID"); kvh("addr", imu_addr, 2); kvh("who", imu_who, 2); kv("init", (uint32_t)imu_up); putc_('\n');
  puts_("LOOP"); kv("c1_tx", loop_tx[0]); kv("c1_rx", loop_rx[0]); kv("c2_tx", loop_tx[1]); kv("c2_rx", loop_rx[1]); putc_('\n');
  puts_("NORM"); kv("c1_init", (uint32_t)c1_init_ok); kv("c2_init", (uint32_t)c2_init_ok); putc_('\n');
}

int main(void) {
  clock_init();
  RCC_AHB1ENR |= 7u;
  RCC_APB2ENR |= 1u << 4;
  pin_af(GPIOA, 9, 7u); pin_af(GPIOA, 10, 7u);
  U_BRR = (SYSCLK + 57600u) / 115200u;
  U_CR1 = (1u << 13) | (1u << 3) | (1u << 2);
  static const unsigned bled[3] = {4u, 5u, 10u};
  for (unsigned i = 0; i < 3u; i++) { pin_set(GPIOB, bled[i], 0); pin_mode(GPIOB, bled[i], 1u); }
  pin_set(GPIOC, 7, 0); pin_mode(GPIOC, 7, 1u);

  pin_checks();
  i2c_init(); imu_up = imu_init();
  RCC_APB1ENR |= (1u << 25) | (1u << 26); (void)RCC_APB1ENR; /* CAN2 needs the CAN1 clock */
  can_filters(); can_pins();
  can_loopback_test();
  c1_init_ok = can_config(CAN1, CAN_BTR);
  c2_init_ok = can_config(CAN2, CAN_BTR);
  print_boot();

  uint32_t next_poll = ms, next_ping = ms, next_stat = ms + 1000u, next_boot = ms + 5000u, next_retry = 0;
  uint32_t imu_ok = 0, imu_err = 0, imu_stale = 0, imu_reinit = 0, imu_last_ok = 0;
  uint32_t seq = 0, match = 0, bad = 0, last_match = 0, rt_last_ms = 0;
  for (;;) {
    uart_pump(); rx_poll();
    uint32_t now = ms, id; uint8_t d[8];

    /* CAN2 echoes every ping back to CAN1 on the shared bus. */
    while (can_recv(CAN2, &id, d, &c2.ovr)) {
      c2.rx++;
      if (id == ID_PING) { c2.req++; if (!can_send(CAN2, ID_ECHO, d)) c2.busy++; } else c2.other++;
    }
    while (can_recv(CAN1, &id, d, &c1.ovr)) {
      c1.rx++;
      uint32_t n = d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
      uint32_t inv = d[4] | ((uint32_t)d[5] << 8) | ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);
      if (id != ID_ECHO) c1.other++;
      else if (inv == ~n && n > last_match && n <= seq) { match++; last_match = n; rt_last_ms = now; }
      else bad++;
    }
    can_completions(CAN1, &c1); can_completions(CAN2, &c2);

    if ((int32_t)(now - next_ping) >= 0) {
      next_ping += 10u;
      uint32_t n = seq + 1u, inv = ~n;
      uint8_t p[8] = {(uint8_t)n, (uint8_t)(n >> 8), (uint8_t)(n >> 16), (uint8_t)(n >> 24),
                      (uint8_t)inv, (uint8_t)(inv >> 8), (uint8_t)(inv >> 16), (uint8_t)(inv >> 24)};
      c1.req++;
      if (can_send(CAN1, ID_PING, p)) seq = n; else c1.busy++;
    }

    if ((int32_t)(now - next_poll) >= 0) {
      next_poll += 5u; /* 200 Hz poll of DATA_RDY; the sensor itself runs at 100 Hz */
      if (imu_up) {
        uint8_t b[15];
        int ok = i2c_read(imu_addr, 0x3Au, b, 15u);
        int flat = 1;
        for (unsigned i = 2; i < 15u; i++) if (b[i] != b[1]) flat = 0;
        if (ok && flat && (b[1] == 0x00u || b[1] == 0xFFu)) ok = 0; /* stuck bus / unpowered sensor */
        if (!ok) { imu_err++; imu_up = 0; next_retry = now + 500u; }
        else if (!(b[0] & 1u)) imu_stale++;
        else {
          imu_ok++; imu_last_ok = now;
          putc_('I'); putc_(','); putu(now); putc_(','); putu(1u);
          for (unsigned i = 0; i < 7u; i++) { putc_(','); puti((int16_t)((uint16_t)b[1 + 2 * i] << 8 | b[2 + 2 * i])); }
          putc_('\n');
        }
      } else if ((int32_t)(now - next_retry) >= 0) {
        imu_reinit++; next_retry = now + 500u;
        i2c_init(); imu_up = imu_init();
      }
    }

    if ((int32_t)(now - next_stat) >= 0) {
      next_stat += 1000u;
      uint32_t e1 = REG(CAN1 + C_ESR), e2 = REG(CAN2 + C_ESR);
      puts_("STAT"); kv("ms", now);
      kv("imu_ok", imu_ok); kv("imu_err", imu_err); kv("imu_stale", imu_stale); kv("imu_reinit", imu_reinit); kv("imu_up", (uint32_t)imu_up);
      kv("c1_req", c1.req); kv("c1_busy", c1.busy); kv("c1_txok", c1.txok); kv("c1_txerr", c1.txerr);
      kv("c1_rx", c1.rx); kv("c1_ovr", c1.ovr); kv("c1_other", c1.other); kv("c1_boff_n", c1.boff_n); kvh("c1_esr", e1, 8);
      kv("c2_rx", c2.rx); kv("c2_req", c2.req); kv("c2_busy", c2.busy); kv("c2_txok", c2.txok); kv("c2_txerr", c2.txerr);
      kv("c2_ovr", c2.ovr); kv("c2_other", c2.other); kv("c2_boff_n", c2.boff_n); kvh("c2_esr", e2, 8);
      kv("sent", seq); kv("match", match); kv("bad", bad); kv("uart_drop", tx_drop); putc_('\n');
      int imu_good = imu_up && now - imu_last_ok < 1000u, can_good = rt_last_ms && now - rt_last_ms < 1000u;
      leds(imu_good, can_good, !imu_good || !can_good);
    }
    if ((int32_t)(now - next_boot) >= 0) { next_boot += 5000u; print_boot(); }
    pin_set(GPIOB, 4, (now / 500u) & 1u);
  }
}

/* ---- startup ---- */
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss, _estack;
void Reset_Handler(void);
static void default_handler(void) { for (;;) {} }
__attribute__((section(".isr_vector"), used))
static void (*const vectors[16 + 97])(void) = {
  [0] = (void (*)(void))&_estack,
  [1] = Reset_Handler,
  [2 ... 14] = default_handler,
  [15] = SysTick_Handler,
  [16 ... 16 + 96] = default_handler,
};
void Reset_Handler(void) {
  VTOR = 0x08000000u;
  /* Started by the ROM bootloader's GO command, the core keeps PRIMASK set (SysTick would
   * never tick, so every ms-based wait hangs) and USART1/CAN keep its configuration.
   * Reset those peripherals and enable interrupts so GO behaves like a power-on reset. */
  RCC_AHB1RSTR = 7u; RCC_AHB1RSTR = 0;
  RCC_APB1RSTR = (1u << 21) | (1u << 25) | (1u << 26); RCC_APB1RSTR = 0;
  RCC_APB2RSTR = 1u << 4; RCC_APB2RSTR = 0;
  SYST_CSR = 0;
  __asm volatile("cpsie i" ::: "memory");
  uint32_t *s = &_sidata, *d = &_sdata;
  while (d < &_edata) *d++ = *s++;
  for (d = &_sbss; d < &_ebss;) *d++ = 0;
  (void)main();
  for (;;) {}
}
