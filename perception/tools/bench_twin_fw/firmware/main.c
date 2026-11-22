/* F446 bench-only DM twin. Derived GPIO/I2C/startup from imu_can_bench. */
#include <stdint.h>
#include "twin.h"
#include "uart_queue.h"
#ifndef M_HZ
#define M_HZ 20u /* per motor; 0 disables M telemetry */
#endif
#if M_HZ < 0 || M_HZ > 1000
#error M_HZ must be 0..1000
#endif
#include <stddef.h>

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
#define CAN_KBPS 500u /* ASSUMED bench-only, never a robot bus setting */
/* 16 tq per bit (1+13+2): prescaler 16 MHz / (16 * kbit/s) */
#define CAN_BTR ((1u << 20) | (12u << 16) | (SYSCLK / (16000u * CAN_KBPS) - 1u))

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
static char txb[TXN];
static uint32_t txh, txt, tx_drop, tx_byte_drop, stat_defer;
static void uart_pump(void) {
  while (txt != txh && (U_SR & (1u << 7))) { U_DR = (uint8_t)txb[txt]; txt = (txt + 1u) % TXN; }
}
static void putc_(char c) {
  uint32_t n = (txh + 1u) % TXN;
  if (n == txt) { tx_byte_drop++; return; }
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
/* Atomic record admission: never truncate a text line or binary envelope. */
static int reserve(unsigned n) {
  if (uart_tx_fits(txh,txt,n,0)) return 1;
  tx_drop++; return 0;
}
static void tx_record(void *ctx, const uint8_t *d, size_t n) {
  (void)ctx;
  if (reserve((unsigned)n)) for(size_t i=0;i<n;i++) putc_((char)d[i]);
}
static uart_rx rxq;
void USART1_IRQHandler(void) {
  uint32_t sr=U_SR;
  if(sr & ((1u<<5)|15u)) {
    uint8_t b=(uint8_t)U_DR; /* SR then DR clears ORE/FE/NE/PE */
    uart_rx_push(&rxq,b,(sr&15u)!=0);
  }
}

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
  return (int)m+1;
}
static int can_recv(uint32_t b, uint32_t *id, uint8_t d[8], uint32_t *ovr) {
  uint32_t rf = REG(b + C_RF0R);
  if (rf & (1u << 4)) { (*ovr)++; REG(b + C_RF0R) = 1u << 4; }
  if (!(rf & 3u)) return 0;
  uint32_t rir = REG(b + C_RIR), lo = REG(b + C_RDLR), hi = REG(b + C_RDHR);
  int valid=!(rir & (4u|2u)) && (REG(b+C_RDTR)&15u)==8u;
  *id = valid ? rir >> 21 : 0xffffffffu;
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


/* ASSUMED one outstanding command per ID, 100ms timeout. DM has no sequence field.
 * Latency uses unsigned DWT cycle differences /16; no cross-wrap division. */
typedef struct { uint32_t cycles, at, seq; uint8_t active, mailbox; } pending_t;
static pending_t pending[4];
static dm_motor motors[4];
static twin_link link;
static uint32_t sent, fed_back, lost, rejected, unsolicited, fb_busy;
static uint32_t latency_last, latency_max, latency_sum, latency_n, model_skip;
static uint32_t last_seq, have_seq;
static void host_command(void *ctx,const twin_command *c) {
  (void)ctx; int k=dm_index(c->id); uint8_t d[8];
  if(k<0 || pending[k].active || (have_seq && !lp_seq_newer(c->seq,last_seq))) { rejected++; return; }
  if(c->special) dm_special(c->special,d); else dm_pack(c->id,&c->cmd,d);
  uint32_t start=DWT_CYCCNT;
  int mailbox=can_send(CAN1,DM_MIT+c->id,d);
  if(!mailbox) { c1.busy++; rejected++; return; }
  pending[k]=(pending_t){start,ms,c->seq,1,(uint8_t)(mailbox-1)};
  last_seq=c->seq; have_seq=1; sent++;
}
static void leds(void) {
  uint8_t b=twin_leds(&link);
  pin_set(GPIOB,5,b&1); pin_set(GPIOB,4,b&2);
  pin_set(GPIOB,10,b&4); pin_set(GPIOC,7,b&8);
}
int main(void) {
  clock_init(); RCC_AHB1ENR|=7u; RCC_APB2ENR|=1u<<4;
  pin_af(GPIOA,9,7u); pin_af(GPIOA,10,7u);
  U_BRR=(SYSCLK+230400u)/460800u; /* 8N1; ROM flash_safe stays 115200 8E1 */
  U_CR1=(1u<<13)|(1u<<3)|(1u<<2)|(1u<<5); /* RXNE interrupt */
  REG(0xe000e104u)=1u<<(37u-32u); /* NVIC ISER1 USART1 */
  static const unsigned pins[3]={4,5,10};
  for(unsigned i=0;i<3;i++) { pin_set(GPIOB,pins[i],0); pin_mode(GPIOB,pins[i],1); }
  pin_set(GPIOC,7,0); pin_mode(GPIOC,7,1);
  /* ASSUMED diagnostic session ID derived from UID; not a unique boot epoch.
   * Host must reset the decoder and sequence state at every MCU reset. */
  uint64_t boot=((uint64_t)REG(0x1fff7a10u)<<32)^REG(0x1fff7a14u)^REG(0x1fff7a18u);
  twin_init(&link,boot,tx_record,host_command,0);
  for(int k=0;k<4;k++) dm_motor_init(&motors[k],dm_ids[k]);
  i2c_init(); int imu_up=imu_init();
  RCC_APB1ENR|=(1u<<25)|(1u<<26); (void)RCC_APB1ENR;
  can_filters(); can_pins();
  int c1_ok=can_config(CAN1,CAN_BTR), c2_ok=can_config(CAN2,CAN_BTR);
  puts_("BOOT fw=bench_twin_dm bench_only=1 not_micro_ros=1 not_agv_ready=1");
  kv("uart_bps",460800); kv("m_hz",M_HZ); kv("can_bps",500000); kv("imu_hz",100); kv("c1_init",c1_ok); kv("c2_init",c2_ok);
  kvh("imu_addr",imu_addr,2); kvh("imu_who",imu_who,2); putc_('\n');
  uint32_t next_poll=ms, next_retry=0, next_stat=ms+1000, step_at=ms;
  uint32_t imu_ok=0,imu_err=0,imu_stale=0, m_decimated=0, m_format_drop=0;
  uint32_t m_at[4]={0}, m_have[4]={0};
  for(;;) {
    uart_pump();
    uint16_t byte;
    /* Bound RX work so a continuous sender cannot starve CAN, TX or STAT. */
    for(unsigned n=0;n<256u && uart_rx_pop(&rxq,&byte);n++) {
      if(byte&256u) twin_rx_gap(&link);
      twin_feed(&link,(uint8_t)byte,ms);
    }
    uint32_t now=ms;
    twin_poll(&link,now); leds();
    unsigned steps=0;
    while((uint32_t)(now-step_at)>0 && steps<10) {
      step_at++; steps++;
      for(int k=0;k<4;k++) dm_motor_step(&motors[k],.001f);
    }
    if(now!=step_at) { model_skip+=now-step_at; step_at=now; }
    uint32_t id; uint8_t d[8];
    while(can_recv(CAN2,&id,d,&c2.ovr)) {
      c2.rx++; int k=dm_index(id);
      if(k<0) { c2.other++; continue; }
      dm_motor_receive(&motors[k],d);
      dm_motor_step(&motors[k],0); /* evaluate new MIT torque before replying */
      dm_feedback f; dm_motor_feedback(&motors[k],&f); dm_pack_feedback(&f,d);
      /* ASSUMED feedback arbitration IDs: reference supplies payload but no reply-ID configuration. */
      if(!can_send(CAN2,TWIN_FEEDBACK_BASE+id,d)) fb_busy++;
    }
    while(can_recv(CAN1,&id,d,&c1.ovr)) {
      uint32_t received=DWT_CYCCNT; c1.rx++;
      dm_feedback f; dm_unpack_feedback(d,&f); int k=dm_index(f.id);
      if(k<0 || id!=TWIN_FEEDBACK_BASE+f.id) { c1.other++; continue; }
      if(pending[k].active && (uint32_t)(now-pending[k].at)<100u) {
        latency_last=(received-pending[k].cycles)/(SYSCLK/1000000u);
        if(latency_last>latency_max) latency_max=latency_last;
        latency_sum+=latency_last; latency_n++; pending[k].active=0; fed_back++;
      } else unsolicited++;
      if(M_HZ && (!m_have[k] || (uint32_t)(now-m_at[k]) >= (1000u+M_HZ-1u)/(M_HZ?M_HZ:1u))) {
        m_have[k]=1; m_at[k]=now;
        char line[80]; size_t n=twin_format(line,sizeof(line),now,&f);
        if(n) tx_record(0,(const uint8_t *)line,n); else m_format_drop++;
      } else m_decimated++;
    }
    for(int k=0;k<4;k++) if(pending[k].active && (uint32_t)(now-pending[k].at)>=100u) {
      /* The mailbox may already have completed and been reused for another ID.
       * Abort only a still-requested transmission belonging to this motor. */
      uint32_t tir=REG(CAN1+C_TIR+0x10u*pending[k].mailbox);
      if((tir&1u) && (tir>>21)==dm_ids[k])
        REG(CAN1+C_TSR)=1u<<(7u+8u*pending[k].mailbox);
      pending[k].active=0; lost++;
    }
    can_completions(CAN1,&c1); can_completions(CAN2,&c2);
    if((int32_t)(now-next_poll)>=0) {
      next_poll=now+5u; /* SOURCE-SUPPORTED imu_can_bench: 200Hz polls, 100Hz sensor */
      if(imu_up) {
        uint8_t b[15]={0}; int ok=i2c_read(imu_addr,0x3a,b,15),flat=1;
        for(unsigned i=2;i<15;i++) if(b[i]!=b[1]) flat=0;
        if(ok && flat && (b[1]==0 || b[1]==255)) ok=0;
        if(!ok) { imu_err++; imu_up=0; next_retry=now+500; }
        else if(!(b[0]&1)) imu_stale++;
        else {
          imu_ok++;
          if(reserve(64)) {
            putc_('I'); putc_(','); putu(now); putc_(','); putu(1);
            for(unsigned i=0;i<7;i++) { putc_(','); puti((int16_t)((uint16_t)b[1+2*i]<<8|b[2+2*i])); }
            putc_('\n');
          }
        }
      } else if((int32_t)(now-next_retry)>=0) { next_retry=now+500; i2c_init(); imu_up=imu_init(); }
    }
    if((int32_t)(now-next_stat)>=0) {
      /* Ordinary records always leave this space available. Retain a due
       * STAT if TX is physically stalled; never advance its deadline on failure. */
      if(uart_tx_fits(txh,txt,STAT_RESERVE,1)) {
        next_stat=now+1000;
        puts_("STAT"); kv("ms",now); kv("sent",sent); kv("fed_back",fed_back); kv("lost",lost);
        kv("pending",sent-fed_back-lost); kv("rejected",rejected); kv("unsolicited",unsolicited);
        kv("lat_last_us",latency_last); kv("lat_mean_us",latency_n?latency_sum/latency_n:0);
        kv("lat_max_us",latency_max); kv("lat_n",latency_n);
        kv("imu_ok",imu_ok); kv("imu_err",imu_err); kv("imu_stale",imu_stale);
        kv("uart_drop",tx_drop); kv("tx_byte_drop",tx_byte_drop); kv("stat_defer",stat_defer);
        kv("rx_drop",rxq.drop); kv("rx_error",rxq.error);
        kv("rx_bytes",rxq.bytes); kv("rx_queued",rxq.queued); kv("rx_processed",rxq.processed);
        kv("text_ok",link.text_ok); kv("rx_gap",link.rx_gap); kv("mux_timeout",link.mux_timeout);
        kv("binary_bytes",link.binary_bytes); kv("binary_overflow",link.binary_overflow);
        kv("echo_encode_drop",link.echo_encode_drop); kv("text_overflow",link.text_overflow);
        kv("m_decimated",m_decimated); kv("m_format_drop",m_format_drop);
        kv("binary_other",link.other); kv("c1_other",c1.other); kv("c2_other",c2.other);
        kv("ad_ok",link.accepted); kv("ad_reject",link.rejected); kv("parse_error",link.parse_errors);
        kv("c1_txok",c1.txok); kv("c1_txerr",c1.txerr); kv("c1_ovr",c1.ovr); kv("c2_ovr",c2.ovr);
        kv("fb_busy",fb_busy); kv("model_skip_ms",model_skip);
        kvh("c1_esr",REG(CAN1+C_ESR),8); kvh("c2_esr",REG(CAN2+C_ESR),8); putc_('\n');
        latency_sum=latency_n=latency_max=0;
      } else stat_defer++;
    }
  }
}
/* Freestanding codec dependencies; bounded lengths from link_proto. */
void *memcpy(void *dst,const void *src,size_t n) {
  uint8_t *d=dst; const uint8_t *s=src; while(n--) *d++=*s++; return dst;
}
void *memset(void *dst,int c,size_t n) { uint8_t *d=dst; while(n--) *d++=(uint8_t)c; return dst; }
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
  [16 ... 16 + 36] = default_handler,
  [16 + 37] = USART1_IRQHandler,
  [16 + 38 ... 16 + 96] = default_handler,
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
