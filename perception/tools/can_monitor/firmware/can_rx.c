#include "f446.h"
#include "can_rx.h"

#define CAN1_BASE 0x40006400u
#define CAN2_BASE 0x40006800u
#define CAN_FMR 0x40006600u
#define CREG(b, o) REG((b) + (o))
#define MCR 0x00u
#define MSR 0x04u
#define RF0R 0x0cu
#define ESR 0x18u
#define BTR 0x1cu
#define RI0R 0x1b0u
#define RDT0R 0x1b4u
#define RDL0R 0x1b8u
#define RDH0R 0x1bcu

static uint32_t overruns[2];

static void gpio_can(unsigned pin) {
  GPIOB_MODER = (GPIOB_MODER & ~(3u << (2u * pin))) | (2u << (2u * pin));
  GPIOB_OSPEEDR |= 3u << (2u * pin);
  GPIOB_PUPDR &= ~(3u << (2u * pin));
  GPIOB_AFRH = (GPIOB_AFRH & ~(15u << (4u * (pin - 8u)))) |
               (9u << (4u * (pin - 8u)));
}

static void accept_all_filter(unsigned bank) {
  const uint32_t f = CAN_FMR;
  REG(f) |= 1u;                         /* FINIT */
  REG(f + 0x1cu) &= ~(1u << bank);    /* deactivate while configuring */
  REG(f + 0x0cu) |= 1u << bank;       /* 32-bit scale */
  REG(f + 0x04u) &= ~(1u << bank);    /* identifier-mask mode */
  REG(f + 0x14u) &= ~(1u << bank);    /* FIFO 0 */
  REG(f + 0x40u + bank * 8u) = 0u;    /* zero mask accepts every ID */
  REG(f + 0x44u + bank * 8u) = 0u;
  REG(f + 0x1cu) |= 1u << bank;
  REG(f) &= ~1u;
}

static void one_init(uint32_t base, uint32_t pclk1_hz) {
  CREG(base, MCR) = 1u;
  while ((CREG(base, MSR) & 1u) == 0u) {}
  CREG(base, MCR) = 1u | (1u << 6); /* INRQ + automatic bus-off recovery */
  /* 42 MHz: / (3 * 14 tq); 16 MHz fallback: / (1 * 16 tq). Both are 1 Mbit/s. */
  uint32_t timing = pclk1_hz == 16000000u ? ((1u << 20) | (12u << 16))
                                          : ((1u << 20) | (10u << 16) | 2u);
  CREG(base, BTR) = (1u << 31) | timing; /* SILM listen-only */
  CREG(base, MCR) &= ~1u;
  while (CREG(base, MSR) & 1u) {}
}

void can_rx_init(uint32_t pclk1_hz) {
  RCC_AHB1ENR |= 1u << 1;
  RCC_APB1ENR |= (1u << 25) | (1u << 26);
  (void)RCC_APB1ENR;
  gpio_can(8); gpio_can(9); gpio_can(12); gpio_can(13);
  REG(CAN_FMR) |= 1u; /* FINIT gates CAN2SB and filter configuration writes. */
  REG(CAN_FMR) = (REG(CAN_FMR) & ~(0x3fu << 8)) | (14u << 8);
  accept_all_filter(0u);  /* CAN1 filter bank 0 */
  accept_all_filter(14u); /* CAN2 filter banks start at CAN2SB = 14 */
  one_init(CAN1_BASE, pclk1_hz);
  one_init(CAN2_BASE, pclk1_hz);
}

int can_rx_read(unsigned bus, can_frame_t *frame) {
  const uint32_t base = bus ? CAN2_BASE : CAN1_BASE;
  const uint32_t rf = CREG(base, RF0R);
  if ((rf & 3u) == 0u) return 0;
  const uint32_t rir = CREG(base, RI0R);
  const uint32_t dlc = CREG(base, RDT0R);
  const uint32_t lo = CREG(base, RDL0R), hi = CREG(base, RDH0R);
  frame->extended = (uint8_t)((rir >> 2) & 1u);
  frame->remote = (uint8_t)((rir >> 1) & 1u);
  frame->id = frame->extended ? ((rir >> 3) & 0x1fffffffu) : ((rir >> 21) & 0x7ffu);
  frame->len = (uint8_t)(dlc & 0x0fu);
  for (unsigned i = 0; i < 4u; ++i) {
    frame->data[i] = (uint8_t)(lo >> (8u * i));
    frame->data[i + 4u] = (uint8_t)(hi >> (8u * i));
  }
  if (rf & (1u << 4)) {
    ++overruns[bus ? 1u : 0u];
    CREG(base, RF0R) |= 1u << 4; /* write-one-to-clear FOVR */
  }
  CREG(base, RF0R) |= 1u << 5; /* release FIFO output mailbox */
  return 1;
}

uint32_t can_rx_fifo_overruns(unsigned bus) { return overruns[bus ? 1u : 0u]; }
uint32_t can_rx_esr(unsigned bus) { return CREG(bus ? CAN2_BASE : CAN1_BASE, ESR); }
