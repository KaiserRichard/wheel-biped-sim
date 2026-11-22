#ifndef CAN_MONITOR_CAN_RX_H
#define CAN_MONITOR_CAN_RX_H
#include <stdint.h>

typedef struct {
  uint32_t id;
  uint8_t data[8];
  uint8_t len;
  uint8_t extended;
  uint8_t remote;
} can_frame_t;

void can_rx_init(uint32_t pclk1_hz);
int can_rx_read(unsigned bus, can_frame_t *frame);
uint32_t can_rx_fifo_overruns(unsigned bus);
uint32_t can_rx_esr(unsigned bus);

#endif
