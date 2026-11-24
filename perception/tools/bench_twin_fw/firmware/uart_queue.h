#ifndef BENCH_UART_QUEUE_H
#define BENCH_UART_QUEUE_H
#include <stdint.h>
#define RXN 1024u
#define TXN 4096u
#define STAT_RESERVE 1536u /* maximum status record, including all 10-digit counters */
/* Single ISR producer / main consumer. Volatile storage and compiler barriers
 * publish data before head, and consume data before releasing tail (Cortex M4). */
typedef struct {
 volatile uint16_t data[RXN];
 volatile uint32_t head, tail, bytes, queued, processed, drop, error;
 uint8_t gap;
} uart_rx;
static inline void queue_barrier(void) { __asm volatile("" ::: "memory"); }
static inline void uart_rx_push(uart_rx *q,uint8_t b,int error) {
 q->bytes++;
 if(error) { q->error++; q->gap=1; return; }
 uint32_t n=(q->head+1u)%RXN;
 if(n==q->tail) { q->drop++; q->gap=1; return; }
 q->data[q->head]=(uint16_t)b | (q->gap?256u:0u); q->gap=0;
 queue_barrier(); q->head=n; q->queued++;
}
static inline int uart_rx_pop(uart_rx *q,uint16_t *b) {
 if(q->tail==q->head) return 0;
 *b=q->data[q->tail]; queue_barrier();
 q->tail=(q->tail+1u)%RXN; q->processed++; return 1;
}
static inline int uart_tx_fits(uint32_t head,uint32_t tail,unsigned n,int status) {
 return (tail+TXN-head-1u)%TXN >= n+(status?0u:STAT_RESERVE);
}
#endif
