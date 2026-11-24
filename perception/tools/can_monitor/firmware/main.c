#include "f446.h"
#include "can_rx.h"
#include "robstride_decode.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rcutils/allocator.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/u_int32_multi_array.h>
#include <uxr/client/transport.h>

#define BAUD 115200u
#define RX_RING_SIZE 512u
#define MOTOR_COUNT 2u
#define STATS_FIELDS 15u

static volatile uint8_t rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head, rx_tail;
static uint32_t timer_clock_hz = 84000000u;
static uint32_t can_clock_hz = 42000000u;
static uint32_t usart1_clock_hz = 84000000u;

typedef struct {
  float position, velocity, torque, temperature;
  uint8_t fault, mode, run_mode;
  uint32_t fault_frame_detail;
  uint32_t frames, feedback, commands, decode_failures;
  uint32_t last_speed;
} motor_state_t;
static motor_state_t motors[MOTOR_COUNT];
static uint32_t total_frames[MOTOR_COUNT];

static void *allocator_allocate(size_t n, void *s) { (void)s; return malloc(n); }
static void allocator_deallocate(void *p, void *s) { (void)s; free(p); }
static void *allocator_reallocate(void *p, size_t n, void *s) { (void)s; return realloc(p, n); }
static void *allocator_zero_allocate(size_t n, size_t z, void *s) { (void)s; return calloc(n, z); }

static uint64_t micros64(void) {
  static uint32_t last;
  static uint64_t high;
  uint32_t now = TIM2_CNT;
  if (now < last) high += 1ull << 32;
  last = now;
  return high | now;
}

int clock_gettime(clockid_t id, struct timespec *ts) {
  (void)id;
  uint64_t us = micros64();
  ts->tv_sec = (time_t)(us / 1000000ull);
  ts->tv_nsec = (long)((us % 1000000ull) * 1000ull);
  return 0;
}

static void clock_init(void) {
  uint32_t i;
  RCC_CR |= 1u << 18;
  RCC_CR |= 1u << 16;
  for (i = 0; i < 200000u && !(RCC_CR & (1u << 17)); ++i) {}
  bool hse_ok = (RCC_CR & (1u << 17)) != 0;
  if (!hse_ok) { RCC_CR &= ~(1u << 16); RCC_CR &= ~(1u << 18); }
  uint32_t m = hse_ok ? 8u : 16u;
  uint32_t source = hse_ok ? 1u : 0u;
  RCC_PLLCFGR = m | (336u << 6) | (1u << 16) | (source << 22) | (7u << 24) | (2u << 28);
  RCC_CR |= 1u << 24;
  for (i = 0; i < 2000000u && !(RCC_CR & (1u << 25)); ++i) {}
  if (!(RCC_CR & (1u << 25))) {
    timer_clock_hz = 16000000u;
    can_clock_hz = 16000000u;
    usart1_clock_hz = 16000000u;
    return;
  }
  FLASH_ACR = 2u | (1u << 8) | (1u << 9) | (1u << 10);
  RCC_CFGR = 4u << 10;
  RCC_CFGR |= 2u;
  for (i = 0; i < 2000000u && (RCC_CFGR & (3u << 2)) != (2u << 2); ++i) {}
}

static void peripherals_init(void) {
  RCC_AHB1ENR |= 7u;
  RCC_APB1ENR |= 1u;
  RCC_APB2ENR |= 1u << 4;
  (void)RCC_APB2ENR;
  for (unsigned pin = 9; pin <= 10; ++pin) {
    GPIOA_MODER = (GPIOA_MODER & ~(3u << (2u * pin))) | (2u << (2u * pin));
    GPIOA_AFRH = (GPIOA_AFRH & ~(15u << (4u * (pin - 8u)))) | (7u << (4u * (pin - 8u)));
  }
  GPIOA_PUPDR = (GPIOA_PUPDR & ~(3u << 20)) | (1u << 20);
  GPIOA_MODER = (GPIOA_MODER & ~(3u << 10)) | (1u << 10);
  GPIOA_BSRR = 1u << 5;
  TIM2_PSC = timer_clock_hz / 1000000u - 1u;
  TIM2_ARR = 0xffffffffu;
  TIM2_EGR = 1u;
  TIM2_CR1 = 1u;
  USART_BRR = (usart1_clock_hz + BAUD / 2u) / BAUD;
  USART_CR1 = (1u << 13) | (1u << 3) | (1u << 2) | (1u << 5);
  NVIC_ISER1 = 1u << (37u - 32u);
}

void USART2_IRQHandler(void) {
  uint32_t sr = USART_SR;
  if (sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NF | USART_SR_PE)) {
    uint8_t byte = (uint8_t)USART_DR;
    if (sr & USART_SR_RXNE) {
      uint16_t next = (uint16_t)((rx_head + 1u) & (RX_RING_SIZE - 1u));
      if (next == rx_tail) { /* Drop newest byte; XRCE framing recovers on a later packet. */ }
      else { rx_ring[rx_head] = byte; rx_head = next; }
    }
  }
}

static bool transport_open(struct uxrCustomTransport *t) { (void)t; return true; }
static bool transport_close(struct uxrCustomTransport *t) { (void)t; return true; }
static size_t transport_write(struct uxrCustomTransport *t, const uint8_t *b, size_t n, uint8_t *e) {
  (void)t;
  for (size_t i = 0; i < n; ++i) { while (!(USART_SR & USART_SR_TXE)) {} USART_DR = b[i]; }
  while (!(USART_SR & USART_SR_TC)) {}
  if (e) *e = 0;
  return n;
}
static size_t transport_read(struct uxrCustomTransport *t, uint8_t *b, size_t n, int timeout_ms, uint8_t *e) {
  (void)t;
  uint64_t start = micros64(), limit = timeout_ms > 0 ? (uint64_t)timeout_ms * 1000ull : 0ull;
  size_t got = 0;
  while (got < n && micros64() - start < limit) {
    if (rx_tail != rx_head) { b[got++] = rx_ring[rx_tail]; rx_tail = (uint16_t)((rx_tail + 1u) & (RX_RING_SIZE - 1u)); }
  }
  if (e) *e = 0;
  return got;
}

static uint32_t signed_as_u32(float v, float scale) {
  int32_t n = (int32_t)(v * scale);
  return (uint32_t)n;
}

static void poll_can(void) {
  can_frame_t f;
  for (unsigned bus = 0; bus < MOTOR_COUNT; ++bus) {
    while (can_rx_read(bus, &f)) {
      motor_state_t *m = &motors[bus];
      ++m->frames;
      ++total_frames[bus];
      rs_decoded_t d;
      int rc = rs_decode_frame(f.id, f.extended && !f.remote, f.data, f.len, &d);
      if (rc != 0) {
        uint8_t type = (uint8_t)((f.id >> 24) & 0x1fu);
        if (f.extended && (type == RS_TYPE_GET_ID || type == RS_TYPE_CONTROL ||
            type == RS_TYPE_FEEDBACK || type == RS_TYPE_ENABLE || type == RS_TYPE_STOP ||
            type == RS_TYPE_SET_ZERO || type == RS_TYPE_READ_PARAM ||
            type == RS_TYPE_WRITE_PARAM || type == RS_TYPE_FAULT_FB ||
            type == RS_TYPE_SAVE_PARAM || type == RS_TYPE_SET_BAUD ||
            type == RS_TYPE_ACTIVE_RPT || type == RS_TYPE_PROTOCOL_SW ||
            type == RS_TYPE_READ_VER)) ++m->decode_failures;
        continue;
      }
      if (d.kind == RS_DECODE_FEEDBACK) {
        m->position = d.position; m->velocity = d.velocity; m->torque = d.torque;
        m->temperature = d.temperature; m->fault = d.fault; m->mode = d.mode;
        ++m->feedback;
      } else if (d.kind == RS_DECODE_COMMAND || d.kind == RS_DECODE_PARAM_WRITE) {
        ++m->commands;
        if (d.has_speed && isfinite(d.commanded_speed))
          m->last_speed = signed_as_u32(d.commanded_speed, 1000.0f);
        if (d.has_run_mode) m->run_mode = d.run_mode;
      } else if (d.kind == RS_DECODE_FAULT) {
        m->fault_frame_detail = d.fault_detail;
      }
    }
  }
}

static bool init_messages(sensor_msgs__msg__JointState *joint,
                          std_msgs__msg__UInt32MultiArray *stats) {
  if (!sensor_msgs__msg__JointState__init(joint) || !std_msgs__msg__UInt32MultiArray__init(stats)) return false;
  if (!rosidl_runtime_c__String__Sequence__init(&joint->name, 2u)) return false;
  joint->position.data = calloc(2u, sizeof(double));
  joint->velocity.data = calloc(2u, sizeof(double));
  joint->effort.data = calloc(2u, sizeof(double));
  stats->data.data = calloc(2u * STATS_FIELDS, sizeof(uint32_t));
  if (!joint->position.data || !joint->velocity.data || !joint->effort.data || !stats->data.data) return false;
  joint->position.capacity = joint->velocity.capacity = joint->effort.capacity = 2u;
  stats->data.capacity = 2u * STATS_FIELDS;
  joint->name.size = joint->position.size = joint->velocity.size = joint->effort.size = 2u;
  stats->data.size = 2u * STATS_FIELDS;
  return rosidl_runtime_c__String__assign(&joint->name.data[0], "left") &&
         rosidl_runtime_c__String__assign(&joint->name.data[1], "right");
}

static bool entities_init(rcl_allocator_t *a, rclc_support_t *support, rcl_node_t *node,
                          rcl_publisher_t *joint_pub, rcl_publisher_t *stats_pub) {
  if (rclc_support_init(support, 0, NULL, a) != RCL_RET_OK) return false;
  if (rclc_node_init_default(node, "can_monitor", "", support) != RCL_RET_OK) return false;
  if (rclc_publisher_init_best_effort(joint_pub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/can_monitor/motors") != RCL_RET_OK) return false;
  if (rclc_publisher_init_best_effort(stats_pub, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32MultiArray), "/can_monitor/stats") != RCL_RET_OK) return false;
  return true;
}

static void entities_fini(rcl_publisher_t *jp, rcl_publisher_t *sp, rcl_node_t *n, rclc_support_t *s) {
  rcl_ret_t rc;
  rc = rcl_publisher_fini(jp, n); (void)rc;
  rc = rcl_publisher_fini(sp, n); (void)rc;
  rc = rcl_node_fini(n); (void)rc;
  rc = rclc_support_fini(s); (void)rc;
}

static void wait_for_agent(void) {
  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) poll_can();
}

int main(void) {
  clock_init(); peripherals_init(); can_rx_init(can_clock_hz);
  rcutils_allocator_t ca = rcutils_get_zero_initialized_allocator();
  ca.allocate = allocator_allocate; ca.deallocate = allocator_deallocate;
  ca.reallocate = allocator_reallocate; ca.zero_allocate = allocator_zero_allocate;
  if (!rcutils_set_default_allocator(&ca)) for (;;) {}
  if (rmw_uros_set_custom_transport(true, NULL, transport_open, transport_close,
                                    transport_write, transport_read) != RMW_RET_OK) for (;;) {}

  for (;;) {
    wait_for_agent();
    rcl_allocator_t a = rcl_get_default_allocator();
    rclc_support_t support = {0};
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_publisher_t jp = rcl_get_zero_initialized_publisher();
    rcl_publisher_t sp = rcl_get_zero_initialized_publisher();
    sensor_msgs__msg__JointState joint = {0};
    std_msgs__msg__UInt32MultiArray stats = {0};
    if (!init_messages(&joint, &stats) || !entities_init(&a, &support, &node, &jp, &sp)) {
      sensor_msgs__msg__JointState__fini(&joint);
      std_msgs__msg__UInt32MultiArray__fini(&stats);
      entities_fini(&jp, &sp, &node, &support);
      continue;
    }
    uint64_t now = micros64(), next_joint = now, next_stats = now;
    uint64_t last_ping = now;
    uint32_t last_total[MOTOR_COUNT] = {total_frames[0], total_frames[1]};
    uint32_t last_feedback[MOTOR_COUNT] = {motors[0].feedback, motors[1].feedback};
    uint32_t last_commands[MOTOR_COUNT] = {motors[0].commands, motors[1].commands};
    bool connected = true;
    while (connected) {
      poll_can();
      now = micros64();
      if (now >= next_joint) {
        for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
          joint.position.data[i] = motors[i].position;
          joint.velocity.data[i] = motors[i].velocity;
          joint.effort.data[i] = motors[i].torque;
        }
        joint.header.stamp.sec = (int32_t)(now / 1000000ull);
        joint.header.stamp.nanosec = (uint32_t)((now % 1000000ull) * 1000ull);
        rcl_ret_t publish_rc = rcl_publish(&jp, &joint, NULL); (void)publish_rc;
        next_joint = now + 50000ull;
      }
      if (now >= next_stats) {
        for (unsigned i = 0; i < MOTOR_COUNT; ++i) {
          uint32_t esr = can_rx_esr(i), k = i * STATS_FIELDS;
          stats.data.data[k + 0] = (total_frames[i] - last_total[i]) * 5u;
          stats.data.data[k + 1] = (motors[i].feedback - last_feedback[i]) * 5u;
          stats.data.data[k + 2] = (motors[i].commands - last_commands[i]) * 5u;
          stats.data.data[k + 3] = motors[i].last_speed;
          stats.data.data[k + 4] = (esr >> 16) & 0xffu;
          stats.data.data[k + 5] = (esr >> 24) & 0xffu;
          stats.data.data[k + 6] = (esr >> 4) & 7u;
          stats.data.data[k + 7] = (esr >> 2) & 1u;
          stats.data.data[k + 8] = can_rx_fifo_overruns(i);
          stats.data.data[k + 9] = motors[i].decode_failures;
          stats.data.data[k + 10] = motors[i].fault;
          stats.data.data[k + 11] = motors[i].mode;
          stats.data.data[k + 12] = (uint32_t)(motors[i].temperature * 10.0f);
          stats.data.data[k + 13] = motors[i].run_mode;
          stats.data.data[k + 14] = motors[i].fault_frame_detail;
          last_total[i] = total_frames[i]; last_feedback[i] = motors[i].feedback;
          last_commands[i] = motors[i].commands;
        }
        rcl_ret_t publish_rc = rcl_publish(&sp, &stats, NULL); (void)publish_rc;
        next_stats = now + 200000ull;
      }
      if (now - last_ping >= 1000000ull) {
        last_ping = now;
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) connected = false;
      }
    }
    sensor_msgs__msg__JointState__fini(&joint);
    std_msgs__msg__UInt32MultiArray__fini(&stats);
    entities_fini(&jp, &sp, &node, &support);
  }
}
