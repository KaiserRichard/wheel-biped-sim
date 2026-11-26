#include "target_estop.h"
#include "target_fdcan.h"
#include "target_crsf.h"
#include "target_usb_cdc.h"
#include "target_iwdg.h"
#include "target_safety_io.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

static CanSched sched;
static SafetyInputs clean_inputs(void) {
 SafetyInputs in = {0};
 in.dt_ms = 2u; in.boot_self_test_ok = 1u; in.dump_continuity_ok = 1u;
 in.v_bus_v = 0.5f; in.v_batt_ocv_v = 24.0f; in.imu_est_valid = 1u;
 in.drives_healthy_6 = 1u; in.host_valid = 1u; in.ground_left = in.ground_right = 1u;
 return in;
}
static void setup(void) {
 stub_reset(); can_sched_init(&sched, NULL); target_estop_init();
 assert(target_fdcan_init(&sched) == HAL_OK);
 target_fdcan_clear_emergency(); target_safety_io_init();
}
static void stop_irq(void) {
 stub_stop_closed = 0u; stub_exti_pending = 0u; target_estop_exti_isr();
}
static void pend_stop(void) { stub_irq_on_enable = stop_irq; }
static void pend_opening(void) {
 stub_exti_pending = TARGET_ESTOP_PIN;
 stub_irq_on_enable = stop_irq;
}
/* HFX2-01: a PD7 pulse has bounced LOW; its ISR cannot run under PRIMASK. */
static void opto_pending_permit_test(void) {
 setup();
 __disable_irq();
 stub_exti_pending = TARGET_ESTOP_OPTO_PIN;
 target_estop_set_permit(1u);
 assert(stub_primask == 1u);
 assert(stub_permit == 0u);
 assert(target_estop_is_tripped() == 1u);
 __enable_irq();
}
static void opto_pending_clear_test(void) {
 setup(); target_estop_force_trip();
 __disable_irq(); stub_exti_pending = TARGET_ESTOP_OPTO_PIN;
 assert(target_estop_try_clear_latch(1u) == 0u);
 assert(stub_primask == 1u && stub_permit == 0u);
 stub_exti_pending = 0u;
 assert(target_estop_try_clear_latch(1u) == 1u);
 __enable_irq();
}
static void opto_pending_tx_test(void) {
 setup();
 assert(can_sched_enqueue_enable_all(&sched, 0u) == CAN_SCHED_OK);
 __disable_irq(); stub_exti_pending = TARGET_ESTOP_OPTO_PIN;
 target_fdcan_flush_tx(&sched, 0u);
 assert(stub_primask == 1u && stub_tx_count == 0u);
 __enable_irq();
}
static void rev4_mapping_test(void) {
 setup();
 const uint8_t buses[6] = {1u, 1u, 1u, 0u, 0u, 0u};
 assert(stub_filter_count[0] == 2u && stub_filter_count[1] == 2u);
 assert(stub_filter_count[2] == 0u && g_hfdcan[2].Init.StdFiltersNbr == 0u);
 for (uint8_t b = 0u; b < 2u; ++b) {
  uint32_t first = b == 0u ? 0x14u : 0x11u;
  assert(stub_filters[b][0].IdType == FDCAN_STANDARD_ID);
  assert(stub_filters[b][0].FilterID1 == first && stub_filters[b][0].FilterID2 == first + 1u);
  assert(stub_filters[b][1].FilterID1 == first + 2u && stub_filters[b][1].FilterID2 == first + 2u);
 }
 for (uint8_t a = 0u; a < 6u; ++a) {
  uint8_t b = buses[a];
  DamiaoAxisConfig cfg; uint8_t sched_bus;
  assert(damiao_get_axis_config(a, &cfg) == DAMIAO_OK);
  assert(can_sched_bus_for_axis(a, &sched_bus) == CAN_SCHED_OK);
  assert(sched_bus == b && cfg.can_bus == b + 1u);
  uint16_t id = (uint16_t)(0x11u + a);
  unsigned matches = 0u;
  for (uint8_t leg = 0u; leg < 2u; ++leg) {
   for (uint8_t i = 0u; i < 2u; ++i) {
    FDCAN_FilterTypeDef f = stub_filters[leg][i];
    assert(f.FilterType == FDCAN_FILTER_DUAL && f.FilterConfig == FDCAN_FILTER_TO_RXFIFO0);
    if (f.FilterID1 == id || f.FilterID2 == id) { assert(leg == b); matches++; }
   }
  }
  assert(matches == 1u);
  /* ISR payload verification and thread dispatcher must agree for every axis. */
  stub_rx_header[b] = (FDCAN_RxHeaderTypeDef){id, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8};
  uint8_t fb[8] = {(uint8_t)(0x11u + a), 0x80u, 0u, 0x80u, 0x08u, 0u, 30u, 31u};
  memcpy(stub_rx_data[b], fb, 8u); stub_rx_ready[b] = 1u;
  HAL_FDCAN_RxFifo0Callback(&g_hfdcan[b], FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
 }
 target_fdcan_poll_status(0u);
 for (uint8_t a = 0u; a < 6u; ++a) { assert(sched.axis_rx[a].seen == 1u); }
 /* Wrong connector and spare must reject every motor ID in software too. */
 for (uint8_t a = 0u; a < 6u; ++a) {
  CanFrame frame = {0}; frame.std_id = (uint16_t)(0x11u + a); frame.dlc = 8u;
  assert(can_sched_rx_frame(&sched, (uint8_t)(1u - buses[a]), &frame, 0u) == CAN_SCHED_ERR_REJECTED_FRAME);
  assert(can_sched_rx_frame(&sched, 2u, &frame, 0u) == CAN_SCHED_ERR_REJECTED_FRAME);
 }
 assert(can_sched_enqueue_enable_all(&sched, 0u) == CAN_SCHED_OK);
 target_fdcan_flush_tx(&sched, 0u);
 assert(stub_tx_count == 6u);
 for (unsigned i = 0u; i < 6u; ++i) {
  assert(stub_tx[i].id >= 1u && stub_tx[i].id <= 6u);
  assert(stub_tx[i].bus == buses[stub_tx[i].id - 1u]);
 }
 for (unsigned b = 0u; b < 3u; ++b) { stub_free[b] = 8u; }
 target_estop_force_trip();
 assert(stub_tx_count == 12u);
 for (unsigned i = 6u; i < 12u; ++i) {
  assert(stub_tx[i].data[7] == 0xFDu);
  assert(stub_tx[i].bus == buses[stub_tx[i].id - 1u]);
 }
}
static void preempt_service(void) {
 stub_now = 101u;
 target_estop_force_trip();
}
static void emergency_clock_test(void) {
 setup();
 stub_now = 100u;
 /* EXTI preempts immediately before service masks IRQs and stamps t=101 us. */
 stub_before_irq_disable = preempt_service;
 target_fdcan_emergency_service();
 assert(stub_tx_count == 6u); /* Never emit the repeat at t=101 us. */
 puts("elapsed_us,disable_frames");
 printf("0,%u\n", (unsigned)stub_tx_count);
 stub_now = 1100u; target_fdcan_emergency_service(); assert(stub_tx_count == 6u);
 printf("999,%u\n", (unsigned)stub_tx_count);
 stub_now = 1101u; target_fdcan_emergency_service(); assert(stub_tx_count == 12u);
 printf("1000,%u\n", (unsigned)stub_tx_count);
 stub_now = 1102u; target_fdcan_emergency_service(); assert(stub_tx_count == 12u);
 printf("1001,%u\n", (unsigned)stub_tx_count);
 __disable_irq(); target_fdcan_emergency_service(); assert(stub_primask == 1u); __enable_irq();
}
/* FW8 F-05: Error_Handler/HardFault path. IRQs stay masked, so no SysTick or
 * FDCAN callback can emit the +1 ms disable repeat; the halt loop must. */
static void fatal_halt_repeat_test(void) {
 setup();
 stub_now = 500u;
 stub_halt_polls_max = 50u; /* 5 ms of halt-loop polling at 100 us per poll */
 target_estop_fatal_halt();
 assert(stub_primask == 1u);
 unsigned disable = 0u;
 unsigned per_id[7] = {0u};
 for (unsigned i = 0u; i < stub_tx_count; ++i) {
  assert(stub_tx[i].data[7] == 0xFDu);
  disable++;
  if (stub_tx[i].id >= 1u && stub_tx[i].id <= 6u) { per_id[stub_tx[i].id]++; }
 }
 printf("fatal_halt: disable_frames=%u over %u us\n", disable, (unsigned)(stub_now - 500u));
 assert(disable == 12u); /* first burst + one +1 ms repeat, no more */
 for (unsigned id = 1u; id <= 6u; ++id) { assert(per_id[id] == 2u); }

 /* A full TX FIFO at trip time: frames that did not fit are sent once room appears. */
 setup();
 stub_now = 500u;
 for (unsigned b = 0u; b < 3u; ++b) { stub_free[b] = 1u; }
 stub_halt_polls_max = 1u;
 target_estop_fatal_halt();
 unsigned first = stub_tx_count;
 for (unsigned b = 0u; b < 3u; ++b) { stub_free[b] = 8u; }
 stub_halt_polls = 0u; stub_halt_polls_max = 50u;
 /* re-enter the loop body as the halted core would keep doing */
 target_estop_fatal_halt();
 printf("fatal_halt with full FIFO: first=%u total=%u\n", first, (unsigned)stub_tx_count);
 assert(first == 2u && stub_tx_count == 12u);
}
static void watchdog_config_test(void) {
 setup(); assert(target_iwdg_init(0u) == HAL_OK);
 assert(stub_iwdg_config.Init.Reload == 52u);
 assert(stub_iwdg_config.Init.Reload == TARGET_IWDG_TIMEOUT_MS - 1u);
 assert(stub_iwdg_config.Init.Prescaler == IWDG_PRESCALER_32);
 /* FROM FILE clock_check.py: 33.6 kHz upper LSI prior. T=32*(RLR+1)/f. */
 assert(32u * (stub_iwdg_config.Init.Reload + 1u) * 1000u >= 50u * 33600u);
}
static void estop_test(void) {
 setup(); TargetEstopState st;
 target_estop_set_permit(1u); assert(stub_permit == 1u);
 target_estop_set_permit(1u); assert(stub_permit == 0u); /* recurring edge, not static permit */
 target_estop_set_permit(1u); assert(stub_permit == 1u);
 /* Interrupt becomes pending between pin read and permit write. */
 stub_on_gpio_read = pend_stop;
 target_estop_set_permit(1u);
 target_estop_get_state(&st);
 assert(st.latched == 1u && st.permit_asserted == 0u && stub_permit == 0u);
 assert(target_estop_try_clear_latch(1u) == 0u); /* PB12 open */
 stub_stop_closed = 1u;
 assert(target_estop_try_clear_latch(0u) == 0u); /* USB hint is no authority */
 stub_exti_pending = TARGET_ESTOP_PIN;
 assert(target_estop_try_clear_latch(1u) == 0u);
 target_estop_set_permit(1u); assert(stub_permit == 0u);
 stub_exti_pending = 0u;
 assert(target_estop_try_clear_latch(1u) == 1u);
 /* Opening arrives during the clear check: it must never be forgotten. */
 stub_on_gpio_read = pend_opening;
 assert(target_estop_try_clear_latch(1u) == 0u);
 target_estop_get_state(&st); assert(st.latched == 1u);
 stub_stop_closed = 1u;
 assert(target_estop_try_clear_latch(1u) == 1u);
 /* Either stop input independently trips; both use LOW=healthy/HIGH=open. */
 stub_opto_closed = 0u;
 assert(target_estop_try_clear_latch(1u) == 0u); /* PD7 open */
 target_estop_get_state(&st); assert(st.pin_open == 1u && st.latched == 1u);
 stub_opto_closed = 1u;
 assert(target_estop_try_clear_latch(1u) == 1u);
 stub_stop_closed = 0u;
 assert(target_estop_try_clear_latch(1u) == 0u); /* PB12 open */
 stub_stop_closed = 1u;
 assert(target_estop_try_clear_latch(1u) == 1u);
 /* Rising edge bounced closed before ISR: still latch. */
 target_estop_exti_isr(); target_estop_get_state(&st); assert(st.latched == 1u);
 assert(target_estop_try_clear_latch(1u) == 1u);
 /* Nested critical section must retain caller's mask. */
 __disable_irq(); target_estop_set_permit(0u); target_estop_get_state(&st);
 assert(stub_primask == 1u); __enable_irq();

 SafetyFsm fsm; SafetyOutput out; SafetyInputs in = clean_inputs();
 safety_fsm_init(&fsm);
 puts("time_ms,pd7_open,reset_edge,hw_ack,state,permit,torque");
 in.estop_open = 1u; safety_fsm_step(&fsm, &in, &out);
 printf("0,1,0,0,%u,%u,%u\n", (unsigned)out.state, out.permit_out, out.torque_enable);
 assert(out.state == SAFETY_STATE_ESTOP);
 in.estop_open = 0u; in.local_reset_req = 1u;
 safety_fsm_step(&fsm, &in, &out); assert(out.state == SAFETY_STATE_ESTOP);
 printf("2,0,0,0,%u,%u,%u\n", (unsigned)out.state, out.permit_out, out.torque_enable);
 in.hw_reset_ack = target_estop_try_clear_latch(1u);
 safety_fsm_step(&fsm, &in, &out); assert(out.state == SAFETY_STATE_DISARMED);
 printf("4,0,1,1,%u,%u,%u\n", (unsigned)out.state, out.permit_out, out.torque_enable);
 safety_fsm_step(&fsm, &in, &out); assert(out.state == SAFETY_STATE_DISARMED);
}
static void can_test(void) {
 setup(); float torques[6] = {1, 1, 1, 1, 1, 1};
 assert(can_sched_enqueue_torques(&sched, torques, 0u) == CAN_SCHED_OK);
 uint8_t counts[3]; for (unsigned b = 0; b < 3; ++b) { counts[b] = sched.bus[b].tx_count; stub_free[b] = 0u; }
 target_estop_force_trip();
 for (unsigned b = 0; b < 3; ++b) { assert(sched.bus[b].tx_count == counts[b]); }
 assert(stub_tx_count == 0u); assert(stub_abort_count == 3u);
 /* Abort completes asynchronously, ISR must drain without main progress. */
 for (unsigned b = 0; b < 3; ++b) { stub_free[b] = 8u; }
 HAL_FDCAN_TxBufferAbortCallback(&g_hfdcan[0], 0xFFu);
 assert(stub_tx_count == 6u);
 stub_now = 999u; target_fdcan_emergency_service(); assert(stub_tx_count == 6u);
 stub_now = 1000u; target_fdcan_emergency_service(); assert(stub_tx_count == 12u);
 target_fdcan_flush_tx(&sched, 0u); /* interrupted tick's queued torque must all be dropped */
 assert(stub_tx_count == 12u);
 for (unsigned i = 0; i < stub_tx_count; ++i) {
  for (unsigned k = 0; k < 7; ++k) { assert(stub_tx[i].data[k] == 0xFFu); }
  assert(stub_tx[i].data[7] == 0xFDu);
 }
 assert(target_estop_try_clear_latch(1u) == 1u);
 for (unsigned b = 0; b < 3; ++b) { assert(sched.bus[b].tx_count == 0u); }
 /* Queue after stale snapshot, then a pending physical opening at final gate. */
 assert(can_sched_enqueue_enable_all(&sched, stub_now) == CAN_SCHED_OK);
 stub_exti_pending = TARGET_ESTOP_PIN;
 target_fdcan_flush_tx(&sched, 0u); assert(stub_tx_count == 12u);
 /* Repeat deadline works through a nearby u32 wrap. */
 stub_exti_pending = 0u;
 assert(target_estop_try_clear_latch(1u) == 1u);
 for (unsigned b = 0; b < 3; ++b) { stub_free[b] = 8u; }
 stub_now = UINT32_MAX - 499u; target_estop_force_trip();
 uint32_t first = stub_tx_count;
 stub_now = 499u; target_fdcan_emergency_service(); assert(stub_tx_count == first);
 stub_now = 500u; target_fdcan_emergency_service(); assert(stub_tx_count == first + 6u);
}
static void rx_test(void) {
 setup();
 for (unsigned i = 0; i < 20; ++i) {
  stub_rx_header[1] = (FDCAN_RxHeaderTypeDef){0x11u, FDCAN_STANDARD_ID, FDCAN_DATA_FRAME, FDCAN_DLC_BYTES_8};
  uint8_t fb[8] = {0x11u, 0x80u, 0x00u, 0x80u, 0x08u, 0x00u, 30u, 31u};
  memcpy(stub_rx_data[1], fb, 8u); stub_rx_ready[1] = 1u; stub_now = i * 100u;
  HAL_FDCAN_RxFifo0Callback(&g_hfdcan[1], FDCAN_IT_RX_FIFO0_NEW_MESSAGE);
  assert(sched.axis_rx[0].seen == 0u); /* ISR has no access to scheduler */
 }
 target_fdcan_poll_status(stub_now);
 assert(sched.axis_rx[0].seen == 1u && sched.axis_rx[0].rx_us == 1400u);
 assert(sched.bus[1].rx_drop_count == 5u);
 assert(sched.axis_rx[0].fb.t_mos_c == 30.0f);
}
static void operator_test(void) {
 setup(); TargetOperator op = {0};
 assert(target_reset_button_raw() == 0u); /* deliberately unwired default */
 assert(target_operator_reset_edge(&op, 1u, 0u) == 0u);
 assert(target_operator_reset_edge(&op, 1u, 40000u) == 0u); /* held at boot */
 assert(target_operator_reset_edge(&op, 0u, 50000u) == 0u);
 assert(target_operator_reset_edge(&op, 0u, 80000u) == 0u);
 assert(target_operator_reset_edge(&op, 1u, 90000u) == 0u);
 assert(target_operator_reset_edge(&op, 0u, 91000u) == 0u); /* bounce */
 assert(target_operator_reset_edge(&op, 1u, 92000u) == 0u);
 assert(target_operator_reset_edge(&op, 1u, 122000u) == 1u);
 assert(target_operator_reset_edge(&op, 1u, 200000u) == 0u);
 assert(target_operator_arm_edge(&op, 1u, 1u, 0u, 1u, 200000u) == 0u); /* boot high */
 assert(target_operator_arm_edge(&op, 1u, 0u, 0u, 1u, 202000u) == 0u);
 assert(target_operator_arm_edge(&op, 1u, 1u, 0u, 1u, 204000u) == 0u); /* no reset */
 assert(target_operator_arm_edge(&op, 1u, 0u, 1u, 1u, 206000u) == 0u);
 assert(target_operator_arm_edge(&op, 1u, 1u, 0u, 1u, 208000u) == 1u);
 assert(target_operator_arm_edge(&op, 1u, 1u, 0u, 1u, 210000u) == 0u);
 assert(target_operator_arm_edge(&op, 1u, 0u, 1u, 1u, UINT32_MAX - 1000u) == 0u);
 assert(target_operator_arm_edge(&op, 1u, 1u, 0u, 1u, 1000u) == 1u); /* ordinary wrap */
 assert(target_operator_arm_edge(&op, 1u, 0u, 1u, 1u, 2000u) == 0u);
 assert(target_operator_arm_edge(&op, 1u, 1u, 0u, 1u, 5002001u) == 0u); /* expired */
 assert(target_operator_arm_edge(&op, 1u, 0u, 1u, 1u, 6000000u) == 0u);
 assert(target_operator_arm_edge(&op, 0u, 0u, 0u, 1u, 6001000u) == 0u);
 assert(target_operator_arm_edge(&op, 1u, 1u, 0u, 1u, 6002000u) == 0u); /* RF loss revokes */
 BalanceTickInput bin = {0}; target_power_unwired_inputs(&bin);
 assert(bin.boot_self_test_ok == 0u && bin.dump_continuity_ok == 0u);
 assert(bin.k1_closed == 0u && bin.k2_closed == 0u && bin.precharge_dv_ok == 0u);
 assert(bin.mirror_mismatch == 1u && bin.drive_enable_fail == 1u);
 SafetyFsm fsm; SafetyOutput out; safety_fsm_init(&fsm);
 SafetyInputs in = clean_inputs(); in.local_arm_req = in.local_reset_req = 1u;
 in.boot_self_test_ok = bin.boot_self_test_ok; safety_fsm_step(&fsm, &in, &out);
 assert(out.permit_out == 0u && out.can_enable_cmd == 0u);
 /* Partial wiring must still fail on unknown contactor/drive verification. */
 safety_fsm_init(&fsm); in.boot_self_test_ok = 1u; in.mirror_mismatch = bin.mirror_mismatch;
 safety_fsm_step(&fsm, &in, &out); assert(out.state == SAFETY_STATE_DISARMED);
}
static void watchdog_test(void) {
 setup(); stub_reset_flags = RCC_FLAG_IWDG1RST;
 target_iwdg_capture_reset_cause(); assert(target_iwdg_init(0u) == HAL_OK);
 TargetIwdgStatus st; target_iwdg_get_status(&st); assert(st.wdg_fault == 1u);
 SafetyFsm fsm; SafetyOutput out; SafetyInputs in = clean_inputs(); safety_fsm_init(&fsm);
 assert(fsm.state == SAFETY_STATE_DISARMED);
 in.wdg_fault = st.wdg_fault; in.local_arm_req = in.local_reset_req = 1u;
 safety_fsm_step(&fsm, &in, &out); assert(out.state == SAFETY_STATE_DISARMED && out.permit_out == 0u);
 target_iwdg_clear_fault(0u); target_iwdg_get_status(&st); assert(st.wdg_fault == 1u);
 target_iwdg_clear_fault(1u); target_iwdg_get_status(&st);
 assert(st.wdg_fault == 0u && (st.reset_cause_bits & TARGET_RESET_CAUSE_IWDG) != 0u);
 in.wdg_fault = st.wdg_fault; safety_fsm_step(&fsm, &in, &out); assert(out.state == SAFETY_STATE_PRECHARGE);
 /* FAULT recovery path as well. */
 fsm.state = SAFETY_STATE_FAULT; in.radio_arm_sw = 0u;
 safety_fsm_step(&fsm, &in, &out); assert(out.state == SAFETY_STATE_DISARMED);
 target_iwdg_service(0u, 1u); target_iwdg_service(31001u, 1u);
 target_iwdg_get_status(&st); assert(st.wdg_fault == 1u && stub_iwdg_refreshes == 1u);
}
static void imu_test(void) {
 setup(); BalanceImuSample sample, copy;
 assert(target_imu_snapshot(&copy) == 0u);
 memset(&sample, 0, sizeof(sample)); sample.tick_ms = 42;
 for (unsigned i = 0; i < 3; ++i) { sample.gyro_rad_s[i] = 0.25 + i; sample.accel_m_s2[i] = 9.0 + i; }
 target_imu_publish_isr(&sample); assert(target_imu_snapshot(&copy) == 1u);
 assert(memcmp(&copy, &sample, sizeof(sample)) == 0);
 assert(target_imu_snapshot(&copy) == 0u); /* No fake fresh replay next tick. */
 __disable_irq(); target_imu_publish_isr(&sample); target_imu_snapshot(&copy);
 assert(stub_primask == 1u); __enable_irq();
}
static size_t rc_wire_axes(uint8_t *wire, uint16_t yaw, uint16_t forward,
                           uint16_t switch_value) {
 uint16_t channels[16]; for (unsigned i = 0; i < 16; ++i) { channels[i] = 992u; }
 channels[0] = yaw; channels[1] = forward;
 channels[4] = channels[5] = channels[6] = channels[7] = switch_value;
 memset(wire, 0, 26u); wire[0] = CRSF_SYNC_BYTE_FC; wire[1] = 24u; wire[2] = CRSF_FRAMETYPE_RC_CHANNELS;
 for (unsigned ch = 0; ch < 16; ++ch) {
  for (unsigned bit = 0; bit < 11; ++bit) {
   unsigned pos = ch * 11u + bit;
   if ((channels[ch] & (1u << bit)) != 0u) { wire[3u + pos / 8u] |= (uint8_t)(1u << (pos % 8u)); }
  }
 }
 wire[25] = target_crsf_crc8_dvb_s2(wire + 2, 23u); return 26u;
}
static size_t rc_wire(uint8_t *wire, uint16_t value) {
 return rc_wire_axes(wire, CRSF_CH_VAL_MID, CRSF_CH_VAL_MID, value);
}
static void crsf_push(const uint8_t *wire, size_t n, uint32_t start) {
 for (size_t i = 0; i < n; ++i) { target_crsf_isr_push_byte(wire[i], start + (uint32_t)i * 24u); }
}
static void radio_test(void) {
 setup(); assert(target_crsf_init() == HAL_OK); uint8_t wire[64]; TargetCrsfSnapshot snap;
 size_t n = rc_wire(wire, CRSF_CH_VAL_MID);
 crsf_push(wire, 13u, 1000u); target_crsf_poll(5000u);
 crsf_push(wire + 13, n - 13u, 1312u); target_crsf_poll(20000u);
 target_crsf_snapshot(20000u, &snap);
 assert(snap.frames_ok == 1u && snap.failsafe_active == 0u && snap.radio_age_ms == 18u);
 assert(snap.v_target_m_s == 0.0f && snap.yaw_target_rad_s == 0.0f);
 /* A frame stamped by the ISR after the loop's earlier clock sample is fresh. */
 assert(target_crsf_init() == HAL_OK);
 n = rc_wire(wire, CRSF_CH_VAL_MID);
 crsf_push(wire, n, 50000u); target_crsf_poll(51000u);
 target_crsf_snapshot(49999u, &snap);
 assert(snap.failsafe_active == 0u && snap.radio_age_ms == 0u);
 assert(snap.v_target_m_s == 0.0f && snap.yaw_target_rad_s == 0.0f);
 n = rc_wire_axes(wire, CRSF_CH_VAL_MAX, CRSF_CH_VAL_MAX, CRSF_CH_VAL_MIN);
 crsf_push(wire, n, 60000u); target_crsf_poll(61000u);
 target_crsf_snapshot(61000u, &snap);
 assert(snap.v_target_m_s == 1.0f);
 assert(snap.yaw_target_rad_s == 1.5f);
 n = rc_wire_axes(wire, CRSF_CH_VAL_MIN, CRSF_CH_VAL_MIN, CRSF_CH_VAL_MIN);
 crsf_push(wire, n, 70000u); target_crsf_poll(71000u);
 target_crsf_snapshot(71000u, &snap);
 assert(snap.v_target_m_s == -1.0f);
 assert(snap.yaw_target_rad_s == -1.5f);
 target_crsf_snapshot(300000u, &snap);
 assert(snap.failsafe_active == 1u && snap.v_target_m_s == 0.0f
        && snap.yaw_target_rad_s == 0.0f);
 assert(target_crsf_init() == HAL_OK);
 n = rc_wire(wire, CRSF_CH_VAL_MIN);
 /* Actual inter-byte gap rejects a partial frame even if drained in one poll. */
 crsf_push(wire, 13u, 21000u); crsf_push(wire + 13, 13u, 25000u);
 target_crsf_poll(30000u); target_crsf_snapshot(30000u, &snap); assert(snap.frames_ok == 0u);
 /* Link statistics LQ=0 followed by preset high stop/arm frames must not change switches. */
 uint8_t stats[14] = {CRSF_SYNC_BYTE_FC, 12u, CRSF_FRAMETYPE_LINK_STATISTICS};
 stats[13] = target_crsf_crc8_dvb_s2(stats + 2, 11u);
 crsf_push(stats, sizeof(stats), 31000u); target_crsf_poll(32000u);
 n = rc_wire(wire, CRSF_CH_VAL_MAX); crsf_push(wire, n, 33000u); target_crsf_poll(35000u);
 target_crsf_snapshot(35000u, &snap); assert(snap.radio_estop_sw == 0u && snap.radio_arm_sw == 0u);
 target_crsf_snapshot(200000u, &snap);
 assert(snap.failsafe_active == 1u && snap.radio_estop_sw == 0u && snap.radio_arm_sw == 0u);
 assert(snap.stand_handoff_req == 0u && snap.voluntary_stop_req == 0u);
 assert(snap.effective_tilt_limit_deg == CRSF_NORMAL_TILT_LIMIT_DEG);
 /* Timeout cannot clear after an entire u32 cycle without a new valid frame. */
 target_crsf_snapshot(1600u, &snap); assert(snap.failsafe_active == 1u && snap.radio_age_ms > 150u);
 assert(target_crsf_init() == HAL_OK); n = rc_wire(wire, CRSF_CH_VAL_MIN);
 uint32_t start = UINT32_MAX - 1000u;
 crsf_push(wire, n, start); target_crsf_poll(UINT32_MAX - 300u);
 target_crsf_snapshot(1000u, &snap); assert(snap.failsafe_active == 0u && snap.radio_age_ms == 1u);
 target_crsf_snapshot(200000u, &snap); assert(snap.failsafe_active == 1u);
 target_crsf_snapshot(UINT32_MAX - 300u, &snap); assert(snap.failsafe_active == 1u);
 crsf_push(wire, n, 300000u); target_crsf_poll(301000u);
 target_crsf_snapshot(301000u, &snap); assert(snap.failsafe_active == 0u);
 /* Overflow does not overwrite unread payload; reader catches up and can recover. */
 assert(target_crsf_init() == HAL_OK);
 for (uint32_t i = 0; i < 300u; ++i) { target_crsf_isr_push_byte(0xAAu, i); }
 HAL_UART_ErrorCallback(&g_huart_crsf); target_crsf_poll(400u);
 target_crsf_snapshot(400u, &snap); assert(snap.framing_errors == 46u);
 crsf_push(wire, n, 1000u); target_crsf_poll(2000u);
 target_crsf_snapshot(2000u, &snap); assert(snap.frames_ok == 1u);
 /* Full FSM policy c: stale radio including masked arm never disables balancing. */
 SafetyConfig cfg = safety_config_with_radio_policy(RADIO_LOSS_BRAKE_BALANCE);
 SafetyFsm fsm; SafetyOutput out; SafetyInputs in = clean_inputs();
 safety_fsm_init_with_config(&fsm, &cfg); fsm.state = SAFETY_STATE_BALANCING;
 in.v_bus_v = 24.0f; in.radio_age_ms = 151u; in.radio_arm_sw = 0u; in.pitch_deg = 15.0f;
 for (unsigned i = 0; i < 30; ++i) {
  safety_fsm_step(&fsm, &in, &out); assert(out.torque_enable == 1u && out.can_disable_cmd == 0u);
 }
 assert(out.state == SAFETY_STATE_CONTROLLED_STOP);
 in.pitch_deg = 0.0f;
 for (unsigned i = 0; i < 500; ++i) { safety_fsm_step(&fsm, &in, &out); assert(out.torque_enable == 1u); }
 assert(out.state == SAFETY_STATE_REST);
 in.radio_estop_sw = 1u; safety_fsm_step(&fsm, &in, &out); assert(out.torque_enable == 0u);
}
static void usb_command(uint8_t type, const uint8_t *payload, uint16_t len, uint32_t seq) {
 UsbLinkHeader hdr = {0}; uint8_t wire[512]; size_t wire_len = 0u;
 hdr.ver_major = USB_LINK_VER_MAJOR; hdr.ver_minor = USB_LINK_VER_MINOR;
 hdr.msg_type = type; hdr.seq = seq; hdr.payload_len = len;
 assert(usb_link_encode(&hdr, payload, wire, sizeof(wire), &wire_len) == USB_LINK_OK);
 target_usb_cdc_rx_push_bytes_isr(wire, (uint32_t)wire_len);
}
/* FW10: drive one control transfer through the HAL PCD callbacks the way the
 * OTG core would: SETUP, then one IN-complete per EP0 packet until the status
 * OUT is armed. Returns the number of EP0 IN data bytes. */
static uint32_t usb_setup(uint8_t bm, uint8_t req, uint16_t value, uint16_t index, uint16_t length) {
 uint8_t s[8] = { bm, req, (uint8_t)(value & 0xFFu), (uint8_t)(value >> 8),
                  (uint8_t)(index & 0xFFu), (uint8_t)(index >> 8),
                  (uint8_t)(length & 0xFFu), (uint8_t)(length >> 8) };
 memcpy(g_hpcd_usb.Setup, s, 8u);
 stub_ep0_in_len = 0u; stub_ep0_in_pkts = 0u;
 uint32_t armed0 = stub_ep0_status_out_armed;
 HAL_PCD_SetupStageCallback(&g_hpcd_usb);
 for (unsigned i = 0u; i < 16u && stub_ep0_status_out_armed == armed0; ++i) {
  HAL_PCD_DataInStageCallback(&g_hpcd_usb, 0u);
 }
 return stub_ep0_in_len;
}
static void usb_configure(void) {
 (void)usb_setup(0x00u, 0x09u, 1u, 0u, 0u); /* SET_CONFIGURATION(1) */
}
static void usb_test(void) {
 setup(); assert(target_usb_cdc_init() == HAL_OK); HAL_PCD_ResetCallback(&g_hpcd_usb);
 /* FW10: data endpoints open on SET_CONFIGURATION, not on bus reset. */
 LogTelemFast fast0 = {0};
 assert(target_usb_cdc_send_telem_fast(99u, 0u, &fast0) == LOG_OK);
 assert(stub_usb_calls == 0u);
 usb_configure();
 LogTelemFast fast = {0}; LogTelemHealth health = {0}; LogBenchRaw bench = {0};
 assert(target_usb_cdc_send_telem_fast(100u, 0u, &fast) == LOG_OK);
 assert(stub_usb_calls == 1u && stub_usb_len > 64u);
 uint8_t saved[512]; uint32_t saved_len = stub_usb_len;
 memcpy(saved, stub_usb_active, saved_len);
 uint8_t payload[52] = {0};
 assert(target_usb_cdc_send_telem_health(101u, 0u, &health) == LOG_OK);
 assert(target_usb_cdc_send_bench_raw(102u, 0u, &bench) == LOG_OK);
 assert(target_usb_cdc_send_frame(1u, 0u, 103u, payload, 6u) == USB_LINK_OK);
 assert(stub_usb_calls == 1u && memcmp(saved, stub_usb_active, saved_len) == 0);
 HAL_PCD_DataInStageCallback(&g_hpcd_usb, 1u);
 assert(target_usb_cdc_send_telem_health(104u, 0u, &health) == LOG_OK);
 assert(stub_usb_calls == 2u);
 HAL_PCD_DataInStageCallback(&g_hpcd_usb, 1u); stub_usb_fail = 1u;
 assert(target_usb_cdc_send_frame(1u, 0u, 105u, payload, 6u) == USB_LINK_ERR_OVERFLOW);
 stub_usb_fail = 0u;
 assert(target_usb_cdc_send_frame(1u, 0u, 106u, payload, 6u) == USB_LINK_OK);
 assert(stub_usb_calls == 3u); /* Failed HAL submission released claim. */
 TargetUsbHostCmd cmd;
 usb_command((uint8_t)USB_MSG_CMD_TWIST, payload, 52u, 1u);
 target_usb_cdc_poll(UINT32_MAX - 1000u, UINT32_MAX - 1000u, &cmd); assert(cmd.host_valid == 1u);
 target_usb_cdc_poll(1000u, UINT64_C(4294968296), &cmd); assert(cmd.host_valid == 1u);
 target_usb_cdc_poll(200000u, UINT64_C(4295167296), &cmd); assert(cmd.host_valid == 0u);
 target_usb_cdc_poll(UINT32_MAX - 1000u, UINT64_C(8589933590), &cmd); assert(cmd.host_valid == 0u);
 usb_command((uint8_t)USB_MSG_CMD_TWIST, payload, 52u, 2u);
 target_usb_cdc_poll(300000u, UINT64_C(8590234592), &cmd); assert(cmd.host_valid == 1u);
 uint8_t mode[6] = {0u, 1u, 0u, 0u, 1u, 0u};
 usb_command((uint8_t)USB_MSG_CMD_MODE, mode, 6u, 3u);
 target_usb_cdc_poll(302000u, 302000u, &cmd); assert(cmd.reset_fault_req == 1u);
 target_usb_cdc_poll(304000u, 304000u, &cmd); assert(cmd.reset_fault_req == 0u);
}
/* ---------------- FW10 scenarios ---------------- */
static uint32_t count_non_disable(void) {
 uint32_t n = 0u;
 for (uint32_t i = 0u; i < stub_tx_count; ++i) {
  if (bringup_frame_is_disable(8u, stub_tx[i].data) == 0u) { n++; }
 }
 return n;
}
static void mit_frame(uint8_t axis, float kp, float torque, uint8_t out[8]) {
 DamiaoAxisConfig cfg; DamiaoMitCmd c = {0};
 assert(damiao_get_axis_config(axis, &cfg) == DAMIAO_OK);
 c.kp = kp; c.torque = torque;
 assert(damiao_pack_mit_cmd(&cfg.limits, &c, out) == DAMIAO_OK);
}
static uint16_t cmd_id(uint8_t axis) {
 DamiaoAxisConfig cfg; assert(damiao_get_axis_config(axis, &cfg) == DAMIAO_OK);
 return cfg.cmd_can_id;
}
static uint8_t bus_of(uint8_t axis) {
 uint8_t b = 0u; assert(can_sched_bus_for_axis(axis, &b) == CAN_SCHED_OK); return b;
}
/* BENCH image: everything the scheduler can queue (enable, MIT torque) dies at
 * the hardware choke point; only 0xFF..FD frames reach the FDCAN TX FIFO. */
static void bench_tx_gate_test(void) {
 static BringupCtx ctx;
 bringup_init(&ctx, BRINGUP_MODE_BENCH);
 target_fdcan_bind_tx_gate(&ctx);
 setup();
 assert(can_sched_enqueue_enable_all(&sched, 0u) == CAN_SCHED_OK);
 target_fdcan_flush_tx(&sched, 0u);
 assert(stub_tx_count == 0u && target_fdcan_tx_gate_blocked() == 6u);
 const float tq[6] = {1.0f, -1.0f, 0.5f, 2.0f, -2.0f, 0.1f};
 assert(can_sched_enqueue_torques(&sched, tq, 10u) == CAN_SCHED_OK);
 target_fdcan_flush_tx(&sched, 0u);
 assert(stub_tx_count == 0u && target_fdcan_tx_gate_blocked() == 12u);
 uint8_t d[8];
 for (uint8_t a = 0u; a < 6u; ++a) {
  damiao_pack_enable(d);
  assert(target_fdcan_send_direct(bus_of(a), cmd_id(a), 8u, d) == HAL_ERROR);
  mit_frame(a, 0.0f, 0.05f, d);
  assert(target_fdcan_send_direct(bus_of(a), cmd_id(a), 8u, d) == HAL_ERROR);
  damiao_pack_disable(d);
  assert(target_fdcan_send_direct(bus_of(a), cmd_id(a), 8u, d) == HAL_OK);
 }
 assert(stub_tx_count == 6u && count_non_disable() == 0u);
 /* Disable to a wrong bus or a non-motor ID is also refused. */
 assert(target_fdcan_send_direct(2u, 0x001u, 8u, d) == HAL_ERROR);
 assert(target_fdcan_send_direct(bus_of(0u), 0x7FFu, 8u, d) == HAL_ERROR);
 /* The emergency path still delivers its disable burst through the gate. */
 for (unsigned b = 0u; b < 3u; ++b) { stub_free[b] = 8u; }
 stub_tx_count = 0u;
 stop_irq();
 assert(stub_tx_count == 6u && count_non_disable() == 0u);
}
/* MOTOR_ID image: only the authorised axis may get enable / pure-torque frames
 * within the cap, and the wired stop overrides the authorisation. */
static void motor_id_tx_gate_test(void) {
 static BringupCtx ctx;
 bringup_init(&ctx, BRINGUP_MODE_MOTOR_ID);
 target_fdcan_bind_tx_gate(&ctx);
 setup();
 BringupTickInput bin; BringupTickOutput bout;
 memset(&bin, 0, sizeof(bin));
 bin.now_us = 1000u; bin.fsm_disarmed = 1u;
 bin.req.valid = 1u; bin.req.seq = 7u; bin.req.rx_us = 1000u;
 bin.req.axis = 2u; bin.req.torque_nm = 0.1f; bin.req.duration_ms = 200u;
 bringup_step(&ctx, &bin, &bout);
 assert(ctx.tx_auth_axis == 2u && bout.n_frames >= 1u);
 uint8_t d[8];
 damiao_pack_enable(d);
 assert(target_fdcan_send_direct(bus_of(2u), cmd_id(2u), 8u, d) == HAL_OK);
 assert(target_fdcan_send_direct(bus_of(0u), cmd_id(0u), 8u, d) == HAL_ERROR);
 mit_frame(2u, 0.0f, 0.1f, d);
 assert(target_fdcan_send_direct(bus_of(2u), cmd_id(2u), 8u, d) == HAL_OK);
 mit_frame(2u, 0.0f, 0.5f, d);
 assert(target_fdcan_send_direct(bus_of(2u), cmd_id(2u), 8u, d) == HAL_ERROR);
 mit_frame(2u, 5.0f, 0.0f, d);
 assert(target_fdcan_send_direct(bus_of(2u), cmd_id(2u), 8u, d) == HAL_ERROR);
 mit_frame(5u, 0.0f, 0.1f, d);
 assert(target_fdcan_send_direct(bus_of(5u), cmd_id(5u), 8u, d) == HAL_ERROR);
 assert(stub_tx_count == 2u);
 /* Wired stop opens: even the authorised axis is refused (send_hw stop gate). */
 stub_stop_closed = 0u;
 mit_frame(2u, 0.0f, 0.1f, d);
 assert(target_fdcan_send_direct(bus_of(2u), cmd_id(2u), 8u, d) == HAL_ERROR);
 /* ... and the next bringup step aborts the pulse with disables on all axes. */
 bin.now_us = 3000u; bin.estop_pin_open = 1u;
 bringup_step(&ctx, &bin, &bout);
 /* 6 abort disables + 1 encoder-poll disable (cool-down), all 0xFF..FD. */
 assert(ctx.tx_auth_axis == BRINGUP_NO_AXIS && ctx.aborts == 1u && bout.n_frames == 7u);
 for (uint8_t f = 0u; f < bout.n_frames; ++f) {
  assert(bringup_frame_is_disable(bout.frames[f].dlc, bout.frames[f].data) == 1u);
 }
}
/* USB enumeration as a host would drive it, through the HAL PCD callbacks. */
static void usb_enum_test(void) {
 setup(); assert(target_usb_cdc_init() == HAL_OK); HAL_PCD_ResetCallback(&g_hpcd_usb);
 assert(usb_setup(0x80u, 0x06u, 0x0100u, 0u, 64u) == 18u);       /* device desc */
 assert(stub_ep0_in[0] == 18u && stub_ep0_in[1] == 1u && stub_ep0_in[7] == 64u);
 assert(stub_ep0_zlp == 0u && stub_ep0_status_out_armed == 1u);
 assert(usb_setup(0x00u, 0x05u, 5u, 0u, 0u) == 0u);              /* SET_ADDRESS */
 assert(stub_usb_address == 5u && stub_ep0_zlp == 1u);
 assert(usb_setup(0x80u, 0x06u, 0x0200u, 0u, 9u) == 9u);         /* config header */
 assert(stub_ep0_in[2] == 67u && stub_ep0_in[3] == 0u);
 assert(usb_setup(0x80u, 0x06u, 0x0200u, 0u, 255u) == 67u);      /* full config */
 assert(stub_ep0_in_pkts == 2u);                                  /* 64 + 3 */
 assert(usb_setup(0x80u, 0x06u, 0x0300u, 0u, 255u) == 4u);       /* LANGID */
 assert(usb_setup(0x80u, 0x06u, 0x0303u, 0x0409u, 255u) == 50u); /* serial */
 assert(stub_ep0_in[2] == '0' && stub_ep0_in[4] == '0' && stub_ep0_in[6] == '3');
 uint32_t stalls = stub_ep0_stalls;
 (void)usb_setup(0x80u, 0x06u, 0x0600u, 0u, 10u);                /* qualifier */
 assert(stub_ep0_stalls == stalls + 2u);                         /* IN + OUT */
 assert((stub_ep_open_mask & 0x00060002u) == 0u);                /* no data EPs yet */
 usb_configure();
 assert((stub_ep_open_mask & 0x00060002u) == 0x00060002u);       /* 0x01, 0x81, 0x82 */
 /* SET_LINE_CODING: OUT data stage, then status IN. */
 uint32_t zlp = stub_ep0_zlp;
 (void)usb_setup(0x21u, 0x20u, 0u, 0u, 7u);
 assert(stub_ep0_out_buf != NULL && stub_ep0_out_len == 7u && stub_ep0_zlp == zlp);
 const uint8_t lc[7] = {0x00u, 0x10u, 0x0Eu, 0x00u, 0u, 0u, 8u};  /* 921600 8N1 */
 memcpy(stub_ep0_out_buf, lc, 7u); stub_rx_count = 7u;
 HAL_PCD_DataOutStageCallback(&g_hpcd_usb, 0u);
 assert(stub_ep0_zlp == zlp + 1u);
 assert(usb_setup(0xA1u, 0x21u, 0u, 0u, 7u) == 7u && memcmp(stub_ep0_in, lc, 7u) == 0);
 (void)usb_setup(0x21u, 0x22u, 3u, 0u, 0u);                      /* DTR|RTS */
 assert(stub_ep0_zlp == zlp + 2u);
 stalls = stub_ep0_stalls;
 (void)usb_setup(0xC0u, 0x55u, 0u, 0u, 4u);                      /* vendor */
 assert(stub_ep0_stalls == stalls + 2u);
 LogTelemFast fast = {0};
 assert(target_usb_cdc_send_telem_fast(1u, 0u, &fast) == LOG_OK && stub_usb_calls == 1u);
 /* Bus reset closes the data path again. */
 HAL_PCD_ResetCallback(&g_hpcd_usb);
 HAL_PCD_DataInStageCallback(&g_hpcd_usb, 1u);
 assert(target_usb_cdc_send_telem_fast(2u, 0u, &fast) == LOG_OK && stub_usb_calls == 1u);
}
static void usb_motor_id_cmd_test(void) {
 setup(); assert(target_usb_cdc_init() == HAL_OK); HAL_PCD_ResetCallback(&g_hpcd_usb);
 TargetUsbHostCmd cmd;
 uint8_t p[8] = {3u, 0u, 150u, 0u, 0u, 0u, 0u, 0u};
 const float t = -0.15f; memcpy(&p[4], &t, 4u);
 usb_command((uint8_t)USB_MSG_CMD_MOTOR_ID, p, 8u, 41u);
 target_usb_cdc_poll(5000u, 5000u, &cmd);
 assert(cmd.motor_id_valid == 1u && cmd.motor_id_axis == 3u && cmd.motor_id_seq == 41u);
 assert(cmd.motor_id_duration_ms == 150u && cmd.motor_id_torque_nm == t);
 assert(cmd.motor_id_rx_us == 5000u);
 uint32_t errs = cmd.rx_proto_errors;
 usb_command((uint8_t)USB_MSG_CMD_MOTOR_ID, p, 6u, 42u);          /* short */
 target_usb_cdc_poll(6000u, 6000u, &cmd);
 assert(cmd.motor_id_seq == 41u && cmd.rx_proto_errors >= errs);
}
static void usb_feedforward_test(void) {
 setup(); assert(target_usb_cdc_init() == HAL_OK);
 uint8_t p[60] = {0};
 const float v = 0.2f, a = 0.15f, j = -0.3f;
 memcpy(p + 28, &v, 4u); memcpy(p + 52, &a, 4u); memcpy(p + 56, &j, 4u);
 usb_command((uint8_t)USB_MSG_CMD_TWIST, p, 60u, 1u);
 TargetUsbHostCmd cmd; BalanceTickInput in = {0};
 target_usb_cdc_poll(1000u, 1000u, &cmd);
 target_usb_apply_motion(&cmd, -0.1f, 0.0f, &in);
 assert(in.v_target_m_s == v && in.accel_ref_m_s2 == a && in.jerk_ref_m_s3 == j);
 /* Millisecond truncation and strict >150 comparison: valid at 150.999 ms. */
 target_usb_cdc_poll(151999u, 151999u, &cmd); assert(cmd.host_valid == 1u);
 target_usb_cdc_poll(152000u, 152000u, &cmd); assert(cmd.host_valid == 0u);
 target_usb_apply_motion(&cmd, -0.1f, 0.2f, &in);
 assert(in.v_target_m_s == -0.1f && in.yaw_target_rad_s == 0.2f);
 assert(in.accel_ref_m_s2 == 0.0f && in.jerk_ref_m_s3 == 0.0f);
 /* Bad extension cannot refresh freshness or replace accepted command. */
 float bad = NAN; memcpy(p + 52, &bad, 4u);
 usb_command((uint8_t)USB_MSG_CMD_TWIST, p, 60u, 2u);
 target_usb_cdc_poll(160000u, 160000u, &cmd);
 assert(cmd.host_valid == 0u && cmd.twist_frames_ok == 1u);
 memcpy(p + 52, &a, 4u);
 usb_command((uint8_t)USB_MSG_CMD_TWIST, p, 56u, 3u);
 target_usb_cdc_poll(180000u, 180000u, &cmd); assert(cmd.twist_frames_ok == 1u);
 bad = 10.01f; memcpy(p + 56, &bad, 4u);
 usb_command((uint8_t)USB_MSG_CMD_TWIST, p, 60u, 4u);
 target_usb_cdc_poll(200000u, 200000u, &cmd); assert(cmd.twist_frames_ok == 1u);
 /* Legacy velocity-only command clears stored feed-forward. */
 usb_command((uint8_t)USB_MSG_CMD_TWIST, p, 52u, 5u);
 target_usb_cdc_poll(220000u, 220000u, &cmd);
 target_usb_apply_motion(&cmd, 0.0f, 0.0f, &in);
 assert(in.host_valid == 1u && in.accel_ref_m_s2 == 0.0f && in.jerk_ref_m_s3 == 0.0f);
 puts("USB feed-forward reaches tick input; stale/legacy clears it; malformed rejected.");
}

static void diagnostic_power_test(void) {
 setup();
 for (unsigned mode = 1u; mode <= 2u; ++mode) {
  static BalanceCtrl ctrl;
  BalanceCtrlConfig cfg = balance_ctrl_default_config(); cfg.safety_cfg.diagnostic_only = 1u;
  balance_ctrl_init(&ctrl, &cfg);
  BringupCtx gate; bringup_init(&gate, (BringupMode)mode);
  BalanceTickInput in = {0}; target_power_unwired_inputs(&in);
  in.local_arm_req = in.local_reset_req = in.hw_reset_ack = 1u;
  in.radio_arm_sw = 1u;
  /* Even without input gating the diagnostic supervisor cannot arm. */
  BalanceTickOutput out; balance_ctrl_tick(&ctrl, &in, &out);
  assert(in.boot_self_test_ok == 0u && in.mirror_mismatch == 1u);
  assert(out.safety_out.state == SAFETY_STATE_DISARMED && out.safety_out.torque_enable == 0u);
  BringupTickInput bin = {0}; bin.now_us = 1000u; bin.fsm_disarmed = 1u;
  bin.req.valid = 1u; bin.req.seq = 1u; bin.req.rx_us = 1000u;
  bin.req.torque_nm = 0.1f; bin.req.duration_ms = 10u;
  BringupTickOutput bout; bringup_step(&gate, &bin, &bout);
  assert((gate.state == BRINGUP_ST_ENABLE) == (mode == 2u));
  /* A physical stop still latches ESTOP; watchdog/CAN faults still latch FAULT. */
  in.estop_open = 1u; balance_ctrl_tick(&ctrl, &in, &out);
  assert(out.safety_out.state == SAFETY_STATE_ESTOP);
  bin.fsm_disarmed = 0u; bin.now_us += 2000u; bringup_step(&gate, &bin, &bout);
  assert(gate.tx_auth_axis == BRINGUP_NO_AXIS);
  balance_ctrl_init(&ctrl, &cfg); in.estop_open = 0u; in.wdg_fault = 1u;
  balance_ctrl_tick(&ctrl, &in, &out);
  assert(out.safety_out.state == SAFETY_STATE_FAULT && out.safety_out.reason == SAFETY_REASON_WDG_FAULT);
 }
 puts("Unwired power inputs preserved; diagnostic DISARMED; MOTOR_ID reachable; stop aborts.");
}

static void loss_boundary_test(void) {
 setup(); assert(target_crsf_init() == HAL_OK);
 uint8_t wire[26]; size_t n = rc_wire(wire, CRSF_CH_VAL_MIN);
 crsf_push(wire, n, 1000u); target_crsf_poll(2000u);
 /* Final byte stamp = 1600 us. */
 TargetCrsfSnapshot snap; target_crsf_snapshot(152599u, &snap);
 assert(snap.radio_age_ms == 150u && snap.failsafe_active == 0u);
 SafetyFsm fsm; safety_fsm_init(&fsm); fsm.state = SAFETY_STATE_BALANCING;
 SafetyInputs in = clean_inputs(); in.v_bus_v = 24.0f; in.radio_arm_sw = 1u;
 target_crsf_apply_to_safety(&snap, &fsm.config, &in.radio_age_ms, &in.radio_arm_sw, &in.radio_estop_sw);
 /* Low arm switch would stop voluntarily, so hold high for the healthy boundary. */
 in.radio_arm_sw = 1u;
 SafetyOutput out; safety_fsm_step(&fsm, &in, &out); assert(out.state == SAFETY_STATE_BALANCING);
 target_crsf_snapshot(152600u, &snap); assert(snap.failsafe_active == 1u);
 target_crsf_apply_to_safety(&snap, &fsm.config, &in.radio_age_ms, &in.radio_arm_sw, &in.radio_estop_sw);
 safety_fsm_step(&fsm, &in, &out);
 assert(out.state == SAFETY_STATE_CONTROLLED_STOP && out.reason == SAFETY_REASON_RADIO_LOSS_BRAKE);
 assert(out.torque_enable == 1u);
 /* Host boundary through the same supervisor: age 150 -> active, 151 -> brake. */
 safety_fsm_init(&fsm); fsm.state = SAFETY_STATE_BALANCING;
 in = clean_inputs(); in.v_bus_v = 24.0f; in.radio_arm_sw = 1u; in.host_age_ms = 150u;
 safety_fsm_step(&fsm, &in, &out); assert(out.state == SAFETY_STATE_BALANCING);
 in.host_age_ms = 151u; in.host_valid = 0u; safety_fsm_step(&fsm, &in, &out);
 assert(out.state == SAFETY_STATE_CONTROLLED_STOP && out.reason == SAFETY_REASON_HOST_LOSS);
 puts("Radio/host: age 150 ms remains BALANCING; age 151 ms enters CONTROLLED_STOP with torque.");
}

int main(int argc, char **argv) {
 assert(argc == 2);
 if (strcmp(argv[1], "usb_feedforward") == 0) { usb_feedforward_test(); puts("PASS"); return 0; }
 if (strcmp(argv[1], "diagnostic_power") == 0) { diagnostic_power_test(); puts("PASS"); return 0; }
 if (strcmp(argv[1], "loss_boundary") == 0) { loss_boundary_test(); puts("PASS"); return 0; }
 if (strcmp(argv[1], "bench_tx_gate") == 0) { bench_tx_gate_test(); puts("PASS"); return 0; }
 if (strcmp(argv[1], "motor_id_tx_gate") == 0) { motor_id_tx_gate_test(); puts("PASS"); return 0; }
 if (strcmp(argv[1], "usb_enum") == 0) { usb_enum_test(); puts("PASS"); return 0; }
 if (strcmp(argv[1], "usb_motor_id_cmd") == 0) { usb_motor_id_cmd_test(); puts("PASS"); return 0; }
 if (strcmp(argv[1], "estop") == 0) { estop_test(); }
 else if (strcmp(argv[1], "opto_pending_permit") == 0) { opto_pending_permit_test(); }
 else if (strcmp(argv[1], "opto_pending_clear") == 0) { opto_pending_clear_test(); }
 else if (strcmp(argv[1], "opto_pending_tx") == 0) { opto_pending_tx_test(); }
 else if (strcmp(argv[1], "rev4_mapping") == 0) { rev4_mapping_test(); }
 else if (strcmp(argv[1], "emergency_clock") == 0) { emergency_clock_test(); }
 else if (strcmp(argv[1], "watchdog_config") == 0) { watchdog_config_test(); }
 else if (strcmp(argv[1], "fatal_halt_repeat") == 0) { fatal_halt_repeat_test(); }
 else if (strcmp(argv[1], "can") == 0) { can_test(); }
 else if (strcmp(argv[1], "rx") == 0) { rx_test(); }
 else if (strcmp(argv[1], "operator") == 0) { operator_test(); }
 else if (strcmp(argv[1], "watchdog") == 0) { watchdog_test(); }
 else if (strcmp(argv[1], "imu") == 0) { imu_test(); }
 else if (strcmp(argv[1], "radio") == 0) { radio_test(); }
 else if (strcmp(argv[1], "usb") == 0) { usb_test(); }
 else { assert(0); }
 puts("PASS"); return 0;
}
