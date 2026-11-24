#include "target_estop.h"
#include "target_fdcan.h"
#include "target_crsf.h"
#include "target_usb_cdc.h"
#include "target_iwdg.h"
#include "target_safety_io.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

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
static void usb_test(void) {
 setup(); assert(target_usb_cdc_init() == HAL_OK); HAL_PCD_ResetCallback(&g_hpcd_usb);
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
int main(int argc, char **argv) {
 assert(argc == 2);
 if (strcmp(argv[1], "estop") == 0) { estop_test(); }
 else if (strcmp(argv[1], "opto_pending_permit") == 0) { opto_pending_permit_test(); }
 else if (strcmp(argv[1], "opto_pending_clear") == 0) { opto_pending_clear_test(); }
 else if (strcmp(argv[1], "opto_pending_tx") == 0) { opto_pending_tx_test(); }
 else if (strcmp(argv[1], "rev4_mapping") == 0) { rev4_mapping_test(); }
 else if (strcmp(argv[1], "emergency_clock") == 0) { emergency_clock_test(); }
 else if (strcmp(argv[1], "watchdog_config") == 0) { watchdog_config_test(); }
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
