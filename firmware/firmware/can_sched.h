/*
 * 3-Bus x 2-Motor CAN TX Scheduler, RX Dispatcher, Bus-Off & Timeout Monitor
 *
 * Implements FIRMWARE_DESIGN_v2.md Section 4.2 (3-bus x 2-motor map, globally
 * unique IDs 0x001-0x006 / 0x011-0x016) and Section 4.3 (priority TX queue,
 * strict per-bus RX filter, AutoRetransmission=DISABLE periodic replacement,
 * emergency TX abort + 0 ms / +1 ms repeat 0xFF..FD disable, ErrorPassive jump
 * inhibit, latched BusOff with DISARMED-only manual recovery, dual-clock
 * supervisor 10 ms / drive 20 ms timeout monitoring, and tear-free S11 triple-
 * buffered ISR mailbox integration).
 *
 * Uses the existing damiao_mit codec (firmware/damiao_mit.h) and verified S11
 * packet policy (firmware/s11_packet_policy.h).
 * Pure C11, zero HAL/RTOS dependencies, zero dynamic memory, fixed-width types.
 */

#ifndef WBR_CAN_SCHED_H
#define WBR_CAN_SCHED_H

#include "damiao_mit.h"
#include "s11_packet_policy.h"

#include <stdint.h>

#define CAN_SCHED_NUM_BUSES              3u
#define CAN_SCHED_MOTORS_PER_BUS         2u
#define CAN_SCHED_NUM_AXES               DAMIAO_NUM_AXES
#define CAN_SCHED_TX_QUEUE_CAP           8u   /* 8 priority TX elements per bus (Sec 4.3) */
#define CAN_SCHED_DISABLE_REPEAT_US      1000u /* Repeat 0xFF..FD at t = +1 ms (Sec 4.3) */
#define CAN_SCHED_SUPERVISOR_TIMEOUT_US  10000u /* 10 ms S11 supervisor cut_hold_ms limit (Sec 4.3) */
#define CAN_SCHED_DRIVE_TIMEOUT_US       20000u /* 20 ms DAMIAO reg 9 TIMEOUT target (Sec 4.3) */
#define CAN_SCHED_ERR_PASSIVE_THRESH     128u
#define CAN_SCHED_BUS_OFF_THRESH         256u

typedef enum {
    CAN_SCHED_OK = 0,
    CAN_SCHED_ERR_NULL_PTR = 1,
    CAN_SCHED_ERR_INVALID_BUS = 2,
    CAN_SCHED_ERR_INVALID_AXIS = 3,
    CAN_SCHED_ERR_QUEUE_EMPTY = 4,
    CAN_SCHED_ERR_QUEUE_FULL = 5,
    CAN_SCHED_ERR_BUS_OFF = 6,
    CAN_SCHED_ERR_REJECTED_FRAME = 7,
    CAN_SCHED_ERR_CODEC = 8,
    CAN_SCHED_ERR_NOT_DISARMED = 9,
    CAN_SCHED_ERR_RECOVERY_INCOMPLETE = 10
} CanSchedStatus;

typedef struct {
    uint16_t std_id;
    uint8_t  dlc;
    uint8_t  is_extended;
    uint8_t  is_rtr;
    uint8_t  data[DAMIAO_CAN_DLC];
} CanFrame;

typedef struct {
    CanFrame frame;
    uint32_t enqueue_us;
    uint8_t  axis;
    uint8_t  is_emergency;
    uint8_t  used;
} CanTxSlot;

typedef struct {
    CanTxSlot tx_queue[CAN_SCHED_TX_QUEUE_CAP];
    uint8_t   tx_count;
    uint16_t  tec;
    uint16_t  rec;
    uint8_t   error_warning;
    uint8_t   error_passive;
    uint8_t   bus_off_hw;
    uint8_t   bus_off_latched;
    uint8_t   recovery_requested;
    uint16_t  bus_off_count;
    uint16_t  tx_drop_count;
    uint16_t  rx_drop_count;
    uint16_t  rx_rejected_count;
    uint16_t  abort_count;
} CanBusState;

typedef struct {
    DamiaoMitFeedback fb;
    uint32_t          rx_us;
    uint32_t          seq;
    uint8_t           seen;
    uint8_t           valid;
    uint8_t           healthy;
} CanAxisRxSlot;

typedef struct {
    uint8_t  expected_enabled_status; /* Default DAMIAO_DRIVE_STATUS_ENABLED_DEFAULT (0x1) */
    uint32_t supervisor_timeout_us;   /* Default 10000 us (10 ms) */
    uint32_t drive_timeout_us;        /* Default 20000 us (20 ms) */
    uint32_t disable_repeat_us;       /* Default 1000 us (+1 ms repeat 0xFD) */
} CanSchedConfig;

typedef struct {
    CanSchedConfig cfg;
    CanBusState    bus[CAN_SCHED_NUM_BUSES];
    CanAxisRxSlot  axis_rx[CAN_SCHED_NUM_AXES];
    S11IsrMailbox  s11_mb;
    uint32_t       last_consumed_seq[CAN_SCHED_NUM_AXES];
    uint8_t        last_consumed_mask;
    uint8_t        any_bus_off_latched;
    uint8_t        disable_repeat_armed;
    uint32_t       disable_first_us;
} CanSched;

typedef struct {
    uint32_t         fresh_mask;
    uint32_t         valid_mask;
    uint32_t         healthy_mask;
    uint32_t         status_unknown_mask;
    uint32_t         supervisor_timeout_mask;
    uint32_t         drive_timeout_mask;
    double           age_ms[CAN_SCHED_NUM_AXES];
    double           angle[CAN_SCHED_NUM_AXES];
    double           rate[CAN_SCHED_NUM_AXES];
    float            torque_fb[CAN_SCHED_NUM_AXES];
    uint32_t         seq[CAN_SCHED_NUM_AXES];
    uint8_t          motor_state[CAN_SCHED_NUM_AXES];
    uint8_t          t_mos_c[CAN_SCHED_NUM_AXES];
    uint8_t          t_coil_c[CAN_SCHED_NUM_AXES];
    uint8_t          drives_healthy_6;
    S11InputSnapshot s11_snap;
} CanSchedRxSnapshot;

typedef struct {
    uint8_t  any_bus_off;
    uint8_t  any_error_passive;
    uint8_t  any_error_warning;
    uint8_t  inhibit_jump;
    uint8_t  bus_off_mask;
    uint8_t  error_passive_mask;
    uint8_t  tec[CAN_SCHED_NUM_BUSES];
    uint8_t  rec[CAN_SCHED_NUM_BUSES];
    uint16_t bus_off_cnt[CAN_SCHED_NUM_BUSES];
    uint16_t rx_drop_cnt[CAN_SCHED_NUM_BUSES];
} CanSchedHealth;

CanSchedConfig can_sched_default_config(void);
void can_sched_init(CanSched *sched, const CanSchedConfig *cfg);

/* Topology helpers matching FIRMWARE_DESIGN_v2.md Section 4.2 (0-based bus_idx 0..2). */
CanSchedStatus can_sched_bus_for_axis(uint8_t axis, uint8_t *out_bus_idx);
CanSchedStatus can_sched_axes_for_bus(uint8_t bus_idx, uint8_t out_axes[CAN_SCHED_MOTORS_PER_BUS]);

/* Per-bus priority TX scheduling. */
CanSchedStatus can_sched_enqueue_torques(CanSched *sched,
                                         const float torque_nm[CAN_SCHED_NUM_AXES],
                                         uint32_t now_us);
CanSchedStatus can_sched_enqueue_mit_cmds(CanSched *sched,
                                          const DamiaoMitCmd cmds[CAN_SCHED_NUM_AXES],
                                          uint32_t now_us);
CanSchedStatus can_sched_enqueue_enable_all(CanSched *sched, uint32_t now_us);

/* Emergency disable: abort pending TX on all 3 buses, enqueue 0xFF..FD immediately on
 * all non-bus-off buses, and arm the +1 ms repeat transmission per Section 4.3. */
CanSchedStatus can_sched_emergency_disable(CanSched *sched, uint32_t now_us);

/* Emit scheduled +1 ms repeat 0xFF..FD disable burst once elapsed. */
void can_sched_service_tx(CanSched *sched, uint32_t now_us);

/* Pop highest-priority (lowest std_id) TX frame from bus_idx (0..2). */
CanSchedStatus can_sched_pop_tx(CanSched *sched, uint8_t bus_idx, CanFrame *out_frame);
uint8_t        can_sched_tx_pending(const CanSched *sched, uint8_t bus_idx);

/* Strict per-bus RX filter & DAMIAO feedback dispatch (bus_idx 0..2) into S11IsrMailbox. */
CanSchedStatus can_sched_rx_frame(CanSched *sched, uint8_t bus_idx,
                                  const CanFrame *frame, uint32_t rx_us);

/* Snapshot all 6 per-axis RX slots via S11IsrMailbox with wrap-safe u32 age & timeout classification. */
CanSchedStatus can_sched_snapshot_rx(CanSched *sched, uint32_t now_us,
                                     CanSchedRxSnapshot *out_snap);

/* 500 Hz tick helper: snapshot tear-free S11IsrMailbox and step verified S11 packet policy. */
CanSchedStatus can_sched_step_s11_500hz(CanSched *sched, S11PacketPolicy *s11,
                                        uint32_t now_us, double dt_ms,
                                        CanSchedRxSnapshot *out_snap,
                                        S11PacketFrame *out_frame);

/* Poll/update hardware FDCAN TEC/REC/ProtocolStatus for bus_idx (0..2).
 * BusOff latches immediately (no auto-recovery) and triggers emergency disable
 * on all remaining healthy buses per Section 4.3. */
CanSchedStatus can_sched_update_bus_status(CanSched *sched, uint8_t bus_idx,
                                           uint16_t tec, uint16_t rec,
                                           uint8_t hw_bus_off,
                                           uint8_t hw_err_passive,
                                           uint8_t hw_err_warning,
                                           uint32_t now_us);

/* DISARMED-only manual bus-off recovery sequence (Section 4.3). */
CanSchedStatus can_sched_request_bus_recovery(CanSched *sched, uint8_t is_disarmed);
CanSchedStatus can_sched_confirm_bus_recovery(CanSched *sched, uint8_t bus_idx,
                                              uint8_t hw_bus_off,
                                              uint8_t all_6_motors_answered);

void can_sched_get_health(const CanSched *sched, CanSchedHealth *out_health);

#endif /* WBR_CAN_SCHED_H */
