#ifndef WBR_C_MSG_PACKERS_H
#define WBR_C_MSG_PACKERS_H

#include "log_packer.h"
#include "usb_link.h"

#include <stddef.h>
#include <stdint.h>

#define MSG_CMD_TWIST_SIZE       56u
#define MSG_CMD_MODE_SIZE        13u
#define MSG_TIME_SYNC_REQ_SIZE    8u
#define MSG_PARAM_OP_SIZE         7u
#define MSG_TIME_SYNC_RESP_SIZE  28u
#define MSG_FAULT_SNAPSHOT_SIZE  (16u + LOG_FLIGHT_RECORD_SIZE)

/* 0x01 CMD_TWIST (56 bytes: 3x u64 + 1x u32 + 7x f32) */
typedef struct {
    uint64_t session_id;
    uint64_t sent_us;
    uint64_t pose_us;
    uint32_t pose_seq;
    float    v_cmd;
    float    yaw_rate_cmd;
    float    height_cmd;
    float    roll_cmd;
    float    pose_x;
    float    pose_y;
    float    pose_yaw;
} MsgCmdTwist;

/* 0x02 CMD_MODE (13 bytes: u64 + u8 + u32) */
typedef struct {
    uint64_t session_id;
    uint8_t  action;
    uint32_t arg;
} MsgCmdMode;

/* 0x03 TIME_SYNC_REQ (8 bytes: u64) */
typedef struct {
    uint64_t pi_t1_us;
} MsgTimeSyncReq;

/* 0x04 PARAM_OP (7 bytes: u8 + u16 + f32) */
typedef struct {
    uint8_t  op;
    uint16_t param_id;
    float    value;
} MsgParamOp;

/* 0x83 TIME_SYNC_RESP (28 bytes: 3x u64 + u32) */
typedef struct {
    uint64_t pi_t1_us;
    uint64_t stm32_rx_us;
    uint64_t stm32_tx_us;
    uint32_t clock_err_bound_us;
} MsgTimeSyncResp;

/* 0x84 FAULT_SNAPSHOT (204 bytes: 16-byte snapshot header + 188-byte LogFlightRecord) */
typedef struct {
    uint32_t        trigger_reason;
    uint64_t        fault_timestamp_us;
    uint16_t        record_index;
    uint16_t        total_records;
    LogFlightRecord record;
} MsgFaultSnapshot;

LogStatus msg_pack_cmd_twist(const MsgCmdTwist *msg, uint8_t *dst, size_t dst_cap, size_t *out_len);
LogStatus msg_unpack_cmd_twist(const uint8_t *src, size_t src_len, MsgCmdTwist *out_msg);
LogStatus msg_frame_cmd_twist(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                              const MsgCmdTwist *msg,
                              uint8_t *wire_buf, size_t wire_cap, size_t *wire_len);

LogStatus msg_pack_cmd_mode(const MsgCmdMode *msg, uint8_t *dst, size_t dst_cap, size_t *out_len);
LogStatus msg_unpack_cmd_mode(const uint8_t *src, size_t src_len, MsgCmdMode *out_msg);
LogStatus msg_frame_cmd_mode(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                             const MsgCmdMode *msg,
                             uint8_t *wire_buf, size_t wire_cap, size_t *wire_len);

LogStatus msg_pack_time_sync_req(const MsgTimeSyncReq *msg, uint8_t *dst, size_t dst_cap, size_t *out_len);
LogStatus msg_unpack_time_sync_req(const uint8_t *src, size_t src_len, MsgTimeSyncReq *out_msg);
LogStatus msg_frame_time_sync_req(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                                  const MsgTimeSyncReq *msg,
                                  uint8_t *wire_buf, size_t wire_cap, size_t *wire_len);

LogStatus msg_pack_param_op(const MsgParamOp *msg, uint8_t *dst, size_t dst_cap, size_t *out_len);
LogStatus msg_unpack_param_op(const uint8_t *src, size_t src_len, MsgParamOp *out_msg);
LogStatus msg_frame_param_op(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                             const MsgParamOp *msg,
                             uint8_t *wire_buf, size_t wire_cap, size_t *wire_len);

LogStatus msg_pack_time_sync_resp(const MsgTimeSyncResp *msg, uint8_t *dst, size_t dst_cap, size_t *out_len);
LogStatus msg_unpack_time_sync_resp(const uint8_t *src, size_t src_len, MsgTimeSyncResp *out_msg);
LogStatus msg_frame_time_sync_resp(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                                   const MsgTimeSyncResp *msg,
                                   uint8_t *wire_buf, size_t wire_cap, size_t *wire_len);

LogStatus msg_pack_fault_snapshot(const MsgFaultSnapshot *msg, uint8_t *dst, size_t dst_cap, size_t *out_len);
LogStatus msg_unpack_fault_snapshot(const uint8_t *src, size_t src_len, MsgFaultSnapshot *out_msg);
LogStatus msg_frame_fault_snapshot(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                                   const MsgFaultSnapshot *msg,
                                   uint8_t *wire_buf, size_t wire_cap, size_t *wire_len);

#endif /* WBR_C_MSG_PACKERS_H */
