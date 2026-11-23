#include "c_msg_packers.h"

#include <string.h>

static void wr_u8(uint8_t *buf, size_t *pos, uint8_t v)
{
    buf[*pos] = v;
    *pos += 1u;
}

static void wr_u16_le(uint8_t *buf, size_t *pos, uint16_t v)
{
    buf[*pos]      = (uint8_t)(v & 0xFFu);
    buf[*pos + 1u] = (uint8_t)((v >> 8u) & 0xFFu);
    *pos += 2u;
}

static void wr_u32_le(uint8_t *buf, size_t *pos, uint32_t v)
{
    buf[*pos]      = (uint8_t)(v & 0xFFu);
    buf[*pos + 1u] = (uint8_t)((v >> 8u) & 0xFFu);
    buf[*pos + 2u] = (uint8_t)((v >> 16u) & 0xFFu);
    buf[*pos + 3u] = (uint8_t)((v >> 24u) & 0xFFu);
    *pos += 4u;
}

static void wr_u64_le(uint8_t *buf, size_t *pos, uint64_t v)
{
    for (uint32_t i = 0u; i < 8u; ++i) {
        buf[*pos + i] = (uint8_t)((v >> (i * 8u)) & 0xFFu);
    }
    *pos += 8u;
}

static void wr_f32_le(uint8_t *buf, size_t *pos, float v)
{
    uint32_t bits = 0u;
    memcpy(&bits, &v, sizeof(bits));
    wr_u32_le(buf, pos, bits);
}

static uint8_t rd_u8(const uint8_t *buf, size_t *pos)
{
    const uint8_t v = buf[*pos];
    *pos += 1u;
    return v;
}

static uint16_t rd_u16_le(const uint8_t *buf, size_t *pos)
{
    const uint16_t v = (uint16_t)((uint16_t)buf[*pos] | ((uint16_t)buf[*pos + 1u] << 8u));
    *pos += 2u;
    return v;
}

static uint32_t rd_u32_le(const uint8_t *buf, size_t *pos)
{
    const uint32_t v = (uint32_t)buf[*pos]
                     | ((uint32_t)buf[*pos + 1u] << 8u)
                     | ((uint32_t)buf[*pos + 2u] << 16u)
                     | ((uint32_t)buf[*pos + 3u] << 24u);
    *pos += 4u;
    return v;
}

static uint64_t rd_u64_le(const uint8_t *buf, size_t *pos)
{
    uint64_t v = 0u;
    for (uint32_t i = 0u; i < 8u; ++i) {
        v |= ((uint64_t)buf[*pos + i] << (i * 8u));
    }
    *pos += 8u;
    return v;
}

static float rd_f32_le(const uint8_t *buf, size_t *pos)
{
    const uint32_t bits = rd_u32_le(buf, pos);
    float v = 0.0f;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

static LogStatus encode_msg_frame(uint8_t msg_type, uint32_t seq, uint64_t timestamp_us,
                                  uint8_t flags, const uint8_t *payload, uint16_t payload_len,
                                  uint8_t *wire_buf, size_t wire_cap, size_t *wire_len)
{
    UsbLinkHeader hdr;
    hdr.ver_major    = USB_LINK_VER_MAJOR;
    hdr.ver_minor    = USB_LINK_VER_MINOR;
    hdr.msg_type     = msg_type;
    hdr.flags        = flags;
    hdr.seq          = seq;
    hdr.timestamp_us = timestamp_us;
    hdr.payload_len  = payload_len;

    const UsbLinkStatus st = usb_link_encode(&hdr, payload, wire_buf, wire_cap, wire_len);
    if (st == USB_LINK_ERR_BUFFER_TOO_SMALL) {
        return LOG_ERR_BUFFER_TOO_SMALL;
    }
    return (st == USB_LINK_OK) ? LOG_OK : LOG_ERR_LINK_ENCODE;
}

LogStatus msg_pack_cmd_twist(const MsgCmdTwist *msg, uint8_t *dst, size_t dst_cap, size_t *out_len)
{
    if (msg == NULL || dst == NULL || out_len == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    *out_len = 0u;
    if (dst_cap < MSG_CMD_TWIST_SIZE) {
        return LOG_ERR_BUFFER_TOO_SMALL;
    }
    size_t pos = 0u;
    wr_u64_le(dst, &pos, msg->session_id);
    wr_u64_le(dst, &pos, msg->sent_us);
    wr_u64_le(dst, &pos, msg->pose_us);
    wr_u32_le(dst, &pos, msg->pose_seq);
    wr_f32_le(dst, &pos, msg->v_cmd);
    wr_f32_le(dst, &pos, msg->yaw_rate_cmd);
    wr_f32_le(dst, &pos, msg->height_cmd);
    wr_f32_le(dst, &pos, msg->roll_cmd);
    wr_f32_le(dst, &pos, msg->pose_x);
    wr_f32_le(dst, &pos, msg->pose_y);
    wr_f32_le(dst, &pos, msg->pose_yaw);
    *out_len = pos;
    return LOG_OK;
}

LogStatus msg_unpack_cmd_twist(const uint8_t *src, size_t src_len, MsgCmdTwist *out_msg)
{
    if (src == NULL || out_msg == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    if (src_len != MSG_CMD_TWIST_SIZE) {
        return LOG_ERR_BAD_LENGTH;
    }
    size_t pos = 0u;
    out_msg->session_id   = rd_u64_le(src, &pos);
    out_msg->sent_us      = rd_u64_le(src, &pos);
    out_msg->pose_us      = rd_u64_le(src, &pos);
    out_msg->pose_seq     = rd_u32_le(src, &pos);
    out_msg->v_cmd        = rd_f32_le(src, &pos);
    out_msg->yaw_rate_cmd = rd_f32_le(src, &pos);
    out_msg->height_cmd   = rd_f32_le(src, &pos);
    out_msg->roll_cmd     = rd_f32_le(src, &pos);
    out_msg->pose_x       = rd_f32_le(src, &pos);
    out_msg->pose_y       = rd_f32_le(src, &pos);
    out_msg->pose_yaw     = rd_f32_le(src, &pos);
    return LOG_OK;
}

LogStatus msg_frame_cmd_twist(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                              const MsgCmdTwist *msg,
                              uint8_t *wire_buf, size_t wire_cap, size_t *wire_len)
{
    uint8_t payload[MSG_CMD_TWIST_SIZE];
    size_t plen = 0u;
    const LogStatus st = msg_pack_cmd_twist(msg, payload, sizeof(payload), &plen);
    if (st != LOG_OK) {
        return st;
    }
    return encode_msg_frame((uint8_t)USB_MSG_CMD_TWIST, seq, timestamp_us, flags,
                            payload, (uint16_t)plen, wire_buf, wire_cap, wire_len);
}

LogStatus msg_pack_cmd_mode(const MsgCmdMode *msg, uint8_t *dst, size_t dst_cap, size_t *out_len)
{
    if (msg == NULL || dst == NULL || out_len == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    *out_len = 0u;
    if (dst_cap < MSG_CMD_MODE_SIZE) {
        return LOG_ERR_BUFFER_TOO_SMALL;
    }
    size_t pos = 0u;
    wr_u64_le(dst, &pos, msg->session_id);
    wr_u8(dst, &pos, msg->action);
    wr_u32_le(dst, &pos, msg->arg);
    *out_len = pos;
    return LOG_OK;
}

LogStatus msg_unpack_cmd_mode(const uint8_t *src, size_t src_len, MsgCmdMode *out_msg)
{
    if (src == NULL || out_msg == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    if (src_len != MSG_CMD_MODE_SIZE) {
        return LOG_ERR_BAD_LENGTH;
    }
    size_t pos = 0u;
    out_msg->session_id = rd_u64_le(src, &pos);
    out_msg->action     = rd_u8(src, &pos);
    out_msg->arg        = rd_u32_le(src, &pos);
    return LOG_OK;
}

LogStatus msg_frame_cmd_mode(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                             const MsgCmdMode *msg,
                             uint8_t *wire_buf, size_t wire_cap, size_t *wire_len)
{
    uint8_t payload[MSG_CMD_MODE_SIZE];
    size_t plen = 0u;
    const LogStatus st = msg_pack_cmd_mode(msg, payload, sizeof(payload), &plen);
    if (st != LOG_OK) {
        return st;
    }
    return encode_msg_frame((uint8_t)USB_MSG_CMD_MODE, seq, timestamp_us, flags,
                            payload, (uint16_t)plen, wire_buf, wire_cap, wire_len);
}

LogStatus msg_pack_time_sync_req(const MsgTimeSyncReq *msg, uint8_t *dst, size_t dst_cap, size_t *out_len)
{
    if (msg == NULL || dst == NULL || out_len == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    *out_len = 0u;
    if (dst_cap < MSG_TIME_SYNC_REQ_SIZE) {
        return LOG_ERR_BUFFER_TOO_SMALL;
    }
    size_t pos = 0u;
    wr_u64_le(dst, &pos, msg->pi_t1_us);
    *out_len = pos;
    return LOG_OK;
}

LogStatus msg_unpack_time_sync_req(const uint8_t *src, size_t src_len, MsgTimeSyncReq *out_msg)
{
    if (src == NULL || out_msg == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    if (src_len != MSG_TIME_SYNC_REQ_SIZE) {
        return LOG_ERR_BAD_LENGTH;
    }
    size_t pos = 0u;
    out_msg->pi_t1_us = rd_u64_le(src, &pos);
    return LOG_OK;
}

LogStatus msg_frame_time_sync_req(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                                  const MsgTimeSyncReq *msg,
                                  uint8_t *wire_buf, size_t wire_cap, size_t *wire_len)
{
    uint8_t payload[MSG_TIME_SYNC_REQ_SIZE];
    size_t plen = 0u;
    const LogStatus st = msg_pack_time_sync_req(msg, payload, sizeof(payload), &plen);
    if (st != LOG_OK) {
        return st;
    }
    return encode_msg_frame((uint8_t)USB_MSG_TIME_SYNC_REQ, seq, timestamp_us, flags,
                            payload, (uint16_t)plen, wire_buf, wire_cap, wire_len);
}

LogStatus msg_pack_param_op(const MsgParamOp *msg, uint8_t *dst, size_t dst_cap, size_t *out_len)
{
    if (msg == NULL || dst == NULL || out_len == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    *out_len = 0u;
    if (dst_cap < MSG_PARAM_OP_SIZE) {
        return LOG_ERR_BUFFER_TOO_SMALL;
    }
    size_t pos = 0u;
    wr_u8(dst, &pos, msg->op);
    wr_u16_le(dst, &pos, msg->param_id);
    wr_f32_le(dst, &pos, msg->value);
    *out_len = pos;
    return LOG_OK;
}

LogStatus msg_unpack_param_op(const uint8_t *src, size_t src_len, MsgParamOp *out_msg)
{
    if (src == NULL || out_msg == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    if (src_len != MSG_PARAM_OP_SIZE) {
        return LOG_ERR_BAD_LENGTH;
    }
    size_t pos = 0u;
    out_msg->op       = rd_u8(src, &pos);
    out_msg->param_id = rd_u16_le(src, &pos);
    out_msg->value    = rd_f32_le(src, &pos);
    return LOG_OK;
}

LogStatus msg_frame_param_op(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                             const MsgParamOp *msg,
                             uint8_t *wire_buf, size_t wire_cap, size_t *wire_len)
{
    uint8_t payload[MSG_PARAM_OP_SIZE];
    size_t plen = 0u;
    const LogStatus st = msg_pack_param_op(msg, payload, sizeof(payload), &plen);
    if (st != LOG_OK) {
        return st;
    }
    return encode_msg_frame((uint8_t)USB_MSG_PARAM_OP, seq, timestamp_us, flags,
                            payload, (uint16_t)plen, wire_buf, wire_cap, wire_len);
}

LogStatus msg_pack_time_sync_resp(const MsgTimeSyncResp *msg, uint8_t *dst, size_t dst_cap, size_t *out_len)
{
    if (msg == NULL || dst == NULL || out_len == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    *out_len = 0u;
    if (dst_cap < MSG_TIME_SYNC_RESP_SIZE) {
        return LOG_ERR_BUFFER_TOO_SMALL;
    }
    size_t pos = 0u;
    wr_u64_le(dst, &pos, msg->pi_t1_us);
    wr_u64_le(dst, &pos, msg->stm32_rx_us);
    wr_u64_le(dst, &pos, msg->stm32_tx_us);
    wr_u32_le(dst, &pos, msg->clock_err_bound_us);
    *out_len = pos;
    return LOG_OK;
}

LogStatus msg_unpack_time_sync_resp(const uint8_t *src, size_t src_len, MsgTimeSyncResp *out_msg)
{
    if (src == NULL || out_msg == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    if (src_len != MSG_TIME_SYNC_RESP_SIZE) {
        return LOG_ERR_BAD_LENGTH;
    }
    size_t pos = 0u;
    out_msg->pi_t1_us           = rd_u64_le(src, &pos);
    out_msg->stm32_rx_us        = rd_u64_le(src, &pos);
    out_msg->stm32_tx_us        = rd_u64_le(src, &pos);
    out_msg->clock_err_bound_us = rd_u32_le(src, &pos);
    return LOG_OK;
}

LogStatus msg_frame_time_sync_resp(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                                   const MsgTimeSyncResp *msg,
                                   uint8_t *wire_buf, size_t wire_cap, size_t *wire_len)
{
    uint8_t payload[MSG_TIME_SYNC_RESP_SIZE];
    size_t plen = 0u;
    const LogStatus st = msg_pack_time_sync_resp(msg, payload, sizeof(payload), &plen);
    if (st != LOG_OK) {
        return st;
    }
    return encode_msg_frame((uint8_t)USB_MSG_TIME_SYNC_RESP, seq, timestamp_us, flags,
                            payload, (uint16_t)plen, wire_buf, wire_cap, wire_len);
}

LogStatus msg_pack_fault_snapshot(const MsgFaultSnapshot *msg, uint8_t *dst, size_t dst_cap, size_t *out_len)
{
    if (msg == NULL || dst == NULL || out_len == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    *out_len = 0u;
    if (dst_cap < MSG_FAULT_SNAPSHOT_SIZE) {
        return LOG_ERR_BUFFER_TOO_SMALL;
    }
    size_t pos = 0u;
    wr_u32_le(dst, &pos, msg->trigger_reason);
    wr_u64_le(dst, &pos, msg->fault_timestamp_us);
    wr_u16_le(dst, &pos, msg->record_index);
    wr_u16_le(dst, &pos, msg->total_records);

    size_t rec_len = 0u;
    const LogStatus st = log_pack_flight_record(&msg->record, &dst[pos], dst_cap - pos, &rec_len);
    if (st != LOG_OK) {
        return st;
    }
    *out_len = pos + rec_len;
    return LOG_OK;
}

LogStatus msg_unpack_fault_snapshot(const uint8_t *src, size_t src_len, MsgFaultSnapshot *out_msg)
{
    if (src == NULL || out_msg == NULL) {
        return LOG_ERR_NULL_PTR;
    }
    if (src_len == LOG_FLIGHT_RECORD_SIZE) {
        const LogStatus st = log_unpack_flight_record(src, src_len, &out_msg->record);
        if (st != LOG_OK) {
            return st;
        }
        out_msg->trigger_reason     = out_msg->record.fast.fault_mask;
        out_msg->fault_timestamp_us = out_msg->record.timestamp_us;
        out_msg->record_index       = 0u;
        out_msg->total_records      = 1u;
        return LOG_OK;
    }
    if (src_len != MSG_FAULT_SNAPSHOT_SIZE) {
        return LOG_ERR_BAD_LENGTH;
    }
    size_t pos = 0u;
    out_msg->trigger_reason     = rd_u32_le(src, &pos);
    out_msg->fault_timestamp_us = rd_u64_le(src, &pos);
    out_msg->record_index       = rd_u16_le(src, &pos);
    out_msg->total_records      = rd_u16_le(src, &pos);
    return log_unpack_flight_record(&src[pos], src_len - pos, &out_msg->record);
}

LogStatus msg_frame_fault_snapshot(uint32_t seq, uint64_t timestamp_us, uint8_t flags,
                                   const MsgFaultSnapshot *msg,
                                   uint8_t *wire_buf, size_t wire_cap, size_t *wire_len)
{
    uint8_t payload[MSG_FAULT_SNAPSHOT_SIZE];
    size_t plen = 0u;
    const LogStatus st = msg_pack_fault_snapshot(msg, payload, sizeof(payload), &plen);
    if (st != LOG_OK) {
        return st;
    }
    return encode_msg_frame((uint8_t)USB_MSG_FAULT_SNAPSHOT, seq, timestamp_us, flags,
                            payload, (uint16_t)plen, wire_buf, wire_cap, wire_len);
}
