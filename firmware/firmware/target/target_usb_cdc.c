/*
 * STM32H723 USB OTG Full-Speed CDC ACM Driver with COBS + CRC-32C Framing
 *
 * Bridges the STM32H7 HAL PCD driver (`USB1_OTG_HS` in Full-Speed internal PHY
 * mode on PA11/PA12, ASSUMED pending HW1) to the portable COBS + CRC-32C link
 * (`firmware/usb_link.c`) and binary telemetry packer (`firmware/log_packer.c`).
 */

#include "target_usb_cdc.h"
#include "target_sync.h"
#include "usb_cdc_ep0.h"
#include <math.h>
#include <string.h>

PCD_HandleTypeDef g_hpcd_usb;

static volatile uint8_t  s_usb_rx_ring[TARGET_USB_RX_RING_SIZE];
static volatile uint32_t s_usb_rx_head = 0u;
static volatile uint32_t s_usb_rx_tail = 0u;

static uint8_t           s_ep1_rx_pkt[TARGET_USB_FS_MAX_PACKET_SIZE];
static uint8_t           s_ep0_out_buf[16];
static uint8_t           s_ep0_out_pending = 0u;
static UsbCdcEp0         s_ep0;      /* FW10: chapter-9 + CDC request handling */
static UsbEp0InXfer      s_ep0_in;
static uint8_t           s_usb_tx_wire_buf[TARGET_USB_TX_BUF_SIZE];
static volatile uint8_t  s_usb_tx_busy = 0u;
static volatile uint8_t  s_usb_configured = 0u;

static UsbLinkRxParser   s_rx_parser;
static TargetUsbHostCmd  s_host_state;
static uint8_t           s_has_twist = 0u;
static uint8_t           s_twist_timed_out = 1u;
static uint32_t          s_last_twist_seq = 0u;
static uint32_t          s_last_twist_rx_us = 0u;

static uint32_t          s_tx_seq_fast = 0u;
static uint32_t          s_tx_seq_health = 0u;
static uint32_t          s_tx_seq_bench = 0u;
static uint32_t          s_tx_seq_generic = 0u;

/* Little-endian helpers matching firmware/log_packer.c */
static uint32_t unpack_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8u) |
           ((uint32_t)p[2] << 16u) |
           ((uint32_t)p[3] << 24u);
}

static uint64_t unpack_u64_le(const uint8_t *p)
{
    return (uint64_t)unpack_u32_le(p) |
           ((uint64_t)unpack_u32_le(p + 4u) << 32u);
}

static float unpack_f32_le(const uint8_t *p)
{
    uint32_t bits = unpack_u32_le(p);
    float val = 0.0f;
    memcpy(&val, &bits, sizeof(float));
    return val;
}

static void pack_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8u) & 0xFFu);
    p[2] = (uint8_t)((v >> 16u) & 0xFFu);
    p[3] = (uint8_t)((v >> 24u) & 0xFFu);
}

static void pack_u64_le(uint8_t *p, uint64_t v)
{
    pack_u32_le(p, (uint32_t)(v & 0xFFFFFFFFu));
    pack_u32_le(p + 4u, (uint32_t)((v >> 32u) & 0xFFFFFFFFu));
}

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    GPIO_InitTypeDef gpio;
    memset(&gpio, 0, sizeof(gpio));

    if (hpcd->Instance == TARGET_USB_INSTANCE) {
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA9 is the VBUS divider sense input; never use it as USART1_TX. */
        gpio.Pin  = TARGET_USB_VBUS_SENSE_PIN;
        gpio.Mode = GPIO_MODE_INPUT;
        gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(TARGET_USB_VBUS_SENSE_GPIO_PORT, &gpio);

        /* PA11 (DM) / PA12 (DP) in AF10 OTG1_FS mode. */
        gpio.Pin       = TARGET_USB_DM_PIN | TARGET_USB_DP_PIN;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = TARGET_USB_GPIO_AF;
        HAL_GPIO_Init(TARGET_USB_GPIO_PORT, &gpio);

        __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();
        HAL_PWREx_EnableUSBVoltageDetector();

        HAL_NVIC_SetPriority(TARGET_USB_IRQn, TARGET_NVIC_PRIO_USB_CDC, 0u);
        HAL_NVIC_EnableIRQ(TARGET_USB_IRQn);
    }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->Instance == TARGET_USB_INSTANCE) {
        __HAL_RCC_USB1_OTG_HS_CLK_DISABLE();
        HAL_GPIO_DeInit(TARGET_USB_GPIO_PORT, TARGET_USB_DM_PIN | TARGET_USB_DP_PIN);
        HAL_NVIC_DisableIRQ(TARGET_USB_IRQn);
    }
}

HAL_StatusTypeDef target_usb_cdc_init(void)
{
    s_usb_rx_head      = 0u;
    s_usb_rx_tail      = 0u;
    s_usb_tx_busy      = 0u;
    s_usb_configured   = 0u;
    s_has_twist        = 0u;
    s_twist_timed_out  = 1u;
    s_last_twist_seq   = 0u;
    s_last_twist_rx_us = 0u;
    s_tx_seq_fast      = 0u;
    s_tx_seq_health    = 0u;
    s_tx_seq_bench     = 0u;
    s_tx_seq_generic   = 0u;

    const uint32_t uid[3] = { HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2() };
    usb_cdc_ep0_init(&s_ep0, uid);
    memset(&s_ep0_in, 0, sizeof(s_ep0_in));
    s_ep0_out_pending  = 0u;

    usb_link_rx_init(&s_rx_parser);
    memset(&s_host_state, 0, sizeof(s_host_state));
    s_host_state.host_age_ms = 0xFFFFFFFFu;
    s_host_state.host_valid  = 0u;

    memset(&g_hpcd_usb, 0, sizeof(g_hpcd_usb));
    g_hpcd_usb.Instance                     = TARGET_USB_INSTANCE;
    g_hpcd_usb.Init.dev_endpoints           = 6u;
    g_hpcd_usb.Init.speed                   = PCD_SPEED_FULL;
    g_hpcd_usb.Init.dma_enable              = DISABLE;
    g_hpcd_usb.Init.ep0_mps                 = USB_OTG_MAX_EP0_SIZE;
    g_hpcd_usb.Init.phy_itface              = PCD_PHY_EMBEDDED;
    g_hpcd_usb.Init.Sof_enable              = DISABLE;
    g_hpcd_usb.Init.low_power_enable        = DISABLE;
    g_hpcd_usb.Init.lpm_enable              = DISABLE;
    g_hpcd_usb.Init.battery_charging_enable = DISABLE;
    g_hpcd_usb.Init.vbus_sensing_enable     = DISABLE;
    g_hpcd_usb.Init.use_dedicated_ep1       = DISABLE;

    if (HAL_PCD_Init(&g_hpcd_usb) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Configure USB OTG FIFO allocation (in 32-bit words). */
    HAL_PCDEx_SetRxFiFo(&g_hpcd_usb, 128u);
    HAL_PCDEx_SetTxFiFo(&g_hpcd_usb, 0u, 64u);
    HAL_PCDEx_SetTxFiFo(&g_hpcd_usb, 1u, 128u);
    HAL_PCDEx_SetTxFiFo(&g_hpcd_usb, 2u, 32u);

    if (HAL_PCD_Start(&g_hpcd_usb) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    /* Bus reset: address 0, unconfigured, only EP0 open (FW10; the data
     * endpoints open on SET_CONFIGURATION(1), not on reset). */
    (void)HAL_PCD_EP_Open(hpcd, 0x00u, USB_OTG_MAX_EP0_SIZE, EP_TYPE_CTRL);
    (void)HAL_PCD_EP_Open(hpcd, 0x80u, USB_OTG_MAX_EP0_SIZE, EP_TYPE_CTRL);
    usb_cdc_ep0_bus_reset(&s_ep0);
    memset(&s_ep0_in, 0, sizeof(s_ep0_in));
    s_ep0_out_pending = 0u;
    s_usb_configured  = 0u;
    s_usb_tx_busy     = 0u;
}

static void usb_open_data_endpoints(PCD_HandleTypeDef *hpcd)
{
    (void)HAL_PCD_EP_Open(hpcd, TARGET_USB_CDC_OUT_EP, TARGET_USB_FS_MAX_PACKET_SIZE, EP_TYPE_BULK);
    (void)HAL_PCD_EP_Open(hpcd, TARGET_USB_CDC_IN_EP,  TARGET_USB_FS_MAX_PACKET_SIZE, EP_TYPE_BULK);
    (void)HAL_PCD_EP_Open(hpcd, TARGET_USB_CDC_CMD_EP, USB_CDC_NOTIFY_MPS, EP_TYPE_INTR);
    s_usb_tx_busy    = 0u;
    s_usb_configured = 1u;
    /* Arm Bulk OUT endpoint for the first incoming packet */
    (void)HAL_PCD_EP_Receive(hpcd, TARGET_USB_CDC_OUT_EP,
                             s_ep1_rx_pkt, TARGET_USB_FS_MAX_PACKET_SIZE);
}

static void usb_close_data_endpoints(PCD_HandleTypeDef *hpcd)
{
    s_usb_configured = 0u;
    (void)HAL_PCD_EP_Close(hpcd, TARGET_USB_CDC_OUT_EP);
    (void)HAL_PCD_EP_Close(hpcd, TARGET_USB_CDC_IN_EP);
    (void)HAL_PCD_EP_Close(hpcd, TARGET_USB_CDC_CMD_EP);
    s_usb_tx_busy = 0u;
}

static void usb_ep0_send_next(PCD_HandleTypeDef *hpcd)
{
    const uint8_t *chunk = NULL;
    uint16_t len = 0u;
    const UsbEp0InStep st = usb_ep0_in_next(&s_ep0_in, &chunk, &len);
    if (st == USB_EP0_IN_SEND) {
        (void)HAL_PCD_EP_Transmit(hpcd, 0x80u, (uint8_t *)chunk, len);
    } else if (st == USB_EP0_IN_STATUS) {
        (void)HAL_PCD_EP_Receive(hpcd, 0x00u, NULL, 0u); /* zero-length status OUT */
    }
}

/* FW10 (closes the FW9 "TODO F-04" partial EP0): chapter-9 standard requests,
 * descriptors and CDC class requests come from firmware/usb_cdc_ep0.c. */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    UsbEp0Action act;
    usb_cdc_ep0_setup(&s_ep0, (const uint8_t *)hpcd->Setup, &act);
    s_ep0_out_pending = 0u;
    memset(&s_ep0_in, 0, sizeof(s_ep0_in));

    switch (act.kind) {
    case USB_EP0_ACT_STATUS_IN:
        if (act.set_address != 0u) {
            (void)HAL_PCD_SetAddress(hpcd, act.address);
        }
        if (act.configure == 1u) {
            usb_open_data_endpoints(hpcd);
        } else if (act.configure == 2u) {
            usb_close_data_endpoints(hpcd);
        }
        if (act.clear_halt_ep != 0u) {
            (void)HAL_PCD_EP_ClrStall(hpcd, act.clear_halt_ep);
        }
        if (act.set_halt_ep != 0u) {
            (void)HAL_PCD_EP_SetStall(hpcd, act.set_halt_ep);
        }
        (void)HAL_PCD_EP_Transmit(hpcd, 0x80u, NULL, 0u);
        break;
    case USB_EP0_ACT_DATA_IN:
        usb_ep0_in_begin(&s_ep0_in, act.data, act.len, act.zlp);
        usb_ep0_send_next(hpcd);
        break;
    case USB_EP0_ACT_DATA_OUT:
        s_ep0_out_pending = 1u;
        (void)HAL_PCD_EP_Receive(hpcd, 0x00u, s_ep0_out_buf, act.len);
        break;
    case USB_EP0_ACT_STALL:
    default:
        (void)HAL_PCD_EP_SetStall(hpcd, 0x80u);
        (void)HAL_PCD_EP_SetStall(hpcd, 0x00u);
        break;
    }
}

void target_usb_cdc_rx_push_bytes_isr(const uint8_t *data, uint32_t len)
{
    uint32_t head = s_usb_rx_head;
    uint32_t tail = s_usb_rx_tail;

    if (data == NULL) {
        return;
    }

    for (uint32_t i = 0u; i < len; ++i) {
        uint32_t next = (head + 1u) & (TARGET_USB_RX_RING_SIZE - 1u);
        if (next == tail) {
            break; /* Ring full; drop excess byte */
        }
        s_usb_rx_ring[head] = data[i];
        head = next;
    }
    __DMB();
    s_usb_rx_head = head;
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    if (epnum == 0u) {
        if (s_ep0_out_pending != 0u) {
            /* SET_LINE_CODING data stage complete -> status IN. */
            uint32_t n = HAL_PCD_EP_GetRxCount(hpcd, 0x00u);
            if (n > sizeof(s_ep0_out_buf)) {
                n = sizeof(s_ep0_out_buf);
            }
            s_ep0_out_pending = 0u;
            (void)usb_cdc_ep0_out_data(&s_ep0, s_ep0_out_buf, (uint16_t)n);
            (void)HAL_PCD_EP_Transmit(hpcd, 0x80u, NULL, 0u);
        }
        /* else: zero-length status OUT of an IN transfer; HAL re-arms SETUP. */
        return;
    }
    if (epnum == (TARGET_USB_CDC_OUT_EP & 0x7Fu)) {
        uint32_t rx_cnt = HAL_PCD_EP_GetRxCount(hpcd, TARGET_USB_CDC_OUT_EP);
        if (rx_cnt > TARGET_USB_FS_MAX_PACKET_SIZE) {
            rx_cnt = TARGET_USB_FS_MAX_PACKET_SIZE;
        }
        target_usb_cdc_rx_push_bytes_isr(s_ep1_rx_pkt, rx_cnt);

        /* Re-arm Bulk OUT endpoint for the next USB packet */
        (void)HAL_PCD_EP_Receive(hpcd, TARGET_USB_CDC_OUT_EP,
                                 s_ep1_rx_pkt, TARGET_USB_FS_MAX_PACKET_SIZE);
    }
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    if (epnum == 0u) {
        usb_ep0_send_next(hpcd); /* next 64-byte chunk, ZLP, or arm status OUT */
        return;
    }
    if (epnum == (TARGET_USB_CDC_IN_EP & 0x7Fu)) {
        s_usb_tx_busy = 0u;
    }
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    (void)hpcd;
    s_usb_configured = 0u;
    s_usb_tx_busy    = 0u;
}

static void target_usb_handle_cmd_twist(uint32_t now_us,
                                        const UsbLinkHeader *hdr,
                                        const uint8_t *payload)
{
    if (hdr->payload_len != TARGET_USB_CMD_TWIST_SIZE
        && hdr->payload_len != TARGET_USB_CMD_TWIST_FF_SIZE) {
        s_host_state.rx_proto_errors++;
        return;
    }

    /* Enforce strictly increasing sequence number after first frame */
    if (s_has_twist != 0u) {
        int32_t seq_diff = (int32_t)(hdr->seq - s_last_twist_seq);
        if (seq_diff <= 0) {
            s_host_state.twist_seq_drops++;
            return;
        }
        /* S12 50 Hz rate gate: reject twist packets arriving faster than 20 ms */
        uint32_t dt_us = now_us - s_last_twist_rx_us;
        if (s_twist_timed_out == 0u && dt_us < TARGET_USB_MIN_TWIST_INTERVAL_US) {
            s_host_state.twist_rate_drops++;
            return;
        }
    }

    float v_cmd    = unpack_f32_le(&payload[28]);
    float yaw_cmd  = unpack_f32_le(&payload[32]);
    float h_cmd    = unpack_f32_le(&payload[36]);
    float roll_cmd = unpack_f32_le(&payload[40]);
    float px       = unpack_f32_le(&payload[44]);
    float py       = unpack_f32_le(&payload[48]);
    /* Legacy twist means constant velocity (zero feed-forward). A host running
     * the FIL trajectory can supply its analytic acceleration and jerk. */
    float accel    = hdr->payload_len == TARGET_USB_CMD_TWIST_FF_SIZE
                   ? unpack_f32_le(&payload[52]) : 0.0f;
    float jerk     = hdr->payload_len == TARGET_USB_CMD_TWIST_FF_SIZE
                   ? unpack_f32_le(&payload[56]) : 0.0f;

    if (!isfinite(v_cmd) || !isfinite(yaw_cmd) || !isfinite(h_cmd) || !isfinite(roll_cmd)
        || !isfinite(accel) || !isfinite(jerk)
        || fabsf(accel) > 1.5f || fabsf(jerk) > 10.0f) {
        s_host_state.rx_proto_errors++;
        return;
    }

    s_host_state.session_id          = unpack_u64_le(&payload[0]);
    s_host_state.sent_us             = unpack_u64_le(&payload[8]);
    s_host_state.pose_us             = unpack_u64_le(&payload[16]);
    s_host_state.pose_seq            = unpack_u32_le(&payload[24]);
    s_host_state.v_cmd_m_s           = v_cmd;
    s_host_state.yaw_rate_cmd_rad_s  = yaw_cmd;
    s_host_state.accel_ref_m_s2      = accel;
    s_host_state.jerk_ref_m_s3       = jerk;
    s_host_state.height_cmd_m        = h_cmd;
    s_host_state.roll_cmd_rad        = roll_cmd;
    s_host_state.pose_x_m            = isfinite(px) ? px : 0.0f;
    s_host_state.pose_y_m            = isfinite(py) ? py : 0.0f;
    s_host_state.pose_yaw_rad        = 0.0f;

    s_last_twist_seq   = hdr->seq;
    s_last_twist_rx_us = now_us;
    s_has_twist        = 1u;
    s_twist_timed_out  = 0u;
    s_host_state.twist_frames_ok++;
}

static void target_usb_handle_cmd_mode(const UsbLinkHeader *hdr,
                                       const uint8_t *payload)
{
    if (hdr->payload_len < 6u) {
        s_host_state.rx_proto_errors++;
        return;
    }

    s_host_state.req_mode           = payload[0];
    s_host_state.arm_req            = payload[1];
    s_host_state.stand_handoff_req  = payload[2];
    s_host_state.voluntary_stop_req = payload[3];
    s_host_state.reset_fault_req    = payload[4];
    s_host_state.jump_req           = payload[5];
    s_host_state.bench_mode_active  = ((hdr->flags & USB_LINK_FLAG_BENCH_MODE) != 0u) ? 1u : 0u;
}

/* FW10 CMD_MOTOR_ID (0x05), 8-byte payload: [0] axis, [1] reserved,
 * [2..3] duration_ms u16 LE, [4..7] torque_nm f32 LE. Stored as a request only;
 * bringup_step() validates it (MOTOR_ID image, wired stop, cap, duration). */
static void target_usb_handle_cmd_motor_id(uint32_t now_us,
                                           const UsbLinkHeader *hdr,
                                           const uint8_t *payload)
{
    if (hdr->payload_len < TARGET_USB_CMD_MOTOR_ID_SIZE) {
        s_host_state.rx_proto_errors++;
        return;
    }
    s_host_state.motor_id_axis        = payload[0];
    s_host_state.motor_id_duration_ms = (uint16_t)((uint16_t)payload[2]
                                                   | ((uint16_t)payload[3] << 8u));
    s_host_state.motor_id_torque_nm   = unpack_f32_le(&payload[4]);
    s_host_state.motor_id_seq         = hdr->seq;
    s_host_state.motor_id_rx_us       = now_us;
    s_host_state.motor_id_valid       = 1u;
}

static void target_usb_handle_time_sync_req(uint64_t now_us64,
                                            const UsbLinkHeader *hdr,
                                            const uint8_t *payload)
{
    uint8_t resp[TARGET_USB_TIME_SYNC_RESP_SIZE];
    if (hdr->payload_len < TARGET_USB_TIME_SYNC_REQ_SIZE) {
        s_host_state.rx_proto_errors++;
        return;
    }

    uint64_t t1_host_us = unpack_u64_le(&payload[0]);
    uint64_t t2_rx_us   = now_us64;
    uint64_t t3_tx_us   = target_micros64();

    pack_u64_le(&resp[0],  t1_host_us);
    pack_u64_le(&resp[8],  t2_rx_us);
    pack_u64_le(&resp[16], t3_tx_us);

    (void)target_usb_cdc_send_frame((uint8_t)USB_MSG_TIME_SYNC_RESP,
                                    0u,
                                    t3_tx_us,
                                    resp,
                                    (uint16_t)sizeof(resp));
}

void target_usb_cdc_poll(uint32_t now_us, uint64_t now_us64, TargetUsbHostCmd *out_cmd)
{
    UsbLinkHeader hdr;
    uint8_t       payload[USB_LINK_MAX_PAYLOAD];

    uint32_t tail = s_usb_rx_tail;
    uint32_t head = s_usb_rx_head;
    __DMB();

    while (tail != head) {
        uint8_t byte = s_usb_rx_ring[tail];
        tail = (tail + 1u) & (TARGET_USB_RX_RING_SIZE - 1u);

        UsbLinkStatus st = usb_link_rx_feed(&s_rx_parser, byte,
                                            &hdr, payload, sizeof(payload));
        if (st == USB_LINK_OK) {
            if (hdr.msg_type == (uint8_t)USB_MSG_CMD_TWIST) {
                target_usb_handle_cmd_twist(now_us, &hdr, payload);
            } else if (hdr.msg_type == (uint8_t)USB_MSG_CMD_MODE) {
                target_usb_handle_cmd_mode(&hdr, payload);
            } else if (hdr.msg_type == (uint8_t)USB_MSG_TIME_SYNC_REQ) {
                target_usb_handle_time_sync_req(now_us64, &hdr, payload);
            } else if (hdr.msg_type == (uint8_t)USB_MSG_CMD_MOTOR_ID) {
                target_usb_handle_cmd_motor_id(now_us, &hdr, payload);
            }
        }
    }
    __DMB();
    s_usb_rx_tail = tail;

    s_host_state.rx_crc_errors   = s_rx_parser.crc_errors;
    s_host_state.rx_proto_errors = s_rx_parser.proto_errors + s_rx_parser.overflow_errors;

    /* Evaluate 150 ms host command freshness TTL for S9 host-loss brake */
    if (s_has_twist == 0u) {
        s_host_state.host_age_ms = 0xFFFFFFFFu;
        s_host_state.host_valid  = 0u;
    } else {
        uint32_t age_us = now_us - s_last_twist_rx_us;
        uint32_t age_ms = age_us / 1000u;
        if (age_ms > TARGET_USB_HOST_TIMEOUT_MS) {
            s_twist_timed_out = 1u;
        }
        s_host_state.host_age_ms = s_twist_timed_out != 0u ? 0xFFFFFFFFu : age_ms;
        s_host_state.host_valid = s_twist_timed_out == 0u ? 1u : 0u;
    }

    if (out_cmd != NULL) {
        *out_cmd = s_host_state;
    }
    s_host_state.reset_fault_req = 0u; /* Consumed hint, never hardware authorization. */
}

void target_usb_apply_motion(const TargetUsbHostCmd *cmd, float radio_v, float radio_yaw,
                             BalanceTickInput *in)
{
    in->host_age_ms = cmd->host_age_ms;
    in->host_valid = cmd->host_valid;
    in->v_target_m_s = cmd->host_valid != 0u ? cmd->v_cmd_m_s : radio_v;
    in->yaw_target_rad_s = cmd->host_valid != 0u ? cmd->yaw_rate_cmd_rad_s : radio_yaw;
    in->accel_ref_m_s2 = cmd->host_valid != 0u ? cmd->accel_ref_m_s2 : 0.0f;
    in->jerk_ref_m_s3 = cmd->host_valid != 0u ? cmd->jerk_ref_m_s3 : 0.0f;
}

static UsbLinkStatus target_usb_cdc_transmit_wire(const uint8_t *wire_buf, size_t wire_len)
{
    if (wire_buf == NULL || wire_len == 0u || wire_len > TARGET_USB_TX_BUF_SIZE) {
        return USB_LINK_ERR_BUFFER_TOO_SMALL;
    }

    uint32_t saved = target_irq_save();
    if (s_usb_configured == 0u || s_usb_tx_busy != 0u) {
        s_host_state.tx_frames_dropped++;
        target_irq_restore(saved);
        return USB_LINK_OK;
    }

    if (wire_buf != s_usb_tx_wire_buf) {
        memcpy(s_usb_tx_wire_buf, wire_buf, wire_len);
    }

    s_usb_tx_busy = 1u;
    if (HAL_PCD_EP_Transmit(&g_hpcd_usb, TARGET_USB_CDC_IN_EP,
                            s_usb_tx_wire_buf, (uint32_t)wire_len) != HAL_OK) {
        s_usb_tx_busy = 0u;
        s_host_state.tx_frames_dropped++;
        target_irq_restore(saved);
        return USB_LINK_ERR_OVERFLOW;
    }

    s_host_state.tx_frames_sent++;
    target_irq_restore(saved);
    return USB_LINK_OK;
}

UsbLinkStatus target_usb_cdc_send_frame(uint8_t msg_type, uint8_t flags,
                                        uint64_t timestamp_us,
                                        const uint8_t *payload, uint16_t payload_len)
{
    uint8_t staging[TARGET_USB_TX_BUF_SIZE];
    UsbLinkHeader hdr;
    size_t        wire_len = 0u;

    hdr.ver_major    = USB_LINK_VER_MAJOR;
    hdr.ver_minor    = USB_LINK_VER_MINOR;
    hdr.msg_type     = msg_type;
    hdr.flags        = flags;
    hdr.seq          = s_tx_seq_generic++;
    hdr.timestamp_us = timestamp_us;
    hdr.payload_len  = payload_len;

    UsbLinkStatus st = usb_link_encode(&hdr, payload,
                                       staging, sizeof(staging),
                                       &wire_len);
    if (st != USB_LINK_OK) {
        return st;
    }

    return target_usb_cdc_transmit_wire(staging, wire_len);
}

LogStatus target_usb_cdc_send_telem_fast(uint64_t timestamp_us, uint8_t flags,
                                         const LogTelemFast *rec)
{
    uint8_t staging[TARGET_USB_TX_BUF_SIZE];
    size_t wire_len = 0u;
    LogStatus st = log_frame_telem_fast(s_tx_seq_fast++, timestamp_us, flags,
                                        rec,
                                        staging, sizeof(staging),
                                        &wire_len);
    if (st != LOG_OK) {
        return st;
    }
    (void)target_usb_cdc_transmit_wire(staging, wire_len);
    return LOG_OK;
}

LogStatus target_usb_cdc_send_telem_health(uint64_t timestamp_us, uint8_t flags,
                                           const LogTelemHealth *rec)
{
    uint8_t staging[TARGET_USB_TX_BUF_SIZE];
    size_t wire_len = 0u;
    LogStatus st = log_frame_telem_health(s_tx_seq_health++, timestamp_us, flags,
                                          rec,
                                          staging, sizeof(staging),
                                          &wire_len);
    if (st != LOG_OK) {
        return st;
    }
    (void)target_usb_cdc_transmit_wire(staging, wire_len);
    return LOG_OK;
}

LogStatus target_usb_cdc_send_bench_raw(uint64_t timestamp_us, uint8_t flags,
                                        const LogBenchRaw *rec)
{
    uint8_t staging[TARGET_USB_TX_BUF_SIZE];
    size_t wire_len = 0u;
    LogStatus st = log_frame_bench_raw(s_tx_seq_bench++, timestamp_us, flags,
                                       rec,
                                       staging, sizeof(staging),
                                       &wire_len);
    if (st != LOG_OK) {
        return st;
    }
    (void)target_usb_cdc_transmit_wire(staging, wire_len);
    return LOG_OK;
}

LogStatus target_usb_cdc_send_fault_snapshot(uint64_t timestamp_us,
                                             const LogFlightRecord *rec)
{
    uint8_t payload[LOG_FLIGHT_RECORD_SIZE];
    size_t  payload_len = 0u;

    LogStatus st = log_pack_flight_record(rec, payload, sizeof(payload), &payload_len);
    if (st != LOG_OK) {
        return st;
    }

    UsbLinkStatus ust = target_usb_cdc_send_frame((uint8_t)USB_MSG_FAULT_SNAPSHOT,
                                                  0u,
                                                  timestamp_us,
                                                  payload,
                                                  (uint16_t)payload_len);
    return (ust == USB_LINK_OK) ? LOG_OK : LOG_ERR_LINK_ENCODE;
}
