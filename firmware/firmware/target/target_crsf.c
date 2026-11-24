/*
 * ExpressLRS CRSF Receiver UART Driver with CRC-8 DVB-S2 & 150 ms Failsafe Timeout
 *
 * Pinout: USART6 on PC6 (TX) / PC7 (RX), AF7, 420000 baud 8N1.
 */

#include "target_crsf.h"
#include "target_sync.h"

#include <math.h>
#include <string.h>

#define CRSF_RX_RING_SIZE       256u
#define CRSF_MAX_VEL_M_S        1.0f
#define CRSF_MAX_YAW_RAD_S      1.5f
#define CRSF_STICK_DEADBAND     0.04f

UART_HandleTypeDef g_huart_crsf;

typedef struct {
    volatile uint32_t rx_head;
    volatile uint32_t rx_tail;
    volatile uint8_t  rx_ring[CRSF_RX_RING_SIZE];
    volatile uint32_t rx_time_us[CRSF_RX_RING_SIZE];
    volatile uint32_t uart_errors;
    volatile uint32_t overruns;
    uint8_t           isr_rx_byte;

    uint8_t           frame_buf[CRSF_MAX_FRAME_LEN];
    uint8_t           frame_pos;
    uint8_t           expected_total_len;
    uint32_t          byte_last_us;

    uint16_t          channels[CRSF_NUM_CHANNELS];
    int8_t            rssi_dbm;
    uint8_t           lq_pct;
    uint8_t           link_stats_seen;
    uint8_t           rc_seen;
    uint8_t           timed_out;
    uint32_t          last_valid_rc_us;

    uint32_t          frames_ok;
    uint32_t          crc_errors;
    uint32_t          framing_errors;
} CrsfDriverState;

static CrsfDriverState s_crsf;

static float normalize_stick(uint16_t raw, float max_out)
{
    int32_t centered = (int32_t)raw - (int32_t)CRSF_CH_VAL_MID;
    const float half_span = (float)(CRSF_CH_VAL_MAX - CRSF_CH_VAL_MID);
    float norm = (float)centered / half_span;
    if (norm < -1.0f) {
        norm = -1.0f;
    } else if (norm > 1.0f) {
        norm = 1.0f;
    }
    if (fabsf(norm) < CRSF_STICK_DEADBAND) {
        return 0.0f;
    }
    return norm * max_out;
}

static void unpack_crsf_rc_channels(const uint8_t payload[CRSF_RC_PAYLOAD_LEN],
                                    uint16_t out_ch[CRSF_NUM_CHANNELS])
{
    out_ch[0]  = (uint16_t)(((uint32_t)payload[0]       | ((uint32_t)payload[1] << 8u)) & 0x07FFu);
    out_ch[1]  = (uint16_t)((((uint32_t)payload[1] >> 3u) | ((uint32_t)payload[2] << 5u)) & 0x07FFu);
    out_ch[2]  = (uint16_t)((((uint32_t)payload[2] >> 6u) | ((uint32_t)payload[3] << 2u) | ((uint32_t)payload[4] << 10u)) & 0x07FFu);
    out_ch[3]  = (uint16_t)((((uint32_t)payload[4] >> 1u) | ((uint32_t)payload[5] << 7u)) & 0x07FFu);
    out_ch[4]  = (uint16_t)((((uint32_t)payload[5] >> 4u) | ((uint32_t)payload[6] << 4u)) & 0x07FFu);
    out_ch[5]  = (uint16_t)((((uint32_t)payload[6] >> 7u) | ((uint32_t)payload[7] << 1u) | ((uint32_t)payload[8] << 9u)) & 0x07FFu);
    out_ch[6]  = (uint16_t)((((uint32_t)payload[8] >> 2u) | ((uint32_t)payload[9] << 6u)) & 0x07FFu);
    out_ch[7]  = (uint16_t)((((uint32_t)payload[9] >> 5u) | ((uint32_t)payload[10] << 3u)) & 0x07FFu);
    out_ch[8]  = (uint16_t)(((uint32_t)payload[11]      | ((uint32_t)payload[12] << 8u)) & 0x07FFu);
    out_ch[9]  = (uint16_t)((((uint32_t)payload[12] >> 3u) | ((uint32_t)payload[13] << 5u)) & 0x07FFu);
    out_ch[10] = (uint16_t)((((uint32_t)payload[13] >> 6u) | ((uint32_t)payload[14] << 2u) | ((uint32_t)payload[15] << 10u)) & 0x07FFu);
    out_ch[11] = (uint16_t)((((uint32_t)payload[15] >> 1u) | ((uint32_t)payload[16] << 7u)) & 0x07FFu);
    out_ch[12] = (uint16_t)((((uint32_t)payload[16] >> 4u) | ((uint32_t)payload[17] << 4u)) & 0x07FFu);
    out_ch[13] = (uint16_t)((((uint32_t)payload[17] >> 7u) | ((uint32_t)payload[18] << 1u) | ((uint32_t)payload[19] << 9u)) & 0x07FFu);
    out_ch[14] = (uint16_t)((((uint32_t)payload[19] >> 2u) | ((uint32_t)payload[20] << 6u)) & 0x07FFu);
    out_ch[15] = (uint16_t)((((uint32_t)payload[20] >> 5u) | ((uint32_t)payload[21] << 3u)) & 0x07FFu);
}

uint8_t target_crsf_crc8_dvb_s2(const uint8_t *data, size_t len)
{
    uint8_t crc = 0u;
    if (data == NULL) {
        return 0u;
    }
    for (size_t i = 0u; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            if ((crc & 0x80u) != 0u) {
                crc = (uint8_t)((uint8_t)(crc << 1u) ^ 0xD5u);
            } else {
                crc = (uint8_t)(crc << 1u);
            }
        }
    }
    return crc;
}

static void handle_verified_crsf_frame(uint8_t frame_type, const uint8_t *payload,
                                       uint8_t payload_len, uint32_t now_us)
{
    if (frame_type == CRSF_FRAMETYPE_LINK_STATISTICS && payload_len >= CRSF_LINK_STATS_PAYLOAD_LEN) {
        const uint8_t active_ant = payload[4];
        const uint8_t raw_rssi = (active_ant != 0u) ? payload[1] : payload[0];
        s_crsf.rssi_dbm = (raw_rssi <= 127u) ? (int8_t)(-(int8_t)raw_rssi) : -127;
        s_crsf.lq_pct = payload[2];
        s_crsf.link_stats_seen = 1u;
        s_crsf.frames_ok++;
    } else if (frame_type == CRSF_FRAMETYPE_RC_CHANNELS && payload_len == CRSF_RC_PAYLOAD_LEN) {
        s_crsf.frames_ok++;

        /* Distinguish genuine fresh RF packets from receiver-held failsafe frames:
         * if Link Statistics reported LQ == 0%, do not refresh last_valid_rc_us so
         * the 150 ms failsafe timeout still trips into policy c (S12_REPORT.md:73). */
        if (s_crsf.link_stats_seen == 0u || s_crsf.lq_pct > 0u) {
            unpack_crsf_rc_channels(payload, s_crsf.channels);
            s_crsf.timed_out = 0u;
            s_crsf.last_valid_rc_us = now_us;
            s_crsf.rc_seen = 1u;
        }
    }
}

static void feed_crsf_byte(uint8_t byte, uint32_t now_us)
{
    /* Reset partial frame state if inter-byte gap exceeds 2.5 ms (CRSF frame is ~620 us @ 420k). */
    if (s_crsf.frame_pos > 0u && (uint32_t)(now_us - s_crsf.byte_last_us) > 2500u) {
        s_crsf.frame_pos = 0u;
        s_crsf.expected_total_len = 0u;
    }
    s_crsf.byte_last_us = now_us;

    if (s_crsf.frame_pos == 0u) {
        if (byte == CRSF_SYNC_BYTE_FC || byte == CRSF_SYNC_BYTE_TX || byte == CRSF_SYNC_BYTE_HANDSET) {
            s_crsf.frame_buf[0] = byte;
            s_crsf.frame_pos = 1u;
        }
        return;
    }

    if (s_crsf.frame_pos == 1u) {
        /* CRSF length byte = Type (1) + Payload (N) + CRC (1), valid range 2..62 */
        if (byte < 2u || byte > (CRSF_MAX_FRAME_LEN - 2u)) {
            s_crsf.frame_pos = 0u;
            s_crsf.framing_errors++;
            return;
        }
        s_crsf.frame_buf[1] = byte;
        s_crsf.expected_total_len = (uint8_t)(byte + 2u);
        s_crsf.frame_pos = 2u;
        return;
    }

    s_crsf.frame_buf[s_crsf.frame_pos] = byte;
    s_crsf.frame_pos++;

    if (s_crsf.frame_pos >= s_crsf.expected_total_len) {
        const uint8_t len_field   = s_crsf.frame_buf[1];
        const uint8_t frame_type  = s_crsf.frame_buf[2];
        const uint8_t payload_len = (uint8_t)(len_field - 2u);
        const uint8_t rx_crc      = s_crsf.frame_buf[s_crsf.expected_total_len - 1u];
        const uint8_t calc_crc    = target_crsf_crc8_dvb_s2(&s_crsf.frame_buf[2], (size_t)(len_field - 1u));

        s_crsf.frame_pos = 0u;
        s_crsf.expected_total_len = 0u;

        if (calc_crc == rx_crc) {
            handle_verified_crsf_frame(frame_type, &s_crsf.frame_buf[3], payload_len, now_us);
        } else {
            s_crsf.crc_errors++;
        }
    }
}

HAL_StatusTypeDef target_crsf_init(void)
{
    memset(&s_crsf, 0, sizeof(s_crsf));
    for (uint8_t i = 0u; i < CRSF_NUM_CHANNELS; ++i) {
        s_crsf.channels[i] = CRSF_CH_VAL_MID;
    }
    /* Channels 4..7 (switches) default LOW (disarmed, e-stop inactive). */
    s_crsf.channels[4] = CRSF_CH_VAL_MIN;
    s_crsf.channels[5] = CRSF_CH_VAL_MIN;
    s_crsf.channels[6] = CRSF_CH_VAL_MIN;
    s_crsf.channels[7] = CRSF_CH_VAL_MIN;
    s_crsf.rssi_dbm    = -120;
    s_crsf.lq_pct      = 0u;

    memset(&g_huart_crsf, 0, sizeof(g_huart_crsf));
    g_huart_crsf.Instance                    = TARGET_CRSF_UART_INSTANCE;
    g_huart_crsf.Init.BaudRate               = TARGET_CRSF_BAUDRATE;
    g_huart_crsf.Init.WordLength             = UART_WORDLENGTH_8B;
    g_huart_crsf.Init.StopBits               = UART_STOPBITS_1;
    g_huart_crsf.Init.Parity                 = UART_PARITY_NONE;
    g_huart_crsf.Init.Mode                   = UART_MODE_TX_RX;
    g_huart_crsf.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    g_huart_crsf.Init.OverSampling           = UART_OVERSAMPLING_16;
    g_huart_crsf.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    g_huart_crsf.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    g_huart_crsf.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&g_huart_crsf) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&g_huart_crsf, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&g_huart_crsf, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        return HAL_ERROR;
    }
    if (HAL_UARTEx_DisableFifoMode(&g_huart_crsf) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_UART_Receive_IT(&g_huart_crsf, &s_crsf.isr_rx_byte, 1u);
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
    if (uartHandle == NULL) {
        return;
    }
    if (uartHandle->Instance == TARGET_CRSF_UART_INSTANCE) {
        /* CTRL rev4 CRSF connector is USART6 on PC6/PC7, AF7. */
        RCC_PeriphCLKInitTypeDef periph_clk;
        GPIO_InitTypeDef gpio;
        memset(&periph_clk, 0, sizeof(periph_clk));
        memset(&gpio, 0, sizeof(gpio));

        periph_clk.PeriphClockSelection  = RCC_PERIPHCLK_USART6;
        periph_clk.Usart16ClockSelection = RCC_USART16910CLKSOURCE_D2PCLK2;
        (void)HAL_RCCEx_PeriphCLKConfig(&periph_clk);

        __HAL_RCC_USART6_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        gpio.Pin       = TARGET_CRSF_TX_PIN | TARGET_CRSF_RX_PIN;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pull      = GPIO_PULLUP;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = TARGET_CRSF_GPIO_AF;
        HAL_GPIO_Init(TARGET_CRSF_GPIO_PORT, &gpio);

        HAL_NVIC_SetPriority(TARGET_CRSF_UART_IRQn, TARGET_NVIC_PRIO_CRSF_UART, 0u);
        HAL_NVIC_EnableIRQ(TARGET_CRSF_UART_IRQn);
    }
}

void target_crsf_isr_push_byte(uint8_t byte, uint32_t rx_us)
{
    const uint32_t next_head = (s_crsf.rx_head + 1u) % CRSF_RX_RING_SIZE;
    if (next_head != s_crsf.rx_tail) {
        s_crsf.rx_ring[s_crsf.rx_head] = byte;
        s_crsf.rx_time_us[s_crsf.rx_head] = rx_us;
        __DMB();
        s_crsf.rx_head = next_head;
    } else {
        s_crsf.overruns++;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != NULL && huart->Instance == TARGET_CRSF_UART_INSTANCE) {
        target_crsf_isr_push_byte(s_crsf.isr_rx_byte, target_micros());
        (void)HAL_UART_Receive_IT(&g_huart_crsf, &s_crsf.isr_rx_byte, 1u);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != NULL && huart->Instance == TARGET_CRSF_UART_INSTANCE) {
        s_crsf.uart_errors++;
        (void)HAL_UART_Receive_IT(&g_huart_crsf, &s_crsf.isr_rx_byte, 1u);
    }
}

void target_crsf_poll(uint32_t now_us)
{
    (void)now_us;
    uint32_t remaining = CRSF_RX_RING_SIZE;
    uint32_t head = s_crsf.rx_head;
    while (s_crsf.rx_tail != head && remaining > 0u) {
        remaining--;
        __DMB();
        const uint8_t b = s_crsf.rx_ring[s_crsf.rx_tail];
        const uint32_t rx_us = s_crsf.rx_time_us[s_crsf.rx_tail];
        __DMB();
        s_crsf.rx_tail = (s_crsf.rx_tail + 1u) % CRSF_RX_RING_SIZE;
        feed_crsf_byte(b, rx_us);
    }
}

void target_crsf_snapshot(uint32_t now_us, TargetCrsfSnapshot *out_snap)
{
    if (out_snap == NULL) {
        return;
    }
    /* UART error/overrun counters are updated in the ISR.  Snapshot them with
     * interrupts masked so telemetry never combines torn or stale increments. */
    uint32_t saved = target_irq_save();
    memset(out_snap, 0, sizeof(*out_snap));
    memcpy(out_snap->channels, s_crsf.channels, sizeof(out_snap->channels));

    uint32_t age_ms = (TARGET_CRSF_TIMEOUT_MS + 1000u);
    if (s_crsf.rc_seen != 0u) {
        /* A just-stamped ISR frame may be a few cycles newer than now_us.
         * Signed modular comparison treats that small forward skew as age 0. */
        const int32_t signed_elapsed = (int32_t)(now_us - s_crsf.last_valid_rc_us);
        const uint32_t elapsed_us = signed_elapsed < 0 ? 0u : (uint32_t)signed_elapsed;
        age_ms = elapsed_us / 1000u;
    }

    if (age_ms > TARGET_CRSF_TIMEOUT_MS) {
        s_crsf.timed_out = 1u;
    }
    const uint8_t failsafe = s_crsf.timed_out;
    if (failsafe != 0u) {
        age_ms = TARGET_CRSF_TIMEOUT_MS + 1u; /* Never resurrect after a full u32 cycle. */
    }
    out_snap->radio_age_ms       = age_ms;
    out_snap->failsafe_active    = failsafe;
    out_snap->active_radio_policy = SAFETY_RADIO_LOSS_BRAKE_BALANCE;

    /* Policy c does not introduce a stricter RF-loss-only torque-off threshold. */
    out_snap->effective_tilt_limit_deg = CRSF_NORMAL_TILT_LIMIT_DEG;

    /* Channel mapping (ASSUMED pending CTRL board & handset channel map check):
     *   Ch 0: Roll/Yaw rate stick
     *   Ch 1: Pitch/Forward velocity stick
     *   Ch 4 (AUX1): Arm switch
     *   Ch 5 (AUX2): Deliberate radio e-stop switch
     *   Ch 6 (AUX3): Stand handoff request
     *   Ch 7 (AUX4): Voluntary powered stop request */
    out_snap->yaw_target_rad_s   = (failsafe != 0u) ? 0.0f : normalize_stick(s_crsf.channels[0], CRSF_MAX_YAW_RAD_S);
    out_snap->v_target_m_s       = (failsafe != 0u) ? 0.0f : normalize_stick(s_crsf.channels[1], CRSF_MAX_VEL_M_S);
    out_snap->radio_arm_sw       = (failsafe == 0u && s_crsf.channels[4] >= CRSF_SW_HIGH_THRESH) ? 1u : 0u;
    out_snap->radio_estop_sw     = (failsafe == 0u && s_crsf.channels[5] >= CRSF_SW_HIGH_THRESH) ? 1u : 0u;
    out_snap->stand_handoff_req  = (failsafe == 0u && s_crsf.channels[6] >= CRSF_SW_HIGH_THRESH) ? 1u : 0u;
    out_snap->voluntary_stop_req = (failsafe == 0u && s_crsf.channels[7] >= CRSF_SW_HIGH_THRESH) ? 1u : 0u;

    out_snap->switch_bitmask = (uint16_t)((out_snap->radio_arm_sw != 0u       ? 0x01u : 0u)
                                        | (out_snap->radio_estop_sw != 0u     ? 0x02u : 0u)
                                        | (out_snap->stand_handoff_req != 0u  ? 0x04u : 0u)
                                        | (out_snap->voluntary_stop_req != 0u ? 0x08u : 0u)
                                        | (out_snap->failsafe_active != 0u    ? 0x80u : 0u));

    out_snap->rssi_dbm       = s_crsf.rssi_dbm;
    out_snap->lq_pct         = s_crsf.lq_pct;
    out_snap->frames_ok      = s_crsf.frames_ok;
    out_snap->crc_errors     = s_crsf.crc_errors;
    out_snap->framing_errors = s_crsf.framing_errors + s_crsf.uart_errors + s_crsf.overruns;
    target_irq_restore(saved);
}

void target_crsf_apply_to_safety(const TargetCrsfSnapshot *snap,
                                 SafetyConfig *safety_cfg,
                                 uint32_t *out_radio_age_ms,
                                 uint8_t *out_radio_arm_sw,
                                 uint8_t *out_radio_estop_sw)
{
    if (snap == NULL) {
        return;
    }
    if (safety_cfg != NULL) {
        safety_cfg->radio_loss_policy = SAFETY_RADIO_LOSS_BRAKE_BALANCE;
        safety_cfg->radio_timeout_ms  = TARGET_CRSF_TIMEOUT_MS;
        safety_cfg->tilt_limit_deg    = snap->effective_tilt_limit_deg;
    }
    if (out_radio_age_ms != NULL) {
        *out_radio_age_ms = snap->radio_age_ms;
    }
    if (out_radio_arm_sw != NULL) {
        *out_radio_arm_sw = snap->radio_arm_sw;
    }
    if (out_radio_estop_sw != NULL) {
        *out_radio_estop_sw = snap->radio_estop_sw;
    }
}
