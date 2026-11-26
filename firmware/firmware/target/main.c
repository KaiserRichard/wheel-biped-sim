/*
 * STM32H723VGT6 Target Firmware Entry Point, Clock Tree, ISR Vector Glue & 500 Hz Loop
 *
 * Ties together:
 *   - DWT 520 MHz cycle counter microsecond timebase (`target_micros`, `target_micros64`)
 *   - Boot reset-cause capture & IWDG1 independent watchdog (`target_iwdg`)
 *   - Wired NC e-stop EXTI inputs (`PB12` and `PD7`, NVIC prio 0) &
 *     watchdog heartbeat output (`PD11`) (`target_estop`)
 *   - FDCAN1 / FDCAN2 / FDCAN3 at 1.000 Mbit/s with N1 hardware RX filters and
 *     TX/RX ISR glue into `can_sched` and `S11IsrMailbox` (`target_fdcan`)
 *   - CRSF (ExpressLRS) UART receiver at 420 kbaud with CRC-8 DVB-S2 and 150 ms
 *     failsafe timeout feeding radio-loss policy c (`RADIO_LOSS_BRAKE_BALANCE`) (`target_crsf`)
 *   - USB OTG FS CDC ACM link with COBS + CRC-32C framing (`target_usb_cdc`)
 *   - 500 Hz deterministic basic timer (`TIM6`) driving `balance_ctrl_tick()` (`target_timer`)
 *   - 250-record circular SRAM flight recorder (`LogFlightRing`)
 *
 * Clock tree & DWT helpers adapted from DM-jump (Core/Src/main.c, User/Bsp/bsp_dwt.c):
 *   Copyright (c) 2024-2026 Tran Nguyen Binh
 *   Project: DM-jump (https://github.com/nguyenbinh-shark/DM-jump), MIT License
 *   Reference: dmBots/wheel-legged (https://github.com/dmBots/wheel-legged)
 *   See firmware/DM_JUMP_LICENSE.txt for the full MIT license notice.
 */

#include "target_hw.h"
#include "target_fdcan.h"
#include "target_crsf.h"
#include "target_usb_cdc.h"
#include "target_iwdg.h"
#include "target_estop.h"
#include "target_timer.h"
#include "target_safety_io.h"
#include "target_imu.h"

#include "control/balance_ctrl.h"
#include "bringup_mode.h"
#include "log_packer.h"

#include <string.h>

/* 500 ms circular flight recorder at 500 Hz (250 records, 50 post-fault ticks = 100 ms) */
#define TARGET_FLIGHT_RING_CAPACITY     250u
#define TARGET_FLIGHT_POST_FAULT_TICKS  50u

/* FW10 compile-time bring-up image (-DWBR_BRINGUP_MODE=0/1/2 from CMake).
 * Default is the most restrictive image: BENCH (torque impossible). */
#ifndef WBR_BRINGUP_MODE
#define WBR_BRINGUP_MODE 1
#endif
#if WBR_BRINGUP_MODE == 0
#define WBR_MODE_NAME "RUN"
#elif WBR_BRINGUP_MODE == 1
#define WBR_MODE_NAME "BENCH"
#elif WBR_BRINGUP_MODE == 2
#define WBR_MODE_NAME "MOTOR_ID"
#else
#error "WBR_BRINGUP_MODE must be 0 (RUN), 1 (BENCH) or 2 (MOTOR_ID)"
#endif
#ifndef WBR_BUILD_TYPE
#define WBR_BUILD_TYPE "Release"
#endif
/* Identification string kept in flash (no timestamps: reproducible images). */
__attribute__((used)) const char g_wbr_build_id[] =
    "WBR-FW10 mode=" WBR_MODE_NAME " build=" WBR_BUILD_TYPE;

/* power_io_bits (TELEM_HEALTH) FW10 additions; bits 0..2 are the FW9 e-stop bits. */
#define TARGET_PIO_BENCH_IMAGE      0x0008u
#define TARGET_PIO_MOTOR_ID_IMAGE   0x0010u
#define TARGET_PIO_IMU_OK           0x0020u
#define TARGET_PIO_PULSE_ACTIVE     0x0040u
#define TARGET_PIO_BENCH_VIOLATION  0x0080u
#define TARGET_STACK_PAINT          0xA5A5A5A5u

extern uint32_t _end[];    /* linker: end of .bss / start of free DTCM */
extern uint32_t _estack[]; /* linker: top of DTCM */
extern uint32_t g_pfnVectors[];

static BringupCtx        s_bringup;
static BringupTickOutput s_bringup_out;
static ImuSample         s_imu_raw;
static uint8_t           s_imu_fresh;
static uint32_t          s_busy_max_us;

static BalanceCtrl       s_ctrl;
static LogFlightRecord   s_flight_storage[TARGET_FLIGHT_RING_CAPACITY];
static LogFlightRing     s_flight_ring;

/* DWT 64-bit cycle tracking state */
static uint32_t          s_cpu_freq_mhz     = TARGET_SYSCLK_MHZ;
static volatile uint32_t s_dwt_last_cyccnt  = 0u;
static volatile uint64_t s_dwt_total_cycles = 0ULL;

/* ============================================================================
 * 1. DWT Microsecond Timebase (adapted from DM-jump User/Bsp/bsp_dwt.c)
 * ============================================================================ */

void target_dwt_init(uint32_t cpu_freq_mhz)
{
    s_cpu_freq_mhz = (cpu_freq_mhz > 0u) ? cpu_freq_mhz : TARGET_SYSCLK_MHZ;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0u;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;

    s_dwt_last_cyccnt  = DWT->CYCCNT;
    s_dwt_total_cycles = 0ULL;
}

uint64_t target_micros64(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t cyccnt = DWT->CYCCNT;
    uint32_t delta  = cyccnt - s_dwt_last_cyccnt;
    s_dwt_last_cyccnt = cyccnt;
    s_dwt_total_cycles += (uint64_t)delta;
    uint64_t total_cycles = s_dwt_total_cycles;

    if (primask == 0u) {
        __enable_irq();
    }

    return total_cycles / (uint64_t)s_cpu_freq_mhz;
}

uint32_t target_micros(void)
{
    return (uint32_t)target_micros64();
}

/* ============================================================================
 * 2. STM32H723 Clock Tree Configuration (24 MHz HSE -> 520 MHz SYSCLK)
 *    Adapted from DM-jump Core/Src/main.c
 * ============================================================================ */

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef rcc_osc;
    RCC_ClkInitTypeDef rcc_clk;
    memset(&rcc_osc, 0, sizeof(rcc_osc));
    memset(&rcc_clk, 0, sizeof(rcc_clk));

    /* Use 520 MHz to avoid the CPUFREQ_BOOST option-byte requirement. */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    /* Configure HSE (24 MHz), HSI48 (USB FS, trimmed by SOF CRS), and PLL1. */
    rcc_osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48 |
                             RCC_OSCILLATORTYPE_LSI   |
                             RCC_OSCILLATORTYPE_HSE;
    rcc_osc.HSEState       = RCC_HSE_ON;
    rcc_osc.LSIState       = RCC_LSI_ON;
    rcc_osc.HSI48State     = RCC_HSI48_ON;
    rcc_osc.PLL.PLLState   = RCC_PLL_ON;
    rcc_osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    rcc_osc.PLL.PLLM       = 12u;
    rcc_osc.PLL.PLLN       = 260u;
    rcc_osc.PLL.PLLP       = 1u;
    rcc_osc.PLL.PLLQ       = 4u;
    rcc_osc.PLL.PLLR       = 2u;
    rcc_osc.PLL.PLLRGE     = RCC_PLL1VCIRANGE_1;
    rcc_osc.PLL.PLLVCOSEL  = RCC_PLL1VCOWIDE;
    rcc_osc.PLL.PLLFRACN   = 0u;
    if (HAL_RCC_OscConfig(&rcc_osc) != HAL_OK) {
        Error_Handler();
    }

    /* SYSCLK = 520 MHz, HCLK = 260 MHz, APB1/2/3/4 = 130 MHz */
    rcc_clk.ClockType      = RCC_CLOCKTYPE_HCLK    | RCC_CLOCKTYPE_SYSCLK |
                             RCC_CLOCKTYPE_PCLK1   | RCC_CLOCKTYPE_PCLK2  |
                             RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    rcc_clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    rcc_clk.SYSCLKDivider  = RCC_SYSCLK_DIV1;
    rcc_clk.AHBCLKDivider  = RCC_HCLK_DIV2;
    rcc_clk.APB3CLKDivider = RCC_APB3_DIV2;
    rcc_clk.APB1CLKDivider = RCC_APB1_DIV2;
    rcc_clk.APB2CLKDivider = RCC_APB2_DIV2;
    rcc_clk.APB4CLKDivider = RCC_APB4_DIV2;

    if (HAL_RCC_ClockConfig(&rcc_clk, FLASH_LATENCY_3) != HAL_OK) {
        Error_Handler();
    }
}

static void PeriphCommonClock_Config(void)
{
    RCC_PeriphCLKInitTypeDef periph_clk;
    memset(&periph_clk, 0, sizeof(periph_clk));

    /*
     * PLL2: 24 MHz HSE / 3 = 8 MHz * 40 = 320 MHz VCO
     * PLL2Q = 320 MHz / 4 = 80.0 MHz FDCAN kernel clock
     * USB clock source = HSI48, synchronized to USB SOF by CRS.
     */
    periph_clk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN | RCC_PERIPHCLK_USB;
    periph_clk.PLL2.PLL2M           = 3u;
    periph_clk.PLL2.PLL2N           = 40u;
    periph_clk.PLL2.PLL2P           = 2u;
    periph_clk.PLL2.PLL2Q           = 4u;
    periph_clk.PLL2.PLL2R           = 2u;
    periph_clk.PLL2.PLL2RGE         = RCC_PLL2VCIRANGE_3;
    periph_clk.PLL2.PLL2VCOSEL      = RCC_PLL2VCOWIDE;
    periph_clk.PLL2.PLL2FRACN       = 0u;
    periph_clk.FdcanClockSelection  = RCC_FDCANCLKSOURCE_PLL2;
    periph_clk.UsbClockSelection    = RCC_USBCLKSOURCE_HSI48;

    if (HAL_RCCEx_PeriphCLKConfig(&periph_clk) != HAL_OK) {
        Error_Handler();
    }
#ifndef TARGET_HOST_TEST
    /* SOF disciplining keeps HSI48 within USB full-speed clock tolerance. */
    RCC_CRSInitTypeDef crs;
    memset(&crs, 0, sizeof(crs));
    crs.Prescaler             = RCC_CRS_SYNC_DIV1;
    crs.Source                = RCC_CRS_SYNC_SOURCE_USB1;
    crs.Polarity              = RCC_CRS_SYNC_POLARITY_RISING;
    crs.ReloadValue           = (48000000u / 1000u) - 1u;
    crs.ErrorLimitValue       = RCC_CRS_ERRORLIMIT_DEFAULT;
    crs.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
    __HAL_RCC_CRS_CLK_ENABLE();
    HAL_RCCEx_CRSConfig(&crs);
#endif
}

void Error_Handler(void)
{
    target_estop_fatal_halt();
}

/* ============================================================================
 * 2b. FW10 Cortex-M7 MPU / cache / vector table
 *     - VTOR pinned to the linked vector table (flash 0x08000000) instead of
 *       relying on the BOOT_ADD0 option byte alone.
 *     - MPU region 0: the CubeMX default speculative-access guard. 4 GB,
 *       subregions 0,1,2,7 disabled (code, SRAM, peripherals, system keep the
 *       default map); 0x60000000-0xDFFFFFFF (unused FMC/OCTOSPI space) is
 *       no-access, never executable. PRIVDEFENA keeps the default map elsewhere.
 *     - I-cache and D-cache on. All RW data and the stack are in DTCM (never
 *       cached); there is no DMA (USB dma_enable = DISABLE, UART/SPI polled or
 *       IT, FDCAN message RAM is device memory), so no cache maintenance needed.
 * ============================================================================ */
static void target_mpu_cache_config(void)
{
    MPU_Region_InitTypeDef region;
    memset(&region, 0, sizeof(region));

    HAL_MPU_Disable();
    region.Enable           = MPU_REGION_ENABLE;
    region.Number           = MPU_REGION_NUMBER0;
    region.BaseAddress      = 0x00000000u;
    region.Size             = MPU_REGION_SIZE_4GB;
    region.SubRegionDisable = 0x87u;
    region.TypeExtField     = MPU_TEX_LEVEL0;
    region.AccessPermission = MPU_REGION_NO_ACCESS;
    region.DisableExec      = MPU_INSTRUCTION_ACCESS_DISABLE;
    region.IsShareable      = MPU_ACCESS_SHAREABLE;
    region.IsCacheable      = MPU_ACCESS_NOT_CACHEABLE;
    region.IsBufferable     = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&region);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

    SCB_EnableICache();
    SCB_EnableDCache();
}

/* Paint free DTCM between the end of .bss and the live stack pointer so the
 * telemetry can report the stack high-water mark (min_stack_words[0]). */
static void target_stack_paint(void)
{
    uint32_t *p = _end;
    const uint32_t *limit = (const uint32_t *)(__get_MSP() - 256u);
    while ((const uint32_t *)p < limit) {
        *p++ = TARGET_STACK_PAINT;
    }
}

static uint32_t target_stack_free_words(void)
{
    const uint32_t *p = _end;
    const uint32_t *top = _estack;
    uint32_t n = 0u;
    while (p < top && *p == TARGET_STACK_PAINT) {
        ++p;
        ++n;
    }
    return n;
}

/* ============================================================================
 * 3. Cortex-M7 Exception & Peripheral Interrupt Vector Handlers
 * ============================================================================ */

void NMI_Handler(void)
{
    Error_Handler();
}

void HardFault_Handler(void)
{
    Error_Handler();
}

void MemManage_Handler(void)
{
    Error_Handler();
}

void BusFault_Handler(void)
{
    Error_Handler();
}

void UsageFault_Handler(void)
{
    Error_Handler();
}

void SysTick_Handler(void)
{
    HAL_IncTick();
    target_fdcan_emergency_service();
}

/* NVIC Priority 0: opto-isolated E-stop status on PD7 */
void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(TARGET_ESTOP_OPTO_PIN);
}

/* NVIC Priority 0: direct NC wired stop on PB12 */
void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(TARGET_ESTOP_PIN);
}

/* NVIC Priority 4: 500 Hz Deterministic Control Loop Timer (TIM6) */
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&g_htim_ctrl);
}

/* NVIC Priority 5: FDCAN1 / FDCAN2 / FDCAN3 Interrupt Line 0 */
void FDCAN1_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&g_hfdcan[0]);
}

void FDCAN2_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&g_hfdcan[1]);
}

void FDCAN3_IT0_IRQHandler(void)
{
    HAL_FDCAN_IRQHandler(&g_hfdcan[2]);
}

/* NVIC Priority 6: ExpressLRS CRSF Receiver UART (USART6) */
void USART6_IRQHandler(void)
{
    HAL_UART_IRQHandler(&g_huart_crsf);
}

/* NVIC Priority 7: USB OTG Full-Speed CDC ACM (USB1_OTG_HS) */
void OTG_HS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&g_hpcd_usb);
}

/* ============================================================================
 * 4. Telemetry Builders for USB CDC (TELEM_FAST @ 250 Hz, TELEM_HEALTH @ 10 Hz)
 * ============================================================================ */

static void target_emit_telem_fast(uint64_t now_us64,
                                   const BalanceTickInput *in,
                                   const BalanceTickOutput *out)
{
    LogTelemFast fast;
    memset(&fast, 0, sizeof(fast));

    fast.fsm_state      = (uint8_t)out->safety_out.state;
    fast.s11_mode       = (uint8_t)out->s11_frame.mode;
    fast.s11_max_misses = (out->s11_frame.max_misses > 255u)
                            ? (uint8_t)255u
                            : (uint8_t)out->s11_frame.max_misses;
    fast.flags          = (uint8_t)((out->safety_out.torque_enable ? 0x01u : 0x00u) |
                                    (out->safety_out.host_lost     ? 0x02u : 0x00u) |
                                    (out->safety_out.radio_lost    ? 0x04u : 0x00u) |
                                    (out->h5_out.standing          ? 0x08u : 0x00u));
    fast.fault_mask     = (uint32_t)out->safety_out.reason;

    fast.euler_rad[0]   = (float)out->est_frame.pitch;
    fast.euler_rad[1]   = (float)out->est_frame.roll;
    fast.euler_rad[2]   = (float)out->est_frame.yaw;
    fast.gyro_rad_s[0]  = (float)out->est_frame.rate[0];
    fast.gyro_rad_s[1]  = (float)out->est_frame.rate[1];
    fast.gyro_rad_s[2]  = (float)out->est_frame.rate[2];
    fast.v_est_m_s      = (float)out->est_frame.speed;
    fast.x_est_m        = s_ctrl.h5.x_est_m;

    fast.leg_length_m[0]  = out->h5_out.leg_length[0];
    fast.leg_length_m[1]  = out->h5_out.leg_length[1];
    fast.leg_theta_rad[0] = out->h5_out.leg_theta[0];
    fast.leg_theta_rad[1] = out->h5_out.leg_theta[1];

    for (uint8_t i = 0u; i < LOG_NUM_AXES; ++i) {
        fast.motor_pos_rad[i]    = (float)out->s11_frame.angle[i];
        fast.motor_vel_rad_s[i]  = (float)out->s11_frame.rate[i];
        fast.motor_tor_cmd_nm[i] = out->torque_cmd[i];
        fast.motor_tor_fb_nm[i]  = s_ctrl.can_sched.axis_rx[i].fb.torque;
        double age_us = s_ctrl.s11.axis[i].age_ms * 1000.0;
        if (!(age_us >= 0.0)) {
            age_us = 0.0;
        }
        if (age_us > 65535.0) {
            age_us = 65535.0;
        }
        fast.enc_age_us[i] = (uint16_t)age_us;
    }
    /* FW10 MOTOR_ID: show the commanded bench pulse on its axis. */
    if (s_bringup_out.active_axis < LOG_NUM_AXES) {
        fast.motor_tor_cmd_nm[s_bringup_out.active_axis] = s_bringup_out.torque_nm;
    }

    fast.s9_trim_rad       = out->h5_out.s9_pitch_ref_rad;
    fast.gyro_bias_y_rad_s = (float)s_ctrl.s10.bias[1];
    fast.v_bus_mv          = (uint16_t)(in->v_bus_v * 1000.0f);
    fast.i_bus_ma          = in->i_bus_ma;

    (void)target_usb_cdc_send_telem_fast(now_us64, 0u, &fast);
}

static void target_emit_telem_health(uint64_t now_us64,
                                     const TargetCrsfSnapshot *rc_snap,
                                     const TargetEstopState *estop_st,
                                     const TargetIwdgStatus *iwdg_st,
                                     const BalanceTickOutput *out)
{
    LogTelemHealth health;
    memset(&health, 0, sizeof(health));

    for (uint8_t b = 0u; b < LOG_NUM_CAN_BUSES; ++b) {
        health.can_tec[b]        = out->can_health.tec[b];
        health.can_rec[b]        = out->can_health.rec[b];
        health.can_busoff_cnt[b] = out->can_health.bus_off_cnt[b];
        health.can_rx_drop[b]    = out->can_health.rx_drop_cnt[b];
    }

    for (uint8_t a = 0u; a < LOG_NUM_AXES; ++a) {
        health.motor_state[a] = s_ctrl.can_sched.axis_rx[a].fb.status_nibble;
        float t_mos = s_ctrl.can_sched.axis_rx[a].fb.t_mos_c;
        if (t_mos < 0.0f) {
            t_mos = 0.0f;
        } else if (t_mos > 255.0f) {
            t_mos = 255.0f;
        }
        health.t_mos_c[a] = (uint8_t)t_mos;

        float t_coil = s_ctrl.can_sched.axis_rx[a].fb.t_coil_c;
        if (t_coil < 0.0f) {
            t_coil = 0.0f;
        } else if (t_coil > 255.0f) {
            t_coil = 255.0f;
        }
        health.t_coil_c[a] = (uint8_t)t_coil;
    }

    health.rc_rssi_dbm   = rc_snap->rssi_dbm;
    health.rc_lq_pct     = rc_snap->lq_pct;
    health.rc_age_ms     = (rc_snap->radio_age_ms > 65535u)
                             ? (uint16_t)65535u
                             : (uint16_t)rc_snap->radio_age_ms;
    health.rc_switches   = rc_snap->switch_bitmask;
    TargetImuStatus imu_st;
    target_imu_get_status(&imu_st);
    uint32_t pio = (estop_st->pin_open        ? 0x01u : 0x00u) |
                   (estop_st->latched         ? 0x02u : 0x00u) |
                   (estop_st->permit_asserted ? 0x04u : 0x00u);
    pio |= (s_bringup.mode == BRINGUP_MODE_BENCH)    ? TARGET_PIO_BENCH_IMAGE : 0u;
    pio |= (s_bringup.mode == BRINGUP_MODE_MOTOR_ID) ? TARGET_PIO_MOTOR_ID_IMAGE : 0u;
    pio |= (imu_st.init_ok != 0u)                    ? TARGET_PIO_IMU_OK : 0u;
    pio |= (s_bringup_out.active_axis < LOG_NUM_AXES) ? TARGET_PIO_PULSE_ACTIVE : 0u;
    pio |= (s_bringup.bench_violations != 0u)        ? TARGET_PIO_BENCH_VIOLATION : 0u;
    pio |= ((uint32_t)(s_bringup_out.last_reject & 0x0Fu)) << 8u;
    pio |= ((uint32_t)(s_bringup_out.state & 0x07u)) << 12u;
    health.power_io_bits = (uint16_t)pio;
    health.reset_cause   = iwdg_st->reset_cause_bits;
    /* Worst-case loop busy time over the last health window, % of 2000 us. */
    uint32_t load = (s_busy_max_us * 100u) / TARGET_CTRL_PERIOD_US;
    health.cpu_load_pct  = (uint8_t)((load > 255u) ? 255u : load);
    s_busy_max_us = 0u;
    uint32_t free_words = target_stack_free_words();
    health.min_stack_words[0] = (uint16_t)((free_words > 65535u) ? 65535u : free_words);
    uint32_t blocked = target_fdcan_tx_gate_blocked();
    health.min_stack_words[1] = (uint16_t)((blocked > 65535u) ? 65535u : blocked);

    (void)target_usb_cdc_send_telem_health(now_us64, 0u, &health);
}

/* FW10 BENCH_RAW_LOG (0x85): raw IMU sensor sample + encoder replies. The
 * three reserved bytes carry the IMU sensor accel/gyro CHIP_ID read-backs and
 * the first failing init step (0xFF = none) for bench stage 2.1. */
static void target_emit_bench_raw(uint64_t now_us64, const BalanceTickOutput *out)
{
    LogBenchRaw raw;
    memset(&raw, 0, sizeof(raw));
    TargetImuStatus imu_st;
    target_imu_get_status(&imu_st);
    raw.imu_ts_us = now_us64;
    if (s_imu_fresh != 0u) {
        for (uint32_t i = 0u; i < 3u; ++i) {
            raw.gyro_rad_s[i] = (float)s_imu_raw.gyro_rad_s[i];
            raw.accel_m_s2[i] = (float)s_imu_raw.accel_m_s2[i];
        }
    }
    raw.imu_temp_c = imu_st.temp_c;
    uint8_t fresh = 0u;
    for (uint8_t i = 0u; i < LOG_NUM_AXES; ++i) {
        const CanAxisRxSlot *slot = &s_ctrl.can_sched.axis_rx[i];
        raw.motor_pos_rad[i]   = slot->fb.pos;
        raw.motor_vel_rad_s[i] = slot->fb.vel;
        raw.motor_rx_us[i]     = slot->rx_us;
        fresh = (uint8_t)(fresh | ((slot->valid != 0u) ? (1u << i) : 0u));
    }
    raw.fresh_mask    = fresh;
    raw.reserved[0]   = imu_st.acc_chip_id;
    raw.reserved[1]   = imu_st.gyr_chip_id;
    raw.reserved[2]   = imu_st.fail_step;
    raw.pitch_s10_rad = (float)out->est_frame.pitch;
    raw.v_s10_m_s     = (float)out->est_frame.speed;
    (void)target_usb_cdc_send_bench_raw(now_us64, 0u, &raw);
}

/* ============================================================================
 * 5. Main Entry Point & 500 Hz Control Loop
 * ============================================================================ */

int main(void)
{
    /* FW10: vector table, MPU guard and caches before anything else. */
    SCB->VTOR = (uint32_t)g_pfnVectors;
    __DSB();
    __ISB();
    target_mpu_cache_config();
    target_stack_paint();

    HAL_Init();
    target_estop_init(); /* Permit LOW before any fallible clock setup. */
    target_board_outputs_init(); /* FW10: heater/buzzer LOW, LEDs off, IMU CS HIGH */
    target_safety_io_init();
    SystemClock_Config();
    PeriphCommonClock_Config();

    target_dwt_init(TARGET_SYSCLK_MHZ);
    target_iwdg_capture_reset_cause();

    /* FW10 bring-up image and its CAN TX gate (bound before any bus starts). */
    bringup_init(&s_bringup, (BringupMode)WBR_BRINGUP_MODE);
    target_fdcan_bind_tx_gate(&s_bringup);
    memset(&s_bringup_out, 0, sizeof(s_bringup_out));
    s_bringup_out.active_axis = BRINGUP_NO_AXIS;
    /* Never fatal: a missing/wrong sensor is reported over USB (stage 2.1). */
    (void)target_imu_init();

    /* Initialize portable 500 Hz BalanceCtrl with radio-loss policy c (BRAKE_BALANCE) */
    BalanceCtrlConfig ctrl_cfg = balance_ctrl_default_config();
    ctrl_cfg.safety_cfg.radio_loss_policy = RADIO_LOSS_BRAKE_BALANCE;
    ctrl_cfg.safety_cfg.diagnostic_only = s_bringup.mode != BRINGUP_MODE_RUN ? 1u : 0u;
    balance_ctrl_init(&s_ctrl, &ctrl_cfg);

    (void)log_ring_init(&s_flight_ring,
                        s_flight_storage,
                        TARGET_FLIGHT_RING_CAPACITY,
                        TARGET_FLIGHT_POST_FAULT_TICKS);
    balance_ctrl_attach_flight_ring(&s_ctrl, &s_flight_ring);

    /* Initialize STM32H723 hardware layer peripherals */
    if (target_fdcan_init(&s_ctrl.can_sched) != HAL_OK) {
        Error_Handler();
    }
    if (target_crsf_init() != HAL_OK) {
        Error_Handler();
    }
    if (target_usb_cdc_init() != HAL_OK) {
        Error_Handler();
    }
    if (target_iwdg_init(target_micros()) != HAL_OK) {
        Error_Handler();
    }
    if (target_timer_500hz_init() != HAL_OK) {
        Error_Handler();
    }

    TargetOperator operator_state = {0};
    uint32_t tick_us       = 0u;
    uint32_t tick_seq      = 0u;
    uint32_t overrun_count = 0u;

    while (1) {
        if (target_timer_500hz_consume_tick(&tick_us, &tick_seq, &overrun_count) == 0u) {
            continue;
        }

        uint32_t now_us   = target_micros();
        uint64_t now_us64 = target_micros64();

        /* 1. Poll CRSF receiver, USB CDC host link, wired E-stop latch, IWDG & FDCAN status */
        TargetCrsfSnapshot rc_snap;
        TargetUsbHostCmd   usb_cmd;
        TargetEstopState   estop_st;
        TargetIwdgStatus   iwdg_st;

        target_crsf_poll(now_us);
        now_us = target_micros();
        now_us64 = target_micros64();
        target_crsf_snapshot(now_us, &rc_snap);
        target_usb_cdc_poll(now_us, now_us64, &usb_cmd);
        target_estop_get_state(&estop_st);
        target_iwdg_get_status(&iwdg_st);
        target_fdcan_poll_status(now_us);

        /* USB reset/arm are hints only. Local reset must be a released-then-pressed
         * debounced GPIO; no assigned reset GPIO means this remains blocked. */
        uint8_t reset_req = target_operator_reset_edge(&operator_state,
                                      target_reset_button_raw(), now_us);
        uint8_t hw_reset_ack = 0u;
        uint8_t safe_reset_state = (s_ctrl.safety.state == SAFETY_STATE_DISARMED
                                  || s_ctrl.safety.state == SAFETY_STATE_ESTOP
                                  || s_ctrl.safety.state == SAFETY_STATE_FAULT) ? 1u : 0u;
        if (reset_req != 0u && safe_reset_state != 0u
            && rc_snap.radio_arm_sw == 0u && rc_snap.radio_estop_sw == 0u) {
            hw_reset_ack = target_estop_try_clear_latch(reset_req);
            if (hw_reset_ack != 0u) {
                target_iwdg_clear_fault(reset_req);
            }
            target_estop_get_state(&estop_st);
            target_iwdg_get_status(&iwdg_st);
        }
        uint8_t arm_edge = target_operator_arm_edge(&operator_state,
                                  rc_snap.failsafe_active == 0u ? 1u : 0u,
                                  rc_snap.radio_arm_sw, reset_req,
                                  (s_ctrl.safety.state == SAFETY_STATE_DISARMED
                                   && estop_st.latched == 0u) ? 1u : 0u, now_us);

        /* 2. Build deterministic 500 Hz BalanceTickInput */
        BalanceTickInput in;
        memset(&in, 0, sizeof(in));

        /* Sample after RX rings are drained so ISR-stamped frames cannot appear
         * to come from the future relative to this control iteration. */
        in.tick_us               = target_micros();
        in.enc_tick_ms           = (int32_t)(tick_seq * BALANCE_CTRL_DT_MS);
        /* FW10: one synchronous IMU sensor sample per 500 Hz tick, stamped on
         * the encoder tick base (S10 derives dt from tick deltas). */
        s_imu_fresh              = target_imu_read(in.enc_tick_ms, &in.imu_samples[0],
                                                   &s_imu_raw);
        in.imu_sample_count      = s_imu_fresh;
        in.use_can_sched_rx      = 1u;

        /* Unwired power-path sensing blocks PRECHARGE, never pretends healthy. */
        target_power_unwired_inputs(&in);
        in.estop_open            = (estop_st.pin_open != 0u || estop_st.latched != 0u) ? 1u : 0u;
        in.bor3_fault            = iwdg_st.bor3_fault;
        in.wdg_fault             = iwdg_st.wdg_fault;

        /* Pilot radio; policy c brakes and retains the normal tilt safety limit. */
        target_crsf_apply_to_safety(&rc_snap,
                                    &s_ctrl.safety.config,
                                    &in.radio_age_ms,
                                    &in.radio_arm_sw,
                                    &in.radio_estop_sw);

        /* USB trajectory incl. feed-forward; stale USB falls back to CRSF. */
        target_usb_apply_motion(&usb_cmd, rc_snap.v_target_m_s, rc_snap.yaw_target_rad_s, &in);

        in.stand_handoff_req     = (rc_snap.stand_handoff_req != 0u || usb_cmd.stand_handoff_req != 0u) ? 1u : 0u;
        in.voluntary_stop_req    = (rc_snap.voluntary_stop_req != 0u || usb_cmd.voluntary_stop_req != 0u) ? 1u : 0u;
        in.jump_req              = usb_cmd.jump_req;
        in.local_arm_req         = arm_edge;
        in.local_reset_req       = (reset_req != 0u || arm_edge != 0u) ? 1u : 0u;
        in.hw_reset_ack          = hw_reset_ack;
        in.local_regrant_req     = rc_snap.stand_handoff_req;
        in.external_inhibit_trim = 0u;

        /* FW10: BENCH / MOTOR_ID images drop every request that could leave
         * DISARMED (no-op in RUN). */
        bringup_gate_tick_input(&s_bringup, &in);

        /* 3. Execute the 8-step 500 Hz balance control pipeline */
        BalanceTickOutput out;
        balance_ctrl_tick(&s_ctrl, &in, &out);

        /* FW10: a torque request from the controller in a bench image is a
         * defect; latch the wired-stop path (disable frames on all axes). */
        if (bringup_check_tick_output(&s_bringup, &out) != 0u) {
            target_estop_force_trip();
        }

        /* FW10: bring-up step (encoder poll, MOTOR_ID pulse) behind the wired stop. */
        BringupTickInput bin;
        memset(&bin, 0, sizeof(bin));
        target_estop_get_state(&estop_st);
        bin.now_us          = target_micros();
        bin.estop_pin_open  = estop_st.pin_open;
        bin.estop_latched   = estop_st.latched;
        bin.estop_pending   = target_estop_is_tripped();
        bin.fsm_disarmed    = (s_ctrl.safety.state == SAFETY_STATE_DISARMED) ? 1u : 0u;
        bin.req.valid       = usb_cmd.motor_id_valid;
        bin.req.seq         = usb_cmd.motor_id_seq;
        bin.req.rx_us       = usb_cmd.motor_id_rx_us;
        bin.req.axis        = usb_cmd.motor_id_axis;
        bin.req.torque_nm   = usb_cmd.motor_id_torque_nm;
        bin.req.duration_ms = usb_cmd.motor_id_duration_ms;
        bringup_step(&s_bringup, &bin, &s_bringup_out);

        /* 4. Service the PD11 watchdog heartbeat and flush 3-bus FDCAN TX queues.
         * FW10: RUN = FSM permit (FW9); BENCH = always LOW; MOTOR_ID = wired
         * stop chain healthy. */
        target_estop_set_permit(bringup_permit(&s_bringup, out.safety_out.permit_out, &bin));
        can_sched_service_tx(&s_ctrl.can_sched, now_us);
        target_fdcan_flush_tx(&s_ctrl.can_sched, out.safety_out.can_abort_tx);
        for (uint8_t f = 0u; f < s_bringup_out.n_frames; ++f) {
            const BringupFrame *bf = &s_bringup_out.frames[f];
            (void)target_fdcan_send_direct(bf->bus, bf->std_id, bf->dlc, bf->data);
        }

        /* 5. Service independent watchdog (only refreshes if balance_heartbeat advanced) */
        target_iwdg_service(now_us, out.balance_heartbeat);

        /* 6. Stream COBS + CRC-32C telemetry over USB CDC. One frame per tick so
         * the single in-flight bulk IN transfer is never contended (FW10; FW9
         * sent health on a fast tick, where it was always dropped as busy):
         * even ticks TELEM_FAST 250 Hz; tick % 50 == 1 TELEM_HEALTH 10 Hz;
         * other odd ticks BENCH_RAW_LOG (~240 Hz) in BENCH / MOTOR_ID images. */
        if ((tick_seq & 1u) == 0u) {
            target_emit_telem_fast(now_us64, &in, &out);
        } else if ((tick_seq % 50u) == 1u) {
            target_emit_telem_health(now_us64, &rc_snap, &estop_st, &iwdg_st, &out);
        } else if (s_bringup.mode != BRINGUP_MODE_RUN) {
            target_emit_bench_raw(now_us64, &out);
        }

        /* FW10 indicators: RUN LED 2 Hz blink = loop alive; FAULT LED = wired
         * stop tripped, IMU sensor init failed, or bench-gate violation. */
        TargetImuStatus imu_st;
        target_imu_get_status(&imu_st);
        target_led_set(((tick_seq / 125u) & 1u) != 0u ? 1u : 0u,
                       (estop_st.latched != 0u || estop_st.pin_open != 0u
                        || imu_st.init_ok == 0u || s_bringup.bench_violations != 0u) ? 1u : 0u);

        const uint32_t busy_us = target_micros() - now_us;
        if (busy_us > s_busy_max_us) {
            s_busy_max_us = busy_us;
        }
    }
}
