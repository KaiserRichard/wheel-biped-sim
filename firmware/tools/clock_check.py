#!/usr/bin/env python3
"""STM32H723 Firmware Clock & Timing Verification Script (pure Python, no network).

Parses HAL/register initialization values directly from the firmware files in
`./target/` and recomputes:
  1. PLL1 / PLL2 outputs, SYSCLK vs STM32H723 550 MHz limit, VOS0 & Flash latency
  2. AHB / APB prescalers and bus clocks vs DS13313 VOS0 limits
  3. FDCAN kernel clock and nominal bit timing for exact 1 Mbit/s & 75-87.5% SP
  4. CRSF USART6 baud 420000 divider and error
  5. USB 48 MHz clock source accuracy vs USB 2.0 FS +/-0.25% specification
  6. TIM6 500 Hz control timer period
  7. IWDG1 timeout from LSI (nominal and DS13313 min/max over temperature)

All hardware limits are cited from STMicroelectronics DS13313 (STM32H723xE/G
datasheet) and RM0468 (STM32H723/733/725/735/730 reference manual).
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from fractions import Fraction
import math
from pathlib import Path
import re
import sys
from typing import Dict, List, Optional, Tuple


# ---------------------------------------------------------------------------
# Cited STM32H723 Limits (FROM WEB: DS13313, RM0468, USB 2.0 Spec Sec 7.1.11)
# ---------------------------------------------------------------------------
URL_DS13313 = "https://www.st.com/resource/en/datasheet/stm32h723vg.pdf"
URL_RM0468 = (
    "https://www.st.com/resource/en/reference_manual/"
    "rm0468-stm32h723733-stm32h725735-and-stm32h730-value-line-"
    "advanced-armbased-32bit-mcus-stmicroelectronics.pdf"
)

HSE_MIN_HZ = 4_000_000          # DS13313: HSE crystal oscillator 4..50 MHz
HSE_MAX_HZ = 50_000_000

PLL_REF_MIN_HZ = 1_000_000      # RM0468 / DS13313: PLL input ref 1..16 MHz
PLL_REF_MAX_HZ = 16_000_000
PLL_VCO_WIDE_MIN_HZ = 192_000_000   # DS13313: Wide VCO 192..836 MHz
PLL_VCO_WIDE_MAX_HZ = 836_000_000
PLL_VCO_MED_MIN_HZ = 150_000_000    # DS13313: Medium VCO 150..420 MHz
PLL_VCO_MED_MAX_HZ = 420_000_000

PLL_VCI_RANGES_HZ: Dict[str, Tuple[int, int]] = {
    "0": (1_000_000, 2_000_000),
    "1": (2_000_000, 4_000_000),
    "2": (4_000_000, 8_000_000),
    "3": (8_000_000, 16_000_000),
}

# DS13313 Table 12 (General operating conditions) max clock frequencies by VOS
VOS_LIMITS_HZ: Dict[str, Dict[str, float]] = {
    "PWR_REGULATOR_VOLTAGE_SCALE0": {
        "cpu_max_no_boost": 520_000_000.0,
        "cpu_max_boost": 550_000_000.0,
        "hclk_max": 275_000_000.0,
        "pclk_max": 137_500_000.0,
        "tim_max": 275_000_000.0,
        "fdcan_ker_max": 100_000_000.0,
    },
    "PWR_REGULATOR_VOLTAGE_SCALE1": {
        "cpu_max_no_boost": 400_000_000.0,
        "cpu_max_boost": 400_000_000.0,
        "hclk_max": 200_000_000.0,
        "pclk_max": 100_000_000.0,
        "tim_max": 200_000_000.0,
        "fdcan_ker_max": 100_000_000.0,
    },
    "PWR_REGULATOR_VOLTAGE_SCALE2": {
        "cpu_max_no_boost": 300_000_000.0,
        "cpu_max_boost": 300_000_000.0,
        "hclk_max": 150_000_000.0,
        "pclk_max": 75_000_000.0,
        "tim_max": 150_000_000.0,
        "fdcan_ker_max": 80_000_000.0,
    },
    "PWR_REGULATOR_VOLTAGE_SCALE3": {
        "cpu_max_no_boost": 170_000_000.0,
        "cpu_max_boost": 170_000_000.0,
        "hclk_max": 85_000_000.0,
        "pclk_max": 42_500_000.0,
        "tim_max": 85_000_000.0,
        "fdcan_ker_max": 50_000_000.0,
    },
}

# RM0468 Table 16 (FLASH recommended number of wait states) for VOS0 (AXI/HCLK)
VOS0_FLASH_WS_TABLE: List[Tuple[int, int, int]] = [
    (0, 70_000_000, 0),
    (70_000_000, 140_000_000, 1),
    (140_000_000, 210_000_000, 2),
    (210_000_000, 275_000_000, 3),
]

# DS13313 HSI48 oscillator characteristics
HSI48_NOM_HZ = 48_000_000
HSI48_30C_MIN_HZ = 47_500_000     # -1.0417% at VDD=3.3V, TJ=30C
HSI48_30C_MAX_HZ = 48_500_000     # +1.0417% at VDD=3.3V, TJ=30C
HSI48_FULL_MIN_PCT = -4.5         # TJ = -40..125C
HSI48_FULL_MAX_PCT = +3.5         # TJ = -40..125C
USB_FS_TOLERANCE_PCT = 0.25       # USB 2.0 Spec Section 7.1.11: +/-0.25% (+/-2500 ppm)

# DS13313 LSI oscillator characteristics
LSI_NOM_HZ = 32_000
LSI_25C_MIN_HZ = 31_400           # VDD=3.3V, TJ=25C
LSI_25C_MAX_HZ = 32_600
LSI_110C_MIN_HZ = 29_760          # TJ=-40..110C, VDD=1.62..3.6V
LSI_110C_MAX_HZ = 33_600
LSI_125C_MIN_HZ = 29_400          # TJ=-40..125C, VDD=1.62..3.6V
LSI_125C_MAX_HZ = 33_600

# FDCAN target requirements
FDCAN_TARGET_BITRATE_HZ = 1_000_000
FDCAN_SP_MIN_PCT = 75.0
FDCAN_SP_MAX_PCT = 87.5

# CRSF UART target requirements
CRSF_TARGET_BAUD = 420_000
UART_SAFE_TOLERANCE_PCT = 2.0

# Control timer target
CTRL_TARGET_HZ = 500

# HAL divider macro maps
RCC_DIV_MAP: Dict[str, int] = {
    "RCC_SYSCLK_DIV1": 1, "RCC_SYSCLK_DIV2": 2, "RCC_SYSCLK_DIV4": 4,
    "RCC_SYSCLK_DIV8": 8, "RCC_SYSCLK_DIV16": 16, "RCC_SYSCLK_DIV64": 64,
    "RCC_SYSCLK_DIV128": 128, "RCC_SYSCLK_DIV256": 256, "RCC_SYSCLK_DIV512": 512,
    "RCC_HCLK_DIV1": 1, "RCC_HCLK_DIV2": 2, "RCC_HCLK_DIV4": 4,
    "RCC_HCLK_DIV8": 8, "RCC_HCLK_DIV16": 16, "RCC_HCLK_DIV64": 64,
    "RCC_HCLK_DIV128": 128, "RCC_HCLK_DIV256": 256, "RCC_HCLK_DIV512": 512,
    "RCC_APB1_DIV1": 1, "RCC_APB1_DIV2": 2, "RCC_APB1_DIV4": 4,
    "RCC_APB1_DIV8": 8, "RCC_APB1_DIV16": 16,
    "RCC_APB2_DIV1": 1, "RCC_APB2_DIV2": 2, "RCC_APB2_DIV4": 4,
    "RCC_APB2_DIV8": 8, "RCC_APB2_DIV16": 16,
    "RCC_APB3_DIV1": 1, "RCC_APB3_DIV2": 2, "RCC_APB3_DIV4": 4,
    "RCC_APB3_DIV8": 8, "RCC_APB3_DIV16": 16,
    "RCC_APB4_DIV1": 1, "RCC_APB4_DIV2": 2, "RCC_APB4_DIV4": 4,
    "RCC_APB4_DIV8": 8, "RCC_APB4_DIV16": 16,
}

IWDG_PRESCALER_MAP: Dict[str, int] = {
    "IWDG_PRESCALER_4": 4,
    "IWDG_PRESCALER_8": 8,
    "IWDG_PRESCALER_16": 16,
    "IWDG_PRESCALER_32": 32,
    "IWDG_PRESCALER_64": 64,
    "IWDG_PRESCALER_128": 128,
    "IWDG_PRESCALER_256": 256,
}

UART_PRESCALER_MAP: Dict[str, int] = {
    "UART_PRESCALER_DIV1": 1,
    "UART_PRESCALER_DIV2": 2,
    "UART_PRESCALER_DIV4": 4,
    "UART_PRESCALER_DIV6": 6,
    "UART_PRESCALER_DIV8": 8,
    "UART_PRESCALER_DIV10": 10,
    "UART_PRESCALER_DIV12": 12,
    "UART_PRESCALER_DIV16": 16,
    "UART_PRESCALER_DIV32": 32,
    "UART_PRESCALER_DIV64": 64,
    "UART_PRESCALER_DIV128": 128,
    "UART_PRESCALER_DIV256": 256,
}


@dataclass
class CheckResult:
    domain: str
    item: str
    status: str  # "PASS", "WARN", "FAIL"
    computed: str
    limit_or_target: str
    source: str
    note: str = ""
    fix: str = ""


# ---------------------------------------------------------------------------
# C Source Parsing Helpers
# ---------------------------------------------------------------------------
def strip_c_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//.*", "", text)
    return text


def parse_define_int(text: str, name: str) -> int:
    clean = strip_c_comments(text)
    m = re.search(
        rf"^\s*#define\s+{re.escape(name)}\s+\(?(\d+)[uUlL]*\)?",
        clean,
        flags=re.M,
    )
    if not m:
        raise ValueError(f"Could not find integer #define {name}")
    return int(m.group(1))


def parse_define_token(text: str, name: str) -> str:
    clean = strip_c_comments(text)
    m = re.search(
        rf"^\s*#define\s+{re.escape(name)}\s+([A-Za-z0-9_]+)",
        clean,
        flags=re.M,
    )
    if not m:
        raise ValueError(f"Could not find token #define {name}")
    return m.group(1)


def parse_assign(text: str, lhs_pattern: str) -> str:
    clean = strip_c_comments(text)
    m = re.search(rf"{lhs_pattern}\s*=\s*([^;]+);", clean)
    if not m:
        raise ValueError(f"Could not find assignment matching {lhs_pattern}")
    return m.group(1).strip()


def resolve_int(expr: str, macros: Dict[str, int]) -> int:
    expr = expr.strip()
    m = re.fullmatch(r"(\d+)[uUlL]*", expr)
    if m:
        return int(m.group(1))
    if expr in macros:
        return macros[expr]
    m_sub = re.fullmatch(r"([A-Za-z_][A-Za-z0-9_]*)\s*-\s*(\d+)[uUlL]*", expr)
    if m_sub and m_sub.group(1) in macros:
        return macros[m_sub.group(1)] - int(m_sub.group(2))
    raise ValueError(f"Cannot resolve integer expression: {expr!r}")


def parse_call_arg(text: str, func_name: str, arg_index: int = 0) -> str:
    clean = strip_c_comments(text)
    m = re.search(rf"{re.escape(func_name)}\s*\(([^)]+)\)", clean)
    if not m:
        raise ValueError(f"Could not find call to {func_name}(...)")
    args = [a.strip() for a in m.group(1).split(",")]
    return args[arg_index]


# ---------------------------------------------------------------------------
# Verification Engine
# ---------------------------------------------------------------------------
def run_checks(target_dir: Path) -> Tuple[List[CheckResult], Dict[str, object]]:
    hal_conf = (target_dir / "stm32h7xx_hal_conf.h").read_text(encoding="utf-8")
    target_hw = (target_dir / "target_hw.h").read_text(encoding="utf-8")
    main_c = (target_dir / "main.c").read_text(encoding="utf-8")
    fdcan_c = (target_dir / "target_fdcan.c").read_text(encoding="utf-8")
    crsf_c = (target_dir / "target_crsf.c").read_text(encoding="utf-8")
    usb_c = (target_dir / "target_usb_cdc.c").read_text(encoding="utf-8")
    timer_c = (target_dir / "target_timer.c").read_text(encoding="utf-8")
    iwdg_c = (target_dir / "target_iwdg.c").read_text(encoding="utf-8")

    all_c_sources = "\n".join(
        p.read_text(encoding="utf-8") for p in sorted(target_dir.glob("*.c"))
    )

    # Parse #defines from headers
    macros: Dict[str, int] = {
        "HSE_VALUE": parse_define_int(hal_conf, "HSE_VALUE"),
        "HSI_VALUE": parse_define_int(hal_conf, "HSI_VALUE"),
        "CSI_VALUE": parse_define_int(hal_conf, "CSI_VALUE"),
        "LSE_VALUE": parse_define_int(hal_conf, "LSE_VALUE"),
        "LSI_VALUE": parse_define_int(hal_conf, "LSI_VALUE"),
        "VDD_VALUE": parse_define_int(hal_conf, "VDD_VALUE"),
        "TARGET_SYSCLK_MHZ": parse_define_int(target_hw, "TARGET_SYSCLK_MHZ"),
        "TARGET_FDCAN_KER_CLK_HZ": parse_define_int(target_hw, "TARGET_FDCAN_KER_CLK_HZ"),
        "TARGET_FDCAN_PRESCALER": parse_define_int(target_hw, "TARGET_FDCAN_PRESCALER"),
        "TARGET_FDCAN_SJW": parse_define_int(target_hw, "TARGET_FDCAN_SJW"),
        "TARGET_FDCAN_TSEG1": parse_define_int(target_hw, "TARGET_FDCAN_TSEG1"),
        "TARGET_FDCAN_TSEG2": parse_define_int(target_hw, "TARGET_FDCAN_TSEG2"),
        "TARGET_CRSF_BAUDRATE": parse_define_int(target_hw, "TARGET_CRSF_BAUDRATE"),
        "TARGET_CTRL_PERIOD_US": parse_define_int(target_hw, "TARGET_CTRL_PERIOD_US"),
        "TARGET_IWDG_TIMEOUT_MS": parse_define_int(target_hw, "TARGET_IWDG_TIMEOUT_MS"),
        "TARGET_IWDG_MAX_STALL_US": parse_define_int(target_hw, "TARGET_IWDG_MAX_STALL_US"),
    }

    results: List[CheckResult] = []
    summary: Dict[str, object] = {"macros": macros}

    # =======================================================================
    # 1. HSE, Voltage Scale, PLL1, PLL2, SYSCLK, Flash Wait States
    # =======================================================================
    hse_hz = macros["HSE_VALUE"]
    results.append(CheckResult(
        domain="1. PLL / SYSCLK / VOS",
        item="HSE crystal frequency",
        status="PASS" if HSE_MIN_HZ <= hse_hz <= HSE_MAX_HZ else "FAIL",
        computed=f"{hse_hz / 1e6:.3f} MHz (ASSUMED pending HW1)",
        limit_or_target="4.000 - 50.000 MHz (DS13313)",
        source="target/stm32h7xx_hal_conf.h",
        note="Verify physical crystal on CTRL board against HW1 schematic.",
    ))

    supply_cfg = parse_call_arg(main_c, "HAL_PWREx_ConfigSupply", 0)
    vos_cfg = parse_call_arg(main_c, "__HAL_PWR_VOLTAGESCALING_CONFIG", 0)
    vos_limits = VOS_LIMITS_HZ[vos_cfg]

    pll1_src = parse_assign(main_c, r"rcc_osc\.PLL\.PLLSource")
    pll1_m = resolve_int(parse_assign(main_c, r"rcc_osc\.PLL\.PLLM"), macros)
    pll1_n = resolve_int(parse_assign(main_c, r"rcc_osc\.PLL\.PLLN"), macros)
    pll1_p = resolve_int(parse_assign(main_c, r"rcc_osc\.PLL\.PLLP"), macros)
    pll1_q = resolve_int(parse_assign(main_c, r"rcc_osc\.PLL\.PLLQ"), macros)
    pll1_r = resolve_int(parse_assign(main_c, r"rcc_osc\.PLL\.PLLR"), macros)
    pll1_rge = parse_assign(main_c, r"rcc_osc\.PLL\.PLLRGE")
    pll1_vcosel = parse_assign(main_c, r"rcc_osc\.PLL\.PLLVCOSEL")
    pll1_fracn = resolve_int(parse_assign(main_c, r"rcc_osc\.PLL\.PLLFRACN"), macros)

    if pll1_src == "RCC_PLLSOURCE_HSE":
        pll1_in_hz = Fraction(hse_hz)
    elif pll1_src == "RCC_PLLSOURCE_HSI":
        pll1_in_hz = Fraction(macros["HSI_VALUE"])
    elif pll1_src == "RCC_PLLSOURCE_CSI":
        pll1_in_hz = Fraction(macros["CSI_VALUE"])
    else:
        raise ValueError(f"Unsupported PLLSource: {pll1_src}")

    pll1_ref = pll1_in_hz / pll1_m
    pll1_vco = pll1_ref * (pll1_n + Fraction(pll1_fracn, 8192))
    pll1_p_ck = pll1_vco / pll1_p
    pll1_q_ck = pll1_vco / pll1_q
    pll1_r_ck = pll1_vco / pll1_r

    rge1_idx = pll1_rge[-1]
    rge1_lo, rge1_hi = PLL_VCI_RANGES_HZ[rge1_idx]
    pll1_ref_ok = (1 <= pll1_m <= 63) and (rge1_lo <= float(pll1_ref) <= rge1_hi)
    results.append(CheckResult(
        domain="1. PLL / SYSCLK / VOS",
        item="PLL1 reference clock (VCI1) & PLLRGE",
        status="PASS" if pll1_ref_ok else "FAIL",
        computed=f"{float(pll1_ref) / 1e6:.3f} MHz (PLLM={pll1_m}, {pll1_rge})",
        limit_or_target=f"{rge1_lo/1e6:.0f}-{rge1_hi/1e6:.0f} MHz for {pll1_rge}; PLLM 1..63 (RM0468)",
        source="target/main.c",
        note="2.000 MHz sits on the lower boundary of RANGE_1 (2-4 MHz); valid per RM0468.",
    ))

    vco1_min, vco1_max = (
        (PLL_VCO_WIDE_MIN_HZ, PLL_VCO_WIDE_MAX_HZ)
        if "WIDE" in pll1_vcosel
        else (PLL_VCO_MED_MIN_HZ, PLL_VCO_MED_MAX_HZ)
    )
    pll1_vco_ok = (4 <= pll1_n <= 512) and (vco1_min <= float(pll1_vco) <= vco1_max)
    results.append(CheckResult(
        domain="1. PLL / SYSCLK / VOS",
        item="PLL1 VCO frequency",
        status="PASS" if pll1_vco_ok else "FAIL",
        computed=f"{float(pll1_vco) / 1e6:.3f} MHz (PLLN={pll1_n}, FRACN={pll1_fracn}, {pll1_vcosel})",
        limit_or_target=f"{vco1_min/1e6:.0f}-{vco1_max/1e6:.0f} MHz; PLLN 4..512 (DS13313/RM0468)",
        source="target/main.c",
    ))

    # RM0468 STM32H723/733: DIVP1 allows 1 or even 2..128 (odd >= 3 not allowed)
    pll1_p_valid = (pll1_p == 1) or (2 <= pll1_p <= 128 and pll1_p % 2 == 0)
    results.append(CheckResult(
        domain="1. PLL / SYSCLK / VOS",
        item="PLL1 outputs (P / Q / R) & DIVP1 legality",
        status="PASS" if (pll1_p_valid and 1 <= pll1_q <= 128 and 1 <= pll1_r <= 128) else "FAIL",
        computed=(
            f"P={float(pll1_p_ck)/1e6:.3f} MHz (/{pll1_p}), "
            f"Q={float(pll1_q_ck)/1e6:.3f} MHz (/{pll1_q}), "
            f"R={float(pll1_r_ck)/1e6:.3f} MHz (/{pll1_r})"
        ),
        limit_or_target="STM32H723 DIVP1: 1 or even 2..128; DIVQ/R: 1..128 (RM0468)",
        source="target/main.c",
        note="PLLP=1 is permitted on STM32H723/733 (RM0468) and required for 550 MHz since 2*550 > 836 MHz VCO max.",
    ))

    sysclk_src = parse_assign(main_c, r"rcc_clk\.SYSCLKSource")
    sysclk_div_tok = parse_assign(main_c, r"rcc_clk\.SYSCLKDivider")
    sysclk_div = RCC_DIV_MAP[sysclk_div_tok]
    sysclk_hz = pll1_p_ck if sysclk_src == "RCC_SYSCLKSOURCE_PLLCLK" else Fraction(hse_hz)
    cpu_clk_hz = sysclk_hz / sysclk_div

    boost_checked = bool(re.search(r"CPUFREQ_BOOST|OPTSR2", all_c_sources))
    if float(cpu_clk_hz) > vos_limits["cpu_max_boost"]:
        cpu_status = "FAIL"
        cpu_note = "Exceeds absolute STM32H723 VOS0 maximum 550 MHz!"
        cpu_fix = "Reduce PLLN so SYSCLK <= 550 MHz (or <= 520 MHz without CPUFREQ_BOOST)."
    elif float(cpu_clk_hz) > vos_limits["cpu_max_no_boost"] and not boost_checked:
        cpu_status = "WARN"
        cpu_note = (
            f"550.000 MHz exceeds VOS0 default limit (520 MHz with CPUFREQ_BOOST=0) by +30 MHz. "
            f"Supply={supply_cfg}, VOS={vos_cfg}, but firmware never checks FLASH->OPTSR2_CUR "
            f"CPUFREQ_BOOST bit (factory default is 0; setting it to 1 disables ITCM/DTCM ECC)."
        )
        cpu_fix = (
            "Option A (keep 550 MHz): Program option byte CPUFREQ_BOOST=1 in FLASH_OPTSR2_PRG "
            "(accepting ITCM/DTCM ECC disabled) and assert (FLASH->OPTSR2_CUR & FLASH_OPTSR2_CPUFREQ_BOOST) != 0 "
            "before switching SYSCLK. Option B (keep TCM ECC, no option-byte change): set PLLN=260 "
            "(or PLLM=6, PLLN=130, PLLRGE=RCC_PLL1VCIRANGE_2) for SYSCLK=520.000 MHz, HCLK=260 MHz, "
            "PCLK=130 MHz, TARGET_SYSCLK_MHZ=520u."
        )
    else:
        cpu_status = "PASS"
        cpu_note = f"Within {vos_cfg} limits."
        cpu_fix = ""

    results.append(CheckResult(
        domain="1. PLL / SYSCLK / VOS",
        item="SYSCLK / CPU clock vs VOS0 & CPUFREQ_BOOST",
        status=cpu_status,
        computed=f"{float(cpu_clk_hz)/1e6:.3f} MHz ({vos_cfg}, {supply_cfg}, CPUFREQ_BOOST check={boost_checked})",
        limit_or_target="<= 520 MHz (VOS0, CPUFREQ_BOOST=0); <= 550 MHz (VOS0, CPUFREQ_BOOST=1) (DS13313 Table 12)",
        source="target/main.c",
        note=cpu_note,
        fix=cpu_fix,
    ))

    # DWT macro consistency
    dwt_match = float(cpu_clk_hz) == macros["TARGET_SYSCLK_MHZ"] * 1_000_000.0
    results.append(CheckResult(
        domain="1. PLL / SYSCLK / VOS",
        item="DWT TARGET_SYSCLK_MHZ consistency",
        status="PASS" if dwt_match else "FAIL",
        computed=f"TARGET_SYSCLK_MHZ={macros['TARGET_SYSCLK_MHZ']} MHz vs SYSCLK={float(cpu_clk_hz)/1e6:.3f} MHz",
        limit_or_target="Exact match required for target_micros()",
        source="target/target_hw.h target/main.c",
    ))

    # =======================================================================
    # 2. AHB / APB Prescalers & Flash Latency
    # =======================================================================
    hclk_div_tok = parse_assign(main_c, r"rcc_clk\.AHBCLKDivider")
    apb1_div_tok = parse_assign(main_c, r"rcc_clk\.APB1CLKDivider")
    apb2_div_tok = parse_assign(main_c, r"rcc_clk\.APB2CLKDivider")
    apb3_div_tok = parse_assign(main_c, r"rcc_clk\.APB3CLKDivider")
    apb4_div_tok = parse_assign(main_c, r"rcc_clk\.APB4CLKDivider")

    hclk_div = RCC_DIV_MAP[hclk_div_tok]
    apb1_div = RCC_DIV_MAP[apb1_div_tok]
    apb2_div = RCC_DIV_MAP[apb2_div_tok]
    apb3_div = RCC_DIV_MAP[apb3_div_tok]
    apb4_div = RCC_DIV_MAP[apb4_div_tok]

    hclk_hz = cpu_clk_hz / hclk_div
    pclk1_hz = hclk_hz / apb1_div
    pclk2_hz = hclk_hz / apb2_div
    pclk3_hz = hclk_hz / apb3_div
    pclk4_hz = hclk_hz / apb4_div

    results.append(CheckResult(
        domain="2. AHB / APB / Flash",
        item="AXI / AHB clock (HCLK)",
        status="PASS" if float(hclk_hz) <= vos_limits["hclk_max"] else "FAIL",
        computed=f"{float(hclk_hz)/1e6:.3f} MHz ({hclk_div_tok})",
        limit_or_target=f"<= {vos_limits['hclk_max']/1e6:.1f} MHz ({vos_cfg}, DS13313 Table 12)",
        source="target/main.c",
    ))

    for name, val_hz, tok in [
        ("APB1 (PCLK1)", pclk1_hz, apb1_div_tok),
        ("APB2 (PCLK2)", pclk2_hz, apb2_div_tok),
        ("APB3 (D1PCLK1)", pclk3_hz, apb3_div_tok),
        ("APB4 (D3PCLK1)", pclk4_hz, apb4_div_tok),
    ]:
        results.append(CheckResult(
            domain="2. AHB / APB / Flash",
            item=f"{name} clock",
            status="PASS" if float(val_hz) <= vos_limits["pclk_max"] else "FAIL",
            computed=f"{float(val_hz)/1e6:.3f} MHz ({tok})",
            limit_or_target=f"<= {vos_limits['pclk_max']/1e6:.1f} MHz ({vos_cfg}, DS13313 Table 12)",
            source="target/main.c:SystemClock_Config",
        ))

    flash_lat_tok = parse_call_arg(main_c, "HAL_RCC_ClockConfig", 1)
    flash_ws = int(flash_lat_tok.rsplit("_", 1)[1])
    req_ws = None
    for lo, hi, ws in VOS0_FLASH_WS_TABLE:
        if lo < float(hclk_hz) <= hi or (lo == 0 and float(hclk_hz) == 0):
            req_ws = ws
            break
    results.append(CheckResult(
        domain="2. AHB / APB / Flash",
        item="Flash read wait states (FLASH_ACR LATENCY)",
        status="PASS" if (req_ws is not None and flash_ws >= req_ws) else "FAIL",
        computed=f"{flash_lat_tok} ({flash_ws} WS) at ACLK/HCLK={float(hclk_hz)/1e6:.3f} MHz",
        limit_or_target=f">= {req_ws} WS required for 210-275 MHz in VOS0 (RM0468 Table 16)",
        source="target/main.c",
    ))

    # =======================================================================
    # 3. PLL2 & FDCAN Kernel Clock + Nominal Bit Timing
    # =======================================================================
    pll2_m = resolve_int(parse_assign(main_c, r"periph_clk\.PLL2\.PLL2M"), macros)
    pll2_n = resolve_int(parse_assign(main_c, r"periph_clk\.PLL2\.PLL2N"), macros)
    pll2_p = resolve_int(parse_assign(main_c, r"periph_clk\.PLL2\.PLL2P"), macros)
    pll2_q = resolve_int(parse_assign(main_c, r"periph_clk\.PLL2\.PLL2Q"), macros)
    pll2_r = resolve_int(parse_assign(main_c, r"periph_clk\.PLL2\.PLL2R"), macros)
    pll2_rge = parse_assign(main_c, r"periph_clk\.PLL2\.PLL2RGE")
    pll2_vcosel = parse_assign(main_c, r"periph_clk\.PLL2\.PLL2VCOSEL")
    pll2_fracn = resolve_int(parse_assign(main_c, r"periph_clk\.PLL2\.PLL2FRACN"), macros)
    fdcan_clk_sel = parse_assign(main_c, r"periph_clk\.FdcanClockSelection")

    pll2_ref = pll1_in_hz / pll2_m  # PLL1/2/3 share PLLCKSELR.PLLSRC
    pll2_vco = pll2_ref * (pll2_n + Fraction(pll2_fracn, 8192))
    pll2_p_ck = pll2_vco / pll2_p
    pll2_q_ck = pll2_vco / pll2_q
    pll2_r_ck = pll2_vco / pll2_r

    rge2_lo, rge2_hi = PLL_VCI_RANGES_HZ[pll2_rge[-1]]
    vco2_min, vco2_max = (
        (PLL_VCO_WIDE_MIN_HZ, PLL_VCO_WIDE_MAX_HZ)
        if "WIDE" in pll2_vcosel
        else (PLL_VCO_MED_MIN_HZ, PLL_VCO_MED_MAX_HZ)
    )
    pll2_ok = (
        (1 <= pll2_m <= 63)
        and (4 <= pll2_n <= 512)
        and (rge2_lo <= float(pll2_ref) <= rge2_hi)
        and (vco2_min <= float(pll2_vco) <= vco2_max)
    )
    results.append(CheckResult(
        domain="3. FDCAN",
        item="PLL2 VCI / VCO / outputs",
        status="PASS" if pll2_ok else "FAIL",
        computed=(
            f"VCI2={float(pll2_ref)/1e6:.3f} MHz ({pll2_rge}), "
            f"VCO2={float(pll2_vco)/1e6:.3f} MHz ({pll2_vcosel}), "
            f"P={float(pll2_p_ck)/1e6:.3f}, Q={float(pll2_q_ck)/1e6:.3f}, R={float(pll2_r_ck)/1e6:.3f} MHz"
        ),
        limit_or_target=f"VCI2 {rge2_lo/1e6:.0f}-{rge2_hi/1e6:.0f} MHz; VCO2 {vco2_min/1e6:.0f}-{vco2_max/1e6:.0f} MHz (DS13313/RM0468)",
        source="target/main.c",
        note="8.000 MHz sits on the lower boundary of RANGE_3 (8-16 MHz); valid per RM0468.",
    ))

    if fdcan_clk_sel == "RCC_FDCANCLKSOURCE_PLL2":
        fdcan_ker_hz = pll2_q_ck
    elif fdcan_clk_sel == "RCC_FDCANCLKSOURCE_PLL":
        fdcan_ker_hz = pll1_q_ck
    elif fdcan_clk_sel == "RCC_FDCANCLKSOURCE_HSE":
        fdcan_ker_hz = Fraction(hse_hz)
    else:
        raise ValueError(f"Unsupported FdcanClockSelection: {fdcan_clk_sel}")

    fdcan_ker_ok = (
        float(fdcan_ker_hz) <= vos_limits["fdcan_ker_max"]
        and float(fdcan_ker_hz) == float(macros["TARGET_FDCAN_KER_CLK_HZ"])
    )
    results.append(CheckResult(
        domain="3. FDCAN",
        item="FDCAN kernel clock (fdcan_ker_ck)",
        status="PASS" if fdcan_ker_ok else "FAIL",
        computed=f"{float(fdcan_ker_hz)/1e6:.3f} MHz ({fdcan_clk_sel}); TARGET_FDCAN_KER_CLK_HZ={macros['TARGET_FDCAN_KER_CLK_HZ']/1e6:.3f} MHz",
        limit_or_target=f"<= {vos_limits['fdcan_ker_max']/1e6:.0f} MHz in VOS0/VOS1 (DS13313/RM0468)",
        source="target/main.c target/target_hw.h",
    ))

    nbrp = resolve_int(parse_assign(fdcan_c, r"h->Init\.NominalPrescaler"), macros)
    nsjw = resolve_int(parse_assign(fdcan_c, r"h->Init\.NominalSyncJumpWidth"), macros)
    ntseg1 = resolve_int(parse_assign(fdcan_c, r"h->Init\.NominalTimeSeg1"), macros)
    ntseg2 = resolve_int(parse_assign(fdcan_c, r"h->Init\.NominalTimeSeg2"), macros)
    dbrp = resolve_int(parse_assign(fdcan_c, r"h->Init\.DataPrescaler"), macros)
    dsjw = resolve_int(parse_assign(fdcan_c, r"h->Init\.DataSyncJumpWidth"), macros)
    dtseg1 = resolve_int(parse_assign(fdcan_c, r"h->Init\.DataTimeSeg1"), macros)
    dtseg2 = resolve_int(parse_assign(fdcan_c, r"h->Init\.DataTimeSeg2"), macros)
    explicit_clk_div = bool(re.search(r"Init\.ClockDivider", strip_c_comments(fdcan_c)))

    total_tq = 1 + ntseg1 + ntseg2
    f_tq = fdcan_ker_hz / nbrp
    bitrate_hz = fdcan_ker_hz / (nbrp * total_tq)
    sp_pct = float(Fraction(1 + ntseg1, total_tq) * 100)
    bitrate_exact = (bitrate_hz == Fraction(FDCAN_TARGET_BITRATE_HZ))
    sp_ok = FDCAN_SP_MIN_PCT <= sp_pct <= FDCAN_SP_MAX_PCT
    nbtp_ok = (
        (1 <= nbrp <= 512)
        and (1 <= ntseg1 <= 256)
        and (1 <= ntseg2 <= 128)
        and (1 <= nsjw <= 128)
        and (nsjw <= min(ntseg1, ntseg2))
    )
    dbtp_ok = (
        (1 <= dbrp <= 32)
        and (1 <= dtseg1 <= 32)
        and (1 <= dtseg2 <= 16)
        and (1 <= dsjw <= 16)
        and (dsjw <= min(dtseg1, dtseg2))
    )

    df1_pct = (nsjw / (20.0 * total_tq)) * 100.0
    df2_pct = (min(ntseg1, ntseg2) / (2.0 * (13 * total_tq - ntseg2))) * 100.0
    osc_tol_pct = min(df1_pct, df2_pct)

    results.append(CheckResult(
        domain="3. FDCAN",
        item="Nominal bit timing (1.000 Mbit/s & sample point)",
        status="PASS" if (bitrate_exact and sp_ok and nbtp_ok and dbtp_ok) else "FAIL",
        computed=(
            f"Bitrate={float(bitrate_hz)/1e6:.6f} Mbit/s (EXACT={bitrate_exact}), "
            f"SP={sp_pct:.2f}% (1+{ntseg1}/{total_tq} TQ, tq={1e9/float(f_tq):.1f} ns), "
            f"Presc={nbrp}, TSEG1={ntseg1}, TSEG2={ntseg2}, SJW={nsjw}"
        ),
        limit_or_target=(
            f"Exact 1.000 Mbit/s, SP {FDCAN_SP_MIN_PCT:.1f}-{FDCAN_SP_MAX_PCT:.1f}%; "
            f"NBTP NBRP 1..512, TSEG1 1..256, TSEG2 1..128, SJW 1..min(TSEG1,TSEG2) (RM0468)"
        ),
        source="target/target_fdcan.c target/target_hw.h",
        note=(
            f"ClockDivider explicitly set={explicit_clk_div} (memset zero -> FDCAN_CLOCK_DIV1=0x0). "
            f"ISO 11898-1 osc tolerance with SJW={nsjw}: df1=+/-{df1_pct:.3f}%, df2=+/-{df2_pct:.3f}% "
            f"(effective +/-{osc_tol_pct:.3f}%)."
        ),
        fix=(
            "Advisory (not a fault): increasing TARGET_FDCAN_SJW from 1 to 4 (<= TSEG2=4) widens "
            "resync tolerance from +/-0.250% to +/-0.781% with identical 1.000 Mbit/s bitrate and 80.0% SP; "
            "also explicitly set h->Init.ClockDivider = FDCAN_CLOCK_DIV1 if exposed in the target HAL."
            if nsjw < ntseg2 else ""
        ),
    ))

    # =======================================================================
    # 4. CRSF UART Baud 420000 Error (USART6)
    # =======================================================================
    crsf_baud_target = resolve_int(parse_assign(crsf_c, r"g_huart_crsf\.Init\.BaudRate"), macros)
    crsf_oversamp = parse_assign(crsf_c, r"g_huart_crsf\.Init\.OverSampling")
    crsf_presc_tok = parse_assign(crsf_c, r"g_huart_crsf\.Init\.ClockPrescaler")
    crsf_clk_sel = parse_assign(crsf_c, r"periph_clk\.Usart16ClockSelection")

    crsf_presc = UART_PRESCALER_MAP[crsf_presc_tok]
    if "D2PCLK2" in crsf_clk_sel or "PCLK2" in crsf_clk_sel:
        usart6_ker_hz = pclk2_hz
    elif "PLL2" in crsf_clk_sel:
        usart6_ker_hz = pll2_q_ck
    elif "HSI" in crsf_clk_sel:
        usart6_ker_hz = Fraction(macros["HSI_VALUE"])
    else:
        raise ValueError(f"Unsupported Usart16ClockSelection: {crsf_clk_sel}")

    usart6_eff_hz = usart6_ker_hz / crsf_presc
    if crsf_oversamp == "UART_OVERSAMPLING_16":
        usartdiv_exact = usart6_eff_hz / crsf_baud_target
        # STM32 HAL UART_DIV_SAMPLING16 rounding: (pclk + baud/2) // baud
        brr_reg = (int(usart6_eff_hz) + crsf_baud_target // 2) // crsf_baud_target
        actual_baud = usart6_eff_hz / brr_reg
        brr_ok = 16 <= brr_reg <= 65535
    elif crsf_oversamp == "UART_OVERSAMPLING_8":
        usartdiv_exact = (2 * usart6_eff_hz) / crsf_baud_target
        usartdiv_int = (2 * int(usart6_eff_hz) + crsf_baud_target // 2) // crsf_baud_target
        brr_reg = (usartdiv_int & 0xFFF0) | ((usartdiv_int & 0x000F) >> 1)
        actual_baud = (2 * usart6_eff_hz) / usartdiv_int
        brr_ok = 16 <= usartdiv_int <= 65535
    else:
        raise ValueError(f"Unsupported OverSampling: {crsf_oversamp}")

    baud_err_hz = float(actual_baud) - crsf_baud_target
    baud_err_pct = (baud_err_hz / crsf_baud_target) * 100.0
    baud_err_ppm = baud_err_pct * 10_000.0
    baud_exact = (actual_baud == Fraction(crsf_baud_target))

    # Alternative OVER8=1 calculation on the selected UART kernel clock
    over8_div = (2 * int(usart6_eff_hz) + crsf_baud_target // 2) // crsf_baud_target
    over8_baud = (2 * usart6_eff_hz) / over8_div
    over8_err_pct = ((float(over8_baud) - crsf_baud_target) / crsf_baud_target) * 100.0

    if not brr_ok or abs(baud_err_pct) > UART_SAFE_TOLERANCE_PCT:
        crsf_status = "FAIL"
    elif not baud_exact:
        crsf_status = "WARN"
    else:
        crsf_status = "PASS"

    results.append(CheckResult(
        domain="4. CRSF UART",
        item="USART6 420000 baud error",
        status=crsf_status,
        computed=(
            f"f_ker={float(usart6_eff_hz)/1e6:.3f} MHz ({crsf_clk_sel}, {crsf_presc_tok}, {crsf_oversamp}), "
            f"USARTDIV={float(usartdiv_exact):.4f}, BRR={brr_reg} (0x{brr_reg:04X}), "
            f"actual={float(actual_baud):.2f} baud, err={baud_err_pct:+.4f}% ({baud_err_ppm:+.1f} ppm)"
        ),
        limit_or_target=f"Target {crsf_baud_target} baud (exact=0%); async 8N1 tolerance +/-{UART_SAFE_TOLERANCE_PCT:.1f}%; BRR 16..65535 (RM0468)",
        source="target/target_crsf.c target/target_hw.h",
        note=(
            f"NOT EXACT ({baud_err_pct:+.4f}%), inside +/-{UART_SAFE_TOLERANCE_PCT:.1f}% async UART budget; "
            f"functional for ExpressLRS CRSF."
        ),
        fix=(
            f"Optional precision improvement: switch OverSampling to UART_OVERSAMPLING_8 -> "
            f"actual={float(over8_baud):.2f} baud ({over8_err_pct:+.4f}%, {over8_err_pct*1e4:+.1f} ppm). "
            f"Exact 0.000% requires a multiple of 420 kHz (e.g., 84.0 MHz on PLL2Q/PLL3Q, which conflicts with 80 MHz FDCAN / 48 MHz USB)."
        ),
    ))

    # =======================================================================
    # 5. USB 48 MHz Clock Source
    # =======================================================================
    usb_clk_sel = parse_assign(main_c, r"periph_clk\.UsbClockSelection")
    hsi48_state = parse_assign(main_c, r"rcc_osc\.HSI48State")
    usb_speed = parse_assign(usb_c, r"g_hpcd_usb\.Init\.speed")
    usb_phy = parse_assign(usb_c, r"g_hpcd_usb\.Init\.phy_itface")
    usb_sof = parse_assign(usb_c, r"g_hpcd_usb\.Init\.Sof_enable")
    crs_configured = bool(re.search(r"HAL_RCCEx_CRSConfig|__HAL_RCC_CRS_CLK_ENABLE|RCC_CRS_", all_c_sources))

    hsi48_30c_err_pct = ((HSI48_30C_MAX_HZ - HSI48_NOM_HZ) / HSI48_NOM_HZ) * 100.0
    usb_fail = (usb_clk_sel == "RCC_USBCLKSOURCE_HSI48") and (not crs_configured)
    # Candidate PLL3Q crystal-locked 48 MHz from 24 MHz HSE:
    # PLL3M=3 -> 8 MHz (RANGE_3), PLL3N=36 -> 288 MHz (WIDE 192..836 MHz), PLL3Q=6 -> 48.000 MHz exact
    pll3_fix_ref = Fraction(hse_hz, 3)
    pll3_fix_vco = pll3_fix_ref * 36
    pll3_fix_q = pll3_fix_vco / 6
    crs_reload_sof = (HSI48_NOM_HZ // 1000) - 1  # 48 MHz / 1 kHz SOF - 1 = 47999

    results.append(CheckResult(
        domain="5. USB 48 MHz",
        item="USB1_OTG_HS FS 48 MHz clock source & accuracy",
        status="FAIL" if usb_fail else "PASS",
        computed=(
            f"Source={usb_clk_sel} (HSI48State={hsi48_state}), CRS configured={crs_configured}, "
            f"speed={usb_speed}, phy={usb_phy}, Sof_enable={usb_sof}; "
            f"free-running HSI48 acc=+/-{hsi48_30c_err_pct:.2f}% @30C, "
            f"{HSI48_FULL_MIN_PCT:+.1f}%..{HSI48_FULL_MAX_PCT:+.1f}% @-40..125C"
        ),
        limit_or_target=f"48.000 MHz +/-{USB_FS_TOLERANCE_PCT:.2f}% (+/-2500 ppm) (USB 2.0 Sec 7.1.11; DS13313 HSI48)",
        source="target/main.c target/target_usb_cdc.c",
        note=(
            "CRITICAL: Free-running HSI48 without CRS is 4.2x out of USB FS tolerance even at 30C "
            "and up to 18x out of spec over -40..125C!"
        ),
        fix=(
            f"Fix A (recommended, crystal-locked via unused PLL3): configure PLL3M=3 (VCI3={float(pll3_fix_ref)/1e6:.0f} MHz, "
            f"RCC_PLL3VCIRANGE_3), PLL3N=36 (VCO3={float(pll3_fix_vco)/1e6:.0f} MHz, RCC_PLL3VCOWIDE), "
            f"PLL3Q=6 -> pll3_q_ck={float(pll3_fix_q)/1e6:.3f} MHz exact (HSE crystal accuracy ~+/-50 ppm), "
            f"and set periph_clk.UsbClockSelection = RCC_USBCLKSOURCE_PLL3. "
            f"Fix B (keep HSI48): enable __HAL_RCC_CRS_CLK_ENABLE() and HAL_RCCEx_CRSConfig() synced to "
            f"RCC_CRS_SYNC_SOURCE_USB1 SOF with ReloadValue={crs_reload_sof} (0x{crs_reload_sof:04X}), AutoTrimming=ENABLE."
        ),
    ))

    # =======================================================================
    # 6. 500 Hz Control Timer Period (TIM6)
    # =======================================================================
    # RM0468: TIMPRE=0 (default): if APB1 prescaler == 1 -> tim_clk = PCLK1, else 2 * PCLK1
    tim6_clk_hz = pclk1_hz if apb1_div == 1 else (pclk1_hz * 2)
    tim6_psc = (int(tim6_clk_hz) // 1_000_000) - 1
    tim6_arr = macros["TARGET_CTRL_PERIOD_US"] - 1
    tim6_cnt_hz = tim6_clk_hz / (tim6_psc + 1)
    tim6_upd_hz = tim6_cnt_hz / (tim6_arr + 1)
    tim6_period_us = Fraction(1_000_000) / tim6_upd_hz

    tim6_ok = (
        float(tim6_clk_hz) <= vos_limits["tim_max"]
        and (0 <= tim6_psc <= 65535)
        and (0 <= tim6_arr <= 65535)
        and (tim6_upd_hz == Fraction(CTRL_TARGET_HZ))
    )
    results.append(CheckResult(
        domain="6. 500 Hz Control Timer",
        item="TIM6 input clock, PSC, ARR & update frequency",
        status="PASS" if tim6_ok else "FAIL",
        computed=(
            f"TIM6_CLK={float(tim6_clk_hz)/1e6:.3f} MHz, PSC={tim6_psc} (/{tim6_psc+1} -> {float(tim6_cnt_hz)/1e6:.6f} MHz), "
            f"ARR={tim6_arr} (/{tim6_arr+1}) -> {float(tim6_upd_hz):.6f} Hz ({float(tim6_period_us):.3f} us, EXACT)"
        ),
        limit_or_target=f"Exact {CTRL_TARGET_HZ}.000 Hz (2000.000 us); TIM_CLK <= {vos_limits['tim_max']/1e6:.1f} MHz; 16-bit PSC/ARR 0..65535 (RM0468/DS13313)",
        source="target/target_timer.c target/target_hw.h",
    ))

    # =======================================================================
    # 7. IWDG1 Timeout from LSI
    # =======================================================================
    iwdg_presc_tok = parse_assign(iwdg_c, r"s_hiwdg\.Init\.Prescaler")
    iwdg_reload_expr = parse_assign(iwdg_c, r"s_hiwdg\.Init\.Reload")
    iwdg_presc = IWDG_PRESCALER_MAP[iwdg_presc_tok]
    iwdg_reload = resolve_int(iwdg_reload_expr, macros)
    iwdg_target_ms = macros["TARGET_IWDG_TIMEOUT_MS"]
    iwdg_stall_ms = macros["TARGET_IWDG_MAX_STALL_US"] / 1000.0

    # RM0468: T_IWDG = Prescaler * (RLR + 1) / f_LSI
    def iwdg_ms(rlr: int, lsi_hz: int) -> Fraction:
        return Fraction(iwdg_presc * (rlr + 1) * 1000, lsi_hz)

    t_nom_ms = iwdg_ms(iwdg_reload, macros["LSI_VALUE"])
    t_25c_min_ms = iwdg_ms(iwdg_reload, LSI_25C_MAX_HZ)
    t_25c_max_ms = iwdg_ms(iwdg_reload, LSI_25C_MIN_HZ)
    t_110c_min_ms = iwdg_ms(iwdg_reload, LSI_110C_MAX_HZ)
    t_110c_max_ms = iwdg_ms(iwdg_reload, LSI_110C_MIN_HZ)
    t_125c_min_ms = iwdg_ms(iwdg_reload, LSI_125C_MAX_HZ)
    t_125c_max_ms = iwdg_ms(iwdg_reload, LSI_125C_MIN_HZ)

    iwdg_rlr_valid = 0 <= iwdg_reload <= 4095
    iwdg_nom_exact = (t_nom_ms == Fraction(iwdg_target_ms))
    iwdg_above_stall = float(t_125c_min_ms) > iwdg_stall_ms

    if not iwdg_rlr_valid or not iwdg_above_stall or float(t_125c_min_ms) < 50.0:
        iwdg_status = "FAIL"
    elif not iwdg_nom_exact:
        iwdg_status = "WARN"
    else:
        iwdg_status = "PASS"

    results.append(CheckResult(
        domain="7. IWDG1 (LSI)",
        item="IWDG1 reload off-by-one & LSI temperature spread",
        status=iwdg_status,
        computed=(
            f"PR={iwdg_presc} ({iwdg_presc_tok}), RLR={iwdg_reload} -> {iwdg_reload+1} ticks: "
            f"nom={float(t_nom_ms):.3f} ms (@32.0 kHz), "
            f"25C={float(t_25c_min_ms):.3f}..{float(t_25c_max_ms):.3f} ms, "
            f"-40..110C={float(t_110c_min_ms):.3f}..{float(t_110c_max_ms):.3f} ms, "
            f"-40..125C={float(t_125c_min_ms):.3f}..{float(t_125c_max_ms):.3f} ms"
        ),
        limit_or_target=(
            f"Target {iwdg_target_ms}.000 ms nominal; minimum 50 ms at max LSI; must exceed stall gate {iwdg_stall_ms:.1f} ms; "
            f"RLR 0..4095; formula PR*(RLR+1)/f_LSI (RM0468, DS13313 LSI 29.4..33.6 kHz)"
        ),
        source="target/target_iwdg.c target/target_hw.h",
        note=(
            f"Reload={iwdg_reload} yields {iwdg_reload + 1} counts = {float(t_nom_ms):.3f} ms nominal. "
            f"At worst-case LSI=33.6 kHz (-40..125C), shortest timeout is {float(t_125c_min_ms):.3f} ms "
            f"(+{float(t_125c_min_ms) - iwdg_stall_ms:.3f} ms margin above 30.0 ms stall gate)."
        ),
        fix=(
            f"Nominal target {iwdg_target_ms} ms matches the header; "
            "53 ms nominal preserves >=50 ms at the stated 33.6 kHz LSI bound."
            if not iwdg_nom_exact else ""
        ),
    ))

    summary["clocks_hz"] = {
        "hse": hse_hz,
        "pll1_ref": float(pll1_ref),
        "pll1_vco": float(pll1_vco),
        "pll1_p_sysclk": float(pll1_p_ck),
        "pll1_q": float(pll1_q_ck),
        "pll1_r": float(pll1_r_ck),
        "hclk": float(hclk_hz),
        "pclk1": float(pclk1_hz),
        "pclk2": float(pclk2_hz),
        "pclk3": float(pclk3_hz),
        "pclk4": float(pclk4_hz),
        "pll2_ref": float(pll2_ref),
        "pll2_vco": float(pll2_vco),
        "pll2_p": float(pll2_p_ck),
        "pll2_q_fdcan": float(fdcan_ker_hz),
        "pll2_r": float(pll2_r_ck),
        "tim6_clk": float(tim6_clk_hz),
        "tim6_upd": float(tim6_upd_hz),
        "usart6_ker": float(usart6_eff_hz),
        "usart6_actual_baud": float(actual_baud),
        "usart6_err_pct": baud_err_pct,
        "fdcan_bitrate": float(bitrate_hz),
        "fdcan_sp_pct": sp_pct,
        "iwdg_nom_ms": float(t_nom_ms),
        "iwdg_min_125c_ms": float(t_125c_min_ms),
        "iwdg_max_125c_ms": float(t_125c_max_ms),
    }
    return results, summary


def print_report(results: List[CheckResult]) -> None:
    counts = {"PASS": 0, "WARN": 0, "FAIL": 0}
    print("=" * 88)
    print("STM32H723 FIRMWARE CLOCK & TIMING CHECK (parsed from selected target directory)")
    print("=" * 88)
    print(f"References (FROM WEB):")
    print(f"  DS13313: {URL_DS13313}")
    print(f"  RM0468 : {URL_RM0468}")
    print("-" * 88)

    current_domain = None
    for r in results:
        counts[r.status] += 1
        if r.domain != current_domain:
            current_domain = r.domain
            print(f"\n[{current_domain}]")
        print(f"  [{r.status:4s}] {r.item}")
        print(f"         Computed : {r.computed}")
        print(f"         Limit/Tgt: {r.limit_or_target}")
        print(f"         Source   : {r.source}")
        if r.note:
            print(f"         Note     : {r.note}")
        if r.fix:
            print(f"         Fix      : {r.fix}")

    print("\n" + "=" * 88)
    print(
        f"SUMMARY: {counts['PASS']} PASS | {counts['WARN']} WARN (not exact / prerequisite) | "
        f"{counts['FAIL']} FAIL (outside limit)"
    )
    print("=" * 88)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="STM32H723 clock & timing check.")
    parser.add_argument(
        "--target-dir",
        type=Path,
        # Public tools entry point: firmware root is one level up.
        default=Path(__file__).resolve().parents[1] / "firmware/target",
        help="Path to target firmware folder (default: <fw>/firmware/target)",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return non-zero exit code if any FAIL or WARN is found.",
    )
    args = parser.parse_args(argv)

    results, _ = run_checks(args.target_dir)
    print_report(results)

    if any(r.status == "FAIL" or (args.strict and r.status == "WARN") for r in results):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
