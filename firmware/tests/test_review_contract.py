"""Host regression checks for the CTRL rev4 CAN roles and the clock/watchdog audit."""
from pathlib import Path
import re
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]  # repository root
FW = ROOT / "firmware"
sys.path.insert(0, str(FW / "tools"))
import clock_check  # noqa: E402


class ReviewContract(unittest.TestCase):
    def test_board_connector_roles(self):
        board = (ROOT / "hardware/CTRL/CTRL.kicad_pcb").read_text(encoding="utf-8")
        expected = {"J5": "CAN1 RIGHT MOTOR", "J6": "CAN2 LEFT MOTOR", "J7": "CAN3 SPARE"}
        for ref, role in expected.items():
            start = board.find(f'(property "Reference" "{ref}"')
            self.assertGreaterEqual(start, 0, ref)
            value = re.search(r'\(property "Value" "([^"]*)"', board[start:])
            self.assertIsNotNone(value, ref)
            self.assertEqual(value.group(1), role)
        doc = (FW / "can_map/REV4_CAN_MAPPING.md").read_text(encoding="utf-8")
        for connector in expected:
            self.assertIn(f"| {connector} |", doc)

    def test_clock_report_matches_configuration(self):
        results, _ = clock_check.run_checks(FW / "firmware/target")
        self.assertFalse([r for r in results if r.status == "FAIL"])
        uart = next(r for r in results if r.domain == "4. CRSF UART")
        self.assertEqual(uart.item, "USART6 420000 baud error")
        self.assertIn("419354.84", uart.computed)
        watchdog = next(r for r in results if r.domain == "7. IWDG1 (LSI)")
        self.assertEqual(watchdog.status, "PASS")
        self.assertIn("RLR=52 -> 53 ticks", watchdog.computed)
        self.assertIn("50.476", watchdog.computed)
        self.assertIn("Reload=52 yields 53 counts = 53.000 ms", watchdog.note)
        for result in results:
            self.assertNotRegex(result.source, r":\d")  # No stale line hints.

    def test_clock_cli_propagates_failures(self):
        # Run CLI with a fabricated failing result; verify the non-strict gate
        # cannot return success for a failed clock audit.
        fake = clock_check.CheckResult("fixture", "bad clock", "FAIL", "0", ">0", "fixture")
        original_checks = clock_check.run_checks
        original_print = clock_check.print_report
        try:
            clock_check.run_checks = lambda _: ([fake], {})
            clock_check.print_report = lambda _: None
            self.assertEqual(clock_check.main([]), 1)
        finally:
            clock_check.run_checks = original_checks
            clock_check.print_report = original_print

    def test_safety_comments_and_conversion_flags(self):
        target = FW / "firmware/target"
        self.assertNotIn("Falling edge", (target / "target_estop.c").read_text(encoding="utf-8"))
        header = (target / "target_estop.h").read_text(encoding="utf-8")
        self.assertIn("both PB12 and PD7 LOW", header)
        for name in ("target_iwdg.c", "target_iwdg.h", "target_hw.h"):
            text = (target / name).read_text(encoding="utf-8")
            self.assertIn("50.476", text)
            self.assertNotIn("reload = 50", text)
        self.assertIn("260000000u", (target / "target_timer.c").read_text(encoding="utf-8"))
        cmake = (FW / "CMakeLists.txt").read_text(encoding="utf-8")
        arm = re.search(r"set\(WBR_STM32H723_FW_CFLAGS(.*?)\n\)", cmake, re.S).group(1)
        host = re.search(r"target_compile_options\(test_target_hal PRIVATE ([^)]+)\)", cmake).group(1)
        for flags in (arm, host):
            self.assertIn("-Wconversion", flags)
            self.assertIn("-Werror", flags)


if __name__ == "__main__":
    unittest.main()
