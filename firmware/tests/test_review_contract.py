"""Host regression checks for HFX2-02 netlist roles and HFX2-04 audit output."""
from pathlib import Path
import re
import sys
import unittest

# FW6FIX (FW6REVIEW D4): the netlist and clock audit are vendored under
# fw/tests/contract so CTest passes from a fresh copy of fw/ alone.
FW = Path(__file__).resolve().parents[1]
CONTRACT = FW / "tests" / "contract"
sys.path.insert(0, str(CONTRACT))
import clock_check


class ReviewContract(unittest.TestCase):
    def test_netlist_connector_roles(self):
        text = (CONTRACT / "ctrl_rev4.net").read_text()
        expected = {"J5": "CAN1 RIGHT MOTOR", "J6": "CAN2 LEFT MOTOR", "J7": "CAN3 SPARE"}
        for ref, role in expected.items():
            self.assertRegex(text, rf'\(comp \(ref "{ref}"\)\s+\(value "{role}"\)')
        doc = (FW / "can_map/REV4_CAN_MAPPING.md").read_text()
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
        self.assertNotIn("Falling edge", (target / "target_estop.c").read_text())
        header = (target / "target_estop.h").read_text()
        self.assertIn("both PB12 and PD7 LOW", header)
        for name in ("target_iwdg.c", "target_iwdg.h", "target_hw.h"):
            text = (target / name).read_text()
            self.assertIn("50.476", text)
            self.assertNotIn("reload = 50", text)
        self.assertIn("260000000u", (target / "target_timer.c").read_text())
        cmake = (FW / "CMakeLists.txt").read_text()
        arm = re.search(r"set\(WBR_STM32H723_FW_CFLAGS(.*?)\n\)", cmake, re.S).group(1)
        host = re.search(r"target_compile_options\(test_target_hal PRIVATE ([^)]+)\)", cmake).group(1)
        for flags in (arm, host):
            self.assertIn("-Wconversion", flags)
            self.assertIn("-Werror", flags)


if __name__ == "__main__":
    unittest.main()
