"""Pytest-free test runner executing all unit and Python<->C cross tests."""
from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tests.test_cross_cref import CROSS_TESTS
from tests.test_unit import UNIT_TESTS


def main() -> int:
    total = 0
    passed = 0

    print("=== pi_link Unit Tests (Pure Python) ===")
    for name, fn in UNIT_TESTS:
        total += 1
        fn()
        passed += 1
        print(f"  [PASS] {name}")

    print("\n=== pi_link Cross Tests (Python <-> C shared library libc_ref.so) ===")
    for name, fn in CROSS_TESTS:
        total += 1
        fn()
        passed += 1
        print(f"  [PASS] {name}")

    print(
        f"\nSUMMARY: {passed}/{total} test suites passed "
        f"({len(UNIT_TESTS)} unit, {len(CROSS_TESTS)} cross)."
    )
    assert passed == total
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
