"""SYNTHETIC Pi commands only: no camera/SoC access and all files under tests."""
import json
import os
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class WrapperTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="synthetic-wrapper-", dir=ROOT / "tests")
        self.addCleanup(self.temp.cleanup)
        self.work = Path(self.temp.name)
        self.bin = self.work / "bin"; self.bin.mkdir()
        self.home = self.work / "home"; self.home.mkdir()
        self.env = dict(os.environ.copy(), CAMERA_SERIAL="ANY")
        self.env.update({"HOME": str(self.home), "PATH": str(self.bin) + os.pathsep + self.env["PATH"]})
        self.script("vcgencmd", """#!/bin/bash
case "$1" in
get_throttled) echo "throttled=${SYNTHETIC_MASK:-0x0}";;
pmic_read_adc) echo "EXT5V_V volt(24)=5.1000V";;
measure_temp) echo "temp=45.0'C";;
esac
""")
        self.script("df", """#!/bin/bash
echo 'Filesystem 1024-blocks Used Available Capacity Mounted on'
echo "SYNTHETIC 999999999 0 ${SYNTHETIC_FREE_KB:-999999999} 0% /synthetic"
""")
        self.script("recorder", """#!/usr/bin/env python3
import json, os, signal, sys, time
from pathlib import Path
run = Path(sys.argv[1])
assert not list(run.iterdir()), "wrapper polluted recorder directory"
stop = False
def signal_stop(*_):
    global stop
    stop = True
signal.signal(signal.SIGTERM, signal_stop)
(run / "synthetic.started").write_text("SYNTHETIC ONLY")
if os.getenv("SYNTHETIC_WAIT_SIGNAL"):
    while not stop:
        time.sleep(0.01)
(run / "meta.json").write_text(json.dumps({"status": "OPEN", "synthetic": True, "clean_stop": True}))
print("SYNTHETIC recorder clean stop", flush=True)
""")
        self.env["ALLAN_RECORDER"] = str(self.bin / "recorder")

    def script(self, name, content):
        path = self.bin / name; path.write_text(content); path.chmod(0o755)

    def run_wrapper(self, name):
        return subprocess.run(["bash", str(ROOT / "record_allan.sh"), "0.01", name],
                              env=self.env, text=True, capture_output=True, timeout=15)

    def test_clean_stop_and_no_overwrite(self):
        result = self.run_wrapper("clean")
        self.assertEqual(result.returncode, 0, result.stderr)
        run = self.home / "wbr-evidence/allan-clean"
        self.assertTrue(json.loads((run / "meta.json").read_text())["clean_stop"])
        self.assertFalse(json.loads((run / "power_summary.json").read_text())["undervoltage_occurred"])
        self.assertIn("207,360,000 CSV bytes/hour", result.stderr)
        self.assertTrue((run / "tool-sha256.txt").exists())
        original = (run / "meta.json").read_text()
        result = self.run_wrapper("clean")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Run path exists", result.stderr)
        self.assertEqual((run / "meta.json").read_text(), original)

    def test_live_undervoltage_and_space_refusal(self):
        self.env["SYNTHETIC_MASK"] = "0x1"
        result = self.run_wrapper("power")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("DIAGNOSTIC - power not clean", result.stdout)
        self.env["SYNTHETIC_FREE_KB"] = "1"
        result = self.run_wrapper("no-space")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("Insufficient free space", result.stderr)
        self.assertFalse((self.home / "wbr-evidence/allan-no-space").exists())

    def test_sigterm_drains_recorder_and_finalizes_power(self):
        self.env["SYNTHETIC_WAIT_SIGNAL"] = "1"
        process = subprocess.Popen(["bash", str(ROOT / "record_allan.sh"), "0.01", "signal"],
                                   env=self.env, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            run = self.home / "wbr-evidence/allan-signal"
            deadline = time.monotonic() + 5
            while not (run / "synthetic.started").exists() and time.monotonic() < deadline:
                time.sleep(0.02)
            self.assertTrue((run / "synthetic.started").exists())
            process.send_signal(signal.SIGTERM)
            stdout, stderr = process.communicate(timeout=10)
            self.assertEqual(process.returncode, 0, stdout + stderr)
            self.assertTrue(json.loads((run / "meta.json").read_text())["clean_stop"])
            power = json.loads((run / "power_summary.json").read_text())
            self.assertGreaterEqual(power["samples"], 2)
        finally:
            if process.poll() is None:
                process.kill(); process.wait()


if __name__ == "__main__":
    unittest.main(verbosity=2)
