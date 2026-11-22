#!/usr/bin/env python3
"""OPEN: Pi-local power evidence, stdlib only. Raw output is never suppressed."""
import argparse
import datetime
import json
import re
import signal
import subprocess
import threading
from pathlib import Path


def throttled_mask(output):
    match = re.search(r"throttled=(0x[0-9a-fA-F]+)", output)
    return int(match.group(1), 16) if match else None


def summarize(masks):
    valid = [mask for mask in masks if mask is not None]
    live = any(mask & 0x1 for mask in valid)
    # Compare every sample with the previous valid sample; final-only misses a reset.
    sticky_rise = any(not a & 0x10000 and b & 0x10000 for a, b in zip(valid, valid[1:]))
    complete = bool(masks) and all(mask is not None for mask in masks)
    return {"status": "OPEN", "samples": len(masks), "monitoring_complete": complete,
            "initial_mask": masks[0] if masks else None, "final_mask": masks[-1] if masks else None,
            "initial_sticky_undervoltage": bool(valid and valid[0] & 0x10000),
            "live_undervoltage_observed": live, "sticky_undervoltage_changed": sticky_rise,
            "undervoltage_occurred": live or sticky_rise,
            "label": "DIAGNOSTIC - power not clean" if live or sticky_rise else
                     ("UNKNOWN - power monitoring incomplete" if not complete else
                      "OPEN - no under-voltage observed at 10 s sampling")}


def command(args):
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=5)
        return {"returncode": result.returncode, "stdout": result.stdout.strip(), "stderr": result.stderr.strip()}
    except (OSError, subprocess.TimeoutExpired) as error:
        return {"returncode": None, "stdout": "", "stderr": str(error)}


def sample():
    result = {"utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
              "get_throttled": command(["vcgencmd", "get_throttled"]),
              "ext5v": command(["vcgencmd", "pmic_read_adc", "EXT5V_V"]),
              "measure_temp": command(["vcgencmd", "measure_temp"]),
              "temperature_source": "Pi SoC; not camera IMU"}
    try:
        result["soc_thermal_millidegrees_c"] = int(Path("/sys/class/thermal/thermal_zone0/temp").read_text())
    except (OSError, ValueError):
        result["soc_thermal_millidegrees_c"] = None
    result["mask"] = throttled_mask(result["get_throttled"]["stdout"]) if result["get_throttled"]["returncode"] == 0 else None
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("summary", type=Path)
    parser.add_argument("ready", type=Path)
    args = parser.parse_args()
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    masks = []
    with args.log.open("x") as log:
        while True:
            reading = sample()
            masks.append(reading["mask"])
            log.write(json.dumps(reading) + "\n"); log.flush()
            if len(masks) == 1:
                args.ready.write_text("baseline logged\n")
            if stop.is_set():
                break
            # Command time is included: next sample starts 10 s after this one.
            elapsed = (datetime.datetime.now(datetime.timezone.utc) -
                       datetime.datetime.fromisoformat(reading["utc"])).total_seconds()
            if stop.wait(max(0, 10 - elapsed)):
                # Always take one final power reading after recorder shutdown.
                final = sample(); masks.append(final["mask"])
                log.write(json.dumps(final) + "\n"); log.flush()
                break
    result = summarize(masks)
    args.summary.write_text(json.dumps(result, indent=2) + "\n")
    print(result["label"], flush=True)


if __name__ == "__main__":
    main()
