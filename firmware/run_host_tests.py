"""Run the CMake-listed host tests with a GCC-compatible C11 compiler."""
from pathlib import Path
import argparse
import os
import re
import subprocess

ROOT = Path(__file__).resolve().parent

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default="gcc")
    parser.add_argument("--release", action="store_true")
    args = parser.parse_args()
    out = ROOT / "build" / ("release" if args.release else "debug")
    out.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["PATH"] = str(Path(args.cc).resolve().parent) + os.pathsep + env.get("PATH", "")
    cmake = (ROOT / "CMakeLists.txt").read_text()
    sources = re.search(r"set\(WBR_FIRMWARE_SOURCES(.*?)\)", cmake, re.S)[1].split()
    common = ["-std=c11", "-Wall", "-Wextra", "-Wpedantic", "-Wshadow", "-Wconversion", "-ffp-contract=off"]
    common += ["-O2", "-DNDEBUG"] if args.release else ["-O0", "-g"]
    include = ["-Ifirmware", "-Is11_verified/tests"]
    # MinGW math classification macros emit float-conversion diagnostics.
    platform_flags = ["-Wno-error=float-conversion"] if os.name == "nt" else []
    objects = []
    for i, source in enumerate(sources):
        obj = out / f"module_{i}.o"
        subprocess.run([args.cc, *common, "-Werror", *platform_flags, *include, "-c", source, "-o", str(obj)], cwd=ROOT, env=env, check=True)
        objects.append(str(obj))
    tests = re.findall(r"^add_fw_test\(\s*(\w+)\s+(\S+)\s*\)", cmake, re.M)
    tests = [(name, "tests/" + source) for name, source in tests]
    tests += re.findall(r"^add_s11_verified_test\(\s*(\w+)\s+(\S+)\s*\)", cmake, re.M)
    passed = 0
    for name, source in tests:
        exe = out / (name + (".exe" if os.name == "nt" else ""))
        subprocess.run([args.cc, *common, "-UNDEBUG", *include, source, *objects, "-lm", "-pthread", "-o", str(exe)], cwd=ROOT, env=env, check=True)
        result = subprocess.run([str(exe)], cwd=ROOT, env=env, capture_output=True, text=True)
        print(result.stdout, end="", flush=True)
        if result.returncode:
            print(result.stderr, flush=True)
            raise SystemExit(result.returncode)
        passed += 1
        print(f"PASS {name}", flush=True)
    print(f"SUMMARY: {passed}/{len(tests)} host tests passed ({'Release' if args.release else 'Debug'}).")

if __name__ == "__main__":
    main()
