"""Run all tests and the full rehearsal; save a portable verification receipt."""
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent


def main():
    started = time.time()
    runs = []
    for args in (["-m", "unittest", "discover", "-s", "tests", "-v"], ["rehearsal.py"]):
        result = subprocess.run([sys.executable] + args, cwd=ROOT,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        print(result.stdout, end="", flush=True)
        runs.append({"args": args, "exit_code": result.returncode})
        if result.returncode:
            raise SystemExit(result.returncode)
    results = json.loads((ROOT / "rehearsal_results.json").read_text())
    assert abs(results["studio_roundtrip"]["offset_error_us"]) < 1e-3
    assert results["studio_roundtrip"]["compare"]["est.pitch"]["rms"] < 2e-6
    assert max(r["max_abs_error_s"] for r in results["delay_timestamps"]["rows"]) < 1e-9
    chain = results["calibration_chain"]["relative_error_p90"]
    assert chain["k"] < .02 and chain["Tc"] < .05 and chain["b"] < .05
    assert chain["lag"] < .1 and chain["k_over_J"] < .02 and chain["J"] < .03
    files = list(ROOT.glob("*.py")) + list(ROOT.glob("*.md"))
    for directory in ("ident", "wbrlog", "tests"):
        files.extend((ROOT / directory).glob("*.py"))
    files.extend(ROOT / f for f in ("m13_loop3_estimation.csv", "rehearsal_results.json"))
    manifest_path = ROOT / "SOURCE_MANIFEST.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["released_files"] = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                                  for p in sorted(files)}
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    files.append(manifest_path)
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    receipt = {"status": "PASS", "measurement_domain": "synthetic",
               "hardware_acceptance": "OPEN: no real stationary log supplied",
               "python": platform.python_version(),
               "generated_local": time.strftime("%Y-%m-%d %H:%M:%S"),
               "runtime_s": time.time() - started, "runs": runs, "sha256": hashes}
    (ROOT / "verification.json").write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    print("Verification PASS", flush=True)


if __name__ == "__main__":
    main()
