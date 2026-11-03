"""Re-run deterministic reference scenarios and compare saved traces."""
import csv
import json
from pathlib import Path

import numpy as np

from wbr.control import controller
from wbr.sim import scenarios

ROOT = Path(__file__).resolve().parents[2]
TRACES = ROOT / "tests" / "traces"


def compare():
    params = controller.load_design()
    gains = json.loads((ROOT / "wbr" / "control" / "gains.json").read_text())
    coef = np.asarray(gains["poly_coef"])
    results = {}
    for name in ("stand", "obstacle"):
        metrics, trace, _ = scenarios.ALL[name](params, coef, seed=0)
        if not metrics["pass"]:
            raise AssertionError(f"{name}: measured run did not meet its saved acceptance rule")
        with (TRACES / f"{name}_integration.csv").open(newline="") as f:
            reader = csv.reader(line for line in f if not line.startswith("#"))
            keys = next(reader)
            baseline = np.asarray([[float(v) for v in row] for row in reader])
        actual = np.column_stack([trace[key] for key in keys])
        if actual.shape != baseline.shape:
            raise AssertionError(f"{name}: shape {actual.shape} differs from reference {baseline.shape}")
        err = np.abs(actual - baseline)
        results[name] = {
            "measured_metrics": metrics,
            "max_abs_error": float(np.max(err)),
            "columns": len(keys),
            "samples": len(actual),
        }
        if not np.allclose(actual, baseline, rtol=1e-10, atol=1e-12):
            raise AssertionError(f"{name}: trace mismatch, max abs error {results[name]['max_abs_error']:.6g}")
        with (TRACES / f"{name}_cad_reference.csv").open(newline="") as f:
            reader = csv.reader(line for line in f if not line.startswith("#"))
            cad_keys = next(reader)
            cad_reference = np.asarray([[float(v) for v in row] for row in reader])
        cad_actual = np.column_stack([trace[key] for key in cad_keys])
        if cad_actual.shape != cad_reference.shape:
            raise AssertionError(f"{name}: CAD reference shape differs")
        by_column = np.max(np.abs(cad_actual - cad_reference), axis=0)
        largest = int(np.argmax(by_column))
        results[name]["cad_reference_max_abs_difference"] = float(by_column[largest])
        results[name]["cad_reference_largest_difference_column"] = cad_keys[largest]
        results[name]["cad_reference_max_abs_difference_by_column"] = {
            key: float(value) for key, value in zip(cad_keys, by_column)
        }
    return results


if __name__ == "__main__":
    print(json.dumps(compare(), indent=2))
