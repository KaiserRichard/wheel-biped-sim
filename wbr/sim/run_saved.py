"""Shared result writer for the named experiment entry points."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from wbr.control.controller import load_design
from wbr.sim.scenarios import ALL

ROOT = Path(__file__).resolve().parents[2]


def run_saved(scenario):
    parser = argparse.ArgumentParser(description=f"Run the {scenario} MuJoCo scenario")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    gains = json.loads((ROOT / "wbr/control/gains.json").read_text())
    metrics, trace, _ = ALL[scenario](
        load_design(), np.asarray(gains["poly_coef"]), seed=args.seed
    )
    out = ROOT / "results"
    out.mkdir(exist_ok=True)
    with (out / f"{scenario}.csv").open("w", newline="") as f:
        f.write(f"# MEASURED by scripts/{scenario}.py; seed={args.seed}.\n")
        keys = list(trace)
        writer = csv.writer(f)
        writer.writerow(keys)
        writer.writerows(zip(*(trace[key] for key in keys)))
    receipt = {
        "status": "MEASURED",
        "script": f"scripts/{scenario}.py",
        "seed": args.seed,
        "output_trace": f"results/{scenario}.csv",
        "metrics": metrics,
    }
    (out / f"{scenario}.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))
