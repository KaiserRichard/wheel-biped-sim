from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))
from wbr.analysis.parity import compare
from publish_check import scan

if __name__ == "__main__":
    privacy_findings = scan()
    if privacy_findings:
        raise SystemExit("\n".join(privacy_findings))
    receipt = {
        "status": "MEASURED",
        "script": "check.py",
        "output": "results/parity.json",
        "comparisons": compare(),
    }
    results = ROOT / "results"
    results.mkdir(exist_ok=True)
    (results / "parity.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))
