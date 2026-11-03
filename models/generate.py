"""Generate the default primitive-only MuJoCo model."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from wbr.control.controller import load_design
from wbr.dynamics.mjcf import build_mjcf

def main():
    target = ROOT / "models" / "wbr.xml"
    target.write_text(build_mjcf(load_design(), planar=False))
    print(target.name)

if __name__ == "__main__":
    main()
