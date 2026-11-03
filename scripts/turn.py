#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from wbr.sim.run_saved import run_saved

if __name__ == "__main__":
    run_saved("turn")
