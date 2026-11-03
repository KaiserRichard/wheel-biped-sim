"""Evaluate a trained detector on a user-created synthetic validation split."""
import argparse
from pathlib import Path
from ultralytics import YOLO

parser = argparse.ArgumentParser()
parser.add_argument("weights", type=Path)
parser.add_argument("data_yaml", type=Path)
args = parser.parse_args()
if not args.weights.is_file() or not args.data_yaml.is_file():
    raise SystemExit("provide local model weights and synthetic dataset YAML")
YOLO(str(args.weights)).val(data=str(args.data_yaml), device="cpu")
