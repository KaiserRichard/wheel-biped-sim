"""Train an Ultralytics detector on a user-created synthetic dataset (CPU only)."""
import argparse
from pathlib import Path
from ultralytics import YOLO

parser = argparse.ArgumentParser()
parser.add_argument("data_yaml", type=Path)
parser.add_argument("--model", type=Path, required=True, help="Path to existing local weights")
args = parser.parse_args()
if not args.data_yaml.is_file() or not args.model.is_file():
    raise SystemExit("provide a local synthetic dataset YAML and existing local weights")
YOLO(str(args.model)).train(data=str(args.data_yaml), device="cpu")
