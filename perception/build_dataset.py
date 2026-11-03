"""Convert Blender synthetic renders and normalized boxes to YOLO format."""
import argparse
import csv
import json
from pathlib import Path
import shutil

CLASSES = {0: "body", 1: "obstacle"}


def build(render_dir, output_dir):
    marker = render_dir / "synthetic_renders.json"
    labels = render_dir / "labels.csv"
    if not marker.is_file() or not labels.is_file():
        raise SystemExit("render_dir must contain Blender-generated labels.csv and synthetic_renders.json")
    if json.loads(marker.read_text()).get("status") != "SYNTHETIC":
        raise SystemExit("only marked synthetic renders can be included")
    rows = list(csv.DictReader(labels.open(newline="")))
    images = sorted({row["image"] for row in rows})
    if not images:
        raise SystemExit("no labeled images found")
    output_dir.mkdir(parents=True, exist_ok=True)
    for index, image in enumerate(images):
        source = render_dir / image
        if not source.is_file() or source.suffix.lower() != ".png":
            raise SystemExit(f"missing synthetic PNG: {image}")
        split = "val" if index % 5 == 0 and len(images) > 1 else "train"
        image_dir = output_dir / "images" / split
        label_dir = output_dir / "labels" / split
        image_dir.mkdir(parents=True, exist_ok=True)
        label_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, image_dir / source.name)
        with (label_dir / f"{source.stem}.txt").open("w") as handle:
            for row in rows:
                if row["image"] != image:
                    continue
                class_id = int(row["class_id"])
                box = [float(row[key]) for key in ("x_center", "y_center", "width", "height")]
                if class_id not in CLASSES or any(value < 0 or value > 1 for value in box):
                    raise SystemExit(f"invalid normalized box: {row}")
                handle.write(f"{class_id} " + " ".join(f"{value:.8f}" for value in box) + "\n")
    lines = [f"path: {output_dir.resolve()}", "train: images/train", "val: images/val", "names:"]
    lines.extend(f"  {class_id}: {name}" for class_id, name in CLASSES.items())
    (output_dir / "data.yaml").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("render_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    build(args.render_dir, args.output_dir)
