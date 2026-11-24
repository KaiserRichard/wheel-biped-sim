# Synthetic perception

The Blender scene generator creates five CPU rendered views and YOLO bounding boxes for a body and an obstacle. `build_dataset.py` converts those labeled synthetic renders to a deterministic train/validation layout. `train.py` and `evaluate.py` require an existing local model file and force CPU execution. No image dataset, photographs, model weights, or rendered samples are included. Blender and Ultralytics execution were not tested here, so no perception accuracy claim is made.

Example workflow on a machine with Blender and the optional Ultralytics environment:

```bash
blender --background --python perception/blender_scene.py -- /tmp/wbr-renders
python perception/build_dataset.py /tmp/wbr-renders /tmp/wbr-dataset
python perception/train.py /tmp/wbr-dataset/data.yaml --model /path/to/local/weights.pt
python perception/evaluate.py /path/to/trained/weights.pt /tmp/wbr-dataset/data.yaml
```
