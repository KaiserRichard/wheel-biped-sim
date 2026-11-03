# Third-party notices and provenance

## DM-jump

This project is a personal refactor that builds on Trần Nguyên Bình's DM-jump, which its repository states is MIT licensed. The robot concept, modelling approach and test scenarios come from that work. The code here was rewritten and reorganized: a per-file comparison of the Python files against DM-jump's `sim/fivebar` sources found at most about 6% of non-comment lines in common. Upstream copyright notice: `Copyright (c) 2024-2026 Trần Nguyên Bình`.

## dmBots/wheel-legged

The project acknowledges dmBots/wheel-legged, which is MIT licensed. No dmBots source file or binary is copied into this tree. Upstream copyright notice: `Copyright (c) 2025 seek`.

## CAD meshes

I checked the source model's `sim/wbr_model/examples/wbr_full/meshes` folder. Its README describes the eleven-mesh set as an export built from owner-provided SOLIDWORKS parts, but gives no redistribution license. I could not establish a license for the meshes or their source CAD. No mesh or CAD part files are included, and the default model is generated from primitive shapes. Clips named `cad-*`, `mass-*` and `scenario-*` are renderings of that CAD model.

## Included media and traces

Files in `media/` are simulation clips rendered from saved runs of this project. `tests/traces/*.csv` are numeric reference traces exported from saved simulation records. No datasets, camera frames, thesis pages, or real-robot photographs are included.

## MIT license text

The two upstream projects are identified above for attribution. The repository's own MIT license applies to its original files only.
