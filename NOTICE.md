# Third-party notices and provenance

## DM-jump

This project is a personal refactor that builds on Trần Nguyên Bình's DM-jump, which its repository states is MIT licensed. The robot concept, modelling approach and test scenarios come from that work. The code here was rewritten and reorganized. A one-off comparison of this repository's Python files against DM-jump's `sim/fivebar` Python found few identical lines, but that comparison isn't included here and didn't cover the MATLAB files, so treat this repository as derived from DM-jump. Upstream copyright notice: `Copyright (c) 2024-2026 Trần Nguyên Bình`.

## dmBots/wheel-legged

The project acknowledges dmBots/wheel-legged, which is MIT licensed. Its model was used as a cross-check, and `media/figures/simulink-dmbots-model.png` shows its Simulink diagram. No dmBots source file is knowingly included, though no complete comparison was made. Upstream copyright notice: `Copyright (c) 2025 seek`.

## CAD meshes

I checked the source model's `sim/wbr_model/examples/wbr_full/meshes` folder. Its README describes the eleven-mesh set as an export built from owner-provided SOLIDWORKS parts, but gives no redistribution license. I could not establish a license for the meshes or their source CAD. No mesh or CAD part files are included, and the default model is generated from primitive shapes. Clips named `cad-*`, `mass-*` and `scenario-*` are renderings of that CAD model.

## Included media and traces

Files in `media/` are simulation clips rendered from saved runs of this project. `tests/traces/*.csv` are numeric reference traces exported from saved simulation records. No datasets, camera frames, thesis pages, or real-robot photographs are included.

## MIT license text

The MIT permission and warranty text below accompanies the two upstream
copyright notices above. See [DM-jump](https://github.com/nguyenbinh-shark/DM-jump)
and [dmBots/wheel-legged](https://github.com/dmBots/wheel-legged).

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

The repository's own MIT license is in [LICENSE](LICENSE).

## P3 analysis slice

The log codec/importer, identification formulas, tests and synthetic rehearsal are imported as a reviewed file snapshot, without private Git history. Human ownership and existing upstream attribution are retained; AI assistance included Claude and Codex. No upstream firmware source is added in this slice. See [analysis provenance](docs/provenance/analyses.md) and the relative source manifest under `analyses/logsysid/` for original and sanitized hashes and limitations. No real-robot logs are included. This preparation does not establish permission for future hardware data or withheld source material.

## Batch 2 simulation methods and C additions

The S5 predictor template, S7 wheel-loss feedforward, S8 stop re-anchor, and S10 estimator are standalone additions from the simulation work. The S5, S7, and S8 methods were exercised around the DM-jump-derived controller in that work. Trần Nguyên Bình and dmBots retain the credit and MIT copyright notices above. This slice uses new files and includes no copies of their chassis tasks, drive code, or hardware headers. The owner review governs the licence for new first-party code and original explanatory text.

## KiCad learning boards

**FROM FILE** · The power board follows the partition and test-point style of the owner's private Edge AI Smart Power Module project. The control board's STM32H723 pin and peripheral map follows Trần Nguyên Bình's CtrlBoard-H7 configuration in MIT-licensed DM-jump. The DM-jump attribution and MIT terms above cover that derivation. [DM-jump](https://github.com/nguyenbinh-shark/DM-jump)

**FROM WEB / FROM FILE** · Official KiCad library material is by the KiCad library contributors and is licensed under [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/) with the KiCad design-use exception. That exception applies to the electronic designs and generated design outputs. The locally bundled KiCad library entries, including modified copies, retain CC BY-SA 4.0. The board copies include project-specific edits to official library material such as the battery connector footprint, MOSFET land pattern, MCU pin metadata and generic IMU sensor footprint. [KiCad library license and exception](https://www.kicad.org/libraries/license/) · [KiCad footprint sources](https://gitlab.com/kicad/libraries/kicad-footprints) · [KiCad symbol sources](https://gitlab.com/kicad/libraries/kicad-symbols)
