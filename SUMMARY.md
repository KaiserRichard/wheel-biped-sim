# Summary

Refactored the WBR simulation into a Python package with five-bar dynamics, one configurable controller, MuJoCo scenarios, analysis, a primitive MJCF generator, reference-trace checks, MATLAB source, perception entry points, and an unverified RGB-D SLAM launch kit. Added the README, license and notices, citation metadata, CI workflow, publish scan, and one simulation clip. Excluded CAD meshes because their redistribution license could not be established.

## Rerun

From this directory, run `python check.py`, then `python -m pytest -q`. `scripts/run_all.sh` runs both. Use the supplied CPU MuJoCo Python environment when available. `python models/generate.py` writes the primitive model. `python scripts/stand.py --seed 0` writes a labeled CSV trace and JSON receipt.

## Results and failures

MEASURED: seed-zero stand and obstacle traces match the saved integration references exactly (0 maximum absolute difference across the recorded columns). MEASURED: the CAD stand comparison has a maximum absolute difference of 1.785469515; the CAD obstacle comparison has a maximum absolute difference of 19.948454409. Those builds have different plant properties and controller setup; the exact causes have not been isolated. MEASURED in archived simulation results: obstacle attempts failed at assumed 2–8 ms added delay. The archived Simscape balance failed with peak pitch 89.8903707414° (details in `docs/simscape_balance.csv`). The Simscape builder is omitted because its input meshes have no verified redistribution license. No MATLAB run, camera run, or hardware test was performed here. Physical robot masses and unverified design values remain ASSUMED.

## Open questions

The source of the CAD mesh export has no verified redistribution license. The exact cause of CAD trace differences, Simscape balance failure, physical mass properties, motor capability, camera behavior, and hardware performance remain unresolved. Synthetic perception label export remains incomplete.
