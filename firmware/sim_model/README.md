# FIL model (not published)

`tests/fil/run_fil.py` drives the firmware against a MuJoCo model of the robot (`draw_001.xml` and eleven STL meshes exported from the CAD model). Those meshes have no established redistribution licence, so the model is not included here; see [NOTICE](../../NOTICE.md).

The host C tests (`ctest`) run without it. To run FIL, place a compatible MuJoCo model at `sim_model/draw_001.xml` with its meshes in `sim_model/meshes/`.
