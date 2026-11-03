# Model scope and evidence

The MuJoCo model is a simulation abstraction. Its primitive geometry and the parameter file are numerical assumptions, not a released CAD design. The five-bar kinematics and balancing controller are organized as Python modules. Saved traces use CSV so the repository contains no binary numeric result dumps.

The parity harness runs the stand and obstacle scenarios with seed zero and compares all common recorded columns at the source timestep. The recorded integration traces match exactly. A CAD obstacle reference has a maximum absolute discrepancy of 19.948454409 in the compared columns. Plant mass and geometry and the controller schedule differ between these builds; the particular cause of the cross-model discrepancy has not been isolated.

The Simscape and delay findings in the README summarize prior simulation evidence. They are not reproduced by `pytest`. No result is hardware validation.
