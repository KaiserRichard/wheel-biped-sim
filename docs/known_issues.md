# Known issues

These are open model, hardware, and release limitations. Simulation findings describe their tested fixtures and are not hardware failure rates. [FROM FILE]

| Issue | Evidence | Effect |
|---|---|---|
| Five-bar branch check | Explained: the flagged alternate branch lies outside the joint-limit region; the regression suite passes 26/26 checks. [COMPUTED: see [branch check](studies/five-bar-branch-check.md)] | Confirm encoder zero, assembly branch, and physical joint limits on the built leg. |
| Timestep sensitivity | Explained for the checked fixtures: the large change came from the test rig’s sampled PD controller. The recommended setting is 0.25 ms with `implicitfast`. [COMPUTED: see [timestep study](studies/simulation-timestep-sensitivity.md)] | Validate against measured controller and actuator timing before making physical accuracy claims. |
| Step crossing and slow-step discrimination | The friction and speed map includes failures and incomplete cases. The gated stop protects the tested slow failures but also stops a crossable slow step; discrimination remains open. [COMPUTED: see [step study](studies/step-crossing-friction-speed.md)] | The stop is not in public firmware, and simulated stop outcomes do not establish physical safety. |
| Control-board hardware layer | Firmware pin assignments do not match the CTRL rev4 schematic for radio UART, wired stop input, and permit output; the USB clock source is out of tolerance. A correction is in progress. [FROM FILE] | Firmware to schematic mapping and clock accuracy require correction and hardware verification. |
| Motor power board | The schematic does not yet include a bulk DC-link capacitor. At least 1 mF is needed for brake-chopper analysis. [FROM FILE / STANDARD PRACTICE] | Complete the capacitor design and rerun the link and chopper analysis before treating the power design as closed. |

## What this teaches

[STANDARD PRACTICE] Geometric branch validity, numerical convergence, terrain contact, firmware pin mapping, and power-stage energy storage are separate verification questions. A passing check in one layer does not close the others.

## What the real robot must confirm

[UNKNOWN] Verify physical joint limits and encoder zero, measured controller timing, contact and stopping behavior, target pin assignments and clock accuracy, and DC-link capacitance and chopper operation on the assembled hardware.
