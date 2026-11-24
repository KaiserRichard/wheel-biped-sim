# Known issues

These are open model, hardware, and release limitations. Simulation findings describe their tested fixtures and are not hardware failure rates. [FROM FILE]

| Issue | Evidence | Effect |
|---|---|---|
| Five-bar branch check | Explained: the flagged alternate branch lies outside the joint-limit region; the regression suite passes 26/26 checks. [COMPUTED: see [branch check](studies/five-bar-branch-check.md)] | Confirm encoder zero, assembly branch, and physical joint limits on the built leg. |
| Timestep sensitivity | Explained for the checked fixtures: the large change came from the test rig’s sampled PD controller. The recommended setting is 0.25 ms with `implicitfast`. [COMPUTED: see [timestep study](studies/simulation-timestep-sensitivity.md)] | Validate against measured controller and actuator timing before making physical accuracy claims. |
| Step crossing and slow-step discrimination | The friction and speed map includes failures and incomplete cases. The gated stop protects the tested slow failures but also stops a crossable slow step; discrimination remains open. [COMPUTED: see [step study](studies/step-crossing-friction-speed.md)] | The stop is not in public firmware, and simulated stop outcomes do not establish physical safety. |
| Control-board hardware layer | The STM32H723 layer is now published in `firmware/firmware/target/`, with pin and CAN mapping matched to CTRL rev4 and the wired-stop gate checking both inputs' pending interrupt flags. It has been checked only in host tests (41/41, target code against a HAL stub), a linked Cortex-M7 build, a clock audit (0 FAIL, one USART6 baud-accuracy WARN) and firmware-in-the-loop simulation. It has not run on a board. [FROM FILE / COMPUTED: see [firmware README](../firmware/README.md#stm32h723-hardware-layer)] | Before powered operation: confirm CAN harness and IDs, wired-stop levels and the external watchdog cut, oscillator tolerances, USB enumeration (currently a stub), and the IMU sensor SPI driver on the board. |
| Log demo data | The log tools are published, but the demo telemetry fixtures and overlay figure are withheld: their pitch channel carries degrees in a field labelled radians. [COMPUTED: pitch rate matches the gyro rate times 55 with correlation 0.997] | Regenerate the fixtures with pitch in radians before publishing a log comparison. |
| Motor power board | The schematic does not yet include a bulk DC-link capacitor. At least 1 mF is needed for brake-chopper analysis. [FROM FILE / STANDARD PRACTICE] | Complete the capacitor design and rerun the link and chopper analysis before treating the power design as closed. |

## What this teaches

[STANDARD PRACTICE] Geometric branch validity, numerical convergence, terrain contact, firmware pin mapping, and power-stage energy storage are separate verification questions. A passing check in one layer does not close the others.

## What the real robot must confirm

[UNKNOWN] Verify physical joint limits and encoder zero, measured controller timing, contact and stopping behavior, target pin assignments and clock accuracy, wired-stop and watchdog response, and DC-link capacitance and chopper operation on the assembled hardware.
