# Known issues

These are open model and release limitations. Values below are results for named simulation fixtures, not hardware failure rates. [FROM FILE]

| Issue | Evidence | Effect |
|---|---|---|
| Five-bar forward-kinematics branch | Firmware-selected circle intersection used the wrong assembly branch in 62 of 500 tested poses. [COMPUTED: the verification report, as indexed in `NUMBERS.md`] | Leg position, force mapping, and sign interpretation need an explicit branch contract. |
| Timestep sensitivity | Peak pitch changed 51.73% and 17.00% between 1 ms and 0.5 ms on two model variants in a stand-and-push fixture. [COMPUTED: provenance is listed in `NUMBERS.md`] | Dynamic results on those variants need numerical follow-up before relying on them. |
| Step obstacles | All six tested 5 mm or 10 mm step fixtures ended in FAULT at the tested nominal friction. [COMPUTED: the benchmark report, as indexed in `NUMBERS.md`] | The current model does not pass those tested scenarios. This does not establish a universal real-world step limit. |
| STM32 hardware layer | The target hardware layer is not public yet; safety review found issues that are being fixed. [FROM FILE: staging release scope] | Host-side firmware-in-the-loop results do not validate target execution or release readiness. |

## What this teaches

A stable constraint solver does not guarantee correct kinematics, and a correct geometric branch does not guarantee timestep convergence. Keep these checks separate and preserve failing examples as regression evidence. [STANDARD PRACTICE]

## What the real robot must confirm

Confirm five-bar assembly branch and encoder zero, compare dynamic traces across refined timesteps, and measure contact/friction before interpreting step performance. Target-layer findings must be resolved and reviewed before hardware claims. [UNKNOWN / STANDARD PRACTICE]
