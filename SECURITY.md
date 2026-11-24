# Security policy

This is a research and learning project for a wheel-legged robot. It is simulation, design files and host-side code; nothing here has been certified for use on real hardware.

## Reporting a vulnerability

Please report security problems privately through GitHub: open the **Security** tab of this repository and choose **Report a vulnerability**. Do not open a public issue for a security problem.

Include what is affected (file or component), how to reproduce it, and the impact you expect. You should get a first reply within two weeks.

## Scope

In scope: code in this repository, including the firmware host build, the Pi USB link, the CAN tools and the publication checker.

Out of scope: real-robot safety. The firmware and boards are design-stage and have not been tested on hardware. Never run them on a robot without your own review, a tether and a working emergency stop.
