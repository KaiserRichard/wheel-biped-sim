# Lab-day runbook

**FROM FILE.** The Phase 1 procedure starts with power, temperature, USB link, disk space, topic rates and a bag-write smoke check. Time-stamp each run, retain its camera settings and transforms, and capture bags in a local evidence directory. [Original lab-day procedure](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/docs/LAB_DAY_EXECUTION.md)

1. **STANDARD PRACTICE.** Confirm camera calibration, time source and topic names before recording. Verify that playback reaches the consumers in an isolated ROS domain.
2. **FROM FILE.** Record a stationary interval, an empty floor, measured obstacles at several ranges, a wall target, and a short slow approach. Record tape-measured height and forward distance before moving the fixture.
3. **FROM FILE.** Save camera info, depth, transforms, advisories and any pose topics relevant to the run. Use `ros2 bag info` to verify the streams and duration before moving to the next run. [Recording and analysis tools](../scripts/camera_day)
4. **STANDARD PRACTICE.** Report false stops, missed obstacles and stale outputs alongside the successful runs. Keep the STM32 as the safety authority when testing a mounted system.
