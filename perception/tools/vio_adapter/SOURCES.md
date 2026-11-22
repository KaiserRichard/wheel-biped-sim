# Source provenance

SOURCE-SUPPORTED: ORB API audit targets commit
`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` (`4452a3c`).
Links below use one-based raw source line numbers inspected in the read-only
reference checkout. Web text extraction collapses some blank lines; its displayed
line numbers differ. This is an API/source review, not a build of the binding.

- [System.h, 107–110](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/include/System.h#L107-L110): synchronized stereo, grayscale input and `TrackStereo` signature.
- [System.cc, 316–325](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc#L316-L325): stereo tracker return is `Tcw`, tracking state copied after the call. [Frame.cc, 474–478 and 523](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Frame.cc#L474-L523) establishes world-to-camera direction. Invert `Tcw` for `T_world_camera`.
- [Tracking.h, 120–129](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/include/Tracking.h#L120-L129): enum values. Map SYSTEM_NOT_READY/NO_IMAGES_YET to STARTING, NOT_INITIALIZED to INITIALIZING, OK/OK_KLT to TRACKING, RECENTLY_LOST/LOST to LOST; unknown values to FAULT.
- [System.h, 174–178](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/include/System.h#L174-L178) and [System.cc, 1321–1325](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc#L1321-L1325): `GetTrackingState` is the most recent frame's copied state, protected by its mutex. It does not establish inertial initialization, freshness, covariance or navigation eligibility.
- [System.h, 129–135](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/include/System.h#L129-L135) and [System.cc, 490–513](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc#L490-L513): `MapChanged` compares Atlas big-change index to a function-static counter; `ResetActiveMap` sets a request flag. [System.cc, 296–308](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc#L296-L308) processes reset requests on the next stereo call. This is not a complete reset/map-switch event log; a static counter is also unsuitable for independent multiple systems.
- [Map.h, 82 and 91](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/include/Map.h#L82-L91) and [Atlas.h, 98 and 116](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/include/Atlas.h#L98-L116): big-change index, map ID, current map. [System.h, 196–214](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/include/System.h#L196-L214): Atlas is private. A reviewed engine extension must provide a copied locked metadata snapshot to `OrbMapObserver`; this library introduces no private-access hack or guessed map ID. Reset generation is an extension requirement, not a stock ORB API.
- [ImuTypes.h, 45–58](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/include/ImuTypes.h#L45-L58): `IMU::Point` constructor is accel xyz, gyro xyz, double seconds; vectors store float. The binding converts centered integer ns to seconds only at the engine boundary.
- [Tracking.cc, 1643–1665](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc#L1643-L1665): trailing integration endpoint retained in ORB's IMU queue. Adapter sends each gyro epoch once, including the first at/after each image. Actual ORB endpoint selection also uses its `mImuPer` tolerance; matching source intent does not verify engine execution.

MEASURED source attribution: `adapter.cpp::drain` and IMU append checks adapt the
first-party `wbr::Input` algorithm in
[d435i_input.h](<local workspace>/Desktop/Research/wbr-perception-phase1-runtime/experiments/2026-10-13-n4b-d435i-prep/d435i_input.h:1),
lines 13–80, from the project preparation experiment (2026-10-13).
The original double seconds become integer ns with an explicit unchanging domain.
Tests preserve
[test_d435i_input.cpp](<local workspace>/Desktop/Research/wbr-perception-phase1-runtime/experiments/2026-10-13-n4b-d435i-prep/test_d435i_input.cpp:1),
lines 16–51: bracketed interpolation, no extrapolation, prefix trimming, future
coverage, endpoint once, ordering, development gap guard and finite input checks.
Stereo pairing is added by exact epoch; unlike the development driver's skew
allowance, this task requires exact equality.

INFERRED: fixed frame names, conservative map-event invalidation, reinitialization
boundaries, queue limits, covariance tangent convention and snapshot ownership are
adapter policies. UNKNOWN: all ORB runtime behavior, locked metadata extension
implementation, full internal reset detection and calibration accuracy.
