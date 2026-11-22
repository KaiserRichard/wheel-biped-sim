# Source and API provenance

SOURCE-SUPPORTED: inspected local upstream source in the read-only reference
checkout, ORB-SLAM3 revision `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`.
These citations describe source contracts, not a Mac/Pi runtime verification.

- [stereo_inertial_euroc.cc, lines 44–60](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Examples/Stereo-Inertial/stereo_inertial_euroc.cc#L44-L60): vocabulary/settings, sequence/times argv and optional output suffix.
- [lines 85–96](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Examples/Stereo-Inertial/stereo_inertial_euroc.cc#L85-L96): EuRoC camera and IMU paths.
- [lines 224–237](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Examples/Stereo-Inertial/stereo_inertial_euroc.cc#L224-L237): `f_SUFFIX.txt` and `kf_SUFFIX.txt` trajectory outputs in the working directory.
- [System.h, lines 103–105](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/include/System.h#L103-L105): caller controls `bUseViewer`, default true. MEASURED source inspection: the local example at line 132 passes false; the example spec requires an equivalently headless binary. No built binary was tested.
- [System.cc, lines 757–772](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc#L757-L772): inertial EuRoC trajectory exports the camera IMU pose, ns and xyzw. This is not robot base_link.
- [Atlas.cc, lines 58–72](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Atlas.cc#L58-L72): creation messages with map ID and keyframe ID; only the former is parsed as map identity.
- [System.cc, lines 672–676](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc#L672-L676): explicit maps-in-Atlas count.
- [Tracking.cc, lines 2130–2152](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc#L2130-L2152), [LocalMapping.cc, lines 1103–1128](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/LocalMapping.cc#L1103-L1128): failure/reset phrases may appear in different subsystems. `IMU initialized`, alternate `Resetting`, `Map count` and `Number of maps` are task-requested/fixture-compatible patterns; their presence in this upstream revision is UNKNOWN.

First-party interfaces reused without modification:

- [convert_trajectory.py, line 38](../frames/convert_trajectory.py#L38): `read_samples`, exact epoch/stamp agreement, explicit formats. [frames.py, lines 124–162](../frames/frames.py#L124-L162): `convert`, frame composition, clock/domain and epoch boundaries. Transform construction/inversion and SDK mount helpers are in lines 41–123. See [frames README](../frames/README.md) for upstream transform/export citations.
- [trajectory evaluator CLI, lines 565–590](../../../wbr-perception-phase1-runtime/tools/trajectory_eval/traj_eval.py#L565-L590) and [README](../../../wbr-perception-phase1-runtime/tools/trajectory_eval/README.md): explicit frames/clocks, format/unit flags, SE3/Sim3/none, RPE seconds, no-plots output. This tool is absent from this worktree; a spec provides its existing source path and the receipt hashes it. The source has no claimed public upstream URL; this is first-party reuse.

Third-party API contracts:

- [CPython 3.11 subprocess.py, Popen lines 747–820](https://github.com/python/cpython/blob/v3.11.0/Lib/subprocess.py#L747-L820), [wait, lines 1255–1276](https://github.com/python/cpython/blob/v3.11.0/Lib/subprocess.py#L1255-L1276), [POSIX execution, lines 1738–1900](https://github.com/python/cpython/blob/v3.11.0/Lib/subprocess.py#L1738-L1900): argv execution, cwd/env, file redirects, new session, timeout. `os.killpg(..., SIGKILL)` terminates the POSIX group after a timeout.
- [PyYAML 6.0.3 load, lines 74–83](https://github.com/yaml/pyyaml/blob/6.0.3/lib/yaml/__init__.py#L74-L83), [SafeLoader, lines 31–39](https://github.com/yaml/pyyaml/blob/6.0.3/lib/yaml/loader.py#L31-L39): safe YAML constructors; the harness adds duplicate mapping-key rejection.
- [NumPy 1.26.4 linalg.py, eigh](https://github.com/numpy/numpy/blob/v1.26.4/numpy/linalg/linalg.py#L1348-L1490) and [norm](https://github.com/numpy/numpy/blob/v1.26.4/numpy/linalg/linalg.py#L2383-L2633): symmetric eigensystem and Euclidean norms. The matrix-to-quaternion eigenproblem is an INFERRED implementation, independently checked using nonidentity inverse-pose fixtures, not a new evaluator.

INFERRED: status precedence, thresholds, repeat/receipt schema, hash-change checks,
coverage proxy and phrase counts are harness policies. UNKNOWN: ORB build/link
behavior, real logs, library dependency hashes, live clocks/calibration and hardware.
