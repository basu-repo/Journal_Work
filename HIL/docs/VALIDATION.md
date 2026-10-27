# HIL verification and acceptance

Delivery checks on Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12 / C++17:

- Bundled FAST-LIO2 source build passed.
- All 15 automated tests passed.
- Example mission and motion primitive generation passed.
- Localization configuration and command generation passed.
- Isolated disarmed startup and clean shutdown passed for FAST-LIO2,
  pose conversion, segmentation, scan conversion, SLAM Toolbox and navigation.

The ROS startup test used domain 84 with no robot driver or live sensor stream.
Navigation correctly held zero command while clock, scan, odometry and map
were absent. This checks startup and input gating; it does not demonstrate
localization accuracy or a completed mission.

Tests verify shortest-path costs against an independent search, obstacle
changes, directional clearance, primitive motion limits, rejected direct
primitives, calibrated pose/covariance conversion, command clamping and
freshness/readiness gating. Keep tests in this handoff so the receiving team
can check installation and subsequent integration changes.

Historical full-mission reports are kept in the main project, outside HIL.
Physical calibration and powered HIL acceptance remain the receiving team's
responsibility. See HARDWARE_CHECKLIST.md and the root README.

For each trial save the source commit, configuration, sensor fields/rates,
clock mode, physical waypoint and final goal errors, independent trajectory
error where available, collisions, stationary duration, recovery count and TF
fallback ages. Independently verify physical arrival with zero obstacle
contacts before counting a mission as successful.

## Package contents

Only application source, required build/runtime dependencies, sensor/mission
configuration templates, build/launch/check scripts, focused tests, licenses,
the checksum manifest and operating instructions belong in this folder.

Compiler dependency checks identified three unused upstream headers:
Exp_mat.h, matplotlibcpp.h and wrapped_cv_mat.hpp. These are omitted along
with the duplicate vendor sensor configuration, historical research report,
overlapping documentation and the two-line ikd-Tree README.
The source's imported shared navigation modules remain required dependencies,
including modules whose optional GNSS behavior is disabled by the runner.
