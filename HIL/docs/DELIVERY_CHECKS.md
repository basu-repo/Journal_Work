# Delivery checks — 2026-10-27

Environment: Ubuntu 24.04, ROS 2 Jazzy, system Python 3.12, C++17.

| Check | Result |
| --- | --- |
| Bundled FAST-LIO2 source build | Passed, colcon build, one package |
| Automated tests | 15 passed |
| Python/YAML parsing and shell syntax | Passed |
| Example mission configuration and primitive compilation | Passed |
| Localization config and command generation | Passed with a temporary calibrated simulation profile |
| Isolated ROS startup | FAST-LIO2, pose adapter, segmentation, scan conversion, SLAM Toolbox and disarmed navigation remained running |
| Isolated ROS shutdown | Clean after fixing optional debug-file closure and Python shutdown ordering |

The runtime test used ROS domain 84, no robot driver, no live sensor stream and
no motion authority. The navigation stack correctly held zero command while
clock/scan/odometry/map inputs were absent. This confirms process startup and
input gating; it does not establish localization accuracy or mission completion.

Tests cover independent shortest-path cost comparison, changing obstacles,
directional clearance, motion limits, direct-primitive rejection handling,
body-frame pose/covariance conversion, watchdog disarm/clamping/stale-command/
readiness loss, readiness freshness/TF, and rejection of mismatched odometry
frames and duplicate stamps.

Two packaging faults discovered by startup/shutdown testing were corrected:
FAST-LIO2 no longer closes a null optional debug-file handle, and the watchdog
and health process stop publishing before their ROS contexts are destroyed.

Historical full-mission simulation results remain in the included execution
report. No new physical HIL mission or new complete Baylands mission was run
as part of packaging. Receiving-platform calibration and powered validation
are required before claiming hardware performance.
