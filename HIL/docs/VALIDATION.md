# Delivery validation and experimental evidence

This delivery packages the current Dijkstra state-lattice implementation and
FAST-LIO2 snapshot source. It also adds a configurable body-to-base adapter,
localization health publisher, launcher and hardware command watchdog.

The validation record must distinguish these checks:
- Source/unit tests: shortest paths against an independent cost search, obstacle
  changes, primitive motion limits, directional clearance, rejected-direct-branch
  handling, calibrated pose conversion and fail-closed command gating.
- Build: bundled FAST-LIO source compiled under ROS 2 Jazzy/C++17.
- Configuration: offline primitive generation and localization command/config generation.
- ROS bench smoke: initialization of packaged components without a powered robot.
- Full-mission evidence: earlier original-workspace simulation runs documented
  in report/FAST_LIO2_NAVIGATION_EXECUTION_RESULTS.md.
- Receiving robot: calibration, sensor checks and powered HIL acceptance remain
  to be performed by the team. Earlier mission results do not validate its hardware.

The preserved report records Baylands successful trials, cross-environment
results and the Warehouse failure. It must not be read as a claim that every
environment succeeds.

For every receiving-team trial save source commit, configs, ROS distro,
clock mode, LiDAR fields/rate, IMU rate, map resolution, physical waypoint error,
final physical goal error, trajectory RMSE (if independent reference available),
collisions, stationary duration, recovery count and TF fallback ages.

A completed mission requires independently verified physical arrival with
zero obstacle contacts. A plotted route or reported waypoint arrival alone
does not establish correct localization.
