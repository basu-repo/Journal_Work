# Verify the installation and record acceptance results

Run the automated tests using the command in the root README before starting
your hardware trials. Repeat them after changing the integration or software.

The tests cover shortest-path costs against an independent search, obstacle
changes, directional clearance, primitive motion limits, rejected direct
primitives, calibrated pose/covariance conversion, command clamping and
freshness/readiness gating.

## Checks completed before delivery

The following checks passed on Ubuntu 24.04 with ROS 2 Jazzy, Python 3.12
and C++17:

- FAST-LIO2 source build.
- All 15 automated tests.
- Example mission and motion primitive generation.
- Localization configuration and command generation.
- Isolated disarmed startup and clean shutdown of FAST-LIO2, pose conversion,
  segmentation, scan conversion, SLAM Toolbox and navigation.

The ROS startup check used domain 84 without a robot driver or live sensor
stream. Navigation held zero command while clock, scan, odometry and map inputs
were absent. These results verify startup and input gating; they do not
establish localization accuracy or physical mission completion.

## Validate your platform

Complete the calibration, sensor checks and powered acceptance tests described
in [the hardware checklist](HARDWARE_CHECKLIST.md) and [the root README](../README.md).
Start with a short controlled route before attempting the full mission.

Save the following information for each trial:

- Source commit and site configuration.
- Sensor message fields, publication rates and clock mode.
- Physical waypoint and final goal errors.
- Independent trajectory error, when a reference is available.
- Collisions, stationary duration and recovery count.
- TF fallback ages.

Independently verify physical arrival and check for obstacle contacts before
counting a mission as successful. A reported waypoint arrival or plotted path
alone does not establish accurate localization.
