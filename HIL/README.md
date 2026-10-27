# FAST-LIO2 + SLAM Toolbox + DijkstraLattice HIL package

This folder is the portable ROS 2 Jazzy delivery for testing the Husky navigation
experiment on another team's robot or Gazebo interface. The navigation model is
**Dijkstra + State Lattice (DijkstraLattice)**. FAST-LIO2 source, snapshot support,
calibrated body-to-base conversion, online mapping, readiness monitoring, planner,
motion primitive compiler and a command watchdog are included.

```text
3D LiDAR + raw IMU -> FAST-LIO2 -> calibrated base odometry/TF
                               -> SLAM Toolbox -> map and map-to-odom TF
3D LiDAR -> ground segmentation -> navigation scan
map + scan + TF -> DijkstraLattice -> watchdog -> robot/Gazebo command interface
```

GNSS is disabled in the generated mission configuration. Coordinates must be in
the live SLAM `map` frame. Ground truth is only an evaluation reference.

## What the receiving team supplies

Their robot/simulator, ROS sensor drivers, base command interface, robot URDF and
static mounting transforms. Supply calibrated LiDAR-to-IMU and IMU-to-base
extrinsics, point time format/units and a common clock. Terrain assets and
vendor-specific hardware drivers are not bundled. Existing Baylands world and
robot assets can connect through the same topics; see [Baylands HIL](docs/BAYLANDS_HIL.md).

## Install and build

Run all commands from this HIL directory in a ROS 2 Jazzy environment:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-yaml python3-dev \
  ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-tf2-ros \
  ros-jazzy-rtabmap-util ros-jazzy-pointcloud-to-laserscan ros-jazzy-slam-toolbox
rosdep install --from-paths vendor/FAST_LIO --ignore-src -r -y --rosdistro jazzy
bash scripts/build_fastlio.sh
source fastlio_ws/install/setup.bash
export PYTHONPATH="$PWD/src:$PWD:${PYTHONPATH:-}"
```

See [dependencies](docs/DEPENDENCIES.md) for prerequisites. Build/install/log
directories are generated locally and excluded from Git.

## Configure once for the receiving platform

```bash
cp config/robot.example.yaml config/robot.site.yaml
cp config/mission.example.yaml config/mission.site.yaml
# Select the template matching the ACTUAL cloud fields:
cp config/fastlio.ouster.example.yaml config/fastlio.site.yaml
# Or use fastlio.velodyne.example.yaml for matching Velodyne messages.
```

Edit the three site files. Hardware templates deliberately contain placeholder
extrinsics. Enter measured values and then set `calibration_confirmed: true`.
Verify field names, types and point time units against the driver.
Gazebo snapshot type 5 is for the validated instantaneous cloud model only;
use [Baylands HIL](docs/BAYLANDS_HIL.md) for its clock/configuration.

The included launcher publishes odom-to-base TF; SLAM Toolbox publishes
map-to-odom. Disable competing driver/EKF transforms for these same child frames.
Robot mounting transforms must remain available from the URDF or driver.

## Run localization, then navigation

Start robot drivers or the Gazebo bridge first. This package does not launch a
physical base driver or create a Gazebo robot.

Terminal 1:
```bash
source /opt/ros/jazzy/setup.bash
source fastlio_ws/install/setup.bash
python3 scripts/preflight.py --robot config/robot.site.yaml --sensors-only
python3 scripts/start_localization.py --robot config/robot.site.yaml
```

Terminal 2 (same ROS domain, clock and environment):
```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/preflight.py --robot config/robot.site.yaml
# Disarmed: computes paths, watchdog outputs only zero velocity.
python3 run_hil.py --robot config/robot.site.yaml --mission config/mission.site.yaml
# After bench checks, controlled test with physical E-stop:
python3 run_hil.py --robot config/robot.site.yaml --mission config/mission.site.yaml --arm
```

If the team already provides the required localization/map/scan/TF interface,
they can omit `start_localization.py`; `run_hil.py` still monitors readiness.
Never run both supplied and existing owners for the same TF edge.
The planner topic is `/dijkstra_lattice/path`.

## Coordinates and data recording

Replace `targets` in the mission YAML with surveyed XY coordinates expressed in
the current `map` frame; names identify each waypoint and the final goal.
Gazebo world XY and SLAM map XY are not interchangeable. Establish the frame
relationship independently before importing world coordinates; do not use live
ground truth to correct a GNSS-denied estimate.

Record ROS data alongside each run, using the selected topic names:

```bash
ros2 bag record -o runs/site_trial_bag /lidar/points /imu/data \
  /fastlio/odometry /fastlio/odometry_base /map /scan /tf /tf_static \
  /localization/ready /hil/planner_cmd_vel /cmd_vel /dijkstra_lattice/path
```

## Verification and scope of evidence

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src:$PWD:${PYTHONPATH:-}"
python3 -m unittest discover -s tests -t . -v
sha256sum -c SHA256SUMS
```

The original simulation evidence is in [the execution report](report/FAST_LIO2_NAVIGATION_EXECUTION_RESULTS.md).
The delivery's build/tests and remaining hardware acceptance work are documented
in [validation](docs/VALIDATION.md). Physical robot performance has not been
validated here. Site speed limits are conservative examples, not reproduced
Baylands benchmark parameters.

## Files

- `vendor/FAST_LIO/`: complete buildable modified FAST-LIO2 source and upstream license.
- `localization/`: calibrated odometry/TF adapter and readiness monitoring.
- `scripts/start_localization.py`: FAST-LIO2, segmentation, scan and online SLAM launcher.
- `src/dijkstra_lattice/`: Dijkstra search, state lattice, tracking and recovery.
- `run_hil.py`: run configuration, readiness monitor, planner and watchdog orchestration.
- `config/`: robot/mission examples and distinct hardware/simulation sensor templates.
- `tests/`: planner, motion feasibility, clearance, pose conversion and command gate tests.
- `docs/`: setup, Baylands integration, hardware checks and validation.
- `THIRD_PARTY_NOTICES.md`: bundled source provenance and modifications.

The exact-time scan TF lookup falls back only to a latest TF with nonnegative
scan-to-TF lag at most 0.15 s. It never replays a cached obstacle cloud.
Continuous-pose primitive rejection forces a detour search instead of retrying
the same direct progress branch.
