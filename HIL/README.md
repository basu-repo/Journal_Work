# FAST-LIO2 + SLAM Toolbox + DijkstraLattice HIL package

Use this package to run the Husky navigation experiment on your robot or through
a Gazebo interface with ROS 2 Jazzy. The navigation model is
**Dijkstra + State Lattice (DijkstraLattice)**. The package includes FAST-LIO2,
online mapping, pose conversion, readiness monitoring, motion primitive
generation, navigation and a command watchdog.

```text
3D LiDAR + raw IMU -> FAST-LIO2 -> calibrated base odometry/TF
                               -> SLAM Toolbox -> map and map-to-odom TF
3D LiDAR -> ground segmentation -> navigation scan
map + scan + TF -> DijkstraLattice -> watchdog -> robot/Gazebo command interface
```

Run the experiment with GNSS disabled. Enter waypoint and goal coordinates in
the live SLAM `map` frame. Use ground truth only to evaluate the results.

## Prepare your platform

Provide your robot or simulator, ROS sensor drivers, base command interface,
robot URDF and static mounting transforms. Measure the LiDAR-to-IMU and
IMU-to-base extrinsics, verify point timestamp formats and units, and use a
common clock.

Use your existing terrain assets and hardware drivers; these are not included
in this package. For Baylands, follow [Baylands HIL](docs/BAYLANDS_HIL.md).

## Install and build

Run the following commands from the HIL directory using the system Python and
ROS 2 Jazzy environment:

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep \
  python3-numpy python3-scipy python3-yaml \
  ros-jazzy-pcl-ros ros-jazzy-pcl-conversions ros-jazzy-tf2-ros \
  ros-jazzy-rtabmap-util ros-jazzy-pointcloud-to-laserscan ros-jazzy-slam-toolbox
```

Initialize rosdep once with `sudo rosdep init` if it has not already been
initialized. Then update rosdep, install the source dependencies and build:

```bash
rosdep update
rosdep install --from-paths vendor/FAST_LIO --ignore-src -r -y --rosdistro jazzy
bash scripts/build_fastlio.sh
source fastlio_ws/install/setup.bash
export PYTHONPATH="$PWD/src:$PWD:${PYTHONPATH:-}"
```

Install the ROS and system dependencies before building. The required IKFoM
and ikd-Tree source is included under `vendor/FAST_LIO/`. The build generates
local build, install and log directories, which are excluded from Git.

## Configure your robot and mission

```bash
cp config/robot.example.yaml config/robot.site.yaml
cp config/mission.example.yaml config/mission.site.yaml
# Select the template matching your LiDAR message fields.
cp config/fastlio.ouster.example.yaml config/fastlio.site.yaml
# For matching Velodyne messages, use fastlio.velodyne.example.yaml instead.
```

Edit the three site files. Replace the placeholder extrinsics with your measured
values, then set `calibration_confirmed: true`. Check field names, field types
and point timestamp units against your sensor driver.

For Gazebo snapshot clouds, use `config/fastlio.simulation.yaml` and follow the
clock and calibration settings in [Baylands HIL](docs/BAYLANDS_HIL.md).
Select snapshot type 5 only for the validated instantaneous cloud model.

Assign each TF transform to one publisher. When using the included localization
stack, let the FAST-LIO adapter publish odom-to-base and SLAM Toolbox publish
map-to-odom. Disable driver or EKF publishers for those same transforms. Keep
the sensor mounting transforms available through your URDF or driver.

## Start localization and navigation

Start your robot drivers or Gazebo bridge first. Launch the robot or simulator
through your existing platform setup.

In terminal 1, check the sensor inputs and start localization:

```bash
source /opt/ros/jazzy/setup.bash
source fastlio_ws/install/setup.bash
python3 scripts/preflight.py --robot config/robot.site.yaml --sensors-only
python3 scripts/start_localization.py --robot config/robot.site.yaml
```

In terminal 2, use the same ROS domain, clock and environment. Check the complete
interface, then run navigation disarmed:

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/preflight.py --robot config/robot.site.yaml
python3 run_hil.py --robot config/robot.site.yaml --mission config/mission.site.yaml
```

Verify the planned route and confirm that the watchdog outputs zero velocity.
Complete the [hardware checks](docs/HARDWARE_CHECKLIST.md), then start a
controlled motion test with the physical E-stop available:

```bash
python3 run_hil.py --robot config/robot.site.yaml --mission config/mission.site.yaml --arm
```

To use your existing localization stack, first confirm that it publishes the
configured localization, map, scan and TF inputs. Then run `run_hil.py` without
starting `scripts/start_localization.py`. The runner monitors readiness in
either setup. Use only one publisher for each TF transform.

View the planned route on `/dijkstra_lattice/path`.

## Set coordinates and record data

Replace `targets` in the mission YAML with surveyed XY coordinates in the
current `map` frame. Give each waypoint and the final goal a name.

Establish the relationship between Gazebo world coordinates and the SLAM map
frame before importing world coordinates. Do not assume that world XY equals
map XY. Keep live ground truth out of the GNSS-denied localization pipeline.

Record ROS data during each run. Replace the topic names below with those in
your robot profile:

```bash
ros2 bag record -o runs/site_trial_bag /lidar/points /imu/data \
  /fastlio/odometry /fastlio/odometry_base /map /scan /tf /tf_static \
  /localization/ready /hil/planner_cmd_vel /cmd_vel /dijkstra_lattice/path
```

## Verify the installation

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src:$PWD:${PYTHONPATH:-}"
python3 -m unittest discover -s tests -t . -v
```

Follow [verification and acceptance](docs/VALIDATION.md) to assess the
installation and record your test results. Complete physical calibration and
powered testing on your platform before reporting hardware performance.
Start with conservative speed limits; the example limits are not the
Baylands benchmark settings.

## Package layout

- `vendor/FAST_LIO/`: modified FAST-LIO2 source and upstream license.
- `localization/`: calibrated odometry/TF adapter and readiness monitor.
- `scripts/start_localization.py`: FAST-LIO2, segmentation, scan and online SLAM launcher.
- `src/dijkstra_lattice/`: Dijkstra search, state lattice, tracking and recovery.
- `run_hil.py`: run configuration, readiness, planner and watchdog orchestration.
- `config/`: robot, mission and hardware/simulation sensor templates.
- `tests/`: planner, motion, clearance, pose conversion and command gate tests.
- `docs/`: Baylands integration, hardware checks and validation instructions.
- `THIRD_PARTY_NOTICES.md`: source provenance and modifications.

The navigation code requests scan transforms at the exact timestamp. When that
transform is unavailable, it accepts the latest TF only if the scan-to-TF lag
is nonnegative and at most 0.15 s. It does not reuse a cached obstacle cloud.
If continuous-pose validation rejects a direct primitive, navigation starts a
detour search instead of repeatedly selecting the same direct primitive.
