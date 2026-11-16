---
title: "Husky HIL Main Instructions"
subtitle: "FAST-LIO2 + SLAM Toolbox + Dijkstra State Lattice"
author: "Hardware-in-the-Loop Team Guide"
date: "16 November 2026"
geometry: margin=20mm
papersize: a4
fontsize: 10pt
colorlinks: true
---

# Scope and safety boundary

This guide separates three different modes. Do not mix their configuration files or command topics.

1. **Verified simulation:** all software runs on the simulation computer through `HIL/verified_stack`.
2. **Gazebo-sensor HIL:** Gazebo supplies LiDAR, IMU and clock; localization and navigation run on the physical Husky computer; the physical Husky remains stationary; commands return only to the simulated Husky.
3. **Physical test:** physical sensors and actuators are used; Gazebo is absent from the control loop.

For HIL, keep the physical E-stop engaged or disable the physical motor driver. Never connect `/sim/husky/cmd_vel` to the Clearpath physical drive interface.

The current physical computer may be installed on a Husky A200 while the packaged Gazebo asset is named `a300`. In this HIL stage, `verified_stack/robot/a300` describes only the simulated vehicle receiving commands. It is not the hardware configuration of the stationary A200. Physical A200 topics, calibration, footprint and drive limits belong in a separate site profile when physical sensors or motion are introduced.

# One-time FAST-LIO2 installation

## Verified simulation checkout

From `HIL/verified_stack`:

```bash
source /opt/ros/jazzy/setup.bash
bash hardware/build_fastlio.sh
source FASTLIO2Experiment/install/setup.bash
test -x FASTLIO2Experiment/install/fast_lio/lib/fast_lio/fastlio_mapping
ros2 pkg prefix fast_lio
```

The reported prefix must be inside `HIL/verified_stack/FASTLIO2Experiment/install`.

## Main HIL/physical package

From `HIL`:

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/build_fastlio.sh
source fastlio_ws/install/setup.bash
ros2 pkg prefix fast_lio
```

The reported prefix must be inside `HIL/fastlio_ws/install`. These are intentionally separate workspaces. Source only the workspace required by the mode being run.

# Reference verified simulation

From `HIL/verified_stack`:

```bash
python3 localization_experiment/run_diagnostic.py \
  --map bayland \
  --robot a300 \
  --model DijkstraLattice \
  --localization slam \
  --slam-backend slam_toolbox \
  --pose-backend fast_lio2 \
  --explore \
  --boundary-follow \
  --straight-only \
  --adaptive-horizon \
  --experimental-horizon-m 5 \
  --no-slam-scan-matching \
  --no-slam-loop-closing \
  --no-record-bag
```

This command is the reference simulation, not the split-computer HIL launcher.

# Gazebo-sensor HIL

## Files supplied for HIL

- `HIL/config/robot.hil.yaml`: simulated sensors, simulated clock and isolated Gazebo command topic.
- `HIL/config/mission.hil.yaml`: short initial targets in the initialized local map frame.
- `HIL/scripts/prepare_hil_bridge.py`: derives an HIL bridge from the verified A300 bridge without editing the reference bridge.

The source bridge is `HIL/verified_stack/robot/a300/bridge.yaml`. The normal verified simulator creates a run-scoped bridge and changes its own command interface. Therefore, do not edit that source file in place for HIL.

## Gazebo computer

Start the packaged Bayland/A300 simulation-only HIL interface. This launches Gazebo, the isolated sensor/command bridge, and `robot_state_publisher`; it does not launch localization or navigation:

```bash
cd HIL/verified_stack
source /opt/ros/jazzy/setup.bash
python3 run_simulation.py \
  --map bayland \
  --robot a300 \
  --model manual \
  --hil-interface \
  --ros-domain <SHARED_DOMAIN_ID>
```

The launcher derives a run-scoped bridge from `HIL/verified_stack/robot/a300/bridge.yaml`. It maps ROS `/sim/husky/cmd_vel` to Gazebo `/cmd_vel`, excludes GNSS, excludes the wheel TF that would compete with FAST-LIO, and publishes the static robot/sensor TF tree.

For inspection only, the equivalent bridge can be generated without starting Gazebo:

```bash
cd HIL
source /opt/ros/jazzy/setup.bash
python3 scripts/prepare_hil_bridge.py --output runs/hil_bridge.yaml
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:="$PWD/runs/hil_bridge.yaml"
```

Gazebo and both ROS computers must use the same `ROS_DOMAIN_ID`. Confirm DDS discovery and bidirectional topic visibility before localization.

## Physical Husky computer, stationary

From `HIL`:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=<SHARED_DOMAIN_ID>
source fastlio_ws/install/setup.bash
python3 scripts/preflight.py --robot config/robot.hil.yaml --sensors-only
python3 scripts/start_localization.py --robot config/robot.hil.yaml
```

In another terminal:

```bash
cd HIL
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=<SHARED_DOMAIN_ID>
source fastlio_ws/install/setup.bash
python3 scripts/preflight.py --robot config/robot.hil.yaml
python3 run_hil.py \
  --robot config/robot.hil.yaml \
  --mission config/mission.hil.yaml
```

This first run is disarmed and must publish zero to `/sim/husky/cmd_vel`. After verifying the physical drive topic receives no messages, set `verified_profile_armed: true` in `robot.hil.yaml` and run:

```bash
python3 run_hil.py \
  --robot config/robot.hil.yaml \
  --mission config/mission.hil.yaml \
  --arm
```

Arming permits commands only to `/sim/husky/cmd_vel` as configured in `robot.hil.yaml`.

# Why FAST-LIO2 still needs LiDAR-to-IMU extrinsics

The transforms are related, but they serve different software interfaces:

- `mapping.extrinsic_T` and `mapping.extrinsic_R` are the **LiDAR-to-IMU calibration consumed internally by FAST-LIO2**. FAST-LIO2 uses them to express LiDAR measurements in the inertial/body frame during estimation. This implementation does not read that calibration from ROS TF.
- `base_link -> lidar_frame` and `base_link -> imu_frame` are **ROS TF mounting transforms** used by downstream ROS nodes, visualization and sensor-message transformation.
- `body_to_base_translation` and `body_to_base_quaternion` tell the supplied adapter how to convert FAST-LIO2's IMU/body pose into the vehicle `base_link` pose.

If both sensors are children of `base_link`, their relative LiDAR-to-IMU transform can be calculated mathematically from those two TF edges. That does not make the FAST-LIO2 parameters optional: the same measured calibration must be entered in the FAST-LIO2 YAML because the estimator does not query TF for it. The YAML values and published static TF must describe the same physical geometry. Do not publish a separate dynamic LiDAR-to-IMU TF edge; publish the static base-to-sensor edges once.

# Physical test

Create site-specific files; do not reuse the simulation HIL profile:

```bash
cd HIL
cp config/robot.example.yaml config/robot.site.yaml
cp config/fastlio.velodyne.example.yaml config/fastlio.site.yaml
cp config/mission.example.yaml config/mission.site.yaml
```

Use the Ouster template instead only when its handler matches the actual PointCloud2 fields. Set `use_sim_time: false`, enter physical topics, measured LiDAR-to-IMU and body-to-base calibration, measured footprint and conservative motion limits. Set `calibration_confirmed: true` only after checking clocks, frame directions and extrinsics.

Run sensor-only preflight, localization, full preflight and disarmed navigation in that order. Use `--arm` only after the disarmed checks and independent E-stop test pass.

# TF ownership

Required dynamic tree:

```text
map -> odom_lidar -> base_link
```

- SLAM Toolbox owns `map -> odom_lidar`.
- The supplied FAST-LIO adapter owns `odom_lidar -> base_link`.
- `robot_state_publisher` owns the static `base_link -> lidar_frame` and `base_link -> imu_frame` transforms.
- FAST-LIO2's native TF is isolated under `/fastlio/internal_tf`.

Verify one publisher per edge:

```bash
ros2 topic info /tf --verbose
ros2 topic info /tf_static --verbose
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link <lidar-frame>
ros2 run tf2_ros tf2_echo base_link <imu-frame>
```

# Acceptance evidence

A trial is not successful merely because the planner reports arrival. Retain the repository commit, YAML files, generated HIL bridge, logs and bag metadata. Confirm that GNSS and truth are absent from localization/planning inputs; clocks and transforms are fresh; stale sensors or localization loss stop commands; the physical drive remains isolated during HIL; and final arrival is independently measured.
