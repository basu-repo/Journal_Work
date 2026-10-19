# Receiving Team Quick Start

The terrain and robot remain in the team's existing ROS 2 system. This package
does not load a Gazebo world, robot model or saved terrain. It starts only the
Dijkstra lattice planner and its command watchdog.

## 1. Publish the interface

Before starting this package, verify that the robot system publishes:

- a live occupancy grid for the current terrain;
- a ground-filtered planar scan;
- robot odometry;
- `/tf` and `/tf_static` with `map -> odom -> base_link` connected;
- a base command input accepting `geometry_msgs/msg/Twist`.

The actual topic names are configurable and do not need to match the examples.

## 2. Configure the robot and mission

```bash
cp config/robot.example.yaml config/robot.site.yaml
cp config/mission.example.yaml config/mission.site.yaml
```

Edit `robot.site.yaml` with their topic names, measured footprint and safe
limits. Edit `mission.site.yaml` with waypoint coordinates in the current
`map` frame.

## 3. Verify without motion

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src:$PWD:$PYTHONPATH"
python3 scripts/preflight.py --robot config/robot.site.yaml
python3 run_hil.py --robot config/robot.site.yaml \
  --mission config/mission.site.yaml
```

Without `--arm`, the watchdog publishes zero velocity. This lets the team
inspect the Dijkstra path and pose alignment before enabling motion.

## 4. Controlled motion test

After completing `HARDWARE_CHECKLIST.md`:

```bash
python3 run_hil.py --robot config/robot.site.yaml \
  --mission config/mission.site.yaml --arm
```

The first powered mission should contain one target 2--3 m directly ahead in
a clear controlled area.
