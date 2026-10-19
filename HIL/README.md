# Dijkstra Lattice Hardware-in-the-Loop Navigation Package

This delivery contains the deterministic `DijkstraLattice` planner separated
from Gazebo. It consumes live ROS 2
sensor, map, odometry and TF topics. It contains no terrain coordinates,
Gazebo truth, contact topics, saved simulation maps or robot bridge.

## Status and evidence boundary

The algorithms were developed in simulation. This package has static and unit
checks, but it has not been validated on the receiving team's physical Husky.
The example waypoint coordinates are placeholders. The team must calibrate the
sensor transforms, footprint, covariance, velocity limits and emergency stop
before any powered test.

## Required interface from the receiving team

- Ubuntu with ROS 2 Jazzy
- Their selected robot base driver
- A `sensor_msgs/msg/LaserScan` navigation scan
- A `nav_msgs/msg/OccupancyGrid` representing their current terrain
- A `nav_msgs/msg/Odometry` localization stream
- A continuous `map -> odom -> base_link` TF chain
- A `geometry_msgs/msg/Twist` hardware command input
- Independent physical emergency stop and a human safety operator

Topic names are selected in the robot profile. The team is responsible for
publishing them before this package starts. `scripts/preflight.py` verifies the
configured topic names, message types and `map -> base_link` transform.
Sensor drivers, localization and SLAM remain part of their existing stack.

## Install

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pip install -r requirements.txt
sudo apt install ros-jazzy-tf2-ros \
  ros-jazzy-geometry-msgs ros-jazzy-nav-msgs ros-jazzy-sensor-msgs
```

Their existing robot stack remains responsible for producing the configured
scan, map, odometry and TF inputs.

## Configure

1. Copy `config/robot.example.yaml` to `config/robot.site.yaml` and enter their
   existing ROS topic names, TF frames, footprint and conservative limits.
2. Copy `config/mission.example.yaml` to `config/mission.site.yaml`.
3. Enter measured targets in the live `map` frame.
4. Keep the base lifted or motors disabled for the first checks.

## Test sequence

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$PWD/src:$PYTHONPATH"

# Source-level checks; no robot motion.
python3 -m unittest discover -s tests -p 'test_*.py'

# Live read-only topic and TF checks.
python3 scripts/preflight.py --robot config/robot.site.yaml

# Planner runs, but watchdog publishes only zero velocity.
python3 run_hil.py --robot config/robot.site.yaml \
  --mission config/mission.site.yaml

# Only after bench validation and an operator-controlled E-stop test:
python3 run_hil.py --robot config/robot.site.yaml \
  --mission config/mission.site.yaml --arm
```

Start with a single target 2--3 m ahead at reduced platform limits. Record a
ROS bag during every hardware test. Stop immediately if the map/robot alignment
in RViz is inconsistent with the physical scene.

## Fail-closed command path

The planner command is remapped to `/hil/planner_cmd_vel`. The watchdog alone
publishes `/cmd_vel`. It outputs zero unless the run was explicitly armed,
navigation is enabled, and planner command, scan and odometry are all fresh.
Velocity and yaw rate are clamped to the limits in the selected robot profile.
This software watchdog supplements the physical emergency stop.

## Package layout

- `src/dijkstra_lattice/`: the complete Dijkstra lattice implementation
- `tests/`: focused Dijkstra, directional-clearance and primitive tests
- `config/`: generic robot profile and example mission
- `scripts/preflight.py`: read-only topic/type and TF validation
- `scripts/prepare_run.py`: merged run configuration and primitive generation
- `scripts/hardware_watchdog.py`: bounded fail-closed command forwarding
- `run_hil.py`: run orchestration without Gazebo
- `docs/HARDWARE_CHECKLIST.md`: acceptance gates for the receiving team

`run_hil.py` always launches the single `dijkstra_lattice` package. There are
no separate A*, D*, D* Lite, DRL, Nav2 or simulator model folders in this
delivery.
