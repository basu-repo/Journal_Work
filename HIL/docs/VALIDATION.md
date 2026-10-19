# Delivery Validation

Validation date: 2026-10-19

## Completed checks

- Python bytecode compilation completed for source, scripts and launcher.
- ROS 2 Jazzy environment was sourced before tests.
- 9 focused Dijkstra planner, directional-clearance and primitive tests passed.
- Configuration smoke test merged the Dijkstra lattice settings, generic robot
  profile and three-target example mission.
- Motion primitive generation produced 192 primitives.
- Public launcher help exposes one model only: DijkstraLattice.
- The public velocity path is planner -> fail-closed watchdog -> `/cmd_vel`.
- Generic robot-profile smoke check confirmed configurable scan, map, odometry
  and command topics without terrain or robot model files.

## Not yet validated

- Physical Husky topic availability and message rates
- Sensor extrinsic calibration and timestamp synchronization
- Live `map -> odom_lidar -> base_link` consistency
- Wheel slip, stopping distance and skid-steer tracking error
- Hardware waypoint arrival accuracy
- Full mission success on the receiving team's test site

The receiving team must complete `HARDWARE_CHECKLIST.md`. Static and unit checks
do not constitute a successful physical-robot experiment.
