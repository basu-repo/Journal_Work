# Hardware Test Acceptance Checklist

## Gate 1: powered-off inspection

- [ ] Physical E-stop cuts motor power and has been demonstrated.
- [ ] A safety operator can reach the E-stop throughout the test.
- [ ] LiDAR, IMU and wheel odometry timestamps use the same clock.
- [ ] Measured LiDAR-to-base transform matches the TF tree.
- [ ] Footprint and speed limits match the selected physical robot.

## Gate 2: motors disabled or wheels lifted

- [ ] `scripts/preflight.py` passes all topic, type and TF checks.
- [ ] RViz map, LiDAR and robot pose remain aligned during hand motion.
- [ ] Positive linear commands correspond to physical forward motion.
- [ ] Positive angular commands correspond to counter-clockwise yaw.
- [ ] Stopping the planner, LiDAR or odometry produces zero command within 1 s.

## Gate 3: restrained low-speed test

- [ ] Reduce `max_linear_speed` and `max_angular_speed` before arming.
- [ ] Use one target 2--3 m ahead in a clear controlled area.
- [ ] Record `/lidar/points`, `/scan`, `/map`, odometry, TF, planner path and commands.
- [ ] Confirm planned path clearance using measured robot footprint.
- [ ] Confirm E-stop under commanded motion.

## Gate 4: waypoint experiment

- [ ] Survey waypoint coordinates in the current map frame.
- [ ] Define success tolerance before the run.
- [ ] Use independent position measurement for evaluation.
- [ ] Preserve complete run logs and bag; report failures as failures.
