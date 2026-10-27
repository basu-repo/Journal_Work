# Baylands, HIL and coordinate frames

This delivery attaches to a team's existing Baylands world and robot interface.
It does not include terrain meshes, a Gazebo world loader or a vendor base driver.
The runtime inputs/outputs are the same in simulation and hardware.

## Gazebo sensor/actuator loop

Start the existing Baylands world and A300 sensor/command bridge. Publish the
cloud, raw IMU, robot static mounting TF and /clock. Disable the simulator's
odom-to-base TF bridge while the supplied FAST-LIO adapter owns that edge.

Copy robot.example.yaml into robot.site.yaml and set use_sim_time: true.
Set localization.fastlio_config: fastlio.simulation.yaml. The included snapshot
configuration is for the original 64-row, 1024-horizontal-sample, 10 Hz GPU
LiDAR with a 50 Hz IMU; verify the actual model before reusing its extrinsics.
The template LiDAR-to-IMU translation is [0.374, 0, 0.435] metres with identity
rotation. Confirm where the IMU/body and base frames lie in your model before
setting body_to_base_translation/quaternion and calibration_confirmed.

Set hardware_command_topic to the simulator command bridge input. Keep the
physical base command channel disconnected during a simulator-only test.
Use the root README localization and navigation commands.
Keep the initial vehicle stationary while FAST-LIO initializes.

## Physical hardware connected to a simulation environment

Choose which sensor and actuator sources are authoritative for the experiment.
If physical sensors drive the estimator, the estimated motion and the simulated
vehicle/environment must represent the same geometry and motion. Merely viewing
Baylands beside an unrelated physical scene does not provide this relationship.

Use system time for physical sensor processing unless the HIL bridge explicitly
synchronizes all sensor stamps to simulation time. Mixing /clock with wall-time
sensor stamps produces invalid TF and blocked navigation.
Do not combine real and simulated scans/IMU on the same topics.
A mirrored robot pose used only for display is distinct from estimator input.

Run localization on the robot/HIL computer with the same configuration used for
the simulator trial, after replacing the sensor model, calibration and limits.
Physical rotating LiDAR normally needs genuine point times; type 5 must not
be used to disguise missing timestamps. Hardware templates cover the bundled
Velodyne/Ouster message layouts; validate actual fields and units.

## Later coordinate tests

The mission target list accepts named x/y values in the current SLAM map frame.
First measure/establish the map-to-site frame alignment at initialization.
A robot's spawn world XY does not automatically equal map XY. Do not copy
Baylands world coordinates into a physical mission or use live Gazebo truth
as a localization correction.

Record ground truth on a separate evaluation topic when available. Keep it
outside the navigation and localization input graph.
