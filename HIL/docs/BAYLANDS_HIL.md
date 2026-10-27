# Baylands HIL setup and coordinate frames

Connect this package to your existing Baylands world and robot interface.
Provide the terrain meshes, Gazebo world launcher and robot driver through
your platform setup. Use the same topic interface for simulation and hardware.

## Run with Gazebo sensors and actuators

Start your Baylands world and A300 sensor/command bridge. Publish the 3D cloud,
raw IMU, static sensor mounting TF and `/clock`. Disable the simulator's
odom-to-base TF bridge when using the included FAST-LIO adapter.

Copy `config/robot.example.yaml` to `config/robot.site.yaml` and set
`use_sim_time: true`. Set `localization.fastlio_config: fastlio.simulation.yaml`.

Check the sensor model before using the simulation template. It was configured
for a 64-row GPU LiDAR with 1,024 horizontal samples at 10 Hz and an IMU at
50 Hz. The template uses a LiDAR-to-IMU translation of
`[0.374, 0, 0.435]` metres and identity rotation.

Verify the IMU/body and base frame locations in your model. Enter the measured
`body_to_base_translation` and `body_to_base_quaternion`, then set
`calibration_confirmed: true`.

Set `hardware_command_topic` to your simulator command bridge input.
Disconnect the physical base command channel during a simulator-only test.
Follow the localization and navigation commands in the root README.
Keep the vehicle stationary while FAST-LIO initializes.

## Connect physical hardware to the simulation

Choose the sensor and actuator sources for the experiment. When using physical
sensors, ensure that the simulated vehicle and environment represent the same
geometry and motion as the physical setup. Use a mirrored pose only for
visualization unless it is explicitly part of your HIL interface.

Use system time for physical sensor processing unless your bridge synchronizes
all sensor timestamps to simulation time. Do not mix wall-time sensor stamps
with `/clock`, or publish real and simulated sensor data on the same topics.

Before running localization on your robot or HIL computer, update the sensor
model, calibration and motion limits. For a physical rotating LiDAR, verify
genuine point acquisition times and their units. Use type 5 only for the
validated Gazebo snapshot clouds. Select the Velodyne or Ouster template only
when its field layout matches your driver.

## Test waypoint coordinates

Enter named XY targets in the current SLAM `map` frame. Establish the
map-to-site alignment at initialization before converting site coordinates.

Do not assume that the robot's Gazebo spawn coordinates equal its SLAM map
coordinates. Survey physical targets separately rather than copying Baylands
world coordinates into a hardware mission.

Record ground truth on a separate evaluation topic when available. Keep it
outside the localization and navigation input graph.
