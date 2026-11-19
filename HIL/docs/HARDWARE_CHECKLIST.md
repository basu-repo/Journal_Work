# Hardware acceptance checks

Complete these checks before enabling motion:

- Verify the message type accepted on `hardware_command_topic`. The default is
  `geometry_msgs/msg/Twist`; use `--twist-stamped-output` when the Clearpath
  interface requires `geometry_msgs/msg/TwistStamped`. Other message types
  require a separate, explicitly validated bridge.
- Confirm that only the watchdog publishes to the enabled base command channel.
- Test that command loss stops the platform through the driver timeout.
- Test the physical E-stop independently of the navigation software.
- Check the measured footprint, base origin, motion limits and reverse clearance.
- Verify that the IMU reports raw acceleration including gravity and angular
  velocity in documented right-handed axes, SI units and a common sensor clock.
- Match the LiDAR fields and timestamp units to the selected FAST-LIO handler.
- Check that the LiDAR-to-IMU and body-to-base calibration agrees with the
  URDF sensor mounting transforms.
- Set `tf_source_topic` to the robot's dynamic TF topic (for example
  `/a201_0000/tf`), `tf_topic` to an isolated HIL topic such as `/hil/tf`, and
  `tf_static_topic` to the robot's static TF topic. Confirm that the frame IDs
  match `base_frame` and the sensor message frame IDs.
- Assign one publisher to map-to-odom and one to odom-to-base.
- Confirm that GNSS and Gazebo ground truth are excluded from localization
  and planning inputs.
- Inspect the classification of ground returns, body returns and low obstacles.
- Run live preflight and verify that readiness drops when odometry, map or
  TF updates stop.
- Confirm that disarmed operation outputs zero hardware velocity.
- With the wheels lifted or motors disabled, verify that armed command
  forwarding stops when command, scan, odometry or readiness becomes stale.
- Independently measure physical waypoint and goal arrival during motion tests.

Keep a human operator and a clear test area during initial powered trials.
Save your site configuration and ROS data for every experiment.
