# Hardware acceptance checks

Complete these checks before enabling motion:

- Verify that your base driver accepts `geometry_msgs/msg/Twist` on
  `hardware_command_topic`. Provide a bridge if it requires
  `TwistStamped` or another message type.
- Confirm that only the watchdog publishes to the enabled base command channel.
- Test that command loss stops the platform through the driver timeout.
- Test the physical E-stop independently of the navigation software.
- Check the measured footprint, base origin, motion limits and reverse clearance.
- Verify that the IMU reports raw acceleration including gravity and angular
  velocity in documented right-handed axes, SI units and a common sensor clock.
- Match the LiDAR fields and timestamp units to the selected FAST-LIO handler.
- Check that the LiDAR-to-IMU and body-to-base calibration agrees with the
  URDF sensor mounting transforms.
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
