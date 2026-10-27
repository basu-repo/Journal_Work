# Physical robot acceptance

Before motion, verify:

- The base driver accepts geometry_msgs/Twist on hardware_command_topic.
  Supply an explicit bridge if the driver requires TwistStamped or another type.
- Only the watchdog publishes to the enabled base command channel.
- Command loss stops the platform through its driver timeout.
- The physical E-stop independently removes motion authority.
- The measured footprint, base origin, limits and reverse clearance are correct.
- The IMU provides raw acceleration including gravity and angular velocity in
  documented right-handed axes, SI units and a common sensor clock.
- LiDAR fields/time units match the selected FAST-LIO handler.
- LiDAR-to-IMU calibration in FAST-LIO and body-to-base calibration in the
  adapter agree with the URDF mounting tree.
- Exactly one owner publishes map-to-odom and one owns odom-to-base.
- No GNSS or Gazebo ground truth feeds localization or planning.
- Ground returns, body returns and low obstacles are classified correctly.
- All topics pass live preflight; readiness drops when odometry/map/TF stops.
- Disarmed operation emits zero hardware velocity.
- Armed bench operation stops when command, scan, odometry or readiness becomes stale.
- Independent observations confirm physical waypoint/goal arrival.

Keep a human operator and clear test area during initial powered trials.
Record site configuration and ROS data with each experiment.
