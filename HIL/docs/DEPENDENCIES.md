# Runtime Dependencies

## ROS 2 packages

- ROS 2 Jazzy Python client and standard messages
- `tf2_ros`
- ROS message packages for Geometry, Navigation, Sensor and Standard messages

The team's existing system supplies the scan, occupancy map, odometry and TF.
Its sensor drivers, mapping and localization implementation remain outside this
delivery.

## Python packages

See `requirements.txt`: NumPy, SciPy and PyYAML. ROS message packages are
provided by the sourced ROS installation rather than pip.

## Platform-provided components

- Robot base driver and motor controller
- Sensor and localization stack that publishes the configured interface
- calibrated static TF publishers
- physical emergency-stop system
