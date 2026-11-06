# Team configuration worksheet

Use this worksheet for the first Husky HIL and physical-robot integration.
Configure the files in the main `HIL/config` directory. The
`HIL/verified_stack` directory reproduces the simulation setup and is not the
physical-robot configuration directory.

## 1. Information to collect from the robot or recorded bag

Please fill in this table before editing the YAML files.

| Required information | Team value |
|---|---|
| PointCloud2 topic | |
| PointCloud2 message fields | |
| Per-point time field and unit | |
| LiDAR frame ID | |
| LiDAR channels / scan lines | |
| LiDAR scan rate (Hz) | |
| IMU topic | |
| IMU frame ID | |
| IMU publication rate (Hz) | |
| Base frame ID | |
| Husky velocity command topic | |
| Velocity command message type | |
| LiDAR-to-IMU translation | |
| LiDAR-to-IMU rotation | |
| IMU/body-to-base translation | |
| IMU/body-to-base quaternion | |
| Existing dynamic TF publishers | |
| Existing static sensor TF publishers | |

Useful commands:

```bash
ros2 bag info <bag-directory>
ros2 topic list -t
ros2 topic echo <pointcloud-topic> --once
ros2 topic echo <imu-topic> --once
ros2 run tf2_tools view_frames
```

The `/a201_0000` prefix should be added only to topics that actually use that
namespace. Do not automatically add it to TF frame names. Confirm frame names
from message headers and the TF tree.

## 2. Create the three site files

From the `HIL` directory:

```bash
cp config/robot.example.yaml config/robot.site.yaml
cp config/fastlio.velodyne.example.yaml config/fastlio.site.yaml
cp config/mission.example.yaml config/mission.site.yaml
```

Use the Ouster template instead of the Velodyne template only when the installed
sensor and PointCloud2 fields match the Ouster handler.

## 3. Edit `config/robot.site.yaml`

### Values that must be set or confirmed

```yaml
base_frame: base_link                 # Confirm the real robot frame.
use_sim_time: false                   # true for Gazebo; false for physical sensors.

localization:
  calibration_confirmed: false        # Set true only after checking all extrinsics.
  fastlio_config: fastlio.site.yaml
  pointcloud_topic: <TEAM_LIDAR_TOPIC>
  imu_topic: <TEAM_RAW_IMU_TOPIC>
  body_to_base_translation: [x, y, z]
  body_to_base_quaternion: [x, y, z, w]
  min_height: 0.15                    # Confirm against the mounting and ground.
  max_height: 2.0                     # Confirm required obstacle height.
  range_min: 0.8                      # Confirm usable LiDAR minimum range.
  range_max: 60.0                     # Confirm usable LiDAR maximum range.
  scan_period: 0.1                    # 0.1 s corresponds to 10 Hz.

footprint:                            # Measured from base_frame, in metres.
  - [rear_x, right_y]
  - [front_x, right_y]
  - [front_x, left_y]
  - [rear_x, left_y]

max_linear_speed: 0.30                # Begin conservatively.
max_angular_speed: 0.30
linear_acceleration: 0.25
angular_acceleration: 0.40
reverse_speed: 0.10
hardware_command_topic: <TEAM_CMD_VEL_TOPIC>
```

The hardware command topic must accept `geometry_msgs/msg/Twist`. Add a bridge
if the Clearpath interface requires `TwistStamped` or another message type.

### Values to leave unchanged for the supplied stack

```yaml
map_frame: map
odom_frame: odom_lidar
odom_topic: /fastlio/odometry_base
localization_mode: slam

localization:
  raw_odom_topic: /fastlio/odometry
  source_frame: camera_init
  source_body_frame: body

scan_topic: /scan
map_topic: /map
planner_command_topic: /hil/planner_cmd_vel
```

These are internal interfaces created or consumed by the HIL package.

## 4. Edit `config/fastlio.site.yaml`

The launcher takes the LiDAR and IMU topics from `robot.site.yaml` and applies
them to FAST-LIO. The duplicated topic values in `fastlio.site.yaml` are not the
authoritative topic settings.

For a physical Velodyne, confirm these values:

```yaml
/**:
  ros__parameters:
    common:
      time_offset_lidar_to_imu: 0.0   # Change only when an offset is measured.
      time_sync_en: false             # Keep false for synchronized sensor clocks.

    preprocess:
      lidar_type: 2                   # Velodyne PointCloud2 handler.
      scan_line: <TEAM_LIDAR_CHANNELS>
      scan_rate: <TEAM_LIDAR_RATE_HZ>
      timestamp_unit: <TEAM_TIME_UNIT_CODE>
      blind: <TEAM_MINIMUM_RANGE>

    mapping:
      extrinsic_T: [x, y, z]          # LiDAR origin expressed in IMU frame.
      extrinsic_R: [r00, r01, r02,
                    r10, r11, r12,
                    r20, r21, r22]
      extrinsic_est_en: false
      det_range: <TEAM_USABLE_RANGE>
```

The physical Velodyne cloud must contain fields compatible with the selected
handler, normally `x`, `y`, `z`, `intensity`, `ring` and `time`. Confirm the
actual per-point time unit. Do not use the Gazebo snapshot type for a physical
rotating LiDAR, and do not generate artificial zero point times.

For the first integration, leave the supplied filter sizes, covariance values,
iteration count and publishing options unchanged.

## 5. Edit `config/mission.site.yaml`

Enter named targets as X/Y coordinates in the initialized SLAM `map` frame:

```yaml
targets:
  - {name: waypoint_1, x: 2.0, y: 0.0}
  - {name: goal, x: 4.0, y: 0.0}
```

These are not latitude/longitude coordinates. Geographic coordinates must be
converted into the initialized local map frame using a surveyed reference.
Start with a short, clear route for the first powered test.

Follow [Mission coordinates and starting-frame setup](docs/MISSION_COORDINATES.md)
for the complete procedure covering local measurements, Gazebo world
coordinates, latitude/longitude conversion, initial heading and validation.

## 6. TF ownership

The supplied stack publishes:

```text
map -> odom_lidar -> base_link
```

- SLAM Toolbox publishes `map -> odom_lidar`.
- The supplied FAST-LIO adapter publishes `odom_lidar -> base_link`.
- FAST-LIO's native TF output is isolated under `/fastlio/internal_tf`.
- The team robot description must publish the static LiDAR and IMU mounting
  transforms from `base_link`.

Disable or isolate any existing Clearpath localization component that publishes
a competing dynamic transform to `base_link`. Wheel-odometry messages may
remain available, but there must be only one publisher for each TF edge and one
dynamic parent for `base_link`.

## 7. Validation order

Build and source the package, then run sensor-only preflight:

```bash
source /opt/ros/jazzy/setup.bash
source fastlio_ws/install/setup.bash
python3 scripts/preflight.py --robot config/robot.site.yaml --sensors-only
```

Start localization and run the complete preflight:

```bash
python3 scripts/start_localization.py --robot config/robot.site.yaml
python3 scripts/preflight.py --robot config/robot.site.yaml
```

Run navigation disarmed first:

```bash
python3 run_hil.py \
  --robot config/robot.site.yaml \
  --mission config/mission.site.yaml
```

Disarmed mode must publish zero hardware velocity. Verify sensor timing, point
fields, TF ownership, localization, map alignment and the planned path before
using `--arm`. Keep the physical E-stop available during every initial powered
test.
