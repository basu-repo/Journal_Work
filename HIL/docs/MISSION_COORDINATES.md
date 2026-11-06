# Mission coordinates and starting-frame setup

The navigation stack does not accept latitude and longitude directly. It accepts
target X/Y coordinates in metres in the current SLAM `map` frame.

This guide explains how to establish the starting reference and prepare mission
targets for simulation, indoor testing and outdoor testing.

## 1. What happens at startup

FAST-LIO2 provides relative motion from its initialization pose. For a new
session, the stationary starting pose is normally treated as approximately:

```text
map position: x = 0 m, y = 0 m
map heading:  yaw = 0 rad
```

The robot's initial forward direction defines positive map X. Positive map Y is
to the robot's left.

```text
                 +X (initial forward)
                  ^
                  |
       +Y (left) <-H-> -Y (right)
                  |
                  v
                 -X
```

The physical robot may be anywhere geographically. FAST-LIO2 alone does not
know its latitude, longitude or orientation relative to true north.

Keep the robot stationary during initialization. Record the physical starting
point and initial heading for every trial. Restarting localization creates a new
local reference and may require regenerating the mission coordinates.

## 2. Local measured missions

For an indoor test or a small marked test area, measure each target relative to
the starting pose.

Example:

- waypoint 1 is 3 m forward and 1 m left;
- the goal is 8 m forward and 2 m right.

```yaml
targets:
  - {name: waypoint_1, x: 3.0, y: 1.0}
  - {name: goal, x: 8.0, y: -2.0}
```

Use the same physical starting mark and initial orientation when repeating the
mission.

## 3. Gazebo or HIL world coordinates

Do not assume that Gazebo world coordinates are equal to SLAM map coordinates.
Let the robot start in the world at `(X0, Y0)` with world yaw `psi0`. For a
world target `(Xt, Yt)`, first calculate:

```text
dX = Xt - X0
dY = Yt - Y0
```

Then rotate the displacement into the robot's initial map frame:

```text
map_x =  cos(psi0) * dX + sin(psi0) * dY
map_y = -sin(psi0) * dX + cos(psi0) * dY
```

Angles are in radians. Validate at least one known marker before starting an
autonomous mission. Simulator ground truth may be recorded for evaluation, but
must not be fed into FAST-LIO2, SLAM or the planner during a GNSS-denied run.

## 4. Outdoor latitude/longitude missions

For an outdoor mission, record a surveyed initialization reference:

| Reference item | Trial value |
|---|---|
| Start latitude | |
| Start longitude | |
| Start altitude, if needed for evaluation | |
| Initial forward bearing from true north | |
| Coordinate datum | WGS84 / other: |
| Survey method and expected accuracy | |

For each waypoint, convert its latitude/longitude into a local east/north
displacement from the surveyed start. Use a geodesy library or the site's
surveyed ENU/UTM coordinates for the official conversion.

For short-distance checks only, an approximate WGS84 local conversion is:

```text
R = 6378137 m
d_lat = radians(target_latitude - start_latitude)
d_lon = radians(target_longitude - start_longitude)

north = R * d_lat
east  = R * cos(radians(start_latitude)) * d_lon
```

Use a proper geodetic conversion for larger areas, high-accuracy work or paths
crossing coordinate-zone boundaries.

Next rotate east/north into the initial robot-aligned map frame. Let `beta` be
the robot's initial forward bearing measured clockwise from true north:

```text
map_x =  east * sin(beta) + north * cos(beta)
map_y = -east * cos(beta) + north * sin(beta)
```

Examples:

- `beta = 0 degrees`: the robot starts facing north; north becomes +X and west
  becomes +Y.
- `beta = 90 degrees`: the robot starts facing east; east becomes +X and north
  becomes +Y.

Enter the resulting metre values in `config/mission.site.yaml`:

```yaml
targets:
  - {name: waypoint_1, x: <converted_map_x>, y: <converted_map_y>}
  - {name: goal, x: <converted_map_x>, y: <converted_map_y>}
```

The current stack does not subscribe to latitude/longitude targets and does not
perform this conversion during the mission. Geographic coordinates are mission
preparation data, not a live GNSS localization input.

## 5. When an existing site map is used

If targets are defined in an existing surveyed map, the initialization process
must establish the transform between that site frame and the active SLAM map.
Do not enter site coordinates directly until this alignment has been measured
and verified.

Record:

- the site-frame coordinates of the robot start;
- the robot's initial orientation in the site frame;
- the transformation method;
- at least one independent validation marker;
- the expected alignment error.

An incorrect initial heading rotates every mission target even when the starting
position is correct.

## 6. Required validation before arming

1. Place the robot on the recorded start mark and align its initial heading.
2. Start the sensors and keep the robot stationary while FAST-LIO2 initializes.
3. Confirm that the live map pose begins near `(0,0)` with the expected heading.
4. Display the mission targets and planned path while navigation is disarmed.
5. Check that each displayed target corresponds to the intended physical mark.
6. Test a short target, normally 2--4 m away, before loading the full mission.
7. Keep GNSS and ground truth outside the estimator and planner during the
   denied test.
8. Independently measure the physical waypoint and goal errors.

Do not arm the robot if the plotted target appears rotated, mirrored or offset
from its physical location. Correct the initialization reference or coordinate
conversion first.
