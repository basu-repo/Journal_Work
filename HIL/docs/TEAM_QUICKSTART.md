# Team quickstart

1. Clone the repository and cd into HIL. Follow README installation and build.
2. Start the selected robot's driver, sensor streams and mounting TF tree.
3. Copy robot, mission and matching FAST-LIO template into *.site.yaml files.
4. Enter actual topics, frame names, footprint, limits, scan filtering,
   point time units and measured extrinsics. Confirm calibration only after verification.
5. Run sensor preflight. Check acceleration near gravity when stationary,
   gyro sign under manual rotation, synchronized stamps and LiDAR-to-base TF.
6. Start included localization. Keep the robot stationary through IMU initialization.
7. Run full preflight and inspect map/robot alignment in RViz.
8. Start run_hil.py disarmed, verify path and zero hardware commands.
9. Test the E-stop and watchdog with lifted wheels or motors disabled.
10. Arm a short controlled test, record a bag, then progress to the Baylands
    HIL mission or surveyed physical waypoint coordinates.

The root README contains exact commands. Do not reuse example coordinates as
surveyed targets. For hardware plus Gazebo, read BAYLANDS_HIL.md before bridging.
