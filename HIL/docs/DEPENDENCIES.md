# Dependencies

Tested build platform: Ubuntu 24.04, ROS 2 Jazzy, C++17, system Python 3.12.
Use the system Python associated with ROS; a separate virtual environment must
also expose the installed ROS Python packages.

Install commands are in the root README. ROS packages needed by FAST-LIO2 are
listed in vendor/FAST_LIO/package.xml; rosdep resolves their system dependencies.
Initialize rosdep once if necessary with sudo rosdep init, then rosdep update.

Runtime dependencies: rclpy, geometry_msgs, nav_msgs, sensor_msgs, std_msgs,
tf2_ros, ament_index_python, rtabmap_util (ground segmentation),
pointcloud_to_laserscan and slam_toolbox. Python libraries: NumPy, SciPy, PyYAML.
C++ dependencies include Eigen3, PCL, ROS message generation and Python headers.
The CMake build uses OpenMP when available.

The package supplies the navigation and localization applications. The team
supplies ROS-compatible hardware drivers, robot description/static transforms,
base command bridge and E-stop. Gazebo Harmonic/ros_gz_bridge are needed only
for the optional simulation/HIL environment and are supplied with that environment.

No network download is needed to obtain FAST-LIO source after cloning this
repository: its required IKFoM and ikd-Tree headers/source are bundled.
Installing ROS/system dependencies still requires a configured package source.
See THIRD_PARTY_NOTICES.md for source provenance.
