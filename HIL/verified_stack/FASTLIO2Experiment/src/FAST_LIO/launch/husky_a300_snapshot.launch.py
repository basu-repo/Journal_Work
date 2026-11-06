from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    default_config = Path(get_package_share_directory("fast_lio")) / "config" / "husky_a300_snapshot.yaml"
    config = LaunchConfiguration("config_file")
    point_filter_num = LaunchConfiguration("point_filter_num")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=str(default_config),
            description="FAST-LIO2 parameter file",
        ),
        DeclareLaunchArgument(
            "point_filter_num",
            default_value="4",
            description="Keep one point from each consecutive N input points",
        ),
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fast_lio_snapshot",
            output="screen",
            parameters=[ParameterFile(config, allow_substs=True), {
                "use_sim_time": True,
                "point_filter_num": ParameterValue(point_filter_num, value_type=int),
            }],
            remappings=[
                ("/Odometry", "/fastlio/odometry"),
                ("/path", "/fastlio/path"),
                ("/cloud_registered", "/fastlio/cloud_registered"),
                ("/cloud_registered_body", "/fastlio/cloud_registered_body"),
                ("/tf", "/fastlio/tf"),
                ("/tf_static", "/fastlio/tf_static"),
            ],
        )
    ])
