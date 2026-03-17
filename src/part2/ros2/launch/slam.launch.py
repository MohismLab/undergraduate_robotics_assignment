import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    slam_config_file = os.path.join(
        get_package_share_directory("slam_toolbox"),
        "config",
        "mapper_params_online_async.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use sim time from /clock",
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    slam_config_file,
                    {
                        "use_sim_time": use_sim_time,
                        "base_frame": "base_link",
                        "odom_frame": "odom",
                        "map_frame": "map",
                        "scan_topic": "/scan",
                    },
                ],
            ),
        ]
    )
