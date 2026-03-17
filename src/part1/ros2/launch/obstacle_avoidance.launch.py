from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mode = LaunchConfiguration("mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="braitenberg",
                description="Obstacle avoidance controller (braitenberg or rule_based).",
            ),
            Node(
                package="part1_ros2",
                executable="obstacle_avoidance",
                name="obstacle_avoidance",
                output="screen",
                arguments=["--mode", mode],
            ),
        ]
    )
