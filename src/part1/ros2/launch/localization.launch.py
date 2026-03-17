from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="part1_ros2",
                executable="localization",
                name="localization",
                output="screen",
            ),
        ]
    )
