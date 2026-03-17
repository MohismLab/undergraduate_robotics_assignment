from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="part2_ros2",
                executable="receive_clock",
                name="receive_clock",
                output="screen",
            ),
        ]
    )
