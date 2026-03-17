#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to map file
    pkg_share = get_package_share_directory("part2_ros2")
    map_yaml_file = os.path.join(pkg_share, "python", "map.yaml")

    # Map server node (lifecycle node)
    map_server = LifecycleNode(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        namespace="",
        output="screen",
        parameters=[
            {
                "yaml_filename": map_yaml_file,
                "use_sim_time": False,
            }
        ],
    )

    # Lifecycle Manager to handle the state transitions
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"node_names": ["map_server"]},
            {"bond_timeout": 4.0},
        ],
    )

    return LaunchDescription(
        [
            map_server,
            lifecycle_manager,
        ]
    )
