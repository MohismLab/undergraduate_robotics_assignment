import os
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)


def generate_launch_description():
    pkg_share = get_package_share_directory("part1_ros2")
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    world_path = os.path.join(pkg_share, "worlds", "assignment.wbt")

    robot_description_path = os.path.join(
        pkg_share, "resource", "turtlebot_webots.urdf"
    )
    ros2_control_params = os.path.join(pkg_share, "resource", "ros2control.yml")

    # Read URDF content for robot_state_publisher
    with open(robot_description_path, "r") as fp:
        robot_description_content = fp.read()

    webots = WebotsLauncher(world=world_path, ros2_supervisor=True)

    # Topic remappings for Jazzy (uses unstamped cmd_vel)
    mappings = [
        # ("/diffdrive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ("/diffdrive_controller/cmd_vel", "/cmd_vel"),
        ("/diffdrive_controller/odom", "/odom"),
    ]

    turtlebot_controller = WebotsController(
        robot_name="TurtleBot3Burger",
        parameters=[
            {
                "robot_description": robot_description_path,
                "use_sim_time": use_sim_time,
                "set_robot_state_publisher": False,
            },
            ros2_control_params,
        ],
        remappings=mappings,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_content,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # ROS control spawners
    controller_manager_timeout = ["--controller-manager-timeout", "50"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""

    diffdrive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["diffdrive_controller"] + controller_manager_timeout,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["joint_state_broadcaster"] + controller_manager_timeout,
    )

    # Wait for the simulation to be ready before starting ROS2 controllers
    waiting_nodes = WaitForControllerConnection(
        target_driver=turtlebot_controller,
        nodes_to_start=[diffdrive_controller_spawner, joint_state_broadcaster_spawner],
    )

    return LaunchDescription(
        [
            webots,
            webots._supervisor,
            turtlebot_controller,
            robot_state_publisher,
            waiting_nodes,
            # Shutdown all nodes when Webots exits
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
