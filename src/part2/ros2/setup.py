from setuptools import find_packages, setup

package_name = "part2_ros2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/potential_field_navigation.launch.py",
                "launch/rrt_navigation.launch.py",
                "launch/send_clock.launch.py",
                "launch/receive_clock.launch.py",
                "launch/stop.launch.py",
                "launch/slam.launch.py",
                "launch/webots_world.launch.py",
                "launch/map.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/resource",
            [
                "resource/turtlebot_webots.urdf",
                "resource/ros2control.yml",
            ],
        ),
        (
            "share/" + package_name + "/worlds",
            [
                "worlds/assignment.wbt",
            ],
        ),
        (
            "share/" + package_name + "/python",
            [
                "../python/potential_field.py",
                "../python/rrt.py",
                "../python/map.yaml",
                "../python/map.pgm",
            ],
        ),
        (
            "share/" + package_name + "/tmp",
            [
                "../../part2/ros2/tmp/webots_exercise.txt",
            ],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="syy",
    maintainer_email="yc57974@um.edu.mo",
    description="Part 2 Webots + ROS 2 nodes for the undergraduate robotics assignment.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "potential_field_navigation = part2_ros2.potential_field_navigation:main",
            "send_clock = part2_ros2.send_clock:main",
            "receive_clock = part2_ros2.receive_clock:main",
            "stop_robot = part2_ros2.stop:main",
            "plot_trajectory_part2 = part2_ros2.plot_trajectory:main",
            "robot_supervisor = part2_ros2.robot_pose_publisher:main",
            "fake_localization = part2_ros2.fake_localization:main",
        ],
    },
)
