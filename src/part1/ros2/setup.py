from setuptools import find_packages, setup

package_name = "part1_ros2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/obstacle_avoidance.launch.py",
                "launch/localization.launch.py",
                "launch/webots_world.launch.py",
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
            "share/" + package_name + "/tmp",
            [
                "../../part1/ros2/tmp/webots_exercise.txt",
            ],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="syy",
    maintainer_email="yc57974@um.edu.mo",
    description="Part 1 Webots + ROS 2 nodes for the undergraduate robotics assignment.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "obstacle_avoidance = part1_ros2.obstacle_avoidance:main",
            "plot_trajectory_part1 = part1_ros2.plot_trajectory:main",
            "robot_supervisor = part1_ros2.robot_pose_publisher:main",
        ],
    },
)
