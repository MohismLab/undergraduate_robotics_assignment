#!/usr/bin/env python3

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import sys
from ament_index_python.packages import get_package_share_directory
# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped

# For groundtruth information.
from tf_transformations import euler_from_quaternion

python_dir = Path(get_package_share_directory("part2_ros2")) / "python"
sys.path.insert(0, str(python_dir))
try:
    import potential_field
except ImportError:
    raise ImportError(
        'Unable to import potential_field.py. Make sure this file is in "{}"'.format(
            directory
        )
    )


ROBOT_RADIUS = 0.105 / 2.0
CYLINDER_POSITION = np.array([0.3, 0.2], dtype=np.float32)
CYLINDER_RADIUS = 0.3
GOAL_POSITION = np.array([1.5, 1.5], dtype=np.float32)
MAX_SPEED = 0.18  # Match TurtleBot3 motor limits (6.67 rad/s * 0.033m = 0.22 m/s)
EPSILON = 0.2

USE_RELATIVE_POSITIONS = False

X = 0
Y = 1
YAW = 2
LOG_PATH = Path(get_package_share_directory('part2_ros2')) / 'tmp' / 'webots_exercise.txt'


def get_distance(x, y):
    return np.sqrt(np.sum((x - y) ** 2))


def feedback_linearized(pose, velocity, epsilon):
    u = 0.0  # [m/s]
    w = 0.0  # [rad/s] going counter-clockwise.

    # MISSING: Implement feedback-linearization to follow the velocity
    # vector given as argument. Epsilon corresponds to the distance of
    # linearized point in front of the robot.

    return u, w


def get_relative_position(absolute_pose, absolute_position):
    relative_position = absolute_position.copy()

    # MISSING: Compute the relative position of absolute_position in the
    # coordinate frame defined by absolute_pose.

    return relative_position


class GroundtruthPose(object):
    def __init__(self, node):
        self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        self._sub = node.create_subscription(
            PoseStamped, "/robot_pose", self.callback, qos_profile=QoSProfile(depth=10)
        )

    def callback(self, msg):
        self._pose[X] = msg.pose.position.x
        self._pose[Y] = msg.pose.position.y
        _, _, yaw = euler_from_quaternion(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )
        self._pose[YAW] = yaw

    @property
    def ready(self):
        return not np.isnan(self._pose[0])

    @property
    def pose(self):
        return self._pose


def get_velocity(point_position, goal_position, obstacle_position):
    # Missing: Implement the potential field method to get the next
    # velocity vector given the point position, goal position and obstacle

    return potential_field.cap(v, max_speed=MAX_SPEED)


class PotentialFieldNode(Node):
    def __init__(self):
        super().__init__("potential_field_navigation")
        qos = QoSProfile(depth=5)
        self._publisher = self.create_publisher(TwistStamped, "/cmd_vel", qos)
        self._groundtruth = GroundtruthPose(self)
        self._pose_history = []
        open(LOG_PATH, "w").close()
        self.create_timer(0.01, self._step)

    def _step(self):
        if not self._groundtruth.ready:
            return

        absolute_point_position = np.array(
            [
                self._groundtruth.pose[X]
                + EPSILON * np.cos(self._groundtruth.pose[YAW]),
                self._groundtruth.pose[Y]
                + EPSILON * np.sin(self._groundtruth.pose[YAW]),
            ],
            dtype=np.float32,
        )

        if USE_RELATIVE_POSITIONS:
            point_position = get_relative_position(
                self._groundtruth.pose, absolute_point_position
            )
            goal_position = get_relative_position(self._groundtruth.pose, GOAL_POSITION)
            obstacle_position = get_relative_position(
                self._groundtruth.pose, CYLINDER_POSITION
            )
            pose = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        else:
            point_position = absolute_point_position
            goal_position = GOAL_POSITION
            obstacle_position = CYLINDER_POSITION
            pose = self._groundtruth.pose

        # Get velocity.
        v = get_velocity(point_position, goal_position, obstacle_position)

        u, w = feedback_linearized(pose, v, epsilon=EPSILON)
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.twist.linear.x = float(u)
        vel_msg.twist.angular.z = float(w)
        self._publisher.publish(vel_msg)

        # Log groundtruth positions in LOG_PATH
        self._pose_history.append(
            np.concatenate([self._groundtruth.pose, absolute_point_position], axis=0)
        )
        if len(self._pose_history) >= 10:
            with open(LOG_PATH, "a") as fp:
                fp.write(
                    "\n".join(",".join(str(v) for v in p) for p in self._pose_history)
                    + "\n"
                )
            self._pose_history = []


def run(args):
    rclpy.init()
    node = PotentialFieldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description="Runs potential field navigation (ROS 2 + Webots)"
    )
    args, _ = parser.parse_known_args()
    run(args)


if __name__ == "__main__":
    main()
