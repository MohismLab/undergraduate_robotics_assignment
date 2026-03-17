#!/usr/bin/env python3

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import numpy as np
import rclpy
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped

# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

LOG_PATH = Path(get_package_share_directory('part1_ros2')) / 'tmp' / 'webots_exercise.txt'


def braitenberg(front, front_left, front_right, left, right):
    u = 0.0  # [m/s]
    w = 0.0  # [rad/s] going counter-clockwise.

    # MISSING: Implement a braitenberg controller that takes the range
    # measurements given in argument to steer the robot.

    return u, w


def rule_based(front, front_left, front_right, left, right):
    u = 0.0  # [m/s]
    w = 0.0  # [rad/s] going counter-clockwise.

    # MISSING: Implement a rule-based controller that avoids obstacles.
    return u, w


class SimpleLaser(object):
    def __init__(self, node):
        self._sub = node.create_subscription(
            LaserScan, "/scan", self.callback, qos_profile=QoSProfile(depth=10)
        )
        self._angles = [0.0, np.pi / 4.0, -np.pi / 4.0, np.pi / 2.0, -np.pi / 2.0]
        self._width = np.pi / 180.0 * 10.0  # 10 degrees cone of view.
        self._measurements = [np.nan] * len(self._angles)
        self._indices = None

    def callback(self, msg):
        # Helper for angles.
        def _within(x, a, b):
            pi2 = np.pi * 2.0
            x %= pi2
            a %= pi2
            b %= pi2
            if a < b:
                return a <= x and x <= b
            return a <= x or x <= b

        # Compute indices the first time.
        if self._indices is None:
            self._indices = [[] for _ in range(len(self._angles))]
            for i, d in enumerate(msg.ranges):
                angle = msg.angle_min + i * msg.angle_increment
                for j, center_angle in enumerate(self._angles):
                    if _within(
                        angle,
                        center_angle - self._width / 2.0,
                        center_angle + self._width / 2.0,
                    ):
                        self._indices[j].append(i)

        ranges = np.array(msg.ranges)
        for i, idx in enumerate(self._indices):
            # We do not take the minimum range of the cone but the 10-th percentile for robustness.
            self._measurements[i] = np.percentile(ranges[idx], 10)

    @property
    def ready(self):
        return not np.isnan(self._measurements[0])

    @property
    def measurements(self):
        return self._measurements


class GroundtruthPose(object):
    def __init__(self, node):
        self._sub = node.create_subscription(
            PoseStamped, "/robot_pose", self.callback, qos_profile=QoSProfile(depth=10)
        )
        self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)

    def callback(self, msg):
        self._pose[0] = msg.pose.position.x
        self._pose[1] = msg.pose.position.y
        _, _, yaw = euler_from_quaternion(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )
        self._pose[2] = yaw

    @property
    def ready(self):
        return not np.isnan(self._pose[0])

    @property
    def pose(self):
        return self._pose


class ObstacleAvoidanceNode(Node):
    def __init__(self, mode):
        super().__init__("obstacle_avoidance")
        self._avoidance_method = globals()[mode]
        qos = QoSProfile(depth=5)
        self._publisher = self.create_publisher(TwistStamped, "/cmd_vel", qos)
        self._laser = SimpleLaser(self)
        # Keep track of groundtruth position for plotting purposes.
        self._groundtruth = GroundtruthPose(self)
        self._pose_history = []
        open(LOG_PATH, "w").close()
        self.create_timer(0.01, self._step)

    def _step(self):
        if not self._laser.ready or not self._groundtruth.ready:
            return

        u, w = self._avoidance_method(*self._laser.measurements)
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.twist.linear.x = float(u)
        vel_msg.twist.angular.z = float(w)
        self._publisher.publish(vel_msg)

        # Log groundtruth positions in LOG_PATH
        self._pose_history.append(self._groundtruth.pose.copy())
        if len(self._pose_history) >= 10:
            with open(LOG_PATH, "a") as fp:
                fp.write(
                    "\n".join(",".join(str(v) for v in p) for p in self._pose_history)
                    + "\n"
                )
            self._pose_history = []


def run(args):
    rclpy.init()
    node = ObstacleAvoidanceNode(args.mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description="Runs obstacle avoidance (ROS 2 + Webots)"
    )
    parser.add_argument(
        "--mode",
        action="store",
        default="braitenberg",
        help="Method.",
        choices=["braitenberg", "rule_based"],
    )
    args, _ = parser.parse_known_args()
    run(args)


if __name__ == "__main__":
    main()
