#!/usr/bin/env python3

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist, TwistStamped


class StopNode(Node):
    def __init__(self):
        super().__init__("stop_robot")
        self._publisher = self.create_publisher(TwistStamped, "/cmd_vel", QoSProfile(depth=5))
        self.create_timer(0.1, self._step)

    def _step(self):
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.twist.linear.x = 0.0
        vel_msg.twist.angular.z = 0.0
        self._publisher.publish(vel_msg)


def run(args):
    rclpy.init()
    node = StopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Runs stop command (ROS 2)")
    args, _ = parser.parse_known_args()
    run(args)


if __name__ == "__main__":
    main()
