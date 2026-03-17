#!/usr/bin/env python3

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

import subprocess


class Clock(Node):
    def __init__(self):
        super().__init__("get_time")
        self._ready = False
        self.create_subscription(
            String, "/date_time", self.callback, qos_profile=QoSProfile(depth=1)
        )
        self.create_timer(0.01, self._maybe_shutdown)

        # Run sudo once.
        subprocess.call(['sudo echo "Authenticated"'], shell=True)

    def callback(self, msg):
        if not self._ready:
            subprocess.call(['sudo date +%F%T -s "{}"'.format(msg.data)], shell=True)
            print("Time set to", msg.data)
            self._ready = True

    def _maybe_shutdown(self):
        if self._ready:
            rclpy.shutdown()


def run():
    rclpy.init()
    node = Clock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    run()


if __name__ == "__main__":
    main()
