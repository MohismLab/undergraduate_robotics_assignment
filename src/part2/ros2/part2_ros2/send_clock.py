#!/usr/bin/env python3

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import datetime
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class SendClockNode(Node):
    def __init__(self):
        super().__init__("send_clock")
        self._publisher = self.create_publisher(
            String, "/date_time", QoSProfile(depth=1)
        )
        self.create_timer(0.001, self._step)

    def _step(self):
        now = self.get_clock().now().nanoseconds / 1e9
        date_time = datetime.datetime.fromtimestamp(int(now)).strftime(
            "%Y%m%d %H:%M:%S"
        )
        self._publisher.publish(String(data=date_time))


def run():
    rclpy.init()
    node = SendClockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    run()


if __name__ == "__main__":
    main()
