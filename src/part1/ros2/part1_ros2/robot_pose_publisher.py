"""
Custom Webots plugin that publishes the robot's absolute groundtruth pose.
This plugin runs on the TurtleBot itself and publishes /robot_pose.
"""

import math
import rclpy
from geometry_msgs.msg import PoseStamped


class RobotPosePublisher:
    def init(self, webots_node, properties):
        # Initialize ROS node
        rclpy.init(args=None)
        self.__node = rclpy.create_node("robot_pose_publisher")

        # Get robot from webots_node
        self.__robot = webots_node.robot

        if not self.__robot.getSupervisor():
            print("Error: Robot is not a Supervisor! Check 'supervisor' field in .wbt")
            return

        # Create publisher for absolute pose
        self.__pose_pub = self.__node.create_publisher(PoseStamped, "/robot_pose", 10)

        self.__node.get_logger().info(
            "RobotPosePublisher initialized, publishing to /robot_pose"
        )

    def step(self):
        # Spin ROS callbacks
        rclpy.spin_once(self.__node, timeout_sec=0)

        # Get robot's position from Webots supervisor API
        # Note: The robot itself can query its own position
        position = self.__robot.getSelf().getPosition()  # [x, y, z]
        orientation = self.__robot.getSelf().getOrientation()  # 3x3 rotation matrix

        # Convert rotation matrix to quaternion
        qw, qx, qy, qz = self._rotation_matrix_to_quaternion(orientation)

        # Create and publish PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.__node.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"

        pose_msg.pose.position.x = position[0]
        pose_msg.pose.position.y = position[1]
        pose_msg.pose.position.z = position[2]

        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.__pose_pub.publish(pose_msg)

    def _rotation_matrix_to_quaternion(self, R):
        """Convert 3x3 rotation matrix to quaternion (w, x, y, z)."""
        # R is a flat list of 9 elements: [R11, R12, R13, R21, R22, R23, R31, R32, R33]
        trace = R[0] + R[4] + R[8]

        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[7] - R[5]) * s
            y = (R[2] - R[6]) * s
            z = (R[3] - R[1]) * s
        elif R[0] > R[4] and R[0] > R[8]:
            s = 2.0 * math.sqrt(1.0 + R[0] - R[4] - R[8])
            w = (R[7] - R[5]) / s
            x = 0.25 * s
            y = (R[1] + R[3]) / s
            z = (R[2] + R[6]) / s
        elif R[4] > R[8]:
            s = 2.0 * math.sqrt(1.0 + R[4] - R[0] - R[8])
            w = (R[2] - R[6]) / s
            x = (R[1] + R[3]) / s
            y = 0.25 * s
            z = (R[5] + R[7]) / s
        else:
            s = 2.0 * math.sqrt(1.0 + R[8] - R[0] - R[4])
            w = (R[3] - R[1]) / s
            x = (R[2] + R[6]) / s
            y = (R[5] + R[7]) / s
            z = 0.25 * s

        return w, x, y, z
