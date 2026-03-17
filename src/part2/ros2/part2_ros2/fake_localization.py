#!/usr/bin/env python3

"""
Fake Localization Node for ROS 2

This node implements the "fake localization" pattern commonly used in ROS:
- Subscribes to ground truth pose from Webots (/robot_pose)
- Listens to the noisy odom->base_link TF from diffdrive controller
- Computes and publishes map->odom TF to compensate for drift

Result:
- odom -> base_link: Noisy (realistic physics from diffdrive)
- map -> odom: Dynamic correction (computed to cancel drift)
- map -> base_link: Accurate (matches ground truth)

This is the standard ROS way to "cheat" for testing while keeping realistic odom.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from tf2_ros import TransformException
import numpy as np
from tf_transformations import quaternion_matrix, quaternion_from_matrix


class FakeLocalization(Node):
    def __init__(self):
        super().__init__("fake_localization")

        # TF buffer and listener for odom->base_link
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # TF broadcaster for map->odom
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribe to ground truth pose
        self._pose_sub = self.create_subscription(
            PoseStamped, "/robot_pose", self._pose_callback, QoSProfile(depth=1)
        )

        # Store latest ground truth
        self._ground_truth = None

        self.get_logger().info("Fake localization started - publishing map->odom TF")

    def _pose_to_matrix(self, pose):
        """Convert pose to 4x4 transformation matrix."""
        q = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        mat = quaternion_matrix(q)
        mat[0, 3] = pose.position.x
        mat[1, 3] = pose.position.y
        mat[2, 3] = pose.position.z
        return mat

    def _transform_to_matrix(self, transform):
        """Convert TransformStamped to 4x4 transformation matrix."""
        q = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        ]
        mat = quaternion_matrix(q)
        mat[0, 3] = transform.translation.x
        mat[1, 3] = transform.translation.y
        mat[2, 3] = transform.translation.z
        return mat

    def _matrix_to_transform(self, mat, stamp, parent_frame, child_frame):
        """Convert 4x4 transformation matrix to TransformStamped."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = mat[0, 3]
        t.transform.translation.y = mat[1, 3]
        t.transform.translation.z = mat[2, 3]

        q = quaternion_from_matrix(mat)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        return t

    def _pose_callback(self, msg):
        """
        When we receive ground truth pose, compute map->odom transform.

        Logic:
        - T_map_base = ground truth pose (what we want)
        - T_odom_base = current odom->base_link TF (noisy)
        - T_map_odom = T_map_base * inv(T_odom_base)
        """
        try:
            # Get current odom->base_link transform
            odom_to_base = self._tf_buffer.lookup_transform(
                "odom", "base_link", rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().warn(
                f"Could not get odom->base_link: {e}", throttle_duration_sec=2.0
            )
            return

        # Convert ground truth (map->base_link) to matrix
        T_map_base = self._pose_to_matrix(msg.pose)

        # Convert odom->base_link to matrix
        T_odom_base = self._transform_to_matrix(odom_to_base.transform)

        # Compute map->odom: T_map_odom = T_map_base * inv(T_odom_base)
        T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)

        # Convert back to TransformStamped and publish
        map_to_odom = self._matrix_to_transform(
            T_map_odom, self.get_clock().now().to_msg(), "map", "odom"
        )

        self._tf_broadcaster.sendTransform(map_to_odom)


def main(args=None):
    rclpy.init(args=args)
    node = FakeLocalization()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
