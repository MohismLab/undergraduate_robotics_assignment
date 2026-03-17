#!/usr/bin/env python3

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
from pathlib import Path as PathLib
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
import sys
from ament_index_python.packages import get_package_share_directory

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist, TwistStamped

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf2_ros
from tf_transformations import euler_from_quaternion

# Import the potential_field.py code rather than copy-pasting.
python_dir = PathLib(get_package_share_directory("part2_ros2")) / "python"
sys.path.insert(0, str(python_dir))
try:
    import rrt
except ImportError:
    raise ImportError(
        'Unable to import potential_field.py. Make sure this file is in "{}"'.format(
            directory
        )
    )


SPEED = 0.18
EPSILON = 0.1

X = 0
Y = 1
YAW = 2


def get_distance(x, y):
    return np.sqrt(np.sum((x - y) ** 2))


def feedback_linearized(pose, velocity, epsilon):
    u = 0.0  # [m/s]
    w = 0.0  # [rad/s] going counter-clockwise.

    # MISSING: Implement feedback-linearization to follow the velocity
    # vector given as argument. Epsilon corresponds to the distance of
    # linearized point in front of the robot.

    return u, w


def get_velocity(position, path_points):
    v = np.zeros_like(position)
    if len(path_points) == 0:
        return v
    # Stop moving if the goal is reached.
    if np.linalg.norm(position - path_points[-1]) < 0.2:
        return v

    # MISSING: Return the velocity needed to follow the
    # path defined by path_points. Assume holonomicity of the
    # point located at position.

    return v


# It is only subscribing to map and goal_pose, not a standard SLAM implementation.
class SLAM(object):
    def __init__(self, node):
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._sub = node.create_subscription(
            OccupancyGrid, "/map", self.callback, qos_profile=map_qos
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, node, spin_thread=True
        )
        self._occupancy_grid = None
        self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)
        self._tf_warning_count = 0  # Throttle TF warnings

    def callback(self, msg):
        values = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.width, msg.info.height)
        )
        processed = np.empty_like(values)
        processed[:] = rrt.FREE
        processed[values < 0] = rrt.UNKNOWN
        processed[values > 50] = rrt.OCCUPIED
        processed = processed.T
        origin = [msg.info.origin.position.x, msg.info.origin.position.y, 0.0]
        resolution = msg.info.resolution
        self._occupancy_grid = rrt.OccupancyGrid(processed, origin, resolution)

    def update(self):
        # Get pose w.r.t. map.
        a = "map"
        b = "base_link"
        try:
            transform = self._tf_buffer.lookup_transform(a, b, Time())
            self._pose[X] = transform.transform.translation.x
            self._pose[Y] = transform.transform.translation.y
            _, _, self._pose[YAW] = euler_from_quaternion(
                [
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w,
                ]
            )
            self._tf_warning_count = 0  # Reset on success
        except Exception as e:
            # Throttle warnings: only print every 50th failure
            self._tf_warning_count += 1
            if self._tf_warning_count <= 1 or self._tf_warning_count % 50 == 0:
                print(f"Waiting for TF map->base_link... ({self._tf_warning_count})")
        pass

    @property
    def ready(self):
        return self._occupancy_grid is not None and not np.isnan(self._pose[0])

    @property
    def pose(self):
        return self._pose

    @property
    def occupancy_grid(self):
        return self._occupancy_grid


class GoalPose(object):
    def __init__(self, node):
        self._sub = node.create_subscription(
            PoseStamped, "/goal_pose", self.callback, qos_profile=QoSProfile(depth=1)
        )
        self._position = np.array([np.nan, np.nan], dtype=np.float32)

    def callback(self, msg):
        # The pose from RViz is with respect to the "map".
        self._position[X] = msg.pose.position.x
        self._position[Y] = msg.pose.position.y
        print("Received new goal position:", self._position)

    @property
    def ready(self):
        return not np.isnan(self._position[0])

    @property
    def position(self):
        return self._position


def get_path(final_node):
    # Construct path from RRT solution.
    if final_node is None:
        return []
    path_reversed = []
    path_reversed.append(final_node)
    while path_reversed[-1].parent is not None:
        path_reversed.append(path_reversed[-1].parent)
    path = list(reversed(path_reversed))
    # Put a point every 5 cm.
    distance = 0.05
    offset = 0.0
    points_x = []
    points_y = []
    for u, v in zip(path, path[1:]):
        center, radius = rrt.find_circle(u, v)
        du = u.position - center
        theta1 = np.arctan2(du[1], du[0])
        dv = v.position - center
        theta2 = np.arctan2(dv[1], dv[0])
        # Check if the arc goes clockwise.
        clockwise = np.cross(u.direction, du).item() > 0.0
        # Generate a point every 5cm apart.
        da = distance / radius
        offset_a = offset / radius
        if clockwise:
            da = -da
            offset_a = -offset_a
            if theta2 > theta1:
                theta2 -= 2.0 * np.pi
        else:
            if theta2 < theta1:
                theta2 += 2.0 * np.pi
        angles = np.arange(theta1 + offset_a, theta2, da)
        offset = distance - (theta2 - angles[-1]) * radius
        points_x.extend(center[X] + np.cos(angles) * radius)
        points_y.extend(center[Y] + np.sin(angles) * radius)
    return zip(points_x, points_y)


class RRTNavigationNode(Node):
    def __init__(self):
        super().__init__("rrt_navigation")
        qos = QoSProfile(depth=5)
        self._publisher = self.create_publisher(TwistStamped, "/cmd_vel", qos)
        self._path_publisher = self.create_publisher(Path, "/path", qos)
        self._slam = SLAM(self)
        self._goal = GoalPose(self)
        self._frame_id = 0
        self._current_path = []
        self._previous_time = self.get_clock().now().nanoseconds / 1e9

        # Stop moving message.
        self._stop_msg = TwistStamped()
        self._stop_msg.twist.linear.x = 0.0
        self._stop_msg.twist.angular.z = 0.0
        self._stop_prepublished = 0

        self.create_timer(0.01, self._step)

    def _step(self):
        # Make sure the robot is stopped initially.
        if self._stop_prepublished < 10:
            self._publisher.publish(self._stop_msg)
            self._stop_prepublished += 1
            return

        self._slam.update()
        current_time = self.get_clock().now().nanoseconds / 1e9

        if not self._goal.ready or not self._slam.ready:
            return

        goal_reached = np.linalg.norm(self._slam.pose[:2] - self._goal.position) < 0.2
        if goal_reached:
            self._publisher.publish(self._stop_msg)
            return

        # Follow path using feedback linearization.
        position = np.array(
            [
                self._slam.pose[X] + EPSILON * np.cos(self._slam.pose[YAW]),
                self._slam.pose[Y] + EPSILON * np.sin(self._slam.pose[YAW]),
            ],
            dtype=np.float32,
        )
        v = get_velocity(position, np.array(self._current_path, dtype=np.float32))
        u, w = feedback_linearized(self._slam.pose, v, epsilon=EPSILON)
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.twist.linear.x = float(u)
        vel_msg.twist.angular.z = float(w)
        self._publisher.publish(vel_msg)

        # Update plan every 2s.
        time_since = current_time - self._previous_time
        if self._current_path and time_since < 2.0:
            return
        self._previous_time = current_time

        # Run RRT.
        start_node, final_node = rrt.rrt(
            self._slam.pose, self._goal.position, self._slam.occupancy_grid
        )
        self._current_path = list(get_path(final_node))
        if not self._current_path:
            print("Unable to reach goal position:", self._goal.position)

        # Publish path to RViz.
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for u in self._current_path:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = path_msg.header.stamp
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = u[X]
            pose_msg.pose.position.y = u[Y]
            path_msg.poses.append(pose_msg)
        self._path_publisher.publish(path_msg)
        self._frame_id += 1


def run(args):
    rclpy.init()
    node = RRTNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Runs RRT navigation (ROS 2 + Webots)")
    args, _ = parser.parse_known_args()
    run(args)


if __name__ == "__main__":
    main()
