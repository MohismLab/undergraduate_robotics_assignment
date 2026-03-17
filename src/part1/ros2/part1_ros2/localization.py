#!/usr/bin/env python3

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import argparse
import copy
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

# Robot motion commands:
# http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped

# Laser scan message:
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

# For displaying particles.
# http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud.html
from sensor_msgs.msg import PointCloud
from sensor_msgs.msg import ChannelFloat32
from geometry_msgs.msg import Point32

LOG_PATH = Path(get_package_share_directory('part1_ros2')) / 'tmp' / 'webots_exercise.txt'


# Constants used for indexing.
X = 0
Y = 1
YAW = 2

ROBOT_RADIUS = 0.105 / 2.0
WALL_OFFSET = 2.0
CYLINDER_POSITION = np.array([0.3, 0.2], dtype=np.float32)
CYLINDER_RADIUS = 0.3 + ROBOT_RADIUS


def braitenberg(front, front_left, front_right, left, right):
    u = 0.0  # [m/s]
    w = 0.0  # [rad/s] going counter-clockwise.
    # MISSING: Implement a braitenberg controller that takes the range
    # measurements given in argument to steer the robot safely.
    return u, w


class Particle(object):
    """Represents a particle."""

    def __init__(self):
        self._pose = np.zeros(3, dtype=np.float32)
        self._weight = 1.0

        # MISSING: Initialize a particle randomly in the arena. Set the values of
        # _pose such that it is a valid pose (i.e., inside the arena walls, but
        # outside the cylinder). Consider implementing is_valid() below and use it
        # in this function.
        self.generate_valid_pose()

    def generate_valid_pose(self):
        """
        This function generates a random point in the arena and verify whether it is valid
        Repeat until a valid position is generated.
        """

        def draw_from_distribution():
            # Uniform distribution
            # return np.random.uniform(-2, 2, size=3)

            # Alternative Gaussian distribution centered at (0, 0)
            # Under my Braitenberg controller, agent appears more frequently in close-center area
            return np.random.randn(3) * 2

        self._pose = draw_from_distribution()

        print("init", self._pose)
        while not self.is_valid():
            self._pose = draw_from_distribution()
            print("re-init", self._pose)

    def is_valid(self):
        # MISSING: Implement a function that returns True if the current particle
        # position is valid. You might need to use this function in __init__()
        # and compute_weight().
        return True

    def move(self, delta_pose):
        # MISSING: Update the particle pose according the motion model.
        # delta_pose is an offset in the particle frame. As motion model,
        # use roughtly 10% standard deviation with respect to the forward
        # and rotational velocity.
        #
        # In a second step, make the necessary modifications to handle the
        # kidnapped robot problem. For example, with a low probability the
        # particle can be repositioned randomly in the arena.
        pass

    def compute_weight(self, front, front_left, front_right, left, right):
        # MISSING: Update the particle weight self._weight according to measurements.
        # You can use the self.ray_trace(angle) function below. Remember to reduce the
        # weight of particles that are outside the arena. As measurement model, use a
        # Gaussian error with a standard deviation of 80 [cm]. Note that the maximum
        # range of the laser-range finder is 3.5 meters (observations beyond this range
        # will show up as infinity).
        pass

    def ray_trace(self, angle):
        """Returns the distance to the first obstacle from the particle."""

        def intersection_segment(x1, x2, y1, y2):
            point1 = np.array([x1, y1], dtype=np.float32)
            point2 = np.array([x2, y2], dtype=np.float32)
            v1 = self._pose[:2] - point1
            v2 = point2 - point1
            v3 = np.array(
                [
                    np.cos(angle + self._pose[YAW] + np.pi / 2.0),
                    np.sin(angle + self._pose[YAW] + np.pi / 2.0),
                ],
                dtype=np.float32,
            )
            t1 = np.cross(v2, v1) / np.dot(v2, v3)
            t2 = np.dot(v1, v3) / np.dot(v2, v3)
            if t1 >= 0.0 and t2 >= 0.0 and t2 <= 1.0:
                return t1
            return float("inf")

        def intersection_cylinder(x, y, r):
            center = np.array([x, y], dtype=np.float32)
            v = np.array(
                [
                    np.cos(angle + self._pose[YAW] + np.pi),
                    np.sin(angle + self._pose[YAW] + np.pi),
                ],
                dtype=np.float32,
            )

            v1 = center - self._pose[:2]
            a = v.dot(v)
            b = 2.0 * v.dot(v1)
            c = v1.dot(v1) - r**2.0
            q = b**2.0 - 4.0 * a * c
            if q < 0.0:
                return float("inf")
            g = 1.0 / (2.0 * a)
            q = g * np.sqrt(q)
            b = -b * g
            d = min(b + q, b - q)
            if d >= 0.0:
                return d
            return float("inf")

        d = min(
            intersection_segment(-WALL_OFFSET, -WALL_OFFSET, -WALL_OFFSET, WALL_OFFSET),
            intersection_segment(WALL_OFFSET, WALL_OFFSET, -WALL_OFFSET, WALL_OFFSET),
            intersection_segment(-WALL_OFFSET, WALL_OFFSET, -WALL_OFFSET, -WALL_OFFSET),
            intersection_segment(-WALL_OFFSET, WALL_OFFSET, WALL_OFFSET, WALL_OFFSET),
            intersection_cylinder(
                CYLINDER_POSITION[X], CYLINDER_POSITION[Y], CYLINDER_RADIUS
            ),
        )
        return d

    @property
    def pose(self):
        return self._pose

    @property
    def weight(self):
        return self._weight


class SimpleLaser(object):
    def __init__(self, node):
        self._sub = node.create_subscription(
            LaserScan, "/scan", self.callback, qos_profile=QoSProfile(depth=10)
        )
        self._angles = [0.0, np.pi / 4.0, -np.pi / 4.0, np.pi / 2.0, -np.pi / 2.0]
        self._width = np.pi / 180.0 * 3.1  # 3.1 degrees cone of view (3 rays).
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


class Motion(object):
    def __init__(self, node):
        self._previous_time = None
        self._delta_pose = np.array([0.0, 0.0, 0.0], dtype=np.float32)
        self._sub = node.create_subscription(
            Odometry, "/odom", self.callback, qos_profile=QoSProfile(depth=10)
        )

    def callback(self, msg):
        u = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._previous_time is None:
            self._previous_time = current_time
            return
        dt = current_time - self._previous_time
        self._delta_pose[X] += u * dt
        self._delta_pose[Y] += 0.0
        self._delta_pose[YAW] += w * dt
        self._previous_time = current_time

    @property
    def ready(self):
        return True

    @property
    def delta_pose(self):
        ret = self._delta_pose.copy()
        self._delta_pose[:] = 0
        return ret


class GroundtruthPose(object):
    def __init__(self, node):
        self._sub = node.create_subscription(
            PoseStamped, "/robot_pose", self.callback, qos_profile=QoSProfile(depth=10)
        )
        self._pose = np.array([np.nan, np.nan, np.nan], dtype=np.float32)

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


class LocalizationNode(Node):
    def __init__(self):
        super().__init__("localization")
        qos = QoSProfile(depth=5)
        self._publisher = self.create_publisher(TwistStamped, "/cmd_vel", qos)
        self._particle_publisher = self.create_publisher(PointCloud, "/particles", qos)
        self._laser = SimpleLaser(self)
        self._motion = Motion(self)
        # Keep track of groundtruth position for plotting purposes.
        self._groundtruth = GroundtruthPose(self)
        self._pose_history = []
        open(LOG_PATH, "w").close()

        num_particles = 50
        self._particles = [Particle() for _ in range(num_particles)]
        self._frame_id = 0
        self._num_particles = num_particles
        self.create_timer(0.01, self._step)

    def _step(self):
        if (
            not self._laser.ready
            or not self._motion.ready
            or not self._groundtruth.ready
        ):
            return

        # Run braitenberg.
        u, w = braitenberg(*self._laser.measurements)
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.twist.linear.x = float(u)
        vel_msg.twist.angular.z = float(w)
        self._publisher.publish(vel_msg)

        # Update particle positions and weights.
        total_weight = 0.0
        delta_pose = self._motion.delta_pose
        for i, p in enumerate(self._particles):
            p.move(delta_pose)
            p.compute_weight(*self._laser.measurements)
            total_weight += p.weight

        # Low variance re-sampling of particles.
        new_particles = []
        random_weight = np.random.rand() * total_weight / self._num_particles
        current_boundary = self._particles[0].weight
        j = 0
        for m in range(len(self._particles)):
            next_boundary = random_weight + m * total_weight / self._num_particles
            while next_boundary > current_boundary:
                j = j + 1
                if j >= self._num_particles:
                    j = self._num_particles - 1
                current_boundary = current_boundary + self._particles[j].weight
            new_particles.append(copy.deepcopy(self._particles[j]))
        self._particles = new_particles

        # Publish particles.
        particle_msg = PointCloud()
        particle_msg.header.stamp = self.get_clock().now().to_msg()
        particle_msg.header.frame_id = "odom"
        intensity_channel = ChannelFloat32()
        intensity_channel.name = "intensity"
        particle_msg.channels.append(intensity_channel)
        for p in self._particles:
            pt = Point32()
            pt.x = float(p.pose[X])
            pt.y = float(p.pose[Y])
            pt.z = 0.05
            particle_msg.points.append(pt)
            intensity_channel.values.append(float(p.weight))
        self._particle_publisher.publish(particle_msg)

        # Log groundtruth and estimated positions in LOG_PATH
        poses = np.array([p.pose for p in self._particles], dtype=np.float32)
        median_pose = np.median(poses, axis=0)
        self._pose_history.append(
            np.concatenate([self._groundtruth.pose, median_pose], axis=0)
        )
        if len(self._pose_history) >= 10:
            with open(LOG_PATH, "a") as fp:
                fp.write(
                    "\n".join(",".join(str(v) for v in p) for p in self._pose_history)
                    + "\n"
                )
            self._pose_history = []
        self._frame_id += 1


def run(args):
    rclpy.init()
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description="Runs a particle filter (ROS 2 + Webots)"
    )
    args, _ = parser.parse_known_args()
    run(args)


if __name__ == "__main__":
    main()
