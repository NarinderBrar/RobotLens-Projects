#!/usr/bin/env python3
"""
Auto-commands /cmd_vel so the PR2 example robot drives itself instead of
sitting still — makes the TF tree and /odom visibly live without needing a
teleop tool. Identical to ../unified/cmd_vel_commander.py: this controller
only ever talks to /cmd_vel and /odom, so it's robot-agnostic.

Bounded random walk rather than a fixed circle: drives forward at a
constant speed, picking a new random turn rate every 1-3 seconds. Tracks
its own position via /odom feedback, and once it strays past
BOUNDARY_RADIUS from the origin, overrides the random turn with a simple
proportional heading controller that steers it back toward center — so it
wanders visibly around the grid without drifting off screen the way an
unconstrained random walk eventually would.

Message types exercised: geometry_msgs/msg/Twist (publish),
nav_msgs/msg/Odometry (subscribe, for boundary feedback).
"""

import math
import random

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

BOUNDARY_RADIUS = 2.5


def yaw_from_quaternion(q):
    # The robot only ever rotates about Z (see robot_node.py's
    # yaw_to_quaternion), so this is the plain 2D case, not a general
    # quaternion-to-Euler conversion.
    return math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class CmdVelCommander(Node):
    def __init__(self):
        super().__init__('cmd_vel_commander')

        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 0.6)
        self.linear_speed = self.get_parameter('linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.wander_angular = 0.0
        self.wander_timer = 0.0

        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'cmd_vel_commander started (bounded random walk, radius={BOUNDARY_RADIUS} m)')

    def _on_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def _tick(self):
        msg = Twist()
        msg.linear.x = self.linear_speed

        distance = math.hypot(self.x, self.y)
        if distance > BOUNDARY_RADIUS:
            desired_heading = math.atan2(-self.y, -self.x)
            heading_error = normalize_angle(desired_heading - self.yaw)
            msg.angular.z = max(-self.max_angular_speed,
                                min(self.max_angular_speed, 1.5 * heading_error))
        else:
            self.wander_timer -= 0.1
            if self.wander_timer <= 0.0:
                self.wander_angular = random.uniform(-self.max_angular_speed,
                                                      self.max_angular_speed)
                self.wander_timer = random.uniform(1.0, 3.0)
            msg.angular.z = self.wander_angular

        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelCommander()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
