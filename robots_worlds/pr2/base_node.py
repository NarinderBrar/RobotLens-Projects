#!/usr/bin/env python3
"""
Base-only controller for the pr2 fixture (pr2.urdf, the same
flattened willowgarage/pr2_common description ../pr2 uses).

Unlike ../pr2/robot_node.py, this node does NOT animate the arms/head/torso/
grippers itself -- those are left for joint_state_publisher_gui to drive
interactively (see launch/pr2.launch.py), so a person can move
PR2's "hands" with sliders instead of watching a canned sine-wave idle. This
node owns only what joint_state_publisher_gui can't provide:

- Integrates /cmd_vel into an odom -> base_footprint transform (simple
  unicycle model, PR2's actual root link), published as both TF and
  nav_msgs/Odometry on /odom.
- Publishes a static map -> odom transform so 'map' is a real root frame.
- Publishes the base's 8 wheel-spin joints on /wheel_joint_states (NOT
  /joint_states directly) so they reflect actual commanded speed instead of
  sitting frozen or becoming another GUI slider. joint_state_publisher_gui is
  launched with source_list:=['wheel_joint_states'], which merges this
  topic's values into its own /joint_states output and skips creating
  sliders for exactly these joint names -- every other movable joint (torso,
  head, arms, grippers) still gets a slider. robot_state_publisher then
  resolves each gripper's 3 <mimic> finger joints from the one real
  *_gripper_l_finger_joint value the GUI publishes -- no need to handle
  those here either.

Everything else (bounded random-walk driving) is identical to ../pr2's
fixture: cmd_vel_commander.py is reused unchanged.

Message types exercised: nav_msgs/Odometry, sensor_msgs/JointState, tf2
(dynamic + static).
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

# "This is the 'effective' wheel radius. Wheel radius for uncompressed wheel
# is 0.079" -- pr2_description/urdf/base_v0/base.urdf.xacro's own comment.
WHEEL_RADIUS = 0.079
WHEEL_JOINTS = [
    f'{corner}_caster_{side}_wheel_joint'
    for corner in ('fl', 'fr', 'bl', 'br')
    for side in ('l', 'r')
]


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class Pr2ExampleBase(Node):
    def __init__(self):
        super().__init__('pr2_example_base')

        self.declare_parameter('publish_rate', 30.0)
        publish_rate = self.get_parameter('publish_rate').value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_map_to_odom()

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.wheel_joint_pub = self.create_publisher(JointState, '/wheel_joint_states', 10)

        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.wheel_angle = 0.0

        self.dt = 1.0 / publish_rate
        self.timer = self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f'pr2_example_base started (publish_rate={publish_rate} Hz)')

    def _publish_map_to_odom(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(t)

    def _cmd_vel_callback(self, msg):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    def _tick(self):
        now = self.get_clock().now().to_msg()

        # Unicycle-model integration -- same as ../pr2/robot_node.py.
        self.x += self.linear_vel * math.cos(self.theta) * self.dt
        self.y += self.linear_vel * math.sin(self.theta) * self.dt
        self.theta += self.angular_vel * self.dt

        # All 8 wheels spin at the same rate for the robot's overall speed --
        # not per-caster-accurate (each caster can also steer), but reads
        # correctly at a glance: faster driving, faster-spinning wheels,
        # stationary when stopped.
        self.wheel_angle += (self.linear_vel / WHEEL_RADIUS) * self.dt

        self._publish_odom_tf(now)
        self._publish_odometry(now)
        self._publish_wheel_joint_states(now)

    def _publish_odom_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = yaw_to_quaternion(self.theta)
        self.tf_broadcaster.sendTransform(t)

    def _publish_odometry(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = yaw_to_quaternion(self.theta)
        msg.twist.twist.linear.x = self.linear_vel
        msg.twist.twist.angular.z = self.angular_vel
        msg.pose.covariance[0] = 0.05   # x variance
        msg.pose.covariance[7] = 0.01   # y variance
        msg.pose.covariance[35] = 0.02  # yaw variance
        self.odom_pub.publish(msg)

    def _publish_wheel_joint_states(self, stamp):
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(WHEEL_JOINTS)
        msg.position = [self.wheel_angle] * len(WHEEL_JOINTS)
        self.wheel_joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Pr2ExampleBase()
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
