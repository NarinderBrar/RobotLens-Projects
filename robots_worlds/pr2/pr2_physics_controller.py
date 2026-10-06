#!/usr/bin/env python3
"""Convert ROS /cmd_vel into real PR2 wheel commands in Gazebo.

Skid-steer kinematics: as of 2026-08-15, run_pr2_gazebo.py's
add_caster_drive_controllers() welds all four casters forward-facing and
fixed instead of actively steering them (see that function's docstring for
the full story -- an unstable steering control loop, and separately a
graspable-object DetachableJoint that never reliably detached, were both
anchoring the base). This controller no longer computes a per-caster steer
angle; it just differentials wheel speed by side, like any other
skid-steer/differential-drive base. `linear.y` is ignored -- with no
steering DOF left, the base cannot strafe sideways (holonomic strafing was
already a Shift+Left/Right bonus in RobotLens's own teleop, not the primary
forward/back + turn interaction).

Confirmed live: forward and backward driving both work end to end through
this exact /cmd_vel -> gz-transport path. Turning (`angular.z`) does not --
tested at multiple speeds and floor-friction values, the base's measured
orientation never changes at all, even though wheel joint state shows some
wheels tracking their differential target. Still open; see
add_caster_drive_controllers()'s docstring for what's been ruled out.
"""
import rclpy
from geometry_msgs.msg import Twist
from gz.msgs10.double_pb2 import Double
from gz.transport13 import Node as GzNode
from rclpy.node import Node

WHEEL_RADIUS = 0.079
# y-offset of each corner from the base centerline; sign gives left/right side.
CASTERS = {'fl': 0.2246, 'fr': -0.2246, 'bl': 0.2246, 'br': -0.2246}
HALF_TRACK_WIDTH = 0.2246


class Pr2PhysicsController(Node):
    def __init__(self):
        super().__init__('pr2_physics_controller')
        self._gz = GzNode()
        self._command = Twist()
        self._wheel_publishers = [
            (y, self._gz.advertise(f'/model/pr2/joint/{corner}_caster_{side}_wheel_joint/cmd_vel', Double))
            for corner, y in CASTERS.items() for side in ('l', 'r')]
        self.create_subscription(Twist, '/cmd_vel', self._on_command, 10)
        self.create_timer(1.0 / 30.0, self._tick)

    def _on_command(self, command):
        self._command = command

    def _tick(self):
        left_speed = self._command.linear.x - self._command.angular.z * HALF_TRACK_WIDTH
        right_speed = self._command.linear.x + self._command.angular.z * HALF_TRACK_WIDTH
        for y, publisher in self._wheel_publishers:
            side_speed = left_speed if y > 0 else right_speed
            velocity = max(-12.0, min(side_speed / WHEEL_RADIUS, 12.0))
            publisher.publish(Double(data=velocity))


def main():
    rclpy.init()
    node = Pr2PhysicsController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
