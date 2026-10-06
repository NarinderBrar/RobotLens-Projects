#!/usr/bin/env python3
"""Republish the PR2 physics fixture's /pr2/true_pose as /odom for Nav2.

The pr2 fixture has no DiffDrive-style odometry (its holonomic caster base is
driven by per-joint controllers), so there is no /odom topic for
nav2's controller_server, velocity_smoother, AMCL, and BT navigator to read.
`pr2_model_pose_root_relay.py` already publishes the true physics-solved
model pose on /pr2/true_pose (and as the odom->base_footprint transform);
this node simply wraps the same pose in a nav_msgs/Odometry message. Velocities
are left empty -- Nav2's controller needs positions from odom, and the DWB
local planner's pose/velocity feedback degrades gracefully with an empty twist.

Purely a pr2_classic glue node; the pr2 fixture itself deliberately does not
own a /odom topic.
"""
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Pr2ClassicOdom(Node):
    def __init__(self):
        super().__init__("pr2_classic_odom")
        self.publisher = self.create_publisher(Odometry, "/odom", 10)
        self.create_subscription(PoseStamped, "/pr2/true_pose", self.receive, 10)

    def receive(self, message):
        odom = Odometry()
        odom.header.stamp = message.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose = message.pose
        self.publisher.publish(odom)


def main():
    rclpy.init()
    node = Pr2ClassicOdom()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
