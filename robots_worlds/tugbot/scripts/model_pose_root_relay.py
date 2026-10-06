#!/usr/bin/env python3
"""Republish Gazebo's true world pose for tugbot as odom->base_link.

Gazebo's DiffDrive plugin only publishes a 2D kinematic /odom estimate (roll
and pitch hardcoded to zero -- see gz-sim's DiffDrive.cc), so it cannot carry
real rigid-body dynamics like the chassis pitching over its caster wheels.
The PosePublisher plugin's publish_model_pose option (enabled in
tugbot/model.sdf's own plugin block) exposes the model's true physics-solved
world pose instead, bridged to ROS /tf as <world>->tugbot. This node relays
that transform as odom->base_link (tugbot's root link -- unlike TurtleBot3
it has no separate base_footprint) so RobotLens's existing TF-root-pose path
(Application::syncUrdfRootTransform) picks up the real 3D pose instead of
falling back to the flattened /odom.

Also republishes the same pose as a plain PoseStamped on /tugbot/true_pose.
/tf carries three interleaved publishers in this example (this relay,
ros_gz_bridge, robot_state_publisher), so a plotting tool that picks a
transform by raw array index (e.g. RobotLens's Plot panel) will pick a
different transform message-to-message -- /tugbot/true_pose is a single,
unambiguous topic for plotting this specific pose.
"""
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

SOURCE_CHILD_FRAME = "tugbot"
TARGET_FRAME_ID = "odom"
TARGET_CHILD_FRAME = "base_link"


class ModelPoseRootRelay(Node):
    def __init__(self):
        super().__init__("model_pose_root_relay")
        self.broadcaster = TransformBroadcaster(self)
        self.pose_publisher = self.create_publisher(PoseStamped, "/tugbot/true_pose", 10)
        self.create_subscription(TFMessage, "/tf", self.receive, 10)

    def receive(self, message):
        for transform in message.transforms:
            if transform.child_frame_id != SOURCE_CHILD_FRAME:
                continue
            out = TransformStamped()
            out.header.stamp = transform.header.stamp
            out.header.frame_id = TARGET_FRAME_ID
            out.child_frame_id = TARGET_CHILD_FRAME
            out.transform = transform.transform
            self.broadcaster.sendTransform(out)

            pose = PoseStamped()
            pose.header.stamp = transform.header.stamp
            pose.header.frame_id = TARGET_FRAME_ID
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            self.pose_publisher.publish(pose)


def main():
    rclpy.init()
    node = ModelPoseRootRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
