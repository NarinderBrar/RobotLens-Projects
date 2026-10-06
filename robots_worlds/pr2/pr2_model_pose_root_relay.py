#!/usr/bin/env python3
"""Republish Gazebo's true world pose for the PR2 model as odom->base_footprint.

The PR2 physics fixture has no DiffDrive (its holonomic caster base is driven
by per-joint controllers, see run_pr2_gazebo.py's add_caster_drive_controllers),
so there is no /odom estimate to relay. The PosePublisher plugin's
publish_model_pose option exposes the model's true physics-solved world pose,
bridged to ROS /tf as <world>->pr2. This node relays that transform as
odom->base_footprint so RobotLens's TF-root-pose path
(Application::syncUrdfRootTransform / shouldUseOdomForRoot) picks up the real
3D pose instead of pinning the base at the origin. Mirrors the other
example fixtures' model_pose_root_relay.py for their bases.

Also republishes the same pose as a plain PoseStamped on /pr2/true_pose. /tf
carries three interleaved publishers in this fixture (this relay,
ros_gz_bridge, and the Gazebo PosePublisher itself), so a plotting tool that
picks a transform by raw array index (e.g. RobotLens's Plot panel) will pick a
different transform message-to-message -- /pr2/true_pose is a single,
unambiguous topic for plotting this specific pose.
"""
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster

SOURCE_CHILD_FRAME = "pr2"
TARGET_FRAME_ID = "odom"
TARGET_CHILD_FRAME = "base_footprint"


class Pr2ModelPoseRootRelay(Node):
    def __init__(self):
        super().__init__("pr2_model_pose_root_relay")
        self.broadcaster = TransformBroadcaster(self)
        self.pose_publisher = self.create_publisher(PoseStamped, "/pr2/true_pose", 10)
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
    node = Pr2ModelPoseRootRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
