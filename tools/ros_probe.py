"""ROS 2 ground truth for quickstart scenarios; run with ROS sourced. Prints one JSON line.

pose   --parent map --frames base_footprint [...]   planar pose [x, y, yaw] of each frame
joints --topic /joint_states                        latest joint names and positions
active --manager /robot1/lifecycle_manager_navigation  waits until the manager reports active
command --topic /cmd_joints --names a b --positions 0.1 0.2  publishes a JointState command
"""

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


def spin_until(node: Node, predicate, timeout: float):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        value = predicate()
        if value:
            return value
    return None


def pose(node: Node, args) -> dict:
    buffer = Buffer()
    TransformListener(buffer, node)

    def lookup(frame):
        try:
            t = buffer.lookup_transform(args.parent, frame, rclpy.time.Time())
        except Exception:  # noqa: BLE001 - not available yet
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z, yaw]

    spin_until(node, lambda: all(lookup(f) for f in args.frames), args.timeout)
    return {"frames": {f: lookup(f) for f in args.frames}}


def joints(node: Node, args) -> dict:
    latest = {}
    node.create_subscription(JointState, args.topic,
                             lambda m: latest.update(name=list(m.name), position=list(m.position)),
                             qos_profile_sensor_data)
    spin_until(node, lambda: latest, args.timeout)
    return latest


def active(node: Node, args) -> dict:
    client = node.create_client(Trigger, f"{args.manager}/is_active")
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=1.0):
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            if future.done() and future.result() is not None and future.result().success:
                return {"active": True}
        time.sleep(0.5)
    return {"active": False}


def command(node: Node, args) -> dict:
    publisher = node.create_publisher(JointState, args.topic, 10)
    message = JointState()
    message.name = list(args.names)
    message.position = [float(p) for p in args.positions]
    deadline = time.monotonic() + args.timeout
    sent = 0
    while time.monotonic() < deadline:
        message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)
        sent += 1
        rclpy.spin_once(node, timeout_sec=0.1)
    return {"sent": sent, "subscribers": publisher.get_subscription_count()}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("pose", "joints", "active", "command"))
    parser.add_argument("--parent", default="map")
    parser.add_argument("--frames", nargs="+", default=["base_footprint"])
    parser.add_argument("--topic", default="/joint_states")
    parser.add_argument("--manager", default="/lifecycle_manager_navigation")
    parser.add_argument("--names", nargs="*", default=[])
    parser.add_argument("--positions", nargs="*", default=[])
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node(f"robotlens_quickstart_probe_{args.mode}")
    try:
        print(json.dumps({"pose": pose, "joints": joints, "active": active, "command": command}[args.mode](node, args)))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
