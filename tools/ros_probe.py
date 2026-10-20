"""ROS 2 ground truth for quickstart scenarios; run with ROS sourced. Prints one JSON line.

pose   --parent map --frames base_footprint [...]   planar pose [x, y, yaw] of each frame
joints --topic /joint_states                        latest joint names and positions
"""

import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("pose", "joints"))
    parser.add_argument("--parent", default="map")
    parser.add_argument("--frames", nargs="+", default=["base_footprint"])
    parser.add_argument("--topic", default="/joint_states")
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node(f"robotlens_quickstart_probe_{args.mode}")
    try:
        print(json.dumps({"pose": pose, "joints": joints}[args.mode](node, args)))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
