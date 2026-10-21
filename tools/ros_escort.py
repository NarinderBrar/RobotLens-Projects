"""Keeps a ROS 2 drone above a ground robot until stopped; run with ROS sourced.

usage: ros_escort.py --drone-odom /drone/odom --robot-odom /robot1/odom --cmd /drone/cmd_vel

Both odometries must share a frame (MuJoCo publishes world odometry). At 10 Hz it commands the
drone's horizontal velocity toward the robot and its climb toward --altitude, with a proportional
gain and a speed limit, rotated into the drone's heading frame. SIGTERM sends a zero command.
"""

import argparse
import math
import signal

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--drone-odom", required=True)
    parser.add_argument("--robot-odom", required=True)
    parser.add_argument("--cmd", required=True)
    parser.add_argument("--altitude", type=float, default=2.5)
    parser.add_argument("--gain", type=float, default=0.8)
    parser.add_argument("--max-speed", type=float, default=1.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("robotlens_quickstart_escort")
    poses = {}

    def store(key):
        def callback(message):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            poses[key] = (p.x, p.y, p.z, yaw)
        return callback

    node.create_subscription(Odometry, args.drone_odom, store("drone"), 10)
    node.create_subscription(Odometry, args.robot_odom, store("robot"), 10)
    publisher = node.create_publisher(Twist, args.cmd, 10)
    stopping = []
    signal.signal(signal.SIGTERM, lambda *_: stopping.append(True))
    signal.signal(signal.SIGINT, lambda *_: stopping.append(True))

    def clamp(value):
        return max(-args.max_speed, min(args.max_speed, args.gain * value))

    while not stopping:
        rclpy.spin_once(node, timeout_sec=0.1)
        if "drone" not in poses or "robot" not in poses:
            continue
        dx, dy, dz, yaw = poses["drone"]
        rx, ry = poses["robot"][:2]
        wx, wy = clamp(rx - dx), clamp(ry - dy)
        command = Twist()
        command.linear.x = math.cos(yaw) * wx + math.sin(yaw) * wy
        command.linear.y = -math.sin(yaw) * wx + math.cos(yaw) * wy
        command.linear.z = clamp(args.altitude - dz)
        publisher.publish(command)
    for _ in range(3):
        publisher.publish(Twist())
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
