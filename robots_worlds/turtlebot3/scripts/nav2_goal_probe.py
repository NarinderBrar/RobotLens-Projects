#!/usr/bin/env python3
import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

# Frame of the RobotLens virtual LiDAR on base_link (ROS2-Gazebo-Test project file).
SCAN_FRAME = "virtual_lidar"


class NavigationProbe(Node):
    def __init__(self, goal_x: float, goal_y: float, goal_yaw: float):
        super().__init__("robotlens_navigation_probe")
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_yaw = goal_yaw
        self.action = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_odom = None
        self.scan_count = 0
        self.scan_beams = 0
        self.path_points = 0
        self.global_costmap_cells = 0
        self.local_costmap_cells = 0
        self.create_subscription(Odometry, "/odom", self.receive_odom, 10)
        self.create_subscription(LaserScan, "/scan", self.receive_scan, 10)
        self.create_subscription(Path, "/plan", self.receive_path, 10)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.receive_global, 10)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.receive_local, 10)

    def receive_odom(self, message):
        self.last_odom = message

    def receive_scan(self, message):
        self.scan_count += 1
        self.scan_beams = max(self.scan_beams, len(message.ranges))

    def receive_path(self, message):
        self.path_points = max(self.path_points, len(message.poses))

    def receive_global(self, message):
        self.global_costmap_cells = max(self.global_costmap_cells, len(message.data))

    def receive_local(self, message):
        self.local_costmap_cells = max(self.local_costmap_cells, len(message.data))

    def pose(self, frame):
        try:
            transform = self.tf_buffer.lookup_transform("map", frame, rclpy.time.Time())
        except Exception:
            return None
        translation = transform.transform.translation
        return [translation.x, translation.y, translation.z]


def spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        value = predicate()
        if value:
            return value
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=1.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()
    rclpy.init()
    node = NavigationProbe(args.x, args.y, args.yaw)
    report = {"goal": [args.x, args.y, args.yaw], "succeeded": False}
    try:
        if not spin_until(node, lambda: node.action.server_is_ready(), 30.0):
            report["error"] = "navigate_to_pose action server unavailable"
            print(json.dumps(report), flush=True)
            return 2
        before_base = spin_until(node, lambda: node.pose("base_link"), 20.0)
        before_scan = spin_until(node, lambda: node.pose(SCAN_FRAME), 20.0)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.x
        goal.pose.pose.position.y = args.y
        goal.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(args.yaw / 2.0)
        sent = node.action.send_goal_async(goal)
        goal_handle = spin_until(node, lambda: sent.result() if sent.done() else None, 10.0)
        if goal_handle is None or not goal_handle.accepted:
            report["error"] = "navigation goal rejected"
            print(json.dumps(report), flush=True)
            return 3
        result_future = goal_handle.get_result_async()
        wrapped = spin_until(node, lambda: result_future.result() if result_future.done() else None,
                             args.timeout)
        if wrapped is None:
            report["error"] = "navigation timed out"
            print(json.dumps(report), flush=True)
            return 4
        after_base = node.pose("base_link")
        after_scan = node.pose(SCAN_FRAME)
        report.update({
            "action_status": wrapped.status,
            "succeeded": wrapped.status == 4,
            "base_before": before_base,
            "base_after": after_base,
            "scan_before": before_scan,
            "scan_after": after_scan,
            "scan_messages": node.scan_count,
            "scan_beams": node.scan_beams,
            "planned_path_points": node.path_points,
            "global_costmap_cells": node.global_costmap_cells,
            "local_costmap_cells": node.local_costmap_cells,
        })
        if after_base:
            report["goal_error_m"] = math.hypot(after_base[0] - args.x, after_base[1] - args.y)
        if after_base and after_scan:
            report["sensor_base_separation_m"] = math.dist(after_base, after_scan)
        print(json.dumps(report), flush=True)
        return 0 if report["succeeded"] else 5
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
