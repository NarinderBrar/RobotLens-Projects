#!/usr/bin/env python3
"""Republish PR2 Gazebo diagnostics to ROS /rosout for RobotLens's Log panel."""
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node


IGNORABLE_PHYSICS_FAILURES = (
    'child link already has a parent joint',
    'Asked to create a joint between links',
    'mimic constraint',
)


class GazeboLogBridge(Node):
    def __init__(self, log_path):
        super().__init__('pr2_gazebo_log_bridge')
        self._path = Path(log_path)
        self._offset = 0
        self.create_timer(0.25, self._drain)
        self.get_logger().info(f'Forwarding Gazebo diagnostics from {self._path}')

    def _drain(self):
        if not self._path.exists():
            return
        with self._path.open('r', encoding='utf-8', errors='replace') as stream:
            stream.seek(self._offset)
            lines = stream.readlines()
            self._offset = stream.tell()
        for line in lines:
            message = line.strip()
            if not message:
                continue
            if any(pattern in message for pattern in IGNORABLE_PHYSICS_FAILURES):
                self.get_logger().warning(
                    'Gazebo skipped unsupported PR2 closed-loop/mimic physics: ' + message)
            elif '[Err]' in message or 'Error ' in message:
                self.get_logger().error('Gazebo: ' + message)
            elif '[Wrn]' in message or 'Warning ' in message:
                self.get_logger().warning('Gazebo: ' + message)


def main():
    if len(sys.argv) != 2:
        raise SystemExit('usage: gazebo_log_bridge.py GAZEBO_LOG_FILE')
    rclpy.init()
    node = GazeboLogBridge(sys.argv[1])
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
