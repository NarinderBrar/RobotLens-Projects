#!/usr/bin/env python3
"""Diagnostics fixture for RobotLens Example 1's Gazebo launch.

Nothing in example1_robotlens_gazebo(_gui).launch.py publishes
diagnostic_msgs/msg/DiagnosticArray on /diagnostics -- gz sim,
robot_state_publisher, ros_gz_bridge, and this example's own custom scripts
never touch it -- so RobotLens's Diagnostics panel has nothing to show when
running that fixture. This node exists purely to give the panel something
real to display, using diagnostic_updater (the same library a real robot
stack would use) wired up against this example's own topics:

- "/joint_states" and "/odom" frequency checks: go bad if Gazebo, the
  bridge, or DiffDrive stop publishing (e.g. kill `gz sim` while this node
  keeps running and watch both go to ERROR).
- "Teleop (cmd_vel)": informational -- reports whether RobotLens's in-app
  teleop is currently sending /cmd_vel commands.
- "Battery": a synthetic sine-wave signal, not a real battery model. It
  exists purely so the panel visibly cycles through OK -> WARN -> ERROR ->
  WARN -> OK on a short, predictable period, without needing to wait for a
  real fault or drain a real battery.

Launched automatically by example1_robotlens_gazebo.launch.py (see its
generate_launch_description). Can also be run standalone in its own
terminal against an already-running fixture:

    source /opt/ros/jazzy/setup.bash
    python3 robots_worlds/example1/diagnostics_node.py

Requires the ros-jazzy-diagnostic-updater apt package (not part of a
typical ROS 2 desktop or Gazebo install, unlike robot_state_publisher/
ros_gz_bridge):

    sudo apt install ros-jazzy-diagnostic-updater
"""
import math
import time

import diagnostic_msgs.msg
import diagnostic_updater
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

HARDWARE_ID = "simple_robot"

# Demo-only synthetic battery signal (see module docstring) -- a full
# 0..100 sine sweep every BATTERY_CYCLE_SECONDS, so both thresholds below
# get crossed twice per cycle.
BATTERY_CYCLE_SECONDS = 90.0
BATTERY_WARN_PERCENT = 30.0
BATTERY_ERROR_PERCENT = 10.0

CMD_VEL_IDLE_SECONDS = 2.0
# Broad on purpose: this checks "did the topic go completely silent" (a
# dead bridge/sim), not "is it publishing at its usual rate" -- this
# example's actual /joint_states and /odom rates depend on Gazebo's physics
# step size, which this fixture doesn't pin down (observed ~965 Hz for
# /joint_states, physics-step-rate, well under this ceiling).
TOPIC_FREQUENCY_BOUNDS = {"min": 1.0, "max": 5000.0}


class DiagnosticsNode(Node):
    def __init__(self):
        super().__init__("example1_diagnostics")
        self._start_time = time.monotonic()
        self._last_cmd_vel_time = None

        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID(HARDWARE_ID)
        self.updater.add("Battery", self._battery_diagnostic)
        self.updater.add("Teleop (cmd_vel)", self._cmd_vel_diagnostic)

        self._joint_states_freq = diagnostic_updater.HeaderlessTopicDiagnostic(
            "/joint_states", self.updater,
            diagnostic_updater.FrequencyStatusParam(TOPIC_FREQUENCY_BOUNDS, 0.3, 5))
        self._odom_freq = diagnostic_updater.HeaderlessTopicDiagnostic(
            "/odom", self.updater,
            diagnostic_updater.FrequencyStatusParam(TOPIC_FREQUENCY_BOUNDS, 0.3, 5))

        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

    def _on_joint_states(self, _msg):
        self._joint_states_freq.tick()

    def _on_odom(self, _msg):
        self._odom_freq.tick()

    def _on_cmd_vel(self, _msg):
        self._last_cmd_vel_time = time.monotonic()

    def _battery_diagnostic(self, stat):
        phase = (time.monotonic() - self._start_time) / BATTERY_CYCLE_SECONDS
        percent = 50.0 + 50.0 * math.sin(2.0 * math.pi * phase)
        if percent < BATTERY_ERROR_PERCENT:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Battery critically low")
        elif percent < BATTERY_WARN_PERCENT:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, "Battery low")
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Battery nominal")
        stat.add("Percent", "%.1f" % percent)
        return stat

    def _cmd_vel_diagnostic(self, stat):
        if self._last_cmd_vel_time is None:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "No teleop input yet")
            return stat
        idle_seconds = time.monotonic() - self._last_cmd_vel_time
        if idle_seconds > CMD_VEL_IDLE_SECONDS:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                        "Idle (%.1fs since last command)" % idle_seconds)
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, "Receiving teleop commands")
        stat.add("Seconds since last cmd_vel", "%.1f" % idle_seconds)
        return stat


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
