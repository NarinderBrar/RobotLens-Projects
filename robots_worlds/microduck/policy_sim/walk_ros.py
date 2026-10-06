#!/usr/bin/env python3
"""Drive the microduck loaded in RobotLens_2's own MuJoCo session with the
same trained ONNX walking policy walk.py uses standalone -- but over ROS2,
against RobotLens_2's physics instead of a separate local simulation.

This node does no physics stepping of its own: it reads the robot's live
state from RobotLens_2's MuJoCoActuatorBridge (src/simulation/
MuJoCoActuatorBridge.hpp) on /joint_states, and the auto-created "MuJoCo
IMU" virtual sensor (src/graphics/displays/VirtualSensorsDisplay.hpp) on
/virtual_imu/data, and writes actuator targets to /joint_command --
RobotLens_2's own MuJoCoBackend::stepSimulation() is what actually
integrates real dynamics each frame. There is exactly one simulation here,
running inside RobotLens_2; this script is just a controller sitting
outside its process boundary.

Prerequisites:
    - RobotLens_2 running with the microduck example loaded, the MuJoCo
      session Playing (see robots_worlds/microduck/mujoco/microduck.xml), and
      the Simulation panel's "Joint Interface" checkbox enabled.
    - The topic names match RobotLens_2's Simulation > Joint Interface fields.

Usage:
    .venv/bin/python walk_ros.py                  # walk forward
    .venv/bin/python walk_ros.py --vx 0 --vyaw 0.6 # turn on the spot
"""

from __future__ import annotations

import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

from walk import (
    ACTION_LEN,
    ACTION_SCALE,
    DEFAULT_POLICIES_DIR,
    HEAD_JOINTS,
    HEAD_LOWPASS,
    HOME_POSE,
    LEGS_LOWPASS,
    OBS_LEN,
    POLICY_JOINTS,
    STANDING_ACTION_SCALE,
    STANDING_THRESHOLD,
    Policy,
)


class RobotLensDuckController(Node):
    def __init__(self, walk_policy: Policy, stand_policy: Policy | None, command: dict, rate_hz: float,
                 imu_topic: str, joint_state_topic: str, joint_command_topic: str):
        super().__init__("microduck_walk_policy_bridge")

        self._walk_policy = walk_policy
        self._stand_policy = stand_policy
        self._command = command

        self._joint_position = dict.fromkeys(POLICY_JOINTS, 0.0)
        self._joint_velocity = dict.fromkeys(POLICY_JOINTS, 0.0)
        self._gyro = np.zeros(3)
        self._gravity = np.array([0.0, 0.0, -1.0])
        self._have_imu = False
        self._have_joints = False

        self._last_action = np.zeros(ACTION_LEN, dtype=np.float32)
        self._previous_targets: np.ndarray | None = None

        self.create_subscription(Imu, imu_topic, self._on_imu, 10)
        self.create_subscription(JointState, joint_state_topic, self._on_joint_states, 10)
        self._command_pub = self.create_publisher(JointState, joint_command_topic, 10)

        self._tick = 0
        self.create_timer(1.0 / rate_hz, self._on_tick)

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        w, x, y, z = q.w, q.x, q.y, q.z
        # World -> body: same quat_rotate_inverse formula walk.py uses,
        # inlined here since it's one call and avoids importing numpy-heavy
        # helpers twice.
        gx, gy, gz = 0.0, 0.0, -1.0
        tx = 2.0 * (y * gz - z * gy)
        ty = 2.0 * (z * gx - x * gz)
        tz = 2.0 * (x * gy - y * gx)
        cx = y * tz - z * ty
        cy = z * tx - x * tz
        cz = x * ty - y * tx
        self._gravity = np.array([gx - w * tx + cx, gy - w * ty + cy, gz - w * tz + cz])
        self._gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self._have_imu = True

    def _on_joint_states(self, msg: JointState) -> None:
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            if name in self._joint_position:
                self._joint_position[name] = pos
                self._joint_velocity[name] = vel
        self._have_joints = True

    def _on_tick(self) -> None:
        if not (self._have_imu and self._have_joints):
            return

        positions = np.array([self._joint_position[j] for j in POLICY_JOINTS])
        velocities = np.array([self._joint_velocity[j] for j in POLICY_JOINTS])

        twist_magnitude = float(np.linalg.norm(self._command["twist"]))
        standing = self._stand_policy is not None and twist_magnitude <= STANDING_THRESHOLD
        net = self._stand_policy if standing else self._walk_policy

        obs = np.zeros(OBS_LEN, dtype=np.float32)
        obs[0:3] = self._gyro
        obs[3:6] = self._gravity
        obs[6:20] = positions - HOME_POSE
        obs[20:34] = velocities
        obs[34:48] = self._last_action
        obs[48:51] = self._command["twist"]
        obs[51:55] = self._command["head"]
        obs[57] = self._command["body_z"]
        obs[58] = self._command["body_roll"]
        obs[59] = self._command["body_pitch"]

        action = net.infer(obs)
        self._last_action = action

        scale = STANDING_ACTION_SCALE if standing else ACTION_SCALE
        targets = HOME_POSE + scale * action.astype(np.float64)

        if self._previous_targets is not None:
            for j in HEAD_JOINTS:
                targets[j] = HEAD_LOWPASS * targets[j] + (1.0 - HEAD_LOWPASS) * self._previous_targets[j]
            for j in range(ACTION_LEN):
                if j in HEAD_JOINTS:
                    continue
                targets[j] = LEGS_LOWPASS * targets[j] + (1.0 - LEGS_LOWPASS) * self._previous_targets[j]
        self._previous_targets = targets

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(POLICY_JOINTS)
        msg.position = targets.tolist()
        self._command_pub.publish(msg)

        self._tick += 1
        if self._tick % 100 == 0:
            label = "stand" if standing else "walk"
            self.get_logger().info(f"tick={self._tick} {label}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--policies-dir", type=str, default=str(DEFAULT_POLICIES_DIR))
    parser.add_argument("--walk-policy", default="alpha_walking.onnx")
    parser.add_argument("--stand-policy", default="alpha_stand.onnx",
                         help="Set to '' to disable standing and always run the walk policy")
    parser.add_argument("--vx", type=float, default=0.3, help="Forward command, m/s-ish")
    parser.add_argument("--vy", type=float, default=0.0, help="Lateral command")
    parser.add_argument("--vyaw", type=float, default=0.0, help="Yaw-rate command")
    parser.add_argument("--rate", type=float, default=50.0, help="Control loop rate, Hz")
    parser.add_argument("--imu-topic", default="/virtual_imu/data")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--joint-command-topic", default="/joint_command")
    args = parser.parse_args()

    from pathlib import Path

    policies_dir = Path(args.policies_dir)
    walk_policy = Policy(policies_dir / args.walk_policy)
    stand_policy = None
    if args.stand_policy:
        stand_path = policies_dir / args.stand_policy
        if stand_path.is_file():
            stand_policy = Policy(stand_path)

    command = {
        "twist": np.array([args.vx, args.vy, args.vyaw], dtype=np.float64),
        "head": np.zeros(4, dtype=np.float64),
        "body_z": 0.0,
        "body_roll": 0.0,
        "body_pitch": 0.0,
    }

    rclpy.init()
    node = RobotLensDuckController(walk_policy, stand_policy, command, args.rate,
                                   args.imu_topic, args.joint_state_topic, args.joint_command_topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
