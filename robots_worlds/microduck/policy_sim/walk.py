#!/usr/bin/env python3
"""Drive the microduck MuJoCo model with Pollen Robotics' trained ONNX walking
policy, in a *physical* simulation: every tick sets position-actuator targets
and lets MuJoCo's forward dynamics (gravity, contacts, joint damping, the
Dynamixel-matched actuator gains already in microduck.xml) integrate real
motion. Nothing here writes qpos/joint angles directly -- there is no
kinematic replay, only forces and torques resolved by mj_step.

This reimplements the observation/action pipeline from the real robot's
firmware (../../../../microduck: duck-control/src/obs.rs, model.rs and
robotd/src/control.rs, policy.rs) against the simulated body instead of
Dynamixel servos and the LSM6DSV16X IMU. See those files for the underlying
contract this mirrors -- the 61-D observation layout, the 0.9 action scale,
and the head/leg low-pass filters all come from there, not from this script.

Usage:
    .venv/bin/python walk.py                       # walk forward
    .venv/bin/python walk.py --vx 0 --vyaw 0.6      # turn on the spot
    .venv/bin/python walk.py --vx 0 --vy 0 --vyaw 0 # stand (needs alpha_stand.onnx)
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
import onnxruntime as ort

# ---------------------------------------------------------------------------
# Robot tables, transcribed from duck-control/src/model.rs and obs.rs.
#
# JOINT_NAMES (duck_ipc_proto) with "mouth" dropped: this MJCF has no mouth
# joint at all, so the model's 14 actuators already ARE the policy's 14
# action slots, in this order -- no index remapping needed on the sim side
# (obs.rs's policy_joints()/scatter_action() exist only to skip a mouth slot
# that this model doesn't have).
# ---------------------------------------------------------------------------
POLICY_JOINTS = [
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
]
ACTION_LEN = len(POLICY_JOINTS)
assert ACTION_LEN == 14

# model.rs DEFAULT_POSITION, with the mouth entry (index 9) dropped -- same
# order as POLICY_JOINTS. The policy observes joint angles relative to this.
HOME_POSE = np.array(
    [
        0.0, -0.0873, -0.4579, -0.0049, 0.4530,  # left leg
        0.3491, 0.3491, 0.0, 0.0,  # neck / head
        0.0, 0.0873, 0.4579, 0.0049, -0.4530,  # right leg
    ],
    dtype=np.float64,
)

# control.rs HEAD_JOINTS (5..9) -- unchanged by the mouth drop, since it sits
# entirely below the removed index.
HEAD_JOINTS = range(5, 9)

OBS_LEN = 61
CONTROL_HZ = 50.0  # robotd's control loop rate
CONTROL_DT = 1.0 / CONTROL_HZ
STANDING_THRESHOLD = 0.05  # policy.rs DEFAULT_STANDING_THRESHOLD

ACTION_SCALE = 0.9  # Tuning::default().action_scale
STANDING_ACTION_SCALE = 1.0  # Tuning::default().standing_action_scale
HEAD_LOWPASS = 0.5  # Tuning::default().head_lowpass -- must match training
LEGS_LOWPASS = 0.7  # Tuning::default().legs_lowpass -- must match training

REPO_ROOT = Path(__file__).resolve().parents[3]  # .../RobotLens_2
DEFAULT_MODEL = REPO_ROOT / "robots_worlds" / "microduck" / "mujoco" / "microduck.xml"
DEFAULT_POLICIES_DIR = REPO_ROOT.parent / "microduck" / "policies"


def quat_rotate_inverse(quat_wxyz: np.ndarray, v: np.ndarray) -> np.ndarray:
    """World vector -> body frame. Same formula as duck-control's imu.rs rotate_inverse."""
    w, x, y, z = quat_wxyz
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    cx = y * tz - z * ty
    cy = z * tx - x * tz
    cz = x * ty - y * tx
    return np.array([vx - w * tx + cx, vy - w * ty + cy, vz - w * tz + cz])


class Policy:
    def __init__(self, path: Path):
        self.session = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"])

    def infer(self, obs: np.ndarray) -> np.ndarray:
        out = self.session.run(None, {"obs": obs.reshape(1, OBS_LEN).astype(np.float32)})
        return out[0][0]


class DuckSim:
    def __init__(self, model_path: Path):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)

        self.qpos_adr = np.array([self.model.joint(j).qposadr[0] for j in POLICY_JOINTS])
        self.dof_adr = np.array([self.model.joint(j).dofadr[0] for j in POLICY_JOINTS])
        self.act_id = np.array([self.model.actuator(j).id for j in POLICY_JOINTS])
        self.trunk_id = self.model.body("trunk_base").id

        stand_keyframe_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "STAND")
        if stand_keyframe_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, stand_keyframe_id)
        self.data.qpos[self.qpos_adr] = HOME_POSE
        self.data.ctrl[self.act_id] = HOME_POSE
        mujoco.mj_forward(self.model, self.data)

        self.last_action = np.zeros(ACTION_LEN, dtype=np.float32)
        self.previous_targets: np.ndarray | None = None

    def sensors(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """gyro (body frame), projected gravity (body frame), joint positions, joint velocities."""
        gravity = quat_rotate_inverse(self.data.xquat[self.trunk_id], np.array([0.0, 0.0, -1.0]))

        vel6 = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.trunk_id, vel6, 1)
        gyro = vel6[:3]

        positions = self.data.qpos[self.qpos_adr].copy()
        velocities = self.data.qvel[self.dof_adr].copy()
        return gyro, gravity, positions, velocities

    def build_observation(self, command: dict) -> np.ndarray:
        gyro, gravity, positions, velocities = self.sensors()

        obs = np.zeros(OBS_LEN, dtype=np.float32)
        obs[0:3] = gyro
        obs[3:6] = gravity
        obs[6:20] = positions - HOME_POSE
        obs[20:34] = velocities
        obs[34:48] = self.last_action
        # obs.rs's command block: twist(3), head(4), body x/y(zero, unbound),
        # body z/roll/pitch, body yaw(zero, unbound). Note the z, roll, pitch
        # order -- not z, pitch, roll.
        obs[48:51] = command["twist"]
        obs[51:55] = command["head"]
        obs[55] = 0.0  # body x -- unbound in training
        obs[56] = 0.0  # body y -- unbound
        obs[57] = command["body_z"]
        obs[58] = command["body_roll"]
        obs[59] = command["body_pitch"]
        obs[60] = 0.0  # body yaw -- unbound
        return obs

    def step(self, command: dict, walk_policy: Policy, stand_policy: Policy | None):
        twist_magnitude = float(np.linalg.norm(command["twist"]))
        standing = stand_policy is not None and twist_magnitude <= STANDING_THRESHOLD
        net = stand_policy if standing else walk_policy

        obs = self.build_observation(command)
        action = net.infer(obs)
        self.last_action = action

        scale = STANDING_ACTION_SCALE if standing else ACTION_SCALE
        targets = HOME_POSE + scale * action.astype(np.float64)

        if self.previous_targets is not None:
            for j in HEAD_JOINTS:
                targets[j] = HEAD_LOWPASS * targets[j] + (1.0 - HEAD_LOWPASS) * self.previous_targets[j]
            for j in range(ACTION_LEN):
                if j in HEAD_JOINTS:
                    continue
                targets[j] = LEGS_LOWPASS * targets[j] + (1.0 - LEGS_LOWPASS) * self.previous_targets[j]
        self.previous_targets = targets

        self.data.ctrl[self.act_id] = targets

        substeps = max(1, round(CONTROL_DT / self.model.opt.timestep))
        for _ in range(substeps):
            mujoco.mj_step(self.model, self.data)

        return "stand" if standing else "walk"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="MJCF to load")
    parser.add_argument("--policies-dir", type=Path, default=DEFAULT_POLICIES_DIR,
                         help="Directory holding the .onnx policy files")
    parser.add_argument("--walk-policy", default="alpha_walking.onnx")
    parser.add_argument("--stand-policy", default="alpha_stand.onnx",
                         help="Set to '' to disable standing and always run the walk policy")
    parser.add_argument("--vx", type=float, default=0.3, help="Forward command, m/s-ish (0.3-0.5 walks "
                         "reliably; below ~0.2 the policy barely marches in place; above ~0.8 it falls)")
    parser.add_argument("--vy", type=float, default=0.0, help="Lateral command")
    parser.add_argument("--vyaw", type=float, default=0.0, help="Yaw-rate command")
    parser.add_argument("--duration", type=float, default=None, help="Seconds to run; default forever")
    parser.add_argument("--no-viewer", action="store_true", help="Run headless (no MuJoCo viewer window)")
    args = parser.parse_args()

    if not args.model.is_file():
        raise SystemExit(f"model not found: {args.model}")

    walk_path = args.policies_dir / args.walk_policy
    if not walk_path.is_file():
        raise SystemExit(f"walk policy not found: {walk_path} (pass --policies-dir)")
    walk_policy = Policy(walk_path)

    stand_policy = None
    if args.stand_policy:
        stand_path = args.policies_dir / args.stand_policy
        if stand_path.is_file():
            stand_policy = Policy(stand_path)
        else:
            print(f"(no standing policy at {stand_path} -- will walk even at zero command)")

    sim = DuckSim(args.model)
    command = {
        "twist": np.array([args.vx, args.vy, args.vyaw], dtype=np.float64),
        "head": np.zeros(4, dtype=np.float64),
        "body_z": 0.0,
        "body_roll": 0.0,
        "body_pitch": 0.0,
    }

    def run_loop(sync_viewer=None):
        start = time.time()
        tick = 0
        while args.duration is None or time.time() - start < args.duration:
            if sync_viewer is not None and not sync_viewer.is_running():
                break
            tick_start = time.time()
            label = sim.step(command, walk_policy, stand_policy)
            if sync_viewer is not None:
                sync_viewer.sync()
            tick += 1
            if tick % (int(CONTROL_HZ) * 2) == 0:
                print(f"[{tick / CONTROL_HZ:6.1f}s] {label}")
            elapsed = time.time() - tick_start
            remaining = CONTROL_DT - elapsed
            if remaining > 0:
                time.sleep(remaining)

    try:
        if args.no_viewer:
            run_loop()
        else:
            with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
                run_loop(viewer)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
