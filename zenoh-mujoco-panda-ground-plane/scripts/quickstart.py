#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

JOINTS = [f"panda_joint{n}" for n in range(1, 8)]
READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
TURNED = [1.2, -0.3, 0.4, -1.8, 0.3, 1.8, 0.0]
CAMERA = {"x": 0.0, "y": 0.0, "z": 0.4, "yaw": -60.0, "pitch": 25.0, "distance": 2.2}

PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms="zenoh",
    simulator="mujoco",
    robots=("panda",),
    requires=("robots/panda.urdf", "stacks/manipulation.json", "stacks/run_manipulation.sh"),
    stacks="stacks/run_manipulation.sh",
    scenario=(
        {"step": "bridge", "robots": ["panda"]},
        {"step": "wait_stacks", "prefix": "", "stacks": ["manipulation"]},
        {"step": "screenshot", "name": "before-motion", "camera": CAMERA},
        {"step": "joint_target", "prefix": "", "truth": "joint_states",
         "joints": {"name": JOINTS, "position": READY}},
        {"step": "screenshot", "name": "ready-pose", "camera": CAMERA},
        {"step": "joint_target", "prefix": "", "truth": "joint_states",
         "joints": {"name": JOINTS, "position": TURNED}},
        {"step": "screenshot", "name": "turned-pose", "camera": CAMERA},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
