#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

JOINTS = [f"panda_joint{n}" for n in range(1, 8)]
READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
POSES = {"arm1": [1.2, -0.3, 0.4, -1.8, 0.3, 1.8, 0.0],
         "arm2": [-1.2, -0.3, 0.4, -1.8, 0.3, 1.8, 0.0]}
CAMERA = {"x": 0.0, "y": 0.0, "z": 0.4, "yaw": 0.0, "pitch": 20.0, "distance": 3.4}


def targets(poses: dict) -> list[dict]:
    return [{"label": arm, "gz_prefix": arm, "truth": f"/{arm}/joint_states",
             "joints": {"name": JOINTS, "position": pose}} for arm, pose in poses.items()]


PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms="ros2",
    simulator="gazebo",
    robots=("panda_arm1", "panda_arm2"),
    requires=("robots/panda_arm1.urdf", "robots/panda_arm2.urdf"),
    scenario=(
        {"step": "screenshot", "name": "before-motion", "camera": CAMERA},
        {"step": "joint_target", "targets": targets({"arm1": READY, "arm2": READY})},
        {"step": "screenshot", "name": "ready-pose", "camera": CAMERA},
        {"step": "joint_target", "targets": targets(POSES)},
        {"step": "screenshot", "name": "turned-pose", "camera": CAMERA},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
