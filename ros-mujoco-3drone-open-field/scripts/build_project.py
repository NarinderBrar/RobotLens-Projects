#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from new_project import Robot, Spec, build

SPEC = Spec(
    root=Path(__file__).resolve().parents[1],
    communication="ros2",
    simulator="mujoco",
    copy={"robots/bebop2.urdf": "ros-mujoco-drone-open-field/robots/bebop2.urdf",
          "robots/meshes": "ros-mujoco-drone-open-field/robots/meshes"},
    robots=tuple(Robot(file="robots/bebop2.urdf", pose=(x, 0.0, 0.0),
                       joint_control={"state_endpoint": f"/drone{n}/joint_states"})
                 for n, x in ((1, -2.0), (2, 0.0), (3, 2.0))),
)

if __name__ == "__main__":
    sys.exit(build(SPEC))
