#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from new_project import Robot, Spec, build

SPEC = Spec(
    root=Path(__file__).resolve().parents[1],
    communication="ros2",
    simulator="mujoco",
    copy={"robots/panda.urdf": "zenoh-mujoco-panda-ground-plane/robots/panda.urdf",
          "robots/meshes": "zenoh-mujoco-panda-ground-plane/robots/meshes"},
    robots=tuple(
        Robot(file="robots/panda.urdf", pose=(0.0, y, 0.0),
              joint_control={"state_endpoint": f"/arm{n}/joint_states"})
        for n, y in ((1, 0.8), (2, -0.8))),
)

if __name__ == "__main__":
    sys.exit(build(SPEC))
