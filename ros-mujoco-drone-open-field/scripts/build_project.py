#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from new_project import Robot, Spec, build

SPEC = Spec(
    root=Path(__file__).resolve().parents[1],
    communication="ros2",
    simulator="mujoco",
    robots=(Robot(file="robots/bebop2.urdf"),),
)

if __name__ == "__main__":
    sys.exit(build(SPEC))
