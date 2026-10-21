#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

DRONE = "bebop2"
COMMAND, ODOM = "/cmd_vel", "/odom"


def fly(label: str, twist: list[float], hold: float, **check) -> dict:
    return {"step": "twist_motion", "hold": hold,
            "commands": [{"topic": COMMAND, "twist": twist}],
            "check": [{"robot": DRONE, "odom": ODOM, "label": label, **check}]}


PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms="ros2",
    simulator="mujoco",
    robots=(DRONE,),
    requires=("robots/bebop2.urdf",),
    view={"target": (0.0, 0.0, 0.5), "yaw": -60.0, "pitch": 20.0, "distance": 4.0,
          "min_distance": 2.5},
    scenario=(
        {"step": "bridge", "robots": [DRONE]},
        {"step": "screenshot", "name": "before-flight"},
        fly("took off", [0.0, 0.0, 0.6, 0.0], 3.5, min_z=1.5),
        fly("flew +x", [0.8, 0.0, 0.0, 0.0], 2.5, min_move=1.5, min_z=1.0),
        {"step": "screenshot", "name": "in-flight"},
        fly("flew +y", [0.0, 0.8, 0.0, 0.0], 2.5, min_move=1.5, min_z=1.0),
        fly("flew -x", [-0.8, 0.0, 0.0, 0.0], 2.5, min_move=1.5, min_z=1.0),
        fly("flew -y", [0.0, -0.8, 0.0, 0.0], 2.5, min_move=1.5, min_z=1.0),
        fly("landed", [0.0, 0.0, -0.6, 0.0], 7.0, max_z=0.3),
        {"step": "screenshot", "name": "after-flight"},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
