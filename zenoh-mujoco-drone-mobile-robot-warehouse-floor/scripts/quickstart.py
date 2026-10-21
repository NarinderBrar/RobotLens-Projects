#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

ROBOT, DRONE = "Mobile_robot", "bebop2"


def fly(label: str, twist: list[float], hold: float, **check) -> dict:
    return {"step": "twist_motion", "hold": hold,
            "commands": [{"key": "drone/cmd_vel", "twist": twist}],
            "check": [{"robot": DRONE, "odom": "drone/odom", "label": label, **check}]}


PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms="zenoh",
    simulator="mujoco",
    robots=(ROBOT, DRONE),
    requires=("worlds/warehouse.xml", "robots/Mobile_robot.urdf", "robots/bebop2.urdf"),
    stacks="stacks/run_navigation.sh",
    view={"target": (-7.0, 0.0, 0.5), "yaw": -60.0, "pitch": 35.0, "distance": 6.0,
          "min_distance": 5.0},
    scenario=(
        {"step": "bridge", "robots": [ROBOT, DRONE]},
        {"step": "start_navigation", "robots": [ROBOT], "managed": False},
        fly("took off", [0.0, 0.0, 0.8, 0.0], 4.0, min_z=2.0),
        {"step": "screenshot", "name": "before-route"},
        {"step": "navigate", "tolerance": 0.35, "timeout": 240,
         "escort": {"drone_odom": "drone/odom", "robot_odom": "robot1/odom",
                    "cmd": "drone/cmd_vel", "altitude": 3.0, "settle": 8.0},
         "goals": [{"robot": ROBOT, "x": -3.5, "y": 0.0, "yaw": 0.0,
                    "truth": "robot1/odom"}]},
        {"step": "twist_motion", "hold": 1.0,
         "check": [{"robot": DRONE, "odom": "drone/odom",
                    "label": "is above the robot at the route's end",
                    "near": "robot1/odom", "max_gap": 1.0, "min_z": 2.0}]},
        {"step": "screenshot", "name": "after-route"},
        fly("moved clear of the robot", [-0.8, 0.0, 0.0, 0.0], 2.0, min_move=1.2, min_z=2.0),
        fly("landed", [0.0, 0.0, -0.6, 0.0], 9.0, max_z=0.3),
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
