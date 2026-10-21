#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

DRONE = "bebop2"
VIEW = {"target": (0.0, 0.0, 0.5), "yaw": -60.0, "pitch": 20.0, "distance": 4.0, "min_distance": 2.5}


def twist(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> dict:
    return {"topic": "/drone/gazebo/command/twist", "type": "gz.msgs.Twist",
            "payload": f"linear: {{x: {x}, y: {y}, z: {z}}}"}


def arm(enabled: bool) -> dict:
    return {"topic": "/drone/enable", "type": "gz.msgs.Boolean",
            "payload": f"data: {str(enabled).lower()}"}


def leg(label: str, x: float, y: float) -> dict:
    return {"step": "gz_motion", "publish": [twist(x, y)], "stop": [twist()], "hold": 2.5,
            "check": [{"model": DRONE, "label": label, "min_move": 1.5, "min_z": 1.0}]}


PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms="ros2",
    simulator="gazebo",
    robots=(DRONE,),
    requires=("robots/bebop2.urdf",),
    view=VIEW,
    scenario=(
        {"step": "screenshot", "name": "before-flight"},
        {"step": "gz_motion", "publish": [arm(True), twist(z=0.6)], "repeat": 3,
         "stop": [twist()], "hold": 4.0,
         "check": [{"model": DRONE, "label": "took off", "min_z": 1.5}]},
        leg("flew +x", 0.8, 0.0),
        {"step": "screenshot", "name": "in-flight"},
        leg("flew +y", 0.0, 0.8),
        leg("flew -x", -0.8, 0.0),
        leg("flew -y", 0.0, -0.8),
        {"step": "gz_motion", "publish": [twist(z=-0.6)], "stop": [twist(), arm(False)],
         "hold": 14.0, "check": [{"model": DRONE, "label": "landed", "max_z": 0.3}]},
        {"step": "screenshot", "name": "after-flight"},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
