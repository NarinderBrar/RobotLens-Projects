#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

DRONES = {1: "bebop2_1", 2: "bebop2_2", 3: "bebop2_3"}
POINTS = {1: (-0.8, 0.0), 2: (0.0, 0.8), 3: (0.8, 0.0)}
VIEW = {"target": (0.0, 0.0, 0.5), "yaw": -60.0, "pitch": 20.0, "distance": 6.0, "min_distance": 3.0}


def twist(n: int, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> dict:
    return {"topic": f"/drone{n}/gazebo/command/twist", "type": "gz.msgs.Twist",
            "payload": f"linear: {{x: {x}, y: {y}, z: {z}}}"}


def arm(n: int, enabled: bool) -> dict:
    return {"topic": f"/drone{n}/enable", "type": "gz.msgs.Boolean",
            "payload": f"data: {str(enabled).lower()}"}


PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms="ros2",
    simulator="gazebo",
    robots=tuple(DRONES.values()),
    requires=tuple(f"robots/bebop2_{n}.urdf" for n in DRONES),
    view=VIEW,
    scenario=(
        {"step": "screenshot", "name": "before-flight"},
        {"step": "gz_motion", "repeat": 2, "hold": 3.0,
         "publish": [m for n in DRONES for m in (arm(n, True), twist(n, z=0.6))],
         "stop": [twist(n) for n in DRONES],
         "check": [{"model": d, "label": "took off", "min_z": 1.5} for d in DRONES.values()]},
        {"step": "gz_motion", "hold": 2.5,
         "publish": [twist(n, *POINTS[n]) for n in DRONES],
         "stop": [twist(n) for n in DRONES],
         "check": [{"model": d, "label": "flew to its own hover point", "min_move": 1.0,
                    "min_z": 1.0} for d in DRONES.values()]},
        {"step": "screenshot", "name": "hovering"},
        {"step": "gz_motion", "hold": 14.0,
         "publish": [twist(n, z=-0.6) for n in DRONES],
         "stop": [m for n in DRONES for m in (twist(n), arm(n, False))],
         "check": [{"model": d, "label": "landed", "max_z": 0.3} for d in DRONES.values()]},
        {"step": "screenshot", "name": "after-flight"},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
