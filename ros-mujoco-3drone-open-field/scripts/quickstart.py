#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

DRONES = {1: "bebop2", 2: "bebop2_copy", 3: "bebop2_copy_2"}
POINTS = {1: (-0.8, 0.0), 2: (0.0, 0.8), 3: (0.8, 0.0)}


def fly(label: str, twists: dict, hold: float, **check) -> dict:
    return {"step": "twist_motion", "hold": hold,
            "commands": [{"topic": f"/drone{n}/cmd_vel", "twist": twist}
                         for n, twist in twists.items()],
            "check": [{"robot": DRONES[n], "odom": f"/drone{n}/odom", "label": label, **check}
                      for n in DRONES]}


PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms="ros2",
    simulator="mujoco",
    robots=tuple(DRONES.values()),
    requires=("robots/bebop2.urdf",),
    view={"target": (0.0, 0.0, 0.5), "yaw": -90.0, "pitch": 15.0, "distance": 6.0,
          "min_distance": 3.0},
    scenario=(
        {"step": "bridge", "robots": list(DRONES.values())},
        {"step": "screenshot", "name": "before-flight"},
        fly("took off", {n: [0.0, 0.0, 0.6, 0.0] for n in DRONES}, 3.5, min_z=1.5),
        fly("flew to its own hover point",
            {n: [POINTS[n][0], POINTS[n][1], 0.0, 0.0] for n in DRONES}, 2.0,
            min_move=1.0, min_z=1.0),
        {"step": "screenshot", "name": "hovering"},
        fly("landed", {n: [0.0, 0.0, -0.6, 0.0] for n in DRONES}, 7.0, max_z=0.3),
        {"step": "screenshot", "name": "after-flight"},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
