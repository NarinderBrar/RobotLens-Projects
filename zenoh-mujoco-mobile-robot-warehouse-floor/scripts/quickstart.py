#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms='zenoh',
    simulator='mujoco',
    robots=('Mobile_robot',),
    requires=('worlds/warehouse.xml', 'robots/Mobile_robot.urdf', 'stacks/run_navigation.sh'),
    stacks="stacks/run_navigation.sh",
    scenario=(
        {"step": "bridge", "robots": ["Mobile_robot"]},
        {"step": "start_navigation", "robots": ["Mobile_robot"], "managed": False},
        {"step": "screenshot", "name": "before-goal"},
        {"step": "navigate", "goals": [{"robot": "Mobile_robot", "x": -3.0, "y": 0.0,
                                         "truth": "robot1/odom"}]},
        {"step": "screenshot", "name": "after-goal"},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
