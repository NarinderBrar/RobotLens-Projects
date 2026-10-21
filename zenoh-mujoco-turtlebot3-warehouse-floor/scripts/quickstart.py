#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

ROBOT = "turtlebot3_burger"

PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms="zenoh",
    simulator="mujoco",
    robots=(ROBOT,),
    requires=("worlds/warehouse.xml", "robots/turtlebot3_burger.urdf", "stacks/run_navigation.sh",
              "stacks/navigation.json"),
    stacks="stacks/run_navigation.sh",
    scenario=(
        {"step": "bridge", "robots": [ROBOT]},
        {"step": "start_navigation", "robots": [ROBOT], "managed": False},
        {"step": "screenshot", "name": "before-goal"},
        {"step": "navigate", "timeout": 120, "goals": [
            {"robot": ROBOT, "x": -4.0, "y": 0.0, "truth": "robot1/odom"}]},
        {"step": "screenshot", "name": "after-goal"},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
