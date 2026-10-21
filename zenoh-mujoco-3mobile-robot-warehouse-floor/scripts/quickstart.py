#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

ROBOTS = ("Mobile_robot", "Mobile_robot_copy", "Mobile_robot_copy_2")
AISLES = (0.0, 4.5, -4.5)

PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms="zenoh",
    simulator="mujoco",
    robots=ROBOTS,
    requires=("worlds/warehouse.xml", "robots/Mobile_robot.urdf", "stacks/run_fleet.sh",
              "stacks/navigation-robot1.json", "stacks/navigation-robot2.json",
              "stacks/navigation-robot3.json"),
    stacks="stacks/run_fleet.sh",
    scenario=(
        {"step": "bridge", "robots": list(ROBOTS)},
        {"step": "start_navigation", "robots": list(ROBOTS), "managed": False},
        {"step": "screenshot", "name": "before-goals"},
        {"step": "navigate", "timeout": 180, "goals": [
            {"robot": robot, "x": -2.5, "y": y, "truth": f"robot{n}/odom"}
            for n, (robot, y) in enumerate(zip(ROBOTS, AISLES), start=1)]},
        {"step": "screenshot", "name": "after-goals"},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
