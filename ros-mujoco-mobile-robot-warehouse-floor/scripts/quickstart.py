#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

ROBOT = "Mobile_robot"

PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms="ros2",
    simulator="mujoco",
    robots=(ROBOT,),
    requires=("worlds/warehouse.xml", "robots/Mobile_robot.urdf",
              "nav2_config/nav2_robot_launch.py", "nav2_config/nav2_params.yaml",
              "nav2_config/warehouse_navigation_map.yaml"),
    scenario=(
        {"step": "bridge", "robots": [ROBOT]},
        {"step": "start_navigation", "robots": [ROBOT],
         "lifecycle_managers": ["/robot1/lifecycle_manager_navigation"]},
        {"step": "screenshot", "name": "before-goal"},
        {"step": "navigate", "tolerance": 0.35, "timeout": 240, "goals": [
            {"robot": ROBOT, "x": -3.5, "y": 0.0, "yaw": 0.0, "truth": "robot1/base_footprint"}]},
        {"step": "screenshot", "name": "after-goal"},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
