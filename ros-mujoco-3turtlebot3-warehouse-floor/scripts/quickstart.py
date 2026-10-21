#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

ROBOTS = ("turtlebot3_burger", "turtlebot3_burger_copy", "turtlebot3_burger_copy_2")
AISLES = (0.0, 4.5, -4.5)

PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms="ros2",
    simulator="mujoco",
    robots=ROBOTS,
    requires=("worlds/warehouse.xml", "robots/turtlebot3_burger.urdf",
              "nav2_config/nav2_robot_launch.py", "nav2_config/nav2_params.yaml",
              "nav2_config/warehouse_navigation_map.yaml"),
    scenario=(
        {"step": "bridge", "robots": list(ROBOTS)},
        {"step": "start_navigation", "robots": list(ROBOTS),
         "lifecycle_managers": [f"/robot{n}/lifecycle_manager_navigation"
                                for n in range(1, len(ROBOTS) + 1)]},
        {"step": "screenshot", "name": "before-goals"},
        {"step": "navigate", "tolerance": 0.35, "timeout": 300, "goals": [
            {"robot": robot, "x": -3.5, "y": y, "yaw": 0.0, "truth": f"robot{n}/base_footprint"}
            for n, (robot, y) in enumerate(zip(ROBOTS, AISLES), start=1)]},
        {"step": "screenshot", "name": "after-goals"},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
