#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

ROBOTS = ("Mobile_robot", "Mobile_robot_copy", "Mobile_robot_copy_2")
ROWS = (-2.5, -4.5, -6.75)

PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms="ros2",
    simulator="gazebo",
    robots=ROBOTS,
    requires=("worlds/open_interior.sdf", "robots/Mobile_robot.urdf",
              "nav2_config/nav2_robot_launch.py", "nav2_config/nav2_params.yaml",
              "nav2_config/nav2_map.yaml"),
    scenario=(
        {"step": "start_navigation", "robots": list(ROBOTS),
         "lifecycle_managers": [f"/robot{n}/lifecycle_manager_navigation"
                                for n in range(1, len(ROBOTS) + 1)]},
        {"step": "screenshot", "name": "before-goals"},
        {"step": "navigate", "tolerance": 0.35, "timeout": 300, "goals": [
            {"robot": robot, "x": -6.0, "y": y, "yaw": 0.0, "truth": f"robot{n}/base_footprint"}
            for n, (robot, y) in enumerate(zip(ROBOTS, ROWS), start=1)]},
        {"step": "screenshot", "name": "after-goals"},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
