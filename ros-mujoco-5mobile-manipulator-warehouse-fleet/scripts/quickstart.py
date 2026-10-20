#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms='ros2',
    simulator='mujoco',
    robots=('Mobile_manipulator', 'Mobile_manipulator_copy', 'Mobile_manipulator_copy_2', 'Mobile_manipulator_copy_3', 'Mobile_manipulator_copy_4'),
    requires=('worlds/warehouse.xml', 'robots/Mobile_manipulator.urdf', 'scripts/setup_fleet.sh'),
    prepare='scripts/setup_fleet.sh',
    scenario=(
        {"step": "start_navigation", "robots": ['Mobile_manipulator', 'Mobile_manipulator_copy', 'Mobile_manipulator_copy_2', 'Mobile_manipulator_copy_3', 'Mobile_manipulator_copy_4']},
        {"step": "screenshot", "name": "before-goals"},
        {"step": "navigate", "tolerance": 0.35, "timeout": 360.0, "goals": [
            {"robot": "Mobile_manipulator", "x": 6.5, "y": 0, "yaw": 0, "truth": "robot1/base_footprint"},
            {"robot": "Mobile_manipulator_copy", "x": -3.6, "y": -1.5, "yaw": 1.5708, "truth": "robot2/base_footprint"},
            {"robot": "Mobile_manipulator_copy_2", "x": 3.6, "y": 1.5, "yaw": -1.5708, "truth": "robot3/base_footprint"},
            {"robot": "Mobile_manipulator_copy_3", "x": 1.0, "y": 4.5, "yaw": 0, "truth": "robot4/base_footprint"},
            {"robot": "Mobile_manipulator_copy_4", "x": -1.0, "y": -4.5, "yaw": 3.1416, "truth": "robot5/base_footprint"},
        ]},
        {"step": "screenshot", "name": "after-goals"},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
