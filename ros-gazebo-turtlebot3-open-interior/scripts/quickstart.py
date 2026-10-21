#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms='ros2',
    simulator='gazebo',
    robots=('turtlebot3_burger',),
    frames=('base_link',),
    requires=('worlds/open_interior.sdf', 'nav2_config/nav2_params.yaml'),
    navigation=True,
    scenario=(
        {"step": "start_navigation", "robots": ["turtlebot3_burger"]},
        {"step": "screenshot", "name": "before-goal"},
        {"step": "navigate", "goals": [{"robot": "turtlebot3_burger", "x": 1.5, "y": 0.0,
                                         "truth": "base_footprint"}]},
        {"step": "screenshot", "name": "after-goal"},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
