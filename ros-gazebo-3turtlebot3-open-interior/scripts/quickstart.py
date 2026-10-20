#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms='ros2',
    simulator='gazebo',
    robots=('tb1', 'tb2', 'tb3'),
    frames=('tb1/base_link', 'tb2/base_link', 'tb3/base_link'),
    requires=('worlds/open_interior.sdf', 'nav2_config/nav2_namespaced_launch.py'),
    before_simulation=({"move": "tb2", "x": 0.0, "y": 0.0},),
    scenario=(
        {"step": "launch", "path": "nav2_config/nav2_namespaced_launch.py", "name": "nav2_tb1",
         "args": ["namespace:=tb1", "map:=nav2_map.yaml", "params_file:=nav2_params_tb1.yaml",
                  "x:=-1.0", "y:=0.5", "yaw:=0.0"]},
        {"step": "start_servers", "robot": "tb1"},
        {"step": "start_servers", "robot": "tb2", "generate_easynav_config": True},
        {"step": "screenshot", "name": "before-goals"},
        {"step": "navigate", "tolerance": 0.3, "goals": [
            {"robot": "tb1", "x": -1.0, "y": 2.2, "yaw": 1.5708, "truth": "tb1/base_footprint"},
            {"robot": "tb2", "x": -1.0, "y": -2.2, "yaw": -1.5708, "via": "easynav", "ns": "tb2",
             "truth": "tb2/base_footprint"}]},
        {"step": "screenshot", "name": "after-goals"},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
