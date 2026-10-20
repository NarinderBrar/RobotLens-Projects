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
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
