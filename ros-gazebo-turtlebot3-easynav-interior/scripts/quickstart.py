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
    requires=('worlds/open_interior.sdf', 'easynav/file_urdf_turtlebot3_burger.params.yaml'),
    navigation=True,
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
