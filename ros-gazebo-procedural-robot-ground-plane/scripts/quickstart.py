#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms='ros2',
    simulator='gazebo',
    robots=('Mobile_manipulator',),
    requires=('robots/Mobile_manipulator.urdf',),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
