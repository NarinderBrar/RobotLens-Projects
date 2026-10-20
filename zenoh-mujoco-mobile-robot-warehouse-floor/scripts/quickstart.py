#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms='zenoh',
    simulator='mujoco',
    robots=('Mobile_robot',),
    requires=('worlds/warehouse.xml', 'robots/Mobile_robot.urdf', 'stacks/run_navigation.sh'),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
