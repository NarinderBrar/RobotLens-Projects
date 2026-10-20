#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms='zenoh',
    simulator='mujoco',
    robots=('Mobile_manipulator', 'Mobile_manipulator_copy', 'Mobile_manipulator_copy_2', 'Mobile_manipulator_copy_3', 'Mobile_manipulator_copy_4'),
    requires=('worlds/warehouse.xml', 'robots/Mobile_manipulator.urdf', 'scripts/setup_fleet.sh'),
    prepare='scripts/setup_fleet.sh',
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
