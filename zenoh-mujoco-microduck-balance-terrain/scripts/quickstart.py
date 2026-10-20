#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms='zenoh',
    simulator='mujoco',
    requires=('src/microduck/mujoco/real/microduck_hybrid.xml', 'training/requirements.txt'),
    setup_steps=(('project Python environment ready', 'test -x .venv/bin/python || (python3 -m venv .venv && .venv/bin/python -m pip install -r training/requirements.txt)'),),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
