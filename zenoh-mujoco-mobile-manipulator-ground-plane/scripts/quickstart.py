#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms='zenoh',
    simulator='mujoco',
    robots=('Mobile_manipulator',),
    requires=('robots/Mobile_manipulator.urdf', 'scripts/run_stacks.sh'),
    stacks="scripts/run_stacks.sh",
    scenario=(
        {"step": "bridge", "robots": ["Mobile_manipulator"]},
        {"step": "wait_stacks", "prefix": "", "stacks": ["navigation", "manipulation"]},
        {"step": "screenshot", "name": "before-scenario"},
        {"step": "navigate", "goals": [{"robot": "Mobile_manipulator", "prefix": "", "x": 1.0,
                                         "y": 0.0, "truth": "odom"}]},
        {"step": "joint_target", "prefix": "", "truth": "joint_states",
         "joints": {"name": ["arm_base_joint", "upper_arm_joint", "forearm_joint",
                             "wrist_roll_joint", "wrist_pitch_joint", "tool_flange_joint"],
                    "position": [0.8, 0.3, 0.5, 0.5, 0.4, 0.3]}},
        {"step": "screenshot", "name": "after-scenario"},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
