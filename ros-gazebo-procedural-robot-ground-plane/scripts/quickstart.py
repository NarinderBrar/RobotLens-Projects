#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

ROBOT = "Autonomous_mobile_robot"
ARM = "Mobile_manipulator"


def wheels(speed: float) -> list[dict]:
    return [{"topic": f"{ROBOT}/{side}_wheel_joint/cmd_vel", "payload": f"data: {speed}"}
            for side in ("left", "right")]


PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms='ros2',
    simulator='gazebo',
    robots=(ROBOT, ARM),
    requires=('robots/Autonomous_mobile_robot.sdf', 'robots/Mobile_manipulator.urdf'),
    scenario=(
        {"step": "screenshot", "name": "before-motion"},
        {"step": "gz_motion", "publish": wheels(4.0), "stop": wheels(0.0), "hold": 4.0,
         "check": [{"model": ROBOT, "label": "drove forward", "min_move": 0.3}]},
        {"step": "gz_motion", "hold": 4.0,
         "publish": [{"topic": f"{ARM}/arm_base_joint/cmd_pos", "payload": "data: 1.2"},
                     {"topic": f"{ARM}/upper_arm_joint/cmd_pos", "payload": "data: 0.6"}],
         "check": [{"model": ARM, "link": "arm_base", "label": "arm base yawed to 1.2 rad",
                    "axis": "yaw", "min_turn": 1.1},
                   {"model": ARM, "link": "upper_arm", "label": "upper arm pitched to 0.6 rad",
                    "axis": "pitch", "min_turn": 0.5}]},
        {"step": "screenshot", "name": "after-motion"},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
