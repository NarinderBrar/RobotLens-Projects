#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from quickstart import Project, run

ROBOT, DRONE = "Mobile_robot", "bebop2"
ROBOT_MODEL = "Mobile_robot_robot1"
TWIST = "/drone/gazebo/command/twist"


def twist(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> dict:
    return {"topic": TWIST, "type": "gz.msgs.Twist",
            "payload": f"linear: {{x: {x}, y: {y}, z: {z}}}"}


def arm(enabled: bool) -> dict:
    return {"topic": "/drone/enable", "type": "gz.msgs.Boolean",
            "payload": f"data: {str(enabled).lower()}"}


PROJECT = Project(
    root=Path(__file__).resolve().parents[1],
    comms="ros2",
    simulator="gazebo",
    robots=(ROBOT, DRONE),
    requires=("worlds/open_interior.sdf", "robots/Mobile_robot.urdf", "robots/bebop2.urdf",
              "nav2_config/nav2_robot_launch.py"),
    view={"target": (-12.0, -3.8, 0.5), "yaw": -60.0, "pitch": 35.0, "distance": 6.0,
          "min_distance": 5.0},
    scenario=(
        {"step": "start_navigation", "robots": [ROBOT],
         "lifecycle_managers": ["/robot1/lifecycle_manager_navigation"]},
        {"step": "gz_motion", "publish": [arm(True), twist(z=0.6)], "stop": [twist()],
         "hold": 4.0, "check": [{"model": DRONE, "label": "took off", "min_z": 1.5}]},
        {"step": "screenshot", "name": "before-route"},
        {"step": "navigate", "tolerance": 0.35, "timeout": 300,
         "escort": {"drone": DRONE, "robot": ROBOT_MODEL, "twist_topic": TWIST,
                    "altitude": 2.5, "settle": 12.0},
         "goals": [{"robot": ROBOT, "x": -6.0, "y": -3.0, "yaw": 0.0,
                    "truth": "robot1/base_footprint"}]},
        {"step": "gz_motion", "publish": [], "hold": 2.0,
         "check": [{"model": DRONE, "label": "is above the robot at the route's end",
                    "near": ROBOT_MODEL, "max_gap": 1.0, "min_z": 1.5}]},
        {"step": "screenshot", "name": "after-route"},
        {"step": "gz_motion", "publish": [twist(z=-0.6)], "stop": [twist(), arm(False)],
         "hold": 8.0, "check": [{"model": DRONE, "label": "landed", "max_z": 0.3}]},
    ),
)

if __name__ == "__main__":
    sys.exit(run(PROJECT))
