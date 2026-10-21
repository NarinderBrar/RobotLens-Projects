#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from new_project import Lidar, Robot, Spec, build

GROUND = "zenoh-mujoco-mobile-robot-warehouse-floor"
DRONE = "ros-mujoco-drone-open-field"

SPEC = Spec(
    root=Path(__file__).resolve().parents[1],
    communication="zenoh",
    simulator="mujoco",
    copy={"robots/Mobile_robot.urdf": f"{GROUND}/robots/Mobile_robot.urdf",
          "robots/bebop2.urdf": f"{DRONE}/robots/bebop2.urdf",
          "robots/meshes": f"{DRONE}/robots/meshes",
          "worlds/warehouse.xml": f"{GROUND}/worlds/warehouse.xml",
          "maps/warehouse.json": f"{GROUND}/maps/warehouse.json",
          "stacks": f"{GROUND}/stacks",
          "scripts/generate_map.py": f"{GROUND}/scripts/generate_map.py",
          "scripts/publish_map.py": f"{GROUND}/scripts/publish_map.py"},
    world="worlds/warehouse.xml",
    robots=(
        Robot(file="robots/Mobile_robot.urdf", pose=(-6.5, 0.0, 0.0),
              lidar=Lidar(link="lidar_link", topic="robot1/scan", xyz=(0.0, 0.0, 0.04)),
              navigation={"stack_kind": "zenoh_external", "namespace_prefix": "robot1",
                          "zenoh_external": {"mode": "attach",
                                             "configuration": "stacks/navigation.json"}}),
        Robot(file="robots/bebop2.urdf", pose=(-7.6, 0.0, 0.0),
              joint_control={"state_endpoint": "/drone/joint_states"}),
    ),
)

if __name__ == "__main__":
    sys.exit(build(SPEC))
