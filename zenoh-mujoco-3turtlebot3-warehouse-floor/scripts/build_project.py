#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from new_project import Lidar, Robot, Spec, build

AISLES = (0.0, 4.5, -4.5)

SPEC = Spec(
    root=Path(__file__).resolve().parents[1],
    communication="zenoh",
    simulator="mujoco",
    copy={"robots/turtlebot3_burger.urdf": "robots_worlds/turtlebot3/robots/turtlebot3_burger.urdf",
          "meshes": "robots_worlds/turtlebot3/meshes",
          "worlds/warehouse.xml": "zenoh-mujoco-mobile-robot-warehouse-floor/worlds/warehouse.xml"},
    world="worlds/warehouse.xml",
    robots=tuple(
        Robot(file="robots/turtlebot3_burger.urdf", pose=(-6.5, y, 0.0),
              lidar=Lidar(link="base_scan", topic=f"robot{n}/scan", xyz=(0.0, 0.0, 0.02)),
              navigation={"stack_kind": "zenoh_external", "namespace_prefix": f"robot{n}",
                          "zenoh_external": {"mode": "attach",
                                             "configuration": f"stacks/navigation-robot{n}.json"}})
        for n, y in enumerate(AISLES, start=1)),
)

if __name__ == "__main__":
    sys.exit(build(SPEC))
