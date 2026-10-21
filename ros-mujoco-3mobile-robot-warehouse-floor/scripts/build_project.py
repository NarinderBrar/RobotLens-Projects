#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from new_project import Lidar, Robot, Spec, build

FLEET = "ros-mujoco-5mobile-manipulator-warehouse-fleet"
AISLES = (0.0, 4.5, -4.5)


def robot(n: int, y: float) -> Robot:
    return Robot(file="robots/Mobile_robot.urdf", pose=(-6.5, y, 0.0),
                 lidar=Lidar(link="lidar_link", topic=f"/robot{n}/scan", xyz=(0.0, 0.0, 0.04)),
                 navigation={"stack_kind": "nav2", "namespace_prefix": f"/robot{n}", "nav2": {
                     "launch_package": "", "launch_file": "nav2_config/nav2_robot_launch.py",
                     "map_yaml": "nav2_config/warehouse_navigation_map.yaml",
                     "params_yaml": "nav2_config/nav2_params.yaml",
                     "args": ["start_x:=-6.5", f"start_y:={y}", "start_yaw:=0.0"],
                     "attach_only": False}})


SPEC = Spec(
    root=Path(__file__).resolve().parents[1],
    communication="ros2",
    simulator="mujoco",
    copy={"robots/Mobile_robot.urdf": "zenoh-mujoco-mobile-robot-warehouse-floor/robots/Mobile_robot.urdf",
          "worlds/warehouse.xml": "zenoh-mujoco-mobile-robot-warehouse-floor/worlds/warehouse.xml",
          "nav2_config/nav2_robot_launch.py": f"{FLEET}/nav2_config/nav2_robot_launch.py",
          "nav2_config/nav2_params.yaml": f"{FLEET}/nav2_config/nav2_params.yaml",
          "nav2_config/warehouse_navigation_map.yaml":
              f"{FLEET}/nav2_config/warehouse_navigation_map.yaml",
          "nav2_config/warehouse_navigation_map.pgm":
              f"{FLEET}/nav2_config/warehouse_navigation_map.pgm",
          "scripts/generate_warehouse_map.py": f"{FLEET}/scripts/generate_warehouse_map.py"},
    world="worlds/warehouse.xml",
    robots=tuple(robot(n, y) for n, y in enumerate(AISLES, start=1)),
)

if __name__ == "__main__":
    sys.exit(build(SPEC))
