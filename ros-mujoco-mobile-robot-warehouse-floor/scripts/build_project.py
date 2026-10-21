#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from new_project import Lidar, Robot, Spec, build

FLEET = "ros-mujoco-5mobile-manipulator-warehouse-fleet"

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
    robots=(
        Robot(file="robots/Mobile_robot.urdf", pose=(-6.5, 0.0, 0.0),
              lidar=Lidar(link="lidar_link", topic="/robot1/scan", xyz=(0.0, 0.0, 0.04)),
              navigation={"stack_kind": "nav2", "namespace_prefix": "/robot1", "nav2": {
                  "launch_package": "", "launch_file": "nav2_config/nav2_robot_launch.py",
                  "map_yaml": "nav2_config/warehouse_navigation_map.yaml",
                  "params_yaml": "nav2_config/nav2_params.yaml",
                  "args": ["start_x:=-6.5", "start_y:=0.0", "start_yaw:=0.0"],
                  "attach_only": False}}),
    ),
)

if __name__ == "__main__":
    sys.exit(build(SPEC))
