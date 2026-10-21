#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from new_project import Lidar, Robot, Spec, build

ROWS = (-2.5, -4.5, -6.75)
TB3 = "ros-gazebo-3turtlebot3-open-interior"
MUJOCO = "ros-mujoco-turtlebot3-warehouse-floor"


def robot(n: int, y: float) -> Robot:
    return Robot(file="robots/Mobile_robot.urdf", pose=(-12.0, y, 0.0),
                 lidar=Lidar(link="lidar_link", topic=f"/robot{n}/scan", xyz=(0.0, 0.0, 0.04)),
                 navigation={"stack_kind": "nav2", "namespace_prefix": f"/robot{n}", "nav2": {
                     "launch_package": "", "launch_file": "nav2_config/nav2_robot_launch.py",
                     "map_yaml": "nav2_config/nav2_map.yaml",
                     "params_yaml": "nav2_config/nav2_params.yaml",
                     "args": ["start_x:=-12.0", f"start_y:={y}", "start_yaw:=0.0",
                              "robot_radius:=0.5"],
                     "attach_only": False}})


SPEC = Spec(
    root=Path(__file__).resolve().parents[1],
    communication="ros2",
    simulator="gazebo",
    copy={"robots/Mobile_robot.urdf": "zenoh-mujoco-mobile-robot-warehouse-floor/robots/Mobile_robot.urdf",
          "worlds/open_interior.sdf": f"{TB3}/worlds/open_interior.sdf",
          "nav2_config/nav2_map.yaml": f"{TB3}/nav2_config/nav2_map.yaml",
          "nav2_config/nav2_map.pgm": f"{TB3}/nav2_config/nav2_map.pgm",
          "nav2_config/nav2_robot_launch.py": f"{MUJOCO}/nav2_config/nav2_robot_launch.py",
          "nav2_config/nav2_params.yaml": f"{MUJOCO}/nav2_config/nav2_params.yaml"},
    world="worlds/open_interior.sdf",
    robots=tuple(robot(n, y) for n, y in enumerate(ROWS, start=1)),
)

if __name__ == "__main__":
    sys.exit(build(SPEC))
