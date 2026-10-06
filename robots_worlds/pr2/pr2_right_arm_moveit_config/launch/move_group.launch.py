#!/usr/bin/env python3
"""Bring up MoveIt 2's move_group for the P2 right-arm/right-gripper planning
groups (see docs/IMPLEMENTATION_PLAN_PR2_MOVEIT_PICK_PLACE.md).

Run robots_worlds/pr2/launch/pr2_robotlens_gazebo.launch.py first
so /joint_states, /clock, and the P1 trajectory/gripper action adapter
(pr2_arm_trajectory_controller.py) are live; move_group's controller manager
executes purely through those actions, never through a direct Gazebo topic.

This package must be colcon-built and sourced before this launch file can
resolve its own share directory. It lives under robots_worlds/pr2/
(a dependency of that fixture, not runnable standalone) -- colcon's
default recursive discovery from the repo root does not descend into
robots_worlds/, so `--paths` is required, not `--packages-select`:

  source /opt/ros/jazzy/setup.bash
  colcon build --paths robots_worlds/pr2/pr2_right_arm_moveit_config
  source install/setup.bash
  python3 robots_worlds/pr2/pr2_right_arm_moveit_config/launch/move_group.launch.py
"""
import os
import sys
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build_moveit_config():
    """Assemble MoveIt parameters from this package's config/ and the
    single-source-of-truth pr2.urdf in the parent pr2 fixture --
    never a private copy (plan section 4, P3 "one source of truth")."""
    example2_dir = Path(__file__).resolve().parents[2]
    return (
        MoveItConfigsBuilder("pr2_right_arm", package_name="pr2_right_arm_moveit_config")
        .robot_description(file_path=example2_dir / "pr2.urdf")
        .robot_description_semantic(file_path="config/pr2_right_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            # RESOLVED 2026-08-15: publish_robot_description defaults to
            # False in moveit_configs_utils' MoveItConfigsBuilder. With it
            # False, move_group never populates its own local
            # `robot_description` parameter from the URDF loaded above --
            # it instead tries to fetch it remotely from a node literally
            # named `robot_state_publisher`, which doesn't exist in this
            # fixture (deliberately renamed to `pr2_tf_publisher`, see this
            # package's own docstring). That remote fetch retries forever
            # ("Failed to get parameters: parameter 'robot_description' is
            # not initialized", logged every ~2s), leaving `robot_
            # description` permanently unset and RobotLens's MoveItAdapter
            # stuck reporting 0 planning groups / 0 end effectors forever
            # (confirmed live: `ros2 param get /move_group
            # robot_description` returned "Parameter not set" without this).
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )


def generate_launch_description():
    moveit_config = build_moveit_config()
    move_group_params = [
        moveit_config.to_dict(),
        {
            # Simulation time comes from the bridged Gazebo /clock (plan
            # section 5: MoveIt must follow the same clock as the rest of
            # the fixture, not wall time).
            "use_sim_time": True,
            "publish_robot_description_semantic": True,
            "allow_trajectory_execution": True,
            "moveit_manage_controllers": True,
        },
    ]
    return LaunchDescription([
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=move_group_params,
        ),
    ])


def main():
    service = LaunchService(argv=sys.argv[1:])
    service.include_launch_description(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.abspath(__file__))))
    return service.run()


if __name__ == "__main__":
    sys.exit(main())
