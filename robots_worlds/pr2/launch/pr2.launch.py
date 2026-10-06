#!/usr/bin/env python3
"""
Launch file for the pr2 RobotLens example. Starts robot_state_publisher
against pr2.urdf (flattened from the real willowgarage/pr2_common
pr2_description package, same file ../../pr2 uses), plus:

- base_node.py: base TF/odom + wheel-joint spin only (see its own docstring).
  With no cmd_vel_commander.py running (see below), linear/angular velocity
  never leaves zero, so this just keeps the robot parked at the origin and
  supplies wheel_joint_states at a constant 0 rad -- no motion, no code
  change needed to get a fully static robot.
- joint_state_publisher (NOT the _gui variant): every other movable joint
  (torso lift, head pan/tilt, both 7-DOF arms, both grippers) defaults to
  its own zero/rest position with no animation and no interactive sliders --
  this is deliberately a static, unanimated view so link-to-link transforms
  can be inspected without motion as a confound. Use
  pr2_rviz.launch.py instead if you want interactive sliders to
  pose joints by hand.
- scripts/scene_description_node.py: publishes the shared open_interior
  warehouse SDF environment through `scene_models`, so RobotLens loads it next
  to PR2.

cmd_vel_commander.py is intentionally NOT run here (unlike ../../pr2's
fixture) -- see above.

The robot is fully kinematic: nothing here depends on Gazebo, and Gazebo
never sees this robot's joints or pose (see ../README.md's "Option A" note).
Run robots_worlds/pr2/gazebo/run_pr2_gazebo.sh if you also want an empty
Gazebo world running alongside for RobotLens's Simulation source to Attach to
(purely environmental -- the world has no model in it).

Usage:
  source /opt/ros/jazzy/setup.bash
  python3 robots_worlds/pr2/launch/pr2.launch.py
"""

import os
import sys
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def parse_launch_args(argv):
    """Parse 'name:=value' launch arguments from a plain argv list."""
    result = []
    for arg in argv:
        if arg.count(':=') == 1 and not arg.startswith(':=') and not arg.endswith(':='):
            name, _, value = arg.partition(':=')
            result.append((name, value))
    return result


def generate_launch_description():
    example_dir = Path(__file__).resolve().parents[1]
    urdf_file = example_dir / 'pr2.urdf'

    with open(urdf_file, 'r', encoding='utf-8') as f:
        robot_description = f.read()

    def run(*relative_path):
        return ExecuteProcess(
            cmd=[sys.executable, str(example_dir.joinpath(*relative_path))],
            output='screen',
        )

    # This fixture's scene publisher, so RobotLens renders the same
    # open_interior warehouse scene as the Gazebo fixture.
    warehouse_publisher = example_dir / 'scripts' / 'scene_description_node.py'

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        # Plain joint_state_publisher, no GUI: reads robot_description off
        # the /robot_description topic robot_state_publisher latches (no
        # explicit parameter needed here). source_list merges in base_node.py's
        # wheel values instead of defaulting them; every other joint sits at
        # its own zero/rest position with no animation.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'source_list': ['wheel_joint_states']}],
        ),
        run('base_node.py'),
        ExecuteProcess(
            cmd=[sys.executable, str(warehouse_publisher)],
            output='screen',
        ),
    ])


def main():
    launch_service = LaunchService(argv=sys.argv[1:])
    launch_service.include_launch_description(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.abspath(__file__)),
            launch_arguments=parse_launch_args(sys.argv[1:]),
        )
    )
    return launch_service.run()


if __name__ == '__main__':
    sys.exit(main())
