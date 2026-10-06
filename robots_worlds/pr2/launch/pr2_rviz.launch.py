#!/usr/bin/env python3
"""
Static viewer for pr2 in RViz2: robot_state_publisher +
joint_state_publisher_gui (sliders for every movable joint, including the
wheels -- there's no base_node.py running here to drive them) + rviz2 with a
saved config. Independent of pr2.launch.py's base_node.py/
cmd_vel_commander.py pair -- this is for posing the model by hand, not
driving it around.

Usage:
  source /opt/ros/jazzy/setup.bash
  python3 robots_worlds/pr2/launch/pr2_rviz.launch.py
"""

import os
import sys
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    example_dir = Path(__file__).resolve().parents[1]
    urdf_path = example_dir / 'pr2.urdf'
    rviz_config_path = example_dir / 'rviz' / 'pr2.rviz'
    robot_description = urdf_path.read_text(encoding='utf-8')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(rviz_config_path)],
        ),
    ])


def main():
    launch_service = LaunchService(argv=sys.argv[1:])
    launch_service.include_launch_description(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.abspath(__file__)),
        )
    )
    return launch_service.run()


if __name__ == '__main__':
    sys.exit(main())
