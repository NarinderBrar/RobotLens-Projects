#!/usr/bin/env python3
"""RobotLens quick-start wrapper for the standard Nav2 bringup."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory("nav2_bringup")

    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map", default_value=PathJoinSubstitution([ThisLaunchFileDir(), "map.yaml"])
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([ThisLaunchFileDir(), "nav2_params.yaml"]),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("autostart", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "map": map_yaml,
                "params_file": params_file,
                "use_sim_time": use_sim_time,
                "autostart": autostart,
            }.items(),
        ),
    ])
