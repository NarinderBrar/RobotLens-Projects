#!/usr/bin/env python3
"""RobotLens quick-start wrapper for a lidar-free Nav2 bringup.

Deliberately bypasses nav2_bringup's bringup_launch.py: its default node set
always inserts velocity_smoother and collision_monitor between
controller_server and the final /cmd_vel. Without a lidar feeding real
observation data, that chain can hold the smoothed/checked velocity back
indefinitely -- Nav2 reports the navigate action as succeeding while the
robot never receives a nonzero /cmd_vel. This mirrors
robots_worlds/turtlebot3/launch/nav2_with_sim.launch.py's exact minimal node set,
which already drives correctly with no lidar.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
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
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_static_tf",
            arguments=["--x", "0", "--y", "0", "--z", "0", "--yaw", "0",
                       "--pitch", "0", "--roll", "0",
                       "--frame-id", "map", "--child-frame-id", "odom"],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time, "yaml_filename": map_yaml}],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time, "autostart": autostart}],
        ),
    ])
