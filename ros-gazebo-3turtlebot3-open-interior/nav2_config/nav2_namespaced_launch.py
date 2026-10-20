#!/usr/bin/env python3
"""Nav2 for one namespaced robot in a multi-robot simulation (generated for automation).

Arguments: namespace (tb1), map, params_file (fully qualified /<ns>/<node> keys), x, y, yaw
(the robot's spawn pose = its initial pose; published as static map -> <ns>/odom).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

NODES = ["map_server", "planner_server", "controller_server", "behavior_server",
         "bt_navigator"]


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = {"use_sim_time": True}
    args = [DeclareLaunchArgument(name, default_value=default) for name, default in (
        ("namespace", "tb1"), ("map", ""), ("params_file", ""), ("x", "0.0"), ("y", "0.0"),
        ("yaw", "0.0"))]
    odom_frame = PythonExpression(["'", namespace, "' + '/odom'"])
    nodes = [Node(package="tf2_ros", executable="static_transform_publisher",
                  name="map_to_odom_static_tf", namespace=namespace,
                  arguments=["--x", LaunchConfiguration("x"), "--y", LaunchConfiguration("y"),
                             "--yaw", LaunchConfiguration("yaw"), "--frame-id", "map",
                             "--child-frame-id", odom_frame],
                  parameters=[use_sim_time], output="screen")]
    packages = {"map_server": "nav2_map_server", "planner_server": "nav2_planner",
                "controller_server": "nav2_controller", "behavior_server": "nav2_behaviors",
                "bt_navigator": "nav2_bt_navigator"}
    for name in NODES:
        extra = {"yaml_filename": LaunchConfiguration("map")} if name == "map_server" else {}
        nodes.append(Node(package=packages[name], executable=name, name=name,
                          namespace=namespace, output="screen",
                          parameters=[params_file, use_sim_time, extra]))
    nodes.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                      name="lifecycle_manager_navigation", namespace=namespace, output="screen",
                      parameters=[use_sim_time, {"autostart": True, "node_names": NODES}]))
    return LaunchDescription(args + nodes)
