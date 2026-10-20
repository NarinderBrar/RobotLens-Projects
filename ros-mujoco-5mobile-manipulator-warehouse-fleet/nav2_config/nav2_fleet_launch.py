from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    starts = (("robot1", -6.5, 0.0, 0.0), ("robot2", -3.6, -4.5, 1.57079632679), ("robot3", 3.6, 4.5, -1.57079632679), ("robot4", -6.5, 4.5, 0.0), ("robot5", 6.5, -4.5, 3.14159265359))
    nodes = [DeclareLaunchArgument("params_file"), DeclareLaunchArgument("map")]
    for namespace, x, y, yaw in starts:
        common = {"use_sim_time": True, "global_frame": "map", "robot_base_frame": f"{namespace}/base_footprint", "odom_frame": f"{namespace}/odom", "odom_topic": f"/{namespace}/odom", "robot_radius": 0.65}
        nodes.append(Node(package="tf2_ros", executable="static_transform_publisher", namespace=namespace, name="map_to_odom", arguments=["--x", str(x), "--y", str(y), "--yaw", str(yaw), "--frame-id", "map", "--child-frame-id", f"{namespace}/odom"], parameters=[{"use_sim_time": True}]))
        nodes.append(Node(package="nav2_map_server", executable="map_server", namespace=namespace, name="map_server", parameters=[params, common, {"yaml_filename": map_file}]))
        for package, executable, name in (("nav2_planner", "planner_server", "planner_server"), ("nav2_controller", "controller_server", "controller_server"), ("nav2_behaviors", "behavior_server", "behavior_server"), ("nav2_bt_navigator", "bt_navigator", "bt_navigator")):
            nodes.append(Node(package=package, executable=executable, namespace=namespace, name=name, parameters=[params, common]))
        nodes.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", namespace=namespace, name="lifecycle_manager_navigation", parameters=[{"use_sim_time": True, "autostart": True, "node_names": ["map_server", "planner_server", "controller_server", "behavior_server", "bt_navigator"]}]))
    return LaunchDescription(nodes)
