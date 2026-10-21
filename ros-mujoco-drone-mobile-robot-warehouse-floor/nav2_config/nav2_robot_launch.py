from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def launch_robot(context):
    namespace = LaunchConfiguration("namespace").perform(context).strip("/")
    params = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    common = {
        "use_sim_time": True,
        "global_frame": "map",
        "robot_base_frame": f"{namespace}/base_footprint",
        "odom_frame": f"{namespace}/odom",
        "odom_topic": f"/{namespace}/odom",
        "robot_radius": 0.65,
    }
    configured_params = RewrittenYaml(
        source_file=params,
        root_key=namespace,
        param_rewrites={
            "amcl.ros__parameters.base_frame_id": f"{namespace}/base_footprint",
            "amcl.ros__parameters.odom_frame_id": f"{namespace}/odom",
            "bt_navigator.ros__parameters.robot_base_frame": f"{namespace}/base_footprint",
            "bt_navigator.ros__parameters.odom_topic": f"/{namespace}/odom",
            "controller_server.ros__parameters.robot_base_frame": f"{namespace}/base_footprint",
            "local_costmap.local_costmap.ros__parameters.global_frame": f"{namespace}/odom",
            "local_costmap.local_costmap.ros__parameters.robot_base_frame": f"{namespace}/base_footprint",
            "local_costmap.local_costmap.ros__parameters.voxel_layer.scan.topic": f"/{namespace}/scan",
            "global_costmap.global_costmap.ros__parameters.robot_base_frame": f"{namespace}/base_footprint",
            "global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic": f"/{namespace}/scan",
            "behavior_server.ros__parameters.global_frame": "map",
            "behavior_server.ros__parameters.local_frame": f"{namespace}/odom",
            "behavior_server.ros__parameters.robot_base_frame": f"{namespace}/base_footprint",
        },
        convert_types=True,
    )
    nodes = [
        Node(package="tf2_ros", executable="static_transform_publisher", namespace=namespace,
             name="map_to_odom", arguments=["--x", "0", "--y", "0", "--yaw", "0",
             "--frame-id", "map", "--child-frame-id", f"{namespace}/odom"],
             parameters=[{"use_sim_time": True}]),
        Node(package="nav2_map_server", executable="map_server", namespace=namespace,
             name="map_server", parameters=[configured_params, common, {"yaml_filename": map_file}]),
        Node(package="nav2_planner", executable="planner_server", namespace=namespace,
             name="planner_server", parameters=[configured_params, common]),
        Node(package="nav2_controller", executable="controller_server", namespace=namespace,
             name="controller_server", parameters=[configured_params, common]),
        Node(package="nav2_behaviors", executable="behavior_server", namespace=namespace,
             name="behavior_server", parameters=[configured_params, common]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", namespace=namespace,
             name="bt_navigator", parameters=[configured_params, common]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", namespace=namespace,
             name="lifecycle_manager_navigation", parameters=[{"use_sim_time": True,
             "autostart": True, "node_names": ["map_server", "planner_server",
             "controller_server", "behavior_server", "bt_navigator"]}]),
    ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace"),
        DeclareLaunchArgument("params_file"),
        DeclareLaunchArgument("map"),
        DeclareLaunchArgument("start_x", default_value="0"),
        DeclareLaunchArgument("start_y", default_value="0"),
        DeclareLaunchArgument("start_yaw", default_value="0"),
        OpaqueFunction(function=launch_robot),
    ])
