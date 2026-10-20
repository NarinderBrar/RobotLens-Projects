#!/usr/bin/env python3
"""Launch the RobotLens turtlebot3 fixture with Nav2 (see ../README.md).

Single launch that owns Gazebo, the ROS-Gazebo bridge, robot state
publishing, and the full Nav2 stack (a static map->odom transform plus
map_server/planner_server/controller_server/behavior_server/bt_navigator
under one lifecycle_manager, from ../nav2/nav2_params.yaml). Launch the
RobotLens desktop UI separately, use `View > Simulation > Attach` to connect
to it, drive the robot with in-app Teleop (Simulation panel > Teleop (Play
Mode)), and use `View > Navigation` to plan/navigate against the live Nav2
stack.

Usage:
  source /opt/ros/jazzy/setup.bash
  python3 examples/turtlebot3/launch/turtlebot3.launch.py
"""
import importlib.util
import os
import sys
import tempfile
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def prepare_world():
    """Generate the physics-enabled TurtleBot3 + open_interior world used by this launch."""
    runner_path = Path(__file__).resolve().parents[1] / "scripts" / "gazebo_runner.py"
    spec = importlib.util.spec_from_file_location("turtlebot3_gazebo_runner", runner_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load Gazebo runner: {runner_path}")
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    runner.write_open_interior_world()
    return runner.GENERATED_WORLD_PATH


def gazebo_environment():
    env = os.environ.copy()
    env.setdefault("HOME", str(Path.home()))
    env["PATH"] = "/usr/bin:/bin:" + env.get("PATH", "")
    # ROS Jazzy can replace Gazebo's command-plugin path with transport-only
    # entries, hiding `gz sim` and `gz sdf`.
    env["GZ_CONFIG_PATH"] = "/usr/share/gz"
    return env

def bridge_config():
    path = Path(tempfile.gettempdir()) / "robotlens_example1_external_gazebo_bridge.yaml"
    path.write_text("""- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS
- ros_topic_name: /cmd_vel
  gz_topic_name: /cmd_vel
  ros_type_name: geometry_msgs/msg/Twist
  gz_type_name: gz.msgs.Twist
  direction: ROS_TO_GZ
- ros_topic_name: /odom
  gz_topic_name: /model/simple_robot/odometry
  ros_type_name: nav_msgs/msg/Odometry
  gz_type_name: gz.msgs.Odometry
  direction: GZ_TO_ROS
- ros_topic_name: /tf
  gz_topic_name: /model/simple_robot/pose
  ros_type_name: tf2_msgs/msg/TFMessage
  gz_type_name: gz.msgs.Pose_V
  direction: GZ_TO_ROS
- ros_topic_name: /joint_states
  gz_topic_name: /world/open_interior/model/simple_robot/joint_state
  ros_type_name: sensor_msgs/msg/JointState
  gz_type_name: gz.msgs.Model
  direction: GZ_TO_ROS
""", encoding="utf-8")
    return path


def generate_launch_description():
    example_dir = Path(__file__).resolve().parent.parent
    urdf = example_dir / "robots" / "turtlebot3_burger.urdf"
    description = urdf.read_text(encoding="utf-8").replace(
        'filename="../meshes/', f'filename="{example_dir}/meshes/')
    world = prepare_world()
    bridge = bridge_config()

    nav2_dir = example_dir / "nav2"
    params_file = str(nav2_dir / "nav2_params.yaml")
    map_yaml = str(nav2_dir / "maps" / "open_interior.yaml")
    lifecycle_nodes = [
        "map_server",
        "planner_server",
        "controller_server",
        "behavior_server",
        "bt_navigator",
    ]

    return LaunchDescription([
        # --- Gazebo + robot state ---
        ExecuteProcess(
            cmd=["gz", "sim", "-s", "--headless-rendering", "-r", str(world)],
            additional_env=gazebo_environment(),
            output="screen",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": description,
                "use_sim_time": LaunchConfiguration("use_sim_time", default="true"),
            }],
            output="screen",
        ),
        # Expose the same open_interior SDF scene to RobotLens as the non-Gazebo
        # fixture; Gazebo loading it does not publish it through ROS.
        ExecuteProcess(
            cmd=[sys.executable, str(example_dir / "scripts" / "scene_description_node.py")],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
                 "-p", f"config_file:={bridge}"],
            output="screen",
        ),
        ExecuteProcess(
            cmd=[sys.executable, __file__, '--relay-node'],
            output="screen",
        ),
        # Exercises RobotLens's Diagnostics panel, which otherwise has nothing
        # to show for this fixture -- see diagnostics_node.py's docstring.
        ExecuteProcess(
            cmd=[sys.executable, str(example_dir / "scripts" / "diagnostics_node.py")],
            output="screen",
        ),

        # --- Nav2 ---
        # No AMCL on this fixture (no lidar): map and odom are declared
        # coincident so the global costmap/planner/bt_navigator have a valid
        # `map` frame without a real localizer. See ../README.md.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_static_tf",
            # Named form (not the deprecated 8-positional-arg form) so this
            # doesn't depend on which tf2_ros release is installed.
            arguments=["--x", "0", "--y", "0", "--z", "0", "--yaw", "0",
                       "--pitch", "0", "--roll", "0",
                       "--frame-id", "map", "--child-frame-id", "odom"],
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params_file, {"yaml_filename": map_yaml}],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[params_file],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                params_file,
                {"node_names": lifecycle_nodes},
            ],
        ),
    ])


def main():
    service = LaunchService(argv=sys.argv[1:])
    service.include_launch_description(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.abspath(__file__))))
    return service.run()


if __name__ == "__main__":
    if '--relay-node' in sys.argv:
        import rclpy
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from rclpy.node import Node as RosNode
        from tf2_msgs.msg import TFMessage
        from tf2_ros import TransformBroadcaster

        SOURCE_CHILD_FRAME = "simple_robot"
        TARGET_FRAME_ID = "odom"
        TARGET_CHILD_FRAME = "base_footprint"

        class ModelPoseRootRelay(RosNode):
            def __init__(self):
                super().__init__("model_pose_root_relay")
                self.broadcaster = TransformBroadcaster(self)
                self.pose_publisher = self.create_publisher(PoseStamped, "/simple_robot/true_pose", 10)
                self.create_subscription(TFMessage, "/tf", self.receive, 10)

            def receive(self, message):
                for transform in message.transforms:
                    if transform.child_frame_id != SOURCE_CHILD_FRAME:
                        continue
                    out = TransformStamped()
                    out.header.stamp = transform.header.stamp
                    out.header.frame_id = TARGET_FRAME_ID
                    out.child_frame_id = TARGET_CHILD_FRAME
                    out.transform = transform.transform
                    self.broadcaster.sendTransform(out)

                    pose = PoseStamped()
                    pose.header.stamp = transform.header.stamp
                    pose.header.frame_id = TARGET_FRAME_ID
                    pose.pose.position.x = transform.transform.translation.x
                    pose.pose.position.y = transform.transform.translation.y
                    pose.pose.position.z = transform.transform.translation.z
                    pose.pose.orientation = transform.transform.rotation
                    self.pose_publisher.publish(pose)

        rclpy.init()
        node = ModelPoseRootRelay()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        sys.exit(main())
