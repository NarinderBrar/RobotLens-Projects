#!/usr/bin/env python3
"""Launch the Gazebo Harmonic tugbot simulation fixture.

This single launch owns Gazebo, the ROS-Gazebo bridge, and robot state
publishing. Run the RobotLens desktop UI separately, use
`View > Simulation > Attach` to connect to it, and drive the robot with
RobotLens's own in-app teleop (Simulation panel > Teleop (Play Mode)) --
arrow keys + numpad, no external terminal needed.

tugbot spawns dynamic straight from its own Fuel-style SDF model (see
scripts/gazebo_runner.py) -- its model.sdf already ships DiffDrive,
PosePublisher, and JointStatePublisher plugins, so unlike TurtleBot3 there's
no URDF->SDF conversion step for the Gazebo side. robot_state_publisher
below still reads a URDF (tugbot/generated/tugbot.urdf, generated from
tugbot/model.sdf by tools/sdf_to_urdf -- regenerate after changing
model.sdf) purely so RobotLens has a robot_description to render; it plays no
part in the physics.

Sensor topics (tugbot's front/back cameras, front/back/omni gpu_lidars, IMU)
are not bridged yet -- this fixture is spawn+drive+view+/joint_states+/odom
only, matching TurtleBot3's and Panda's own Gazebo-physics fixtures.
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
    """Generate the physics-enabled warehouse + tugbot world used by this launch."""
    runner_path = Path(__file__).resolve().parents[1] / "scripts" / "gazebo_runner.py"
    spec = importlib.util.spec_from_file_location("tugbot_gazebo_runner", runner_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load Gazebo runner: {runner_path}")
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    world = runner.write_warehouse_world()
    return world, runner.RESOURCE_DIR


def gazebo_environment(resource_dir):
    env = os.environ.copy()
    env.setdefault("HOME", str(Path.home()))
    env["PATH"] = "/usr/bin:/bin:" + env.get("PATH", "")
    # ROS Jazzy can replace Gazebo's command-plugin path with transport-only
    # entries, hiding `gz sim` and `gz sdf`.
    env["GZ_CONFIG_PATH"] = "/usr/share/gz"
    # Resolves tugbot's model://tugbot/... and warehouse's model://warehouse/...
    # mesh URIs -- see gazebo_runner.py's module docstring.
    env["GZ_SIM_RESOURCE_PATH"] = str(resource_dir)
    return env


def bridge_config():
    path = Path(tempfile.gettempdir()) / "robotlens_tugbot_external_gazebo_bridge.yaml"
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
  gz_topic_name: /model/tugbot/odometry
  ros_type_name: nav_msgs/msg/Odometry
  gz_type_name: gz.msgs.Odometry
  direction: GZ_TO_ROS
- ros_topic_name: /tf
  gz_topic_name: /model/tugbot/pose
  ros_type_name: tf2_msgs/msg/TFMessage
  gz_type_name: gz.msgs.Pose_V
  direction: GZ_TO_ROS
- ros_topic_name: /joint_states
  gz_topic_name: /world/tugbot_view/model/tugbot/joint_state
  ros_type_name: sensor_msgs/msg/JointState
  gz_type_name: gz.msgs.Model
  direction: GZ_TO_ROS
""", encoding="utf-8")
    return path


def generate_launch_description():
    example_dir = Path(__file__).resolve().parents[1]
    urdf = example_dir / "tugbot" / "generated" / "tugbot.urdf"
    description = urdf.read_text(encoding="utf-8")
    world, resource_dir = prepare_world()
    bridge = bridge_config()

    return LaunchDescription([
        ExecuteProcess(
            cmd=["gz", "sim", "-s", "--headless-rendering", "-r", str(world)],
            additional_env=gazebo_environment(resource_dir),
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
        # Expose the same warehouse SDF scene to RobotLens as the non-Gazebo
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
            cmd=[sys.executable, str(example_dir / "scripts" / "model_pose_root_relay.py")],
            output="screen",
        ),
        # Exercises RobotLens's Diagnostics panel, which otherwise has nothing
        # to show for this fixture -- see diagnostics_node.py's docstring.
        ExecuteProcess(
            cmd=[sys.executable, str(example_dir / "scripts" / "diagnostics_node.py")],
            output="screen",
        ),
    ])


def main():
    service = LaunchService(argv=sys.argv[1:])
    service.include_launch_description(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.abspath(__file__))))
    return service.run()


if __name__ == "__main__":
    sys.exit(main())
