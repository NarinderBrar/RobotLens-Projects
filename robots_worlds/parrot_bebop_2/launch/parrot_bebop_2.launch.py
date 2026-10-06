#!/usr/bin/env python3
"""Launch the Gazebo Harmonic Parrot Bebop 2 multicopter simulation fixture.

This single launch owns Gazebo, the ROS-Gazebo bridge, robot state
publishing, and the flight-controller arming step. Run the RobotLens desktop UI
separately, use `View > Simulation > Attach` to connect to it, and fly the
drone with RobotLens's own in-app teleop (Simulation panel > Teleop (Play Mode)):

  - Up/Down arrows or Keypad 8/2: forward/back body-frame pitch velocity
  - Shift + Left/Right: strafe
  - Keypad 4/6 or Shift+8/2: left/right yaw rate
  - Keypad +/- or PageUp/PageDown: climb/descend
  - Space or Shift+5: hover in place (send zero velocity)

The drone's MulticopterVelocityControl system only flies once armed; the
`gazebo_runner.py arm` process publishes true on /parrot_bebop_2/enable at the
start of this launch.
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
    """Generate the physics-enabled Bebop 2 + open_interior world used by this launch."""
    runner_path = Path(__file__).resolve().parents[1] / "scripts" / "gazebo_runner.py"
    spec = importlib.util.spec_from_file_location("explorer_r2_gazebo_runner", runner_path)
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
    path = Path(tempfile.gettempdir()) / "robotlens_explorer_r2_gazebo_bridge.yaml"
    path.write_text("""- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS
- ros_topic_name: /cmd_vel
  gz_topic_name: /parrot_bebop_2/gazebo/command/twist
  ros_type_name: geometry_msgs/msg/Twist
  gz_type_name: gz.msgs.Twist
  direction: ROS_TO_GZ
- ros_topic_name: /odom
  gz_topic_name: /model/parrot_bebop_2/odometry
  ros_type_name: nav_msgs/msg/Odometry
  gz_type_name: gz.msgs.Odometry
  direction: GZ_TO_ROS
- ros_topic_name: /tf
  gz_topic_name: /model/parrot_bebop_2/pose
  ros_type_name: tf2_msgs/msg/TFMessage
  gz_type_name: gz.msgs.Pose_V
  direction: GZ_TO_ROS
- ros_topic_name: /joint_states
  gz_topic_name: /world/open_interior/model/parrot_bebop_2/joint_state
  ros_type_name: sensor_msgs/msg/JointState
  gz_type_name: gz.msgs.Model
  direction: GZ_TO_ROS
""", encoding="utf-8")
    return path


def generate_launch_description():
    example_dir = Path(__file__).resolve().parents[1]
    urdf = example_dir / "robots" / "parrot_bebop_2.urdf"
    description = urdf.read_text(encoding="utf-8").replace(
        'filename="meshes/', f'filename="{example_dir}/meshes/')
    world = prepare_world()
    bridge = bridge_config()

    actions = [
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
            cmd=[sys.executable, str(example_dir / "scripts" / "model_pose_root_relay.py")],
            output="screen",
        ),
        # Arms the MulticopterVelocityControl flight controller: publishes
        # enable=true on /parrot_bebop_2/enable for a window, so the drone
        # flies instead of sitting disarmed on the floor.
        ExecuteProcess(
            cmd=[sys.executable, str(example_dir / "scripts" / "gazebo_runner.py"), "arm"],
            output="screen",
        ),
        # Exercises RobotLens's Diagnostics panel, which otherwise has nothing
        # to show for this fixture -- see diagnostics_node.py's docstring.
        ExecuteProcess(
            cmd=[sys.executable, str(example_dir / "scripts" / "diagnostics_node.py")],
            output="screen",
        ),
    ]
    return LaunchDescription(actions)


def main():
    service = LaunchService(argv=sys.argv[1:])
    service.include_launch_description(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.abspath(__file__))))
    return service.run()


if __name__ == "__main__":
    sys.exit(main())
