#!/usr/bin/env python3
"""Launch the Gazebo Harmonic Panda arm fixture, headless.

This single launch owns Gazebo, the ROS-Gazebo bridge, and robot state
publishing. Run the RobotLens desktop UI separately, use
`View > Simulation > Attach` to connect to it, and set the panda source
to Simulation to see its live /joint_states-driven pose.

The arm is spawned static (see scripts/gazebo_runner.py) -- nothing
commands its joints yet, so it holds its default pose. This fixture is
spawn+view+/joint_states only, no ros2_control/MoveIt wiring.
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
    """Generate the open_interior world with the static panda arm included."""
    runner_path = Path(__file__).resolve().parents[1] / "scripts" / "gazebo_runner.py"
    spec = importlib.util.spec_from_file_location("panda_gazebo_runner", runner_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load Gazebo runner: {runner_path}")
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    runner.write_open_interior_world()
    return runner.GENERATED_WORLD_PATH, runner.RESOURCE_DIR


def gazebo_environment(resource_dir):
    env = os.environ.copy()
    env.setdefault("HOME", str(Path.home()))
    env["PATH"] = "/usr/bin:/bin:" + env.get("PATH", "")
    # ROS Jazzy can replace Gazebo's command-plugin path with transport-only
    # entries, hiding `gz sim` and `gz sdf`.
    env["GZ_CONFIG_PATH"] = "/usr/share/gz"
    # Resolves robots/'s own model://panda/... mesh URIs -- see
    # gazebo_runner.py's ensure_resource_symlink().
    env["GZ_SIM_RESOURCE_PATH"] = str(resource_dir)
    return env


def bridge_config():
    path = Path(tempfile.gettempdir()) / "robotlens_panda_external_gazebo_bridge.yaml"
    path.write_text("""- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS
- ros_topic_name: /joint_states
  gz_topic_name: /world/open_interior/model/panda/joint_state
  ros_type_name: sensor_msgs/msg/JointState
  gz_type_name: gz.msgs.Model
  direction: GZ_TO_ROS
""", encoding="utf-8")
    return path


def generate_launch_description():
    example_dir = Path(__file__).resolve().parents[1]
    urdf = example_dir / "robots" / "panda.urdf"
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
    ])


def main():
    service = LaunchService(argv=sys.argv[1:])
    service.include_launch_description(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.abspath(__file__))))
    return service.run()


if __name__ == "__main__":
    sys.exit(main())
