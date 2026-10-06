#!/usr/bin/env python3
"""Launch the PR2 Gazebo simulation fixture for RobotLens.

One command owns Gazebo (the shared open_interior warehouse world with a
physics-enabled PR2 driven by real caster joint controllers), the ROS-Gazebo
bridge, the PR2 root-pose relay, the cmd_vel->caster relay, and the warehouse
scene description.

Run the RobotLens desktop UI separately, use `View > Simulation > Attach` to
connect, and drive PR2 through RobotLens's own in-app teleop (Simulation panel >
Teleop (Play Mode)) -- its keyboard /cmd_vel is consumed by
`pr2_physics_controller.py`, which converts it into per-caster steering and
wheel-velocity commands over gz-transport.

PR2's holonomic caster base has no built-in odometry plugin, so the root pose
is carried by the bridged Gazebo model pose relayed as odom->base_footprint
(`pr2_model_pose_root_relay.py`) and /joint_states come straight from Gazebo.

Run with:
  source /opt/ros/jazzy/setup.bash
  python3 robots_worlds/pr2/launch/pr2_robotlens_gazebo.launch.py
"""
import importlib.util
import os
import sys
import tempfile
from pathlib import Path

from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def prepare_world():
    """Generate the warehouse + physics PR2 world via run_pr2_gazebo.py.

    RESOLVED 2026-08-15: the graspable object's DetachableJoint plugin
    (add_object_attachment_plugin, `attach_on_start` per gz-sim's own
    default) starts the gripper rigidly attached to the object -- and
    pr2_grasp_controller.py's startup-detach retry logic does not reliably
    win that race before a base-drive command arrives. With the gripper
    still attached and the object resting on the table, the whole assembly
    behaves like the base is physically anchored: wheels visibly spin
    (confirmed via joint-state readback) but the base does not translate at
    all, confirmed live by A/B testing this exact flag with everything else
    identical -- with_graspable_object=True: <2 cm of travel over a 2 s full
    forward command; =False: ~0.56 m, matching real rolling contact. The
    object is only meaningful for the P3 pick-place task, which needs
    `pick_place:=true` anyway, so spawn it only in that case -- this
    function runs at plain Python import time (before ROS 2 launch's own
    argument substitution is available), so `pick_place` is read directly
    off sys.argv rather than through LaunchConfiguration.
    """
    runner_path = Path(__file__).resolve().parents[1] / "gazebo" / "run_pr2_gazebo.py"
    spec = importlib.util.spec_from_file_location("pr2_gazebo_runner", runner_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load Gazebo runner: {runner_path}")
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    pick_place = "pick_place:=false" not in sys.argv
    runner.write_warehouse_world(
        with_drive_controllers=True,
        with_manipulation_controllers=True,
        with_graspable_object=pick_place,
    )
    return runner.GENERATED_WORLD_PATH


def gazebo_environment():
    """Whitelisted env for `gz sim`, mirroring run_pr2_gazebo.py's gz_env().

    A ROS-sourced shell intermittently breaks the Ruby `gz` dispatcher (the
    `sim` subcommand disappears), so only HOME + PATH are passed. Deliberately
    does NOT inherit a shell-local GZ_PARTITION: this fixture and a
    desktop-launched RobotLens must both stay on gz-transport's default partition
    or Attach sees no services at all (see docs/GAZEBO_EXAMPLES.md).
    """
    env = {"HOME": os.environ.get("HOME", ""), "PATH": "/usr/bin:/bin"}
    # ROS Jazzy can replace Gazebo's command-plugin path with transport-only
    # entries, hiding `gz sim`; keep the dispatcher's config visible.
    env["GZ_CONFIG_PATH"] = "/usr/share/gz"
    for key in ("DISPLAY", "XAUTHORITY"):
        if key in os.environ:
            env[key] = os.environ[key]
    return env


def bridge_config():
    path = Path(tempfile.gettempdir()) / "robotlens_pr2_robotlens_gazebo_bridge.yaml"
    path.write_text("""- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS
  qos_profile: CLOCK
- ros_topic_name: /tf
  gz_topic_name: /model/pr2/pose
  ros_type_name: tf2_msgs/msg/TFMessage
  gz_type_name: gz.msgs.Pose_V
  direction: GZ_TO_ROS
- ros_topic_name: /joint_states
  gz_topic_name: /world/open_interior/model/pr2/joint_state
  ros_type_name: sensor_msgs/msg/JointState
  gz_type_name: gz.msgs.Model
  direction: GZ_TO_ROS
- ros_topic_name: /tf
  gz_topic_name: /model/graspable_object/pose
  ros_type_name: tf2_msgs/msg/TFMessage
  gz_type_name: gz.msgs.Pose_V
  direction: GZ_TO_ROS
""", encoding="utf-8")
    return path


def generate_launch_description():
    example_dir = Path(__file__).resolve().parents[1]
    # The native SDF17 model is delivered to RobotLens through
    # scene_description_publisher. Keep the URDF only for a complete ROS TF
    # tree; its distinct node name prevents RobotLens's live URDF fetcher
    # (which intentionally queries only /robot_state_publisher) from loading
    # the old flattened model instead of the SDF scene model.
    tf_robot_description = (example_dir / "pr2.urdf").read_text(encoding="utf-8")
    world = prepare_world()
    bridge = bridge_config()

    def run(*relative_path):
        return ExecuteProcess(
            cmd=[sys.executable, str(example_dir.joinpath(*relative_path))],
            output="screen",
        )

    return LaunchDescription([
        # Run the /pick_place task server too (default true). Set
        # `pick_place:=false` when using scripts/idle_to_approach.py, which
        # drives the same helpers directly and so needs no /pick_place server.
        DeclareLaunchArgument("pick_place", default_value="true"),
        ExecuteProcess(
            cmd=["gz", "sim", "-s", "--headless-rendering", "-r", str(world)],
            additional_env=gazebo_environment(),
            output="screen",
        ),
        # Turns bridged /joint_states into the complete PR2 link TF tree.
        # Its non-default name deliberately leaves RobotLens to load the
        # native SDF17 PR2 from scene_models instead of robot_description.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="pr2_tf_publisher",
            parameters=[{
                "robot_description": tf_robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time", default="true"),
            }],
            output="screen",
        ),
        ExecuteProcess(
            cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
                 "-p", f"config_file:={bridge}"],
            output="screen",
        ),
        # Convert ROS /cmd_vel into real per-caster steering/wheel commands.
        # Must run on the same gz-transport config as `gz sim` (/usr/share/gz):
        # inheriting the ROS-sourced GZ_CONFIG_PATH (the vendored gz transport)
        # puts this node on a transport domain the sim's plugins don't see, so
        # wheel/steer commands never arrive (the robot then ignores /cmd_vel).
        ExecuteProcess(
            cmd=[sys.executable, str(example_dir.joinpath("pr2_physics_controller.py"))],
            additional_env=gazebo_environment(),
            output="screen",
        ),
        # Standard ROS 2 trajectory/gripper actions -> the P1 Gazebo arm
        # controllers. This owns all manipulation command validation. Needs
        # the same restricted gz-transport env as pr2_physics_controller.py
        # above -- its position_cmd publishers otherwise land on a transport
        # domain `gz sim` never sees, so commands silently go nowhere.
        ExecuteProcess(
            cmd=[sys.executable, str(example_dir.joinpath("pr2_arm_trajectory_controller.py")),
                 "--ros-args", "-p", "use_sim_time:=true"],
            additional_env=gazebo_environment(),
            output="screen",
        ),
        # Relay the Gazebo model pose as odom->base_footprint so RobotLens's
        # Simulation-mode root transform follows real physics.
        run("pr2_model_pose_root_relay.py"),
        # P3 grasp/object-ownership controller: frees the graspable object
        # (the DetachableJoint plugin starts attached), seeds and
        # synchronizes the MoveIt planning scene, and exposes the attach/
        # detach primitives. Needs the same restricted gz-transport env as
        # the two ExecuteProcess entries above for the same reason.
        # Conditional on `pick_place`, same as the object itself (see
        # prepare_world()'s docstring) -- with no graspable object spawned
        # this has nothing to attach/detach/sync.
        ExecuteProcess(
            cmd=[sys.executable, str(example_dir.joinpath("pr2_grasp_controller.py")),
                 "--ros-args", "-p", "use_sim_time:=true"],
            additional_env=gazebo_environment(),
            output="screen",
            condition=IfCondition(LaunchConfiguration("pick_place")),
        ),
        # P4 pick-and-place task: composes move_group (P2) and P1/P3's
        # actions/services into one cancellable PickPlace action. Pure ROS
        # 2 (rclpy actions/services/TF only, no direct gz-transport calls),
        # so it doesn't need the restricted gazebo_environment() the three
        # entries above do. Requires move_group.launch.py (P2) already
        # running, same as any other MoveGroup client -- this launch file
        # does not start it (see that launch file's own docstring for why
        # it's kept separate).
        ExecuteProcess(
            cmd=[sys.executable, str(example_dir.joinpath("pick_place_task.py")),
                 "--ros-args", "-p", "use_sim_time:=true"],
            output="screen",
            condition=IfCondition(LaunchConfiguration("pick_place")),
        ),
        # Expose the warehouse scene to RobotLens (same open_interior world).
        ExecuteProcess(
            cmd=[sys.executable, str(example_dir / "scripts" / "scene_description_node.py")],
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
