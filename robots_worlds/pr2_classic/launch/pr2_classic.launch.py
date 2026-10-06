#!/usr/bin/env python3
"""The pr2_classic "every classic ROS tool at once" launch file.

One command brings up the full old-school ROS 2 tool stack around the same
physics PR2 the ../pr2 fixture drives, all talking to the same ROS graph:

  - gz sim (Gazebo, visible GUI) running the shared warehouse world with the
    physics PR2 (casters + right-arm + gripper + graspable object controllers)
  - ros_gz_bridge (/clock, /tf, /gz_joint_states) + the pr2 fixture's
    pr2_physics_controller.py, pr2_model_pose_root_relay.py,
    pr2_arm_trajectory_controller.py, pr2_grasp_controller.py (the latter two
    remapped to read the real physics on /gz_joint_states)
  - robot_state_publisher + joint_state_publisher_gui (the classic sliders;
    they are the authoritative /joint_states source for the display model, so
    dragging a slider really moves the robot in RViz -- physics is separate)
  - rviz2 with the pr2 fixture's saved pr2.rviz config
  - rqt (docked Console + Topic Monitor perspective) and rqt_graph (the
    computation-graph viewer)
  - MoveIt 2 move_group from ../pr2/pr2_right_arm_moveit_config (skipped with
    a warning when that package isn't built/sourced yet)
  - Nav2 bringup (map_server/AMCL/planner/controller/BT navigator; skipped
    with a warning when ros-jazzy-nav2-* isn't installed)

Launch arguments:
  use_sim_time:=true|false   Follow the bridged Gazebo /clock (default true).
  moveit:=true|false         Start move_group (default: auto -- on only if the
                             pr2_right_arm_moveit_config package is built and
                             the workspace overlay is sourced).
  nav2:=true|false           Start Nav2 bringup (default: auto -- on only if
                             nav2_bringup is installed).

Usage:
  source /opt/ros/jazzy/setup.bash
  python3 robots_worlds/pr2_classic/launch/pr2_classic.launch.py
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

# The pr2 fixture is the single source of truth for every shared asset:
# pr2.urdf, the rviz config, the Gazebo world generator, the physics/arm/
# grasp controllers, the scene publisher, and the MoveIt 2 package.
PR2_DIR = Path(__file__).resolve().parents[2] / 'pr2'
EXAMPLE_DIR = Path(__file__).resolve().parents[1]
BRIDGE_CONFIG_PATH = Path(tempfile.gettempdir()) / 'robotlens_pr2_classic_bridge.yaml'


def package_available(package_name):
    """True when the ROS 2 package is installed and discoverable via the
    ament index (i.e. the workspace overlay that contains it is sourced)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory(package_name)
        return True
    except (ImportError, LookupError):
        return False


def prepare_world():
    """Generate the warehouse + physics PR2 world via the pr2 fixture's
    run_pr2_gazebo.py (same world the pr2_robotlens_gazebo launch uses)."""
    runner_path = PR2_DIR / 'gazebo' / 'run_pr2_gazebo.py'
    spec = importlib.util.spec_from_file_location('pr2_gazebo_runner', runner_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'Cannot load Gazebo runner: {runner_path}')
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    runner.write_warehouse_world(
        with_drive_controllers=True,
        with_manipulation_controllers=True,
        with_graspable_object=True,
    )
    return runner.GENERATED_WORLD_PATH


def gazebo_environment():
    """Whitelisted env for `gz sim`, mirroring run_pr2_gazebo.py's gz_env().

    A ROS-sourced shell intermittently breaks the Ruby `gz` dispatcher (the
    `sim` subcommand disappears), so only HOME + PATH are passed. Deliberately
    does NOT inherit a shell-local GZ_PARTITION: this fixture and a
    desktop-launched RobotLens must both stay on gz-transport's default
    partition or Attach sees no services at all (see
    docs/GAZEBO_EXAMPLES.md). Same restricted environment the pr2 fixture
    uses for every node that talks to gz-transport.
    """
    env = {'HOME': os.environ.get('HOME', ''), 'PATH': '/usr/bin:/bin'}
    env['GZ_CONFIG_PATH'] = '/usr/share/gz'
    for key in ('DISPLAY', 'XAUTHORITY'):
        if key in os.environ:
            env[key] = os.environ[key]
    return env


def bridge_config():
    """Same ros_gz_bridge config as the pr2 physics fixture: /clock, /tf for
    both PR2 and the graspable object, and /joint_states. Nav2's /odom is
    synthesized by pr2_classic_odom.py, not bridged."""
    BRIDGE_CONFIG_PATH.write_text("""- ros_topic_name: /clock
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
- ros_topic_name: /gz_joint_states
  gz_topic_name: /world/open_interior/model/pr2/joint_state
  ros_type_name: sensor_msgs/msg/JointState
  gz_type_name: gz.msgs.Model
  direction: GZ_TO_ROS
- ros_topic_name: /tf
  gz_topic_name: /model/graspable_object/pose
  ros_type_name: tf2_msgs/msg/TFMessage
  gz_type_name: gz.msgs.Pose_V
  direction: GZ_TO_ROS
""", encoding='utf-8')
    return BRIDGE_CONFIG_PATH


def generate_launch_description():
    world = prepare_world()
    bridge = bridge_config()

    robot_description = (PR2_DIR / 'pr2.urdf').read_text(encoding='utf-8')
    rviz_config = PR2_DIR / 'rviz' / 'pr2.rviz'
    nav2_params = EXAMPLE_DIR / 'nav2' / 'nav2_params.yaml'
    nav2_map = EXAMPLE_DIR / 'nav2' / 'pr2_classic_map.yaml'

    moveit_available = package_available('pr2_right_arm_moveit_config')
    nav2_share = None
    if package_available('nav2_bringup'):
        try:
            from ament_index_python.packages import get_package_share_directory
            nav2_share = get_package_share_directory('nav2_bringup')
        except LookupError:
            nav2_share = None
    if not moveit_available:
        print('pr2_classic: pr2_right_arm_moveit_config not found on the ament '
              'index -- skipping MoveIt 2 (build it and source install/setup.bash '
              'first)', file=sys.stderr)
    if nav2_share is None:
        print('pr2_classic: nav2_bringup not found on the ament index -- skipping '
              'Nav2 (see README.md for the install command)', file=sys.stderr)

    def run(*relative_path):
        return ExecuteProcess(
            cmd=[sys.executable, str(EXAMPLE_DIR.joinpath(*relative_path))],
            output='screen',
        )

    def run_in_pr2(*relative_path):
        return ExecuteProcess(
            cmd=[sys.executable, str(PR2_DIR.joinpath(*relative_path))],
            output='screen',
        )

    nav2_bringup = None
    if nav2_share is not None:
        nav2_bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'params_file': str(nav2_params),
                'map': str(nav2_map),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': 'true',
            }.items(),
            condition=IfCondition(LaunchConfiguration('nav2')),
        )

    actions = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'moveit', default_value='true' if moveit_available else 'false'),
        DeclareLaunchArgument(
            'nav2', default_value='true' if nav2_share is not None else 'false'),

        # Gazebo, visible GUI -- the same physics world as the pr2 fixture's
        # headless launch, just without -s/--headless-rendering so the Gazebo
        # window itself is part of the tool stack being exercised.
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', str(world)],
            additional_env=gazebo_environment(),
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '--ros-args',
                 '-p', f'config_file:={bridge}'],
            output='screen',
        ),
        # ROS /cmd_vel -> real per-caster steering/wheel commands. Must run on
        # the same gz-transport config as `gz sim` (see gazebo_environment()).
        ExecuteProcess(
            cmd=[sys.executable, str(PR2_DIR.joinpath('pr2_physics_controller.py'))],
            additional_env=gazebo_environment(),
            output='screen',
        ),
        # Standard ROS 2 trajectory/gripper actions -> the Gazebo arm
        # controllers (the MoveIt execution path). They consume the real
        # physics joint states on /gz_joint_states (not the display model's
        # /joint_states published by joint_state_publisher_gui below), so
        # execution feedback stays authoritative while the sliders drive the
        # RViz model.
        ExecuteProcess(
            cmd=[sys.executable, str(PR2_DIR.joinpath('pr2_arm_trajectory_controller.py')),
                 '--ros-args', '-p', 'use_sim_time:=true',
                 '-r', '/joint_states:=/gz_joint_states'],
            additional_env=gazebo_environment(),
            output='screen',
        ),
        # Relay the Gazebo model pose as odom->base_footprint TF.
        run_in_pr2('pr2_model_pose_root_relay.py'),
        # Nav2 needs a nav_msgs/Odometry topic; the PR2 fixture has none.
        run('pr2_classic_odom.py'),
        # Grasp/object-ownership controller (seeds the MoveIt planning scene).
        # Same /gz_joint_states remap as the arm controller: real physics, not
        # the display model.
        ExecuteProcess(
            cmd=[sys.executable, str(PR2_DIR.joinpath('pr2_grasp_controller.py')),
                 '--ros-args', '-p', 'use_sim_time:=true',
                 '-r', '/joint_states:=/gz_joint_states'],
            additional_env=gazebo_environment(),
            output='screen',
        ),
        # Expose the warehouse scene to RobotLens.
        ExecuteProcess(
            cmd=[sys.executable, str(PR2_DIR / 'scripts' / 'scene_description_node.py')],
            output='screen',
        ),
        # Classic joint_state_publisher_gui sliders: the authoritative
        # /joint_states publisher for the display model (robot_state_publisher
        # + RViz). No source_list -- dragging a slider moves the model and
        # nothing fights back. The physics world publishes separately on
        # /gz_joint_states, which only the controllers consume.
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(rviz_config)],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
        ),
        # rqt with a saved perspective (Console + Topic Monitor docked), so
        # it opens with live content instead of the blank default perspective.
        ExecuteProcess(
            cmd=['rqt', '--perspective-file', str(EXAMPLE_DIR / 'rqt' / 'pr2_classic.perspective')],
            output='screen',
        ),
        # rqt_graph: the computation-graph viewer, its own window.
        ExecuteProcess(cmd=['rqt_graph'], output='screen'),
        # MoveIt 2 move_group (from the pr2 fixture's moveit config package).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(PR2_DIR / 'pr2_right_arm_moveit_config' / 'launch' / 'move_group.launch.py')),
            condition=IfCondition(LaunchConfiguration('moveit')),
        ),
        # The static map->odom identity transform keeps the TF tree complete
        # without a localizing AMCL (no /scan is bridged). Gated on the same
        # nav2 argument so it disappears when Nav2 is disabled.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='pr2_classic_map_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('nav2')),
        ),
    ]
    # Nav2 bringup. Only ever present when the package is actually installed
    # (None is not a valid launch action), then further gated on nav2:=false.
    if nav2_bringup is not None:
        actions.append(nav2_bringup)

    return LaunchDescription(actions)


def parse_launch_args(argv):
    """Parse 'name:=value' launch arguments from a plain argv list."""
    result = []
    for arg in argv:
        if arg.count(':=') == 1 and not arg.startswith(':=') and not arg.endswith(':='):
            name, _, value = arg.partition(':=')
            result.append((name, value))
    return result


def main():
    launch_service = LaunchService(argv=sys.argv[1:])
    launch_service.include_launch_description(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.abspath(__file__)),
            launch_arguments=parse_launch_args(sys.argv[1:]),
        )
    )
    return launch_service.run()


if __name__ == '__main__':
    sys.exit(main())
