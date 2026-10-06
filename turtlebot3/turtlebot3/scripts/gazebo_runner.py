#!/usr/bin/env python3
"""
RobotLens turtlebot3 Gazebo fixture runner.

Spawns one of the four example1 robot models into an otherwise-empty gz-sim
world, for physics and ROS bridge testing:
  - simple_robot: robots/turtlebot3_burger.urdf, converted to SDF via sdformat's built-in URDF
    conversion (`gz sdf -p`), with DiffDrive, PosePublisher, and
    JointStatePublisher systems. It has no simulated sensors.
  - explorer_r2_robot / panda_robot: already Fuel-style SDF model folders
    (model.config + model.sdf), spawned directly via <include><uri>, no
    conversion needed.

Usage:
  examples/turtlebot3/scripts/gazebo_runner.py up [model] [--headless]
  examples/turtlebot3/scripts/gazebo_runner.py down
  examples/turtlebot3/scripts/gazebo_runner.py status

  model is one of: simple_robot (default), explorer_r2_robot, panda_robot

Environment note (same issue as examples/gazebo/run_gazebo_example.sh):
sourcing ROS 2 Jazzy can break the `gz` CLI's subcommand dispatch, so `gz` is
invoked with a whitelisted environment (HOME/PATH/DISPLAY/XAUTHORITY only)
rather than whatever shell this script was run from.
"""

import argparse
import os
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

EXAMPLE_DIR = Path(__file__).resolve().parents[1]
WORLDS_DIR = Path(__file__).resolve().parents[2] / 'worlds'
URDF_PATH = EXAMPLE_DIR / 'robots' / 'turtlebot3_burger.urdf'
STATE_DIR = Path(os.environ.get('TMPDIR', '/tmp')) / 'robotlens_example1_gazebo'
GENERATED_WORLD_PATH = STATE_DIR / 'example1_world.sdf'
PID_FILE = STATE_DIR / 'gz_sim.pid'
LOG_FILE = STATE_DIR / 'gz_sim.log'

# Fuel-style SDF model folders spawned directly via <include>, keyed by the
# CLI model name -- as opposed to 'simple_robot', which goes through the URDF
# conversion path below.
SDF_MODEL_FOLDERS = {
    'explorer_r2_robot': EXAMPLE_DIR / 'explorer_r2_robot',
    'panda_robot': EXAMPLE_DIR / 'panda_robot',
}

# Boilerplate mirrors examples/gazebo/worlds/double_pendulum.sdf's own
# physics/plugin/light/ground_plane setup, for consistency with the other
# gz-sim fixture already in this repo.
WORLD_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="example1_view">
    <gravity>0 0 -9.81</gravity>
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
{model_block}
  </world>
</sdf>
"""


def gz_env(resource_path=None):
    env = {'HOME': os.environ.get('HOME', ''), 'PATH': '/usr/bin:/bin'}
    for key in ('DISPLAY', 'XAUTHORITY'):
        if key in os.environ:
            env[key] = os.environ[key]
    if resource_path is not None:
        # Needed to resolve a Fuel-style model's own internal
        # 'model://<name>/...' mesh URIs (e.g. panda's model.sdf uses
        # model://panda/meshes/...) -- see include_block_for_sdf_model()'s
        # comment. GZ_SIM_RESOURCE_PATH must point at the *parent* of the
        # model folder, not the folder itself.
        env['GZ_SIM_RESOURCE_PATH'] = str(resource_path)
    return env


def require_gz():
    if shutil.which('gz', path=gz_env()['PATH']) is None:
        sys.exit("error: 'gz' (Gazebo Harmonic) not found on /usr/bin:/bin")


def convert_urdf_to_model_block():
    """Runs `gz sdf -p robots/turtlebot3_burger.urdf` (sdformat's built-in URDF->SDF converter,
    no ros_gz_sim/sdformat_urdf dependency needed) and returns the <model>
    element with mesh URIs rewritten from relative to absolute -- confirmed
    empirically that gz-sim resolves bare absolute mesh paths directly, so
    the generated world file below doesn't need to live next to meshes/ the
    way the vendored SDF Fuel models (explorer_r2_robot/panda) do.
    """
    result = subprocess.run(
        ['gz', 'sdf', '-p', str(URDF_PATH)],
        env=gz_env(), capture_output=True, text=True, check=False,
    )
    if result.returncode != 0 or not result.stdout.strip():
        sys.exit(f"error: 'gz sdf -p' failed to convert {URDF_PATH}:\n{result.stderr}")

    root = ET.fromstring(result.stdout)
    model = root.find('model')
    if model is None:
        sys.exit(f"error: converted SDF has no <model>:\n{result.stdout}")
    model.set('name', 'simple_robot')
    static_tag = model.find('static')
    if static_tag is None:
        static_tag = ET.Element('static')
        model.insert(0, static_tag)
    static_tag.text = 'false'
    for link in model.findall('link'):
        gravity_tag = link.find('gravity')
        if gravity_tag is None:
            gravity_tag = ET.SubElement(link, 'gravity')
        gravity_tag.text = 'true'

    for uri in model.iter('uri'):
        if uri.text and uri.text.startswith('../meshes/'):
            uri.text = str(EXAMPLE_DIR / uri.text[len('../'):])

    # Enable the same physics feedback used by the RobotLens connector.
    pose = model.find('pose')
    if pose is None:
        pose = ET.Element('pose')
        model.insert(0, pose)
    pose.text = '0 0 0.9144 0 0 0'  # 1 ft rest height + 2 ft vertical offset
    diff_drive = ET.SubElement(model, 'plugin', {
        'filename': 'gz-sim-diff-drive-system',
        'name': 'gz::sim::systems::DiffDrive',
    })
    for tag, value in (
        ('left_joint', 'wheel_left_joint'), ('right_joint', 'wheel_right_joint'),
        ('wheel_separation', '0.16'), ('wheel_radius', '0.033'),
        ('topic', '/cmd_vel'), ('odom_topic', '/model/simple_robot/odometry'),
        ('tf_topic', '/tf'), ('frame_id', 'odom'),
        ('child_frame_id', 'base_footprint'), ('odom_publish_frequency', '30')):
        ET.SubElement(diff_drive, tag).text = value
    pose_publisher = ET.SubElement(model, 'plugin', {
        'filename': 'gz-sim-pose-publisher-system',
        'name': 'gz::sim::systems::PosePublisher',
    })
    for tag, value in (('publish_link_pose', 'true'),
                        ('publish_collision_pose', 'false'),
                        ('publish_visual_pose', 'false'),
                        ('publish_nested_model_pose', 'false'),
                        # publish_model_pose defaults to publish_nested_model_pose's
                        # value when omitted (gz-sim PosePublisher.cc), so leaving
                        # this unset silently suppressed the model's own true world
                        # pose (the only pose with real physics pitch/roll -- link
                        # poses below are relative to the model, i.e. always near
                        # identity). RobotLens's TF consumer needs this to render
                        # physically-accurate root motion instead of Gazebo
                        # DiffDrive's 2D-only /odom fallback (yaw only, no pitch).
                        ('publish_model_pose', 'true'),
                        ('use_pose_vector_msg', 'true')):
        ET.SubElement(pose_publisher, tag).text = value
    ET.SubElement(model, 'plugin', {
        'filename': 'gz-sim-joint-state-publisher-system',
        'name': 'gz::sim::systems::JointStatePublisher',
    })
    return model


def write_open_interior_world():
    """Build the open_interior world and insert the dynamic simple robot."""
    world_path = WORLDS_DIR / "open_interior" / "open_interior.sdf"
    world_text = world_path.read_text(encoding="utf-8")
    world_text = world_text.replace(
        """    <!-- Sun -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>""",
        """    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>""",
    )
    robot_xml = ET.tostring(convert_urdf_to_model_block(), encoding="unicode")
    if "</world>" not in world_text:
        sys.exit(f"error: open_interior world has no closing world tag: {world_path}")
    GENERATED_WORLD_PATH.parent.mkdir(parents=True, exist_ok=True)
    GENERATED_WORLD_PATH.write_text(
        world_text.replace("</world>", robot_xml + "\n</world>", 1),
        encoding="utf-8",
    )

def include_block_for_sdf_model(model_name):
    """<include><uri>{absolute folder}</uri></include> -- gz-sim loads a
    Fuel-style model.config folder directly, no conversion needed. The
    model's own internal 'model://<name>/...' mesh URIs (e.g. panda's
    model.sdf uses model://panda/meshes/...) are NOT resolved by pointing
    <include><uri> at the folder alone though -- confirmed empirically that
    gz-sim still needs GZ_SIM_RESOURCE_PATH set to the folder's *parent* dir
    (EXAMPLE_DIR) so it can search for the model subdirectory to satisfy that
    URI scheme; cmd_up sets that in the child's environment for sdf-folder
    models.
    """
    folder = SDF_MODEL_FOLDERS[model_name]
    if not (folder / 'model.config').exists():
        sys.exit(f"error: {folder} has no model.config")
    return f'    <include>\n      <uri>{folder}</uri>\n    </include>\n'


def write_world_file(model_name):
    if model_name == 'simple_robot':
        model = convert_urdf_to_model_block()
        model_block = ET.tostring(model, encoding='unicode')
    else:
        model_block = include_block_for_sdf_model(model_name)
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    GENERATED_WORLD_PATH.write_text(
        WORLD_TEMPLATE.format(model_block=model_block), encoding='utf-8'
    )


def pid_alive():
    if not PID_FILE.exists():
        return None
    try:
        pid = int(PID_FILE.read_text().strip())
    except ValueError:
        PID_FILE.unlink(missing_ok=True)
        return None
    try:
        os.kill(pid, 0)
    except OSError:
        PID_FILE.unlink(missing_ok=True)
        return None
    return pid


def cmd_up(args):
    require_gz()
    running = pid_alive()
    if running is not None:
        sys.exit(f"a fixture is already running (pid {running}); run 'down' first")

    write_world_file(args.model)
    print(f'generated world: {GENERATED_WORLD_PATH}')

    gz_args = ['gz', 'sim', str(GENERATED_WORLD_PATH), '-r']
    if args.headless:
        gz_args += ['-s', '--headless-rendering']

    # Only the sdf-folder models (explorer_r2_robot/panda_robot) reference
    # model://<name>/... URIs that need resolving; simple_robot's converted
    # model block already has absolute mesh paths (see
    # convert_urdf_to_model_block's comment).
    resource_path = None
    if args.model != 'simple_robot':
        resource_path = STATE_DIR / 'resources'
        resource_path.mkdir(parents=True, exist_ok=True)
        aliases = {
            'panda': EXAMPLE_DIR / 'panda_robot',
            'explorer_r2_sensor_config_1': EXAMPLE_DIR / 'explorer_r2_robot',
        }
        for alias, folder in aliases.items():
            link = resource_path / alias
            if link.is_symlink():
                link.unlink()
            if folder.is_dir():
                link.symlink_to(folder, target_is_directory=True)

    print(f"starting gz sim ({args.model}" +
          (', headless)...' if args.headless else ', GUI)...'))
    log_handle = open(LOG_FILE, 'w', encoding='utf-8')
    proc = subprocess.Popen(
        gz_args, env=gz_env(resource_path), stdout=log_handle, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    PID_FILE.write_text(str(proc.pid))

    time.sleep(2)
    if proc.poll() is not None or pid_alive() is None:
        sys.exit(f'error: gz sim exited immediately; see {LOG_FILE}')
    print(f'OK: gz sim running (pid {proc.pid}). Log: {LOG_FILE}')
    if args.model == 'simple_robot':
        print('simple_robot physics enabled: /cmd_vel, /model/simple_robot/odometry, '
              '/model/simple_robot/pose, and joint_state are available.')
    if args.foreground:
        def forward_signal(signum, _frame):
            try:
                os.killpg(proc.pid, signum)
            except ProcessLookupError:
                pass

        signal.signal(signal.SIGINT, forward_signal)
        signal.signal(signal.SIGTERM, forward_signal)
        try:
            proc.wait()
        finally:
            PID_FILE.unlink(missing_ok=True)
            log_handle.close()


def cmd_down(_args):
    pid = pid_alive()
    if pid is None:
        print('not running.')
        return
    os.kill(pid, signal.SIGTERM)
    PID_FILE.unlink(missing_ok=True)
    print(f'stopped (pid {pid}).')


def cmd_status(_args):
    pid = pid_alive()
    print(f"gz sim: {'running (pid ' + str(pid) + ')' if pid else 'not running'}")
    if LOG_FILE.exists():
        print(f'log: {LOG_FILE}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    up_parser = sub.add_parser('up', help='start gz sim with a model spawned')
    up_parser.add_argument(
        'model', nargs='?', default='simple_robot',
        choices=['simple_robot', *SDF_MODEL_FOLDERS],
        help='which model to view (default: simple_robot)',
    )
    up_parser.add_argument(
        '--headless', action='store_true',
        help='run headless (-s --headless-rendering) instead of the GUI client',
    )
    up_parser.add_argument(
        '--foreground', action='store_true',
        help='wait for gz sim and forward shutdown signals to it',
    )
    sub.add_parser('down', help='stop gz sim')
    sub.add_parser('status', help='show whether gz sim is running')
    args = parser.parse_args()

    {'up': cmd_up, 'down': cmd_down, 'status': cmd_status}[args.action](args)


if __name__ == '__main__':
    main()
