#!/usr/bin/env python3
"""
RobotLens explorer_r2 (Parrot Bebop 2) Gazebo fixture runner.

Builds a physics-enabled world containing the Parrot Bebop 2 quadcopter and
manages a standalone `gz sim` instance for it.

The drone URDF (robots/parrot_bebop_2.urdf) is converted to SDF via sdformat's
built-in URDF conversion (`gz sdf -p`), then augmented with the standard
gz-sim multicopter stack (see /usr/share/gz/gz-sim8/worlds/multicopter_velocity_control.sdf):

  - four MulticopterMotorModel plugins, one per rotor joint, applying real
    thrust and reaction torque from commanded rotor speeds,
  - one MulticopterVelocityControl plugin turning a body-frame velocity command
    (gz.msgs.Twist, linear = velocity in the body frame, angular.z = yaw rate)
    into per-rotor speed commands,
  - an OdometryPublisher (3D) for /model/parrot_bebop_2/odometry,
  - a PosePublisher for /model/parrot_bebop_2/pose (bridged to /tf) and a
    JointStatePublisher.

The velocity controller is deliberately started disarmed (gz-sim's own
default), so the launch runs `gazebo_runner.py arm` to publish true on
/parrot_bebop_2/enable -- the drone-arm equivalent of a robot's e-stop release.
RobotLens's in-app teleop (Simulation > Teleop (Play Mode), Up/Down arrows, Shift
+ Left/Right strafe, Keypad +/- or PageUp/PageDown for climb/descend, Left/Right
to yaw) flies it directly through /cmd_vel.

Usage:
  robots_worlds/explorer_r2/scripts/gazebo_runner.py up [--headless]
  robots_worlds/explorer_r2/scripts/gazebo_runner.py down
  robots_worlds/explorer_r2/scripts/gazebo_runner.py status
  robots_worlds/explorer_r2/scripts/gazebo_runner.py arm

Environment note (same issue as robots_worlds/gazebo/run_gazebo_example.sh):
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
URDF_PATH = EXAMPLE_DIR / 'robots' / 'parrot_bebop_2.urdf'
MODEL_NAME = 'parrot_bebop_2'
ENABLE_TOPIC = f'/{MODEL_NAME}/enable'
STATE_DIR = Path(os.environ.get('TMPDIR', '/tmp')) / 'robotlens_explorer_r2_gazebo'
GENERATED_WORLD_PATH = STATE_DIR / 'explorer_r2_world.sdf'
PID_FILE = STATE_DIR / 'gz_sim.pid'
LOG_FILE = STATE_DIR / 'gz_sim.log'

# One entry per rotor: (joint, link, motor turning direction, control-plugin
# yaw direction, actuator index). The two CCW rotors (FL, RR) and two CW rotors
# (FR, RL) cancel reaction yaw at hover.
ROTORS = [
    ('rotor_fl_joint', 'rotor_fl_link', 'ccw', 1, 0),
    ('rotor_fr_joint', 'rotor_fr_link', 'cw', -1, 1),
    ('rotor_rl_joint', 'rotor_rl_link', 'cw', -1, 2),
    ('rotor_rr_joint', 'rotor_rr_link', 'ccw', 1, 3),
]

# Sized so four rotors can lift the 0.503 kg Bebop comfortably (hover is around
# 60% of max rotor speed at maxRotVelocity). forceConstant (control plugin) and
# motorConstant (motor plugin) must match for the same rotor.
MOTOR_CONSTANT = 2.5e-06
MOMENT_CONSTANT = 4.0e-03
MAX_ROT_VELOCITY = 1200.0

ARM_DURATION_SECONDS = 30.0
ARM_PERIOD_SECONDS = 0.5

# Boilerplate mirrors robots_worlds/worlds/open_interior/open_interior.sdf's own
# physics/plugin/light/ground_plane setup, for consistency with the other
# gz-sim fixture already in this repo.
WORLD_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="explorer_r2_view">
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
        env['GZ_SIM_RESOURCE_PATH'] = str(resource_path)
    return env


def require_gz():
    if shutil.which('gz', path=gz_env()['PATH']) is None:
        sys.exit("error: 'gz' (Gazebo Harmonic) not found on /usr/bin:/bin")


def add_plugin(model, filename, name, params):
    plugin = ET.SubElement(model, 'plugin', {'filename': filename, 'name': name})
    for tag, value in params.items():
        ET.SubElement(plugin, tag).text = value
    return plugin


def convert_urdf_to_model_block():
    """Run `gz sdf -p robots/parrot_bebop_2.urdf` and return the <model>
    element with mesh URIs rewritten from relative to absolute and the
    multicopter plugin stack attached. sdformat's built-in URDF->SDF converter
    merges the fixed base_footprint->base_link into a single body link named
    base_footprint, so that link is the control plugin's center-of-mass link.
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
    model.set('name', MODEL_NAME)
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
        if uri.text and uri.text.startswith('meshes/'):
            uri.text = str(EXAMPLE_DIR / uri.text)

    pose = model.find('pose')
    if pose is None:
        pose = ET.Element('pose')
        model.insert(0, pose)
    pose.text = '0 0 0.1 0 0 0'

    for joint, link, direction, _control_direction, actuator in ROTORS:
        add_plugin(model, 'gz-sim-multicopter-motor-model-system',
                   'gz::sim::systems::MulticopterMotorModel', {
                       'robotNamespace': MODEL_NAME,
                       'jointName': joint,
                       'linkName': link,
                       'turningDirection': direction,
                       'timeConstantUp': '0.0125',
                       'timeConstantDown': '0.025',
                       'maxRotVelocity': str(MAX_ROT_VELOCITY),
                       'motorConstant': str(MOTOR_CONSTANT),
                       'momentConstant': str(MOMENT_CONSTANT),
                       'commandSubTopic': 'gazebo/command/motor_speed',
                       'actuator_number': str(actuator),
                       'rotorDragCoefficient': '8.06428e-05',
                       'rollingMomentCoefficient': '1e-06',
                       'motorSpeedPubTopic': f'motor_speed/{actuator}',
                       'rotorVelocitySlowdownSim': '10',
                       'motorType': 'velocity',
                   })

    control = add_plugin(model, 'gz-sim-multicopter-control-system',
                         'gz::sim::systems::MulticopterVelocityControl', {
                             'robotNamespace': MODEL_NAME,
                             'commandSubTopic': 'gazebo/command/twist',
                             'enableSubTopic': 'enable',
                             'comLinkName': 'base_footprint',
                             'velocityGain': '2.7 2.7 2.7',
                             'attitudeGain': '2 3 0.15',
                             'angularRateGain': '0.4 0.52 0.18',
                             'maximumLinearAcceleration': '2 2 2',
                             'maximumLinearVelocity': '3 3 3',
                             'maximumAngularVelocity': '1 1 1',
                         })
    rotor_config = ET.SubElement(control, 'rotorConfiguration')
    for _joint, _link, _direction, control_direction, _actuator in ROTORS:
        rotor = ET.SubElement(rotor_config, 'rotor')
        ET.SubElement(rotor, 'jointName').text = _joint
        ET.SubElement(rotor, 'forceConstant').text = str(MOTOR_CONSTANT)
        ET.SubElement(rotor, 'momentConstant').text = str(MOMENT_CONSTANT)
        ET.SubElement(rotor, 'direction').text = str(control_direction)

    add_plugin(model, 'gz-sim-odometry-publisher-system',
               'gz::sim::systems::OdometryPublisher', {
                   'dimensions': '3',
                   'odom_frame': 'odom',
                   'robot_base_frame': 'base_footprint',
               })
    add_plugin(model, 'gz-sim-pose-publisher-system',
               'gz::sim::systems::PosePublisher', {
                   'publish_link_pose': 'true',
                   'publish_collision_pose': 'false',
                   'publish_visual_pose': 'false',
                   'publish_nested_model_pose': 'false',
                   # publish_model_pose defaults to publish_nested_model_pose's
                   # value when omitted (gz-sim PosePublisher.cc), so leaving
                   # this unset silently suppressed the model's own true world
                   # pose -- RobotLens's TF consumer needs it to render the drone's
                   # full 3D flight pose instead of a flat identity link pose.
                   'publish_model_pose': 'true',
                   'use_pose_vector_msg': 'true',
               })
    add_plugin(model, 'gz-sim-joint-state-publisher-system',
               'gz::sim::systems::JointStatePublisher', {})
    return model


def write_open_interior_world():
    """Build the open_interior world and insert the dynamic multicopter drone."""
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
    return GENERATED_WORLD_PATH


def write_world_file():
    model = convert_urdf_to_model_block()
    model_block = ET.tostring(model, encoding='unicode')
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

    write_world_file()
    print(f'generated world: {GENERATED_WORLD_PATH}')

    gz_args = ['gz', 'sim', str(GENERATED_WORLD_PATH), '-r']
    if args.headless:
        gz_args += ['-s', '--headless-rendering']

    print('starting gz sim (' + ('headless)...' if args.headless else 'GUI)...'))
    log_handle = open(LOG_FILE, 'w', encoding='utf-8')
    proc = subprocess.Popen(
        gz_args, env=gz_env(), stdout=log_handle, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    PID_FILE.write_text(str(proc.pid))

    time.sleep(2)
    if proc.poll() is not None or pid_alive() is None:
        sys.exit(f'error: gz sim exited immediately; see {LOG_FILE}')
    print(f'OK: gz sim running (pid {proc.pid}). Log: {LOG_FILE}')
    print(f'arm the controller: {sys.argv[0]} arm')
    print('/cmd_vel drives body-frame velocity: linear.z up, angular.z yaw.')

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


def cmd_world(_args):
    require_gz()
    path = write_open_interior_world()
    print(path)


def cmd_arm(_args):
    """Publish true on /parrot_bebop_2/enable repeatedly for a window, so the
    MulticopterVelocityControl plugin arms whenever it subscribes. Uses
    gz-transport directly (like the CLI `gz topic -p`) because the ROS-sourced
    launch environment can hide the `gz` subcommand dispatcher."""
    from gz import transport13 as gz_transport
    from gz.msgs10 import boolean_pb2

    node = gz_transport.Node()
    publisher = node.advertise(ENABLE_TOPIC, boolean_pb2.Boolean)
    message = boolean_pb2.Boolean()
    message.data = True
    print(f'arming {MODEL_NAME} multicopter controller on {ENABLE_TOPIC} ...')
    deadline = time.monotonic() + ARM_DURATION_SECONDS
    while time.monotonic() < deadline:
        publisher.publish(message)
        time.sleep(ARM_PERIOD_SECONDS)
    print('armed.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    up_parser = sub.add_parser('up', help='start gz sim with the drone spawned')
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
    sub.add_parser('arm', help='arm the multicopter velocity controller')
    sub.add_parser('world', help='generate the open_interior world and print its path')
    args = parser.parse_args()

    {'up': cmd_up, 'down': cmd_down, 'status': cmd_status, 'arm': cmd_arm,
     'world': cmd_world}[args.action](args)


if __name__ == '__main__':
    main()
