#!/usr/bin/env python3
"""
pr2's "view PR2 in Gazebo" fixture runner: loads the vendored native
PR2_SDF17/model.sdf and spawns it, with physics enabled,
into an otherwise-empty gz-sim world so you can look at it in Gazebo's own
renderer.

This is a *separate, standalone* way to view PR2 -- distinct from
../launch/pr2.launch.py's fully-kinematic RobotLens fixture (see
../README.md's "Design" section). Nothing here talks to ROS or RobotLens; PR2
is just spawned as a real gz-sim model with real collision/physics and
gravity, so you can orbit around it in the Gazebo GUI. Don't run this
alongside pr2.launch.py expecting them to show "the same robot" --
they're two independent PR2s in two independent renderers.

No base drive plugin: PR2's real base is 4 independently-steered casters (8
wheels total, a holonomic drive), and gz-sim has no built-in plugin for that
shape of base the way it has DiffDrive for a two-wheel rover (see
../README.md's "Design" section for the fuller version of this reasoning).
So the wheels/casters just roll freely under physics and no joint is
actuated -- PR2 will sit wherever gravity settles it (upright on its base;
unpowered arms/head/torso will sag from their authored zero pose according
to each joint's <dynamics> damping, same as any unpowered real robot with
its motors off). That's still useful for exactly what this script is for:
seeing the real collision geometry and mass properties render and behave
under real physics, cross-checking against RobotLens's own rendering of the
same URDF.

The native SDF retains the PR2 geometry, collision shapes, and SDFormat 1.7
frame semantics directly. It encodes actuator effort limits as zero, however;
the runner fills those controller-limit values from the matching flattened
URDF without using the URDF for Gazebo geometry or topology.

Usage:
  robots_worlds/pr2/gazebo/run_pr2_gazebo.py up [--headless]
  robots_worlds/pr2/gazebo/run_pr2_gazebo.py down
  robots_worlds/pr2/gazebo/run_pr2_gazebo.py status

Environment note (same issue as ../../gazebo/run_gazebo_example.sh and
the other example fixtures): sourcing ROS 2 Jazzy can break
the `gz` CLI's subcommand dispatch, so `gz` is invoked with a whitelisted
environment (HOME/PATH/DISPLAY/XAUTHORITY only) rather than whatever shell
this script was run from.
"""

import argparse
import importlib.util
import os
import re
import shutil
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

EXAMPLE_DIR = Path(__file__).resolve().parents[1]
WORLDS_DIR = Path(__file__).resolve().parents[2] / 'worlds'
URDF_PATH = EXAMPLE_DIR / 'pr2.urdf'
SDF17_MODEL_DIR = EXAMPLE_DIR / 'PR2_SDF17'
SDF17_MODEL_PATH = SDF17_MODEL_DIR / 'model.sdf'


def _load_grasp_object():
    """Import the sibling grasp_object.py by path (not a package)."""
    spec = importlib.util.spec_from_file_location(
        'grasp_object', EXAMPLE_DIR / 'grasp_object.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


grasp_object = _load_grasp_object()
STATE_DIR = Path(os.environ.get('TMPDIR', '/tmp')) / 'robotlens_pr2_view_gazebo'
GENERATED_WORLD_PATH = STATE_DIR / 'pr2_view_world.sdf'
PID_FILE = STATE_DIR / 'gz_sim.pid'
LOG_FILE = STATE_DIR / 'gz_sim.log'
LOG_BRIDGE_PID_FILE = STATE_DIR / 'gazebo_log_bridge.pid'

# This is the only arm chain P0 validates and P1 will actuate. Keep this list
# in kinematic order: controller, MoveIt, and task configuration must use this
# exact ordering rather than relying on Gazebo's arbitrary joint-state order.
RIGHT_ARM_JOINTS = (
    'r_shoulder_pan_joint',
    'r_shoulder_lift_joint',
    'r_upper_arm_roll_joint',
    'r_elbow_flex_joint',
    'r_forearm_roll_joint',
    'r_wrist_flex_joint',
    'r_wrist_roll_joint',
)
# Matches pr2_right_arm.srdf's own <group_state name="home">. Without an
# `initial_position` override, add_right_arm_controllers()'s
# JointPositionController plugins default their setpoint to 0.0 -- which is
# inside every joint's URDF <limit> but OUTSIDE r_elbow_flex_joint/
# r_wrist_flex_joint's <safety_controller> soft limits (soft_upper_limit
# -0.15/-0.1, see pr2.urdf). MoveIt's CheckStartStateBounds planning-request
# adapter enforces those soft limits, not the hard ones, so with no P1
# adapter running (pick_place:=false) to move the arm first, it never left
# this all-zero spawn pose and every /plan_kinematic_path request was
# rejected before OMPL ever ran -- confirmed live via move_group's own
# (non-debug, always-on) ERROR log:
#   "Joint 'r_elbow_flex_joint' from the starting state is outside bounds
#    by: [...] should be in the range [-2.1213 ], [-0.15 ]."
# This surfaced as MotionPlanResponse::error_code::FAILURE with no other
# diagnostic, since the request never reached the OMPL pipeline that
# --log-level debug was pointed at. Fixed the same way as the left arm/head
# (add_left_arm_static_pose/add_head_static_pose): seed each joint's
# controller `initial_position` so physics never steps through 0.0 at all.
RIGHT_ARM_HOME_POSE = {
    'r_shoulder_pan_joint': 0.0,
    'r_shoulder_lift_joint': 0.0,
    'r_upper_arm_roll_joint': 0.0,
    'r_elbow_flex_joint': -0.2,
    'r_forearm_roll_joint': 0.0,
    'r_wrist_flex_joint': -0.15,
    'r_wrist_roll_joint': 0.0,
}
RIGHT_GRIPPER_COMMAND_JOINT = 'r_gripper_l_finger_joint'
# Mirrors RIGHT_ARM_JOINTS. Never actuated for real tasks (P1/P4 only ever
# validate the right arm, per the plan's scope) -- this is only a static
# resting pose so the left arm stops sagging under gravity like an
# unpowered real robot's would, for a less distracting view while testing
# the right arm. Not part of the validated pick-and-place chain.
LEFT_ARM_JOINTS = (
    'l_shoulder_pan_joint',
    'l_shoulder_lift_joint',
    'l_upper_arm_roll_joint',
    'l_elbow_flex_joint',
    'l_forearm_roll_joint',
    'l_wrist_flex_joint',
    'l_wrist_roll_joint',
)
# pr2.urdf's <safety_controller> soft limits for l_elbow_flex_joint/
# l_wrist_flex_joint are identical to the right arm's (both sides use the
# same flexion sign convention, unlike shoulder_lift/upper_arm_roll/
# shoulder_pan which mirror), so the right arm's already-validated "home"
# values are reused directly here. The shoulder is deliberately NOT at the
# right arm's (0, 0): l_shoulder_pan_joint sits at +0.7854 rad (45 deg,
# swung outward to the left) and l_shoulder_lift_joint at +1.1345 rad
# (65 deg, dropped down from level) -- both positive about their +z/+y
# axes, well inside soft limits -- for a relaxed, open stance instead of
# the dead straight droop (elbows bent -0.4, wrists curled -0.3).
LEFT_ARM_STATIC_POSE = {
    'l_shoulder_pan_joint': 0.7854,
    'l_shoulder_lift_joint': 1.1345,
    'l_upper_arm_roll_joint': 0.0,
    'l_elbow_flex_joint': -0.4,
    'l_forearm_roll_joint': 0.0,
    'l_wrist_flex_joint': -0.3,
    'l_wrist_roll_joint': 0.0,
}
# Same reasoning as LEFT_ARM_JOINTS: never actuated for real tasks, just
# stops the head from swinging freely under gravity. 0.0/0.0 (level, facing
# forward) is inside both joints' soft limits (head_pan_joint:
# [-2.857, 2.857], head_tilt_joint: [-0.3712, 1.29626]).
HEAD_JOINTS = ('head_pan_joint', 'head_tilt_joint')
HEAD_STATIC_POSE = {'head_pan_joint': 0.0, 'head_tilt_joint': 0.0}

# Same boilerplate as ../pr2_world.sdf, just with a `{model_block}` slot.
WORLD_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="pr2_view">
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


def gz_env():
    env = {'HOME': os.environ.get('HOME', ''), 'PATH': '/usr/bin:/bin'}
    # Both the external Gazebo process and the desktop app use gz-transport's
    # default partition. Deliberately do not inherit a shell-local
    # GZ_PARTITION here, because RobotLens launched from the desktop would
    # otherwise remain on default and Attach sees no services at all.
    for key in ('DISPLAY', 'XAUTHORITY'):
        if key in os.environ:
            env[key] = os.environ[key]
    return env


def require_gz():
    if shutil.which('gz', path=gz_env()['PATH']) is None:
        sys.exit("error: 'gz' (Gazebo Harmonic) not found on /usr/bin:/bin")


# Each of PR2's two grippers is a four-bar parallel linkage: pr2.urdf declares
# it with closed kinematic loops (e.g. both l_gripper_r_parallel_root_joint
# AND l_wrist_roll_joint claim l_wrist_roll_link as <child>). Rigid-body
# engines require a joint tree, not a graph, so sdformat/gz-sim keeps
# whichever joint for a given child it parsed first and silently drops every
# later one that targets the same child (logged as "child link already has a
# parent joint", confirmed empirically -- harmless when the dropped joint's
# own <parent> link has some OTHER valid attachment elsewhere). But for
# *_gripper_l_parallel_link and *_gripper_r_parallel_link specifically, every
# single joint that names them is one of these dropped ones -- meaning they
# end up on nobody's child list at all: fully unconstrained rigid bodies with
# real mass/collision, in free fall from frame one. Confirmed empirically: at
# real_time_factor 1 these are visibly diverging within seconds (one measured
# at z=-629m after ~11s of headless sim). See fix_orphaned_gripper_links().
_GRIPPER_PARALLEL_LINKS = tuple(
    f'{side}_gripper_{finger}_parallel_link' for side in ('l', 'r') for finger in ('l', 'r')
)


def fix_orphaned_gripper_links(model):
    """Welds each of _GRIPPER_PARALLEL_LINKS to its wrist via a synthesized
    fixed joint, so gz-sim's physics has something to hold it up instead of
    letting it free-fall. Uses no explicit <joint><pose> -- the link's own
    <pose> is already an absolute, correctly-resolved rest pose (sdformat
    computed it via the full URDF kinematic chain, including the loop joints
    it's about to drop; confirmed by inspecting the converted SDF: orphaned
    links carry a <pose> with no relative_to, i.e. already model-frame,
    unlike every properly-attached link's <pose relative_to="its own
    joint">0 0 0 0 0 0</pose>), so an unadorned fixed joint just locks that
    already-correct offset in place instead of needing it recomputed here.
    """
    joints = model.findall('joint')
    for link_name in _GRIPPER_PARALLEL_LINKS:
        # Prefer the "_root_joint" (wrist-side) candidate over "_tip_joint"
        # (finger-side) -- mechanically the parallel link is wrist-mounted.
        candidates = [j for j in joints
                     if j.find('parent').text == link_name and
                     j.get('name', '').endswith('_root_joint')]
        if not candidates:
            continue  # link isn't actually orphaned (or URDF names changed)
        attach_to = candidates[0].find('child').text

        fixed = ET.SubElement(model, 'joint', {
            'name': f'{link_name}_gz_view_fixed', 'type': 'fixed',
        })
        ET.SubElement(fixed, 'parent').text = attach_to
        ET.SubElement(fixed, 'child').text = link_name


def fix_wheel_friction(model):
    """Give PR2's wheel collisions an explicit floor-friction coefficient.

    RESOLVED 2026-08-15 (was: dead code). This originally only overwrote the
    text of *existing* <mu>/<mu2> elements via `collision.iter('mu')` --
    but the native PR2_SDF17 model's wheel <collision> blocks have no
    <surface> element at all (confirmed by inspecting the generated world
    SDF directly: just <geometry>, nothing else), so `.iter('mu')` found
    nothing and this function silently did nothing on every run. The
    docstring's claim about "the URDF's Gazebo-classic mu1=mu2=100.0"
    welding the wheels was based on the URDF's <gazebo reference> tags,
    which never apply here either -- this fixture's geometry comes from the
    native SDF17 file, not a URDF-to-SDF conversion, so those tags are
    inert. Real symptom this masked: with no friction element at all, the
    wheels get whatever gz-sim/dartsim's own default contact friction is,
    which combined with this fixture's soft contact penetration (~2.4 cm
    sink under the robot's weight, see fix_wheel_effort) was not enough to
    stop them slipping in place under real torque -- wheels spin, the base
    barely moves. Fixed by actually creating the <surface><friction><ode>
    element per wheel collision instead of only trying to edit one.
    """
    for link in model.findall('link'):
        if not link.get('name', '').endswith('_wheel_link'):
            continue
        for collision in link.findall('collision'):
            surface = collision.find('surface')
            if surface is None:
                surface = ET.SubElement(collision, 'surface')
            friction = surface.find('friction')
            if friction is None:
                friction = ET.SubElement(surface, 'friction')
            ode = friction.find('ode')
            if ode is None:
                ode = ET.SubElement(friction, 'ode')
            mu = ode.find('mu')
            if mu is None:
                mu = ET.SubElement(ode, 'mu')
            mu.text = '1.0'
            mu2 = ode.find('mu2')
            if mu2 is None:
                mu2 = ET.SubElement(ode, 'mu2')
            mu2.text = '1.0'


def fix_wheel_effort(model):
    """Give the wheel joints enough motor torque to overcome the floor contact.

    The URDF's <limit effort="7"/> becomes a DART SERVO force limit;
    gz-physics SERVO velocity commands respect that limit regardless of
    what a controller plugin's own cmd_max requests, and the soft DART
    contact lets the 116 kg PR2 sink into the floor plane, whose rolling
    resistance exceeds 7 N.m. The wheels then spin a fraction of a radian
    and stall (confirmed empirically). Raise the limit so a commanded wheel
    can actually roll the robot forward.

    (2026-08-15: previously also covered *_rotation_joint, back when
    add_caster_drive_controllers actively steered the casters with a
    JointPositionController. That joint is now a permanent <fixed> weld --
    see that function's docstring -- so it has no controllable effort limit
    left to raise.)
    """
    for joint in model.findall('joint'):
        name = joint.get('name', '')
        if not name.endswith('_wheel_joint'):
            continue
        limit = joint.find('axis/limit')
        if limit is None:
            continue
        effort = limit.find('effort')
        if effort is not None:
            effort.text = '200'
        velocity = limit.find('velocity')
        if velocity is not None:
            velocity.text = '15'


def validate_right_arm_model(model):
    """Fail conversion when the PR2 model cannot safely support P1.

    The URDF-to-SDF conversion can silently change topology when it encounters
    unsupported constructs. Validate the exact arm chain, its limits, and the
    collision/inertia-bearing child links before adding controllers. The
    gripper is checked only as a single command joint: its closed-loop mimic
    mechanism is deliberately not accepted as independently controllable.
    """
    joints_by_name = {joint.get('name'): joint for joint in model.findall('joint')}
    links_by_name = {link.get('name'): link for link in model.findall('link')}
    missing = [name for name in RIGHT_ARM_JOINTS if name not in joints_by_name]
    if missing:
        raise RuntimeError(f'converted PR2 model is missing right-arm joints: {missing}')

    for name in RIGHT_ARM_JOINTS:
        joint = joints_by_name[name]
        if joint.get('type') not in ('revolute', 'continuous'):
            raise RuntimeError(f'{name} has unsupported joint type {joint.get("type")!r}')
        axis = joint.find('axis')
        limit = joint.find('axis/limit')
        child = joint.findtext('child')
        if axis is None or not axis.findtext('xyz') or limit is None:
            raise RuntimeError(f'{name} is missing axis or limit data after conversion')
        if limit.findtext('effort') is None or limit.findtext('velocity') is None:
            raise RuntimeError(f'{name} is missing effort or velocity limit after conversion')
        child_link = links_by_name.get(child)
        if child_link is None:
            raise RuntimeError(f'{name} references missing child link {child!r}')
        if child_link.find('inertial') is None or child_link.find('collision') is None:
            raise RuntimeError(f'{name} child link {child!r} lacks inertia or collision data')

    gripper = joints_by_name.get(RIGHT_GRIPPER_COMMAND_JOINT)
    if gripper is None or gripper.get('type') != 'revolute':
        raise RuntimeError(
            f'converted PR2 model lacks revolute gripper joint {RIGHT_GRIPPER_COMMAND_JOINT}')
    if gripper.find('axis/limit/lower') is None or gripper.find('axis/limit/upper') is None:
        raise RuntimeError(f'{RIGHT_GRIPPER_COMMAND_JOINT} lacks finite position limits')
    gripper_child = links_by_name.get(gripper.findtext('child'))
    if gripper_child is None or gripper_child.find('collision') is None:
        raise RuntimeError(f'{RIGHT_GRIPPER_COMMAND_JOINT} child link lacks collision data')


def add_caster_drive_controllers(model):
    """Attach real Gazebo joint controllers for PR2's caster base.

    Wheel velocity commands use JointController's standard per-joint cmd_vel
    topics; steering position commands use explicit, stable topics consumed by
    pr2_physics_controller.py.  The model remains dynamic: contact forces,
    gravity, friction, and the rigid-body solver determine its motion.

    RESOLVED 2026-08-15 (was: all four casters spin chaotically from the
    moment physics starts, even with zero /cmd_vel ever published --
    steering joint velocities oscillating up to +/-10 rad/s, positions
    drifting continuously; commanding pure +X drove the base sideways in Y
    instead). Root cause: `gz-sim-joint-controller-system` without
    `use_force_commands` calls `SetVelocity` directly every step (see
    JointController.cc's `else if (!useForceCommands)` branch) -- a
    kinematic override, not a physical torque. DART must inject whatever
    constraint force is needed to hit that exact velocity every step,
    including through the ground-contact friction that couples the wheel to
    the steering joint above it, which can be arbitrarily large and
    discontinuous. That unphysical, torque-unlimited coupling is what excited
    the steering DOF, not anything about the commanded direction. Fixed by
    giving the wheel controller `use_force_commands` and a real PID gain, so
    DART treats it as an ordinary torque-limited actuator instead of a
    kinematic constraint.

    The wheel joint's own URDF <limit effort="7"> was tried first and proved
    too weak: same empirically-confirmed situation as
    RIGHT_ARM_EFFORT_OVERRIDE_NM above (a real per-joint torque rating with
    no gravity/friction compensation in this fixture's plain PD loop can't
    actually move the load it was rated for on real PR2 hardware) -- with
    cmd_max=7 the wheels stayed pinned near 0 rad/s indefinitely under a
    full forward command. Raising cmd_max to 25 (and p_gain to 15.0) below
    gets the wheels themselves spinning at the commanded rate again
    (confirmed via joint-state readback), which is what this function is
    responsible for.

    STILL OPEN as of 2026-08-15: with the wheels spinning at the commanded
    rate and the steering loop below holding heading (within a few degrees,
    stable, not drifting -- needed raising steering to p_gain=120/d_gain=20
    and extending fix_wheel_effort to the *_rotation_joint's own SDF hard
    limit too, since a plugin's cmd_max is silently capped by that limit
    regardless of what the plugin requests), the base still barely
    translates. Ruled out so far, each confirmed live:
    - Friction coefficient: `fix_wheel_friction()` turned out to be dead
      code (see its own docstring) and was fixed to actually create a
      <surface><friction><ode><mu> element; tested at both mu=1.0 and an
      extreme mu=5.0 with no measurable difference in how far the base
      travels.
    - Steering direction/coordination: forcing all four casters to a fixed
      angle (bypassing pr2_physics_controller.py's own steer computation
      entirely, straight over gz-transport) and spinning the wheels still
      produced only ~2 cm of travel over 2 s of full-speed commanded wheel
      velocity (vs. the ~0.6 m a true rolling contact at that rate would
      cover) -- ruling out any bug in the commanded angle itself.
    - Wheel/floor friction coefficient and contact stiffness: tested up to
      mu=5.0 on both surfaces with no measurable difference.
    - A nearby obstacle: PR2 spawns at the warehouse world's origin, and
      nothing else in open_interior.sdf is placed within 2 m of it.

    RESOLVED 2026-08-15, later same day (steering): the STEERING CONTROL
    LOOP itself, not friction or contact physics, was preventing the base
    from translating up to this point. Diagnostic proof: with the
    caster_rotation_joint welded to a <fixed> joint (steering DOF physically
    removed, zero possibility of any reaction-torque coupling or per-corner
    drift) and the exact same wheel friction/torque/gains as every failed
    attempt above, an isolated test world (no arm, no graspable object, just
    this function's wheel controllers) covered 0.34 m of real forward
    travel in 3 s at 3.8 rad/s -- close to the ~0.6 m true rolling contact
    predicts. Even a stable-*looking* steering loop (p_gain=120, holding
    within a few degrees in point samples) apparently still carries enough
    closed-loop lag/transient error across four independently-actuated
    corners to cancel most of the net drive force in practice -- getting
    genuinely zero-lag steering tracking on four separate joints each
    fighting real wheel reaction torque is a much harder control problem
    than it looked. Given real PR2 keyboard teleop only ever commands
    forward/back + turn-in-place by default (holonomic strafing is a
    Shift+Left/Right bonus in EditorShell's teleop, not the primary
    interaction -- see core::computeTeleopTwist), this function now welds
    all four casters forward-facing and fixed instead of actively steering
    them, trading holonomic strafing for (hopefully) working forward/turn
    driving. pr2_physics_controller.py's kinematics changed to match
    (skid-steer: differential wheel speed per side, no steer angle at all).

    RESOLVED 2026-08-15, still later: welding steering fixed the isolated
    test world, but the REAL fixture (pr2_robotlens_gazebo.launch.py, full
    warehouse + arm + graspable object) still barely translated -- A/B
    tested live with everything else identical: `with_graspable_object=True`
    covered <2 cm in a 2 s full forward command, `=False` covered ~0.56 m.
    Root cause: `add_object_attachment_plugin`'s DetachableJoint starts
    attached (gz-sim's own default), and pr2_grasp_controller.py's
    startup-detach retry logic does not reliably win that race before a
    drive command arrives -- with the gripper still rigidly attached to an
    object resting on a static table, the whole assembly behaves like the
    base is anchored, regardless of anything this function does. Fixed in
    pr2_robotlens_gazebo.launch.py's prepare_world(): the graspable object
    (and pr2_grasp_controller.py, which has nothing to do without it) is now
    only spawned when `pick_place:=true`, since it's only ever meaningful
    for the P3 pick-place task anyway.

    STILL OPEN as of 2026-08-15: with all of the above fixed, forward
    (~0.27 m per 2 s hold at 0.3 m/s) and backward driving work and were
    confirmed live end to end through the real /cmd_vel -> pr2_physics_
    controller.py -> gz-transport path. Turning (angular.z) does not --
    tested at both a modest 0.5 rad/s and a strong 2.0 rad/s command, and at
    both the normal mu=1.0 and an elevated mu=3.0 (testing whether more
    lateral grip helps a skid-steer turn), the base's orientation measured
    zero change in every case, confirmed via both /tf and raw gz-transport
    /model/pr2/pose. Wheel joint state during a turn command shows some
    wheels tracking their differential target and others staying near zero
    (an inconsistent, partial pattern, not a clean "nothing happens" or "all
    wheels correct" split), suggesting the four-corner skid-steer geometry
    itself (an 8-wheel rectangular footprint, not 2 wheels) may need a
    different kinematic model or per-wheel tuning than plain differential
    left/right speed -- not yet investigated further.
    """
    for corner in ('fl', 'fr', 'bl', 'br'):
        rotation_joint = next(
            j for j in model.findall('joint')
            if j.get('name') == f'{corner}_caster_rotation_joint')
        rotation_joint.set('type', 'fixed')
        axis = rotation_joint.find('axis')
        if axis is not None:
            rotation_joint.remove(axis)
        # The `name` must be the exact plugin symbol the library registers
        # (gz-sim strips the `gz::sim::v8::` namespace): SystemLoader fails
        # with "library does not contain requested plugin" if a suffix like
        # `_fl` is appended here. Duplicate plugin names are fine -- each
        # <plugin> element becomes its own system instance, and the
        # per-instance <joint_name> parameter tells them apart.
        for side in ('l', 'r'):
            wheel = ET.SubElement(model, 'plugin', {
                'filename': 'gz-sim-joint-controller-system',
                'name': 'gz::sim::systems::JointController',
            })
            ET.SubElement(wheel, 'joint_name').text = f'{corner}_caster_{side}_wheel_joint'
            ET.SubElement(wheel, 'use_force_commands').text = 'true'
            # See the wheel-effort-override note in this function's
            # docstring: the real URDF effort=7 rating can't overcome this
            # fixture's rolling/static contact friction at all.
            ET.SubElement(wheel, 'p_gain').text = '15.0'
            ET.SubElement(wheel, 'i_gain').text = '0.0'
            ET.SubElement(wheel, 'd_gain').text = '0.1'
            ET.SubElement(wheel, 'cmd_max').text = '25'
            ET.SubElement(wheel, 'cmd_min').text = '-25'


# The right-arm light-damping joints' real URDF damping (0.1 N*m*s/rad) is
# the lightest of any right-arm joint by an order of magnitude. The DART
# solver's residual chatter on these fast, low-inertia axes has almost
# nothing to dissipate into, and its worst moments exceed ARM_GOAL_TOLERANCE
# -- the r_wrist_roll_joint instability documented in add_right_arm_
# controllers' docstring. These joints get a common, empirically-validated
# damping of 2.0 N*m*s/rad (elbow flex is 1.0, shoulder pan/lift are 10.0)
# written into the generated SDF, which makes the chatter settle within
# tolerance without touching any joint's limits or effort ceiling.
RIGHT_ARM_LIGHT_DAMPING_OVERRIDE_NM_S_PER_RAD = 2.0
RIGHT_ARM_LIGHT_DAMPING_JOINTS = (
    'r_upper_arm_roll_joint', 'r_forearm_roll_joint',
    'r_wrist_flex_joint', 'r_wrist_roll_joint',
)


def fix_right_arm_light_damping(model):
    """Raise the right-arm light-damping joints' SDF damping (see the
    override constants above). The converted SDF carries pr2.urdf's
    <dynamics> through into each joint's <axis><dynamics><damping>, so the
    element to patch is that nested one; fall back to inserting the missing
    <dynamics>/<damping> and validate the parsed value is a number before
    replacing it."""
    joints_by_name = {joint.get('name'): joint for joint in model.findall('joint')}
    for joint_name in RIGHT_ARM_LIGHT_DAMPING_JOINTS:
        joint = joints_by_name[joint_name]
        axis = joint.find('axis')
        if axis is None:
            raise RuntimeError(f'{joint_name} has no SDF <axis> to hold damping')
        dynamics = axis.find('dynamics')
        if dynamics is None:
            dynamics = ET.SubElement(axis, 'dynamics')
        damping = dynamics.find('damping')
        if damping is None:
            damping = ET.SubElement(dynamics, 'damping')
        try:
            float(damping.text)
        except (TypeError, ValueError):
            raise RuntimeError(
                f'{joint_name} has unparsable SDF damping {damping.text!r}')
        damping.text = str(RIGHT_ARM_LIGHT_DAMPING_OVERRIDE_NM_S_PER_RAD)


def add_right_arm_controllers(model):
    """Attach position controllers to the P0-validated right arm and gripper.

    Each controller has an explicit topic instead of the plugin default. This
    keeps the Gazebo transport contract stable for the forthcoming ROS
    trajectory adapter and prevents accidental commands to mimic joints.
    Position limits remain enforced by that adapter; `cmd_min`/`cmd_max` bound
    the PID effort requested from the physics engine, not joint position --
    each is set to that joint's own URDF <limit effort> (RIGHT_ARM_EFFORT_
    OVERRIDE_NM excepted, see below), never higher otherwise. This is a plain
    PD position controller with no gravity compensation, so it carries
    steady-state droop under load; the gain below is tuned empirically to
    keep that droop within the P1 adapter's ARM_GOAL_TOLERANCE for the
    fixture's validated home/ready poses.

    RESOLVED 2026-08-12 (was: r_shoulder_lift_joint settles near ~0.94-0.98
    rad regardless of commanded target or this plugin's cmd_max, even up to
    2000 N*m). Root cause: gz-sim's DART backend enforces each joint's own
    SDF <axis><limit><effort> -- carried through unmodified from pr2.urdf's
    <limit effort="30"> -- as a hard per-step actuator-torque ceiling,
    completely independent of the JointPositionController plugin's cmd_max
    parameter. Every earlier experiment (cmd_max 30/100/2000) only edited
    the plugin's own parameter and never that separate SDF element, so all
    of them were silently capped at the same real 30 N*m ceiling the whole
    time -- confirmed empirically by holding cmd_max fixed and instead
    raising the *joint's* <axis><limit><effort> alone, which immediately let
    it track a commanded target. (That saturation-vs-gain confound is also
    what made raising p_gain look like it worsened the error: a
    proportional term that's already clipped at the ceiling doesn't get any
    stronger, so the softer secondary effects of a higher gain -- more
    aggressive but no bigger -- read as "worse" without ever probing what
    was really bounding it.) 30 N*m is r_shoulder_lift_joint's real
    URDF-declared rating, but this fixture's controller has no gravity
    compensation term (unlike the real PR2's actual joint firmware) and this
    is the one right-arm joint that alone carries the whole extended arm's
    gravity moment -- with zero actuation it free-falls to a passive
    equilibrium around ~1.0-1.05 rad and 30 N*m of PD authority can only
    nudge it about 0.1 rad off of that. RIGHT_ARM_EFFORT_OVERRIDE_NM raises
    only this joint's ceiling (SDF axis limit and plugin cmd_max together)
    to an empirically-validated 300 N*m, paired with a raised p_gain/d_gain
    (kept at the other joints' 10:1 ratio) so steady-state droop lands
    within ARM_GOAL_TOLERANCE (0.05 rad) at both the "home" (0 rad) and
    "ready" (0.3 rad) SRDF poses, and at a real MoveIt-planned/IK-solved
    grasp pose near the P3 graspable object -- confirmed live, including
    through a real `move_group` (not just direct Gazebo commanding).
    r_shoulder_lift_joint's gain deliberately was NOT raised past 1200 to
    chase a tighter residual at that grasp pose: 1800 and 2500 were each
    tried and reverted after they measurably worsened r_shoulder_pan_joint
    and r_elbow_flex_joint's tracking on an unrelated, previously-clean
    "home" goal -- a stiffer shoulder_lift servo transmits more disturbance
    through the kinematic chain to its parent and downstream joints than a
    softer one does. Prefer picking an achievable target pose over
    reflexively raising gain again.

    FOLLOW-UP 2026-08-12: the same live reach-and-grasp check found
    r_upper_arm_roll_joint and r_forearm_roll_joint hit the same
    saturated-ceiling defect away from the home/ready family (their axes
    carry little gravity torque when the arm is roughly extended, which is
    all home/ready exercise, but a bent elbow/wrist swings the downstream
    mass off-axis, adding real torque their 30/10 N*m ratings can't cover).
    Given the same fix (SDF axis effort + plugin cmd_max raised together),
    confirmed converging afterward. Their gain was left at the default
    400/40 rather than matching shoulder_lift's 1200 -- both needed far
    less extra torque than shoulder_lift did, and the coupling regression
    above is a reason to prefer the smallest gain that works, not the
    largest that's merely safe.

    UNRESOLVED as of 2026-08-12, narrowed but not fixed: r_wrist_roll_joint
    intermittently fails to hold a commanded target, and unlike every joint
    above this is NOT the saturated-ceiling defect and NOT a controller-gain
    problem. Isolated with the plugin's cmd_max/p_gain/d_gain all set to
    literal 0 (i.e. the JointPositionController applying zero commanded
    effort) in an otherwise-empty world (no warehouse, no graspable object,
    no move_group, nothing else commanding anything): the joint's reported
    velocity still oscillates at high frequency, alternating sign roughly
    every /joint_states sample at ~1.7-2.2 rad/s, with this session's
    p_gain 400 (default), 50, and 0 all producing materially the same
    oscillation -- ruling out both this function's plugin entirely (it
    cannot be the cause of motion it isn't applying any torque to produce)
    and the earlier gripper-mimic-joint defect (fixed separately below;
    still worth keeping fixed, but confirmed NOT the source of this -- the
    oscillation predates and is independent of it, reproducing even with
    the whole gripper substituted for an empty world). The link's lumped
    mass/inertia (r_wrist_roll_link, which the URDF's own comment flags as
    "dummy masses, to be removed -- wrist roll masses are on gripper_palm")
    was checked in the generated SDF and is physically sane after lumping
    (0.681 kg, a well-conditioned inertia tensor, no degenerate/near-zero
    principal moments) -- ruling out a bad-lumping explanation too. This
    now looks like a genuine gz-sim/DART numerical solver instability
    specific to this joint's dynamics (a continuous joint with real URDF
    damping of only 0.1, the lightest of any right-arm joint), independent
    of any Gazebo-side plugin this function controls. In practice: the
    joint sometimes settles close enough to a commanded target to pass
    ARM_GOAL_TOLERANCE, sometimes doesn't, unpredictably, and the position
    error observed when it fails (up to ~0.9 rad seen against a large,
    multi-joint-extreme target) is plausibly this same chatter's amplitude
    landing wherever it happens to be sampled, not the joint being "stuck"
    the way the RESOLVED defects above were. Fixing this would need
    physics-engine-level investigation (solver iteration counts, joint
    damping/friction tuning, or a DART-specific workaround) rather than
    anything this function's controller parameters can address -- don't
    spend more time on cmd_max/p_gain/d_gain experiments for this joint,
    they've been tried at multiple values with no effect on the underlying
     oscillation.
    """
    # RESOLVED 2026-08-13 via joint damping tuning (the fix path the
    # UNRESOLVED note above names): the right-arm light-damping joints
    # (upper_arm_roll, forearm_roll, wrist_flex, wrist_roll) all carry the
    # URDF's real 0.1 N*m*s/rad damping, the lightest of any right-arm joint
    # by an order of magnitude (shoulder pan/lift are 10.0, elbow flex is
    # 1.0), so the DART solver's residual chatter on these fast, low-inertia
    # axes has almost nothing to dissipate into and its worst moments
    # exceed ARM_GOAL_TOLERANCE. fix_right_arm_light_damping() raises these
    # joints' SDF <dynamics><damping> to a common 2.0 N*m*s/rad in the
    # generated world, matching the elbow's order of magnitude. This is a
    # solver-stability parameter only -- no change to any joint's limits or
    # effort ceiling. Verified live against the P4 pre-grasp (the pose that
    # previously failed tolerance the most): the joint now settles and
    # holds its commanded target within tolerance on repeated runs.
    #
    # Empirically tuned (see RESOLVED/FOLLOW-UP notes above): these need
    # more than their real URDF effort rating to hold the arm's weight
    # without gravity compensation; the other four don't, in every
    # configuration tested so far -- r_wrist_roll_joint is the exception,
    # see the damping fix above; it deliberately has no effort override
    # here.
    # (effort_nm, p_gain, d_gain) -- d_gain keeps the other joints' 10:1
    # p_gain:d_gain ratio.
    RIGHT_ARM_EFFORT_OVERRIDE_NM = {
        'r_shoulder_lift_joint': (300.0, 1200.0, 120.0),
        'r_upper_arm_roll_joint': (60.0, 400.0, 40.0),
        'r_forearm_roll_joint': (100.0, 400.0, 40.0),
    }
    joints_by_name = {joint.get('name'): joint for joint in model.findall('joint')}
    for joint_name in RIGHT_ARM_JOINTS:
        override = RIGHT_ARM_EFFORT_OVERRIDE_NM.get(joint_name)
        if override is None:
            effort = joints_by_name[joint_name].findtext('axis/limit/effort')
            p_gain, d_gain = '400.0', '40.0'
        else:
            effort_nm, p_gain_nm, d_gain_nm = override
            effort = str(effort_nm)
            p_gain, d_gain = str(p_gain_nm), str(d_gain_nm)
            # DART enforces this SDF-level ceiling independently of the
            # plugin's own cmd_max below (see RESOLVED note above) -- both
            # must be raised together.
            joints_by_name[joint_name].find('axis/limit/effort').text = effort
        controller = ET.SubElement(model, 'plugin', {
            'filename': 'gz-sim-joint-position-controller-system',
            'name': 'gz::sim::systems::JointPositionController',
        })
        ET.SubElement(controller, 'joint_name').text = joint_name
        ET.SubElement(controller, 'topic').text = f'/pr2/right_arm/{joint_name}/position_cmd'
        ET.SubElement(controller, 'p_gain').text = p_gain
        ET.SubElement(controller, 'd_gain').text = d_gain
        ET.SubElement(controller, 'cmd_max').text = effort
        ET.SubElement(controller, 'cmd_min').text = f'-{effort}'
        # See RIGHT_ARM_HOME_POSE above: without this, the plugin's implicit
        # 0.0 default setpoint violates elbow_flex/wrist_flex's soft limits
        # and MoveIt's CheckStartStateBounds adapter rejects every plan.
        ET.SubElement(controller, 'initial_position').text = str(RIGHT_ARM_HOME_POSE[joint_name])

    gripper = ET.SubElement(model, 'plugin', {
        'filename': 'gz-sim-joint-position-controller-system',
        'name': 'gz::sim::systems::JointPositionController',
    })
    ET.SubElement(gripper, 'joint_name').text = RIGHT_GRIPPER_COMMAND_JOINT
    ET.SubElement(gripper, 'topic').text = '/pr2/right_gripper/position_cmd'
    ET.SubElement(gripper, 'p_gain').text = '5.0'
    ET.SubElement(gripper, 'd_gain').text = '0.2'
    ET.SubElement(gripper, 'cmd_max').text = '5.0'
    ET.SubElement(gripper, 'cmd_min').text = '-5.0'

    # UNRESOLVED-turned-ROOT-CAUSE, 2026-08-12: r_wrist_roll_joint's
    # intermittent tracking failures (see the r_wrist_roll_joint FOLLOW-UP
    # note above -- left there for history, but this is the actual fix)
    # traced to three OTHER joints, not wrist_roll itself.
    # r_gripper_r_finger_joint, r_gripper_l_finger_tip_joint, and
    # r_gripper_r_finger_tip_joint are each declared in pr2.urdf as
    # <mimic joint="r_gripper_l_finger_joint" multiplier="1" offset="0"/>
    # -- but gz-sim's DART backend does not implement mimic constraints at
    # all (it logs "the chosen physics engine does not support mimic
    # constraints, so no constraint will be created" and silently drops
    # it). That leaves all three completely unactuated and unsynchronized
    # with the one joint this fixture actually commands, so they free-swing
    # under gravity/contact -- confirmed empirically: r_gripper_r_finger_
    # joint/r_gripper_r_finger_tip_joint's velocity was seen oscillating
    # +/-0.3-0.5 rad/s every /joint_states sample (never settling), while
    # the untouched, uncommanded left gripper's equivalent joints sat
    # perfectly still. r_wrist_roll_joint is simply the nearest actuated
    # joint with the least torque authority (10 N*m, the lowest of any
    # right-arm joint) to absorb that disturbance, so it was the one
    # visibly failing tolerance -- raising *its* effort never addressed the
    # real source and was the wrong joint to chase.
    #
    # Fix: give each mimic-in-name-only joint its own JointPositionController
    # subscribed to the SAME command topic as the real commanded joint, with
    # multiplier=1/offset=0 (matching the URDF's own mimic parameters
    # exactly) -- so it tracks the same target the mimic constraint would
    # have enforced, manually, since gz-sim can't do it natively.
    for mimic_joint in ('r_gripper_r_finger_joint', 'r_gripper_l_finger_tip_joint',
                        'r_gripper_r_finger_tip_joint'):
        mimic_controller = ET.SubElement(model, 'plugin', {
            'filename': 'gz-sim-joint-position-controller-system',
            'name': 'gz::sim::systems::JointPositionController',
        })
        ET.SubElement(mimic_controller, 'joint_name').text = mimic_joint
        ET.SubElement(mimic_controller, 'topic').text = '/pr2/right_gripper/position_cmd'
        ET.SubElement(mimic_controller, 'p_gain').text = '5.0'
        ET.SubElement(mimic_controller, 'd_gain').text = '0.2'
        ET.SubElement(mimic_controller, 'cmd_max').text = '5.0'
        ET.SubElement(mimic_controller, 'cmd_min').text = '-5.0'


def add_left_arm_static_pose(model):
    """Hold the left arm at LEFT_ARM_STATIC_POSE instead of letting it sag
    unpowered under gravity (see this module's docstring for why the base
    fixture leaves unpowered joints to sag -- that's still correct for
    joints nothing else here cares about, but a fully-drooped left arm
    next to the right arm being actively tested reads as broken rather
    than intentional). Not part of the validated right-arm pick-and-place
    chain -- purely cosmetic, static, and never commanded to move.

    Reuses the SAME per-joint effort/gain overrides already validated on
    the right arm's identical joints (RIGHT_ARM_EFFORT_OVERRIDE_NM in
    add_right_arm_controllers): l_shoulder_lift_joint and
    l_upper_arm_roll_joint/l_forearm_roll_joint need more than their real
    URDF effort rating to hold a no-gravity-compensation PD loop, for the
    same reason the right arm's did (see that function's RESOLVED/
    FOLLOW-UP docstring notes) -- these are physically identical joints
    (mirrored, same masses/inertias/dynamics), so the same fix applies.
    """
    joints_by_name = {joint.get('name'): joint for joint in model.findall('joint')}
    left_override_nm = {
        'l_shoulder_lift_joint': (300.0, 1200.0, 120.0),
        'l_upper_arm_roll_joint': (60.0, 400.0, 40.0),
        'l_forearm_roll_joint': (100.0, 400.0, 40.0),
    }
    for joint_name in LEFT_ARM_JOINTS:
        override = left_override_nm.get(joint_name)
        if override is None:
            effort = joints_by_name[joint_name].findtext('axis/limit/effort')
            p_gain, d_gain = '400.0', '40.0'
        else:
            effort_nm, p_gain_nm, d_gain_nm = override
            effort = str(effort_nm)
            p_gain, d_gain = str(p_gain_nm), str(d_gain_nm)
            joints_by_name[joint_name].find('axis/limit/effort').text = effort
        controller = ET.SubElement(model, 'plugin', {
            'filename': 'gz-sim-joint-position-controller-system',
            'name': 'gz::sim::systems::JointPositionController',
        })
        ET.SubElement(controller, 'joint_name').text = joint_name
        ET.SubElement(controller, 'topic').text = f'/pr2/left_arm/{joint_name}/position_cmd'
        ET.SubElement(controller, 'p_gain').text = p_gain
        ET.SubElement(controller, 'd_gain').text = d_gain
        ET.SubElement(controller, 'cmd_max').text = effort
        ET.SubElement(controller, 'cmd_min').text = f'-{effort}'
        # No P1-style adapter exists for the left arm (never actuated for
        # real tasks), so nothing ever publishes to the topic above --
        # `initial_position` (confirmed present in gz-sim's
        # JointPositionController.cc: read via
        # `_sdf->HasElement("initial_position")` and stored directly as
        # the controller's internal setpoint) sets the target once at
        # plugin-load time instead, before physics ever steps, so the arm
        # never passes through the plugin's own 0.0 default at all.
        ET.SubElement(controller, 'initial_position').text = str(LEFT_ARM_STATIC_POSE[joint_name])


def add_head_static_pose(model):
    """Hold the head level and forward-facing (HEAD_STATIC_POSE) instead of
    letting head_pan_joint/head_tilt_joint swing freely under gravity --
    same reasoning and mechanism as add_left_arm_static_pose. No override
    needed: unlike the arm, empirically confirmed to hold within a small
    fraction of a radian at each joint's own real URDF effort rating."""
    joints_by_name = {joint.get('name'): joint for joint in model.findall('joint')}
    for joint_name in HEAD_JOINTS:
        effort = joints_by_name[joint_name].findtext('axis/limit/effort')
        controller = ET.SubElement(model, 'plugin', {
            'filename': 'gz-sim-joint-position-controller-system',
            'name': 'gz::sim::systems::JointPositionController',
        })
        ET.SubElement(controller, 'joint_name').text = joint_name
        ET.SubElement(controller, 'topic').text = f'/pr2/head/{joint_name}/position_cmd'
        ET.SubElement(controller, 'p_gain').text = '400.0'
        ET.SubElement(controller, 'd_gain').text = '40.0'
        ET.SubElement(controller, 'cmd_max').text = effort
        ET.SubElement(controller, 'cmd_min').text = f'-{effort}'
        ET.SubElement(controller, 'initial_position').text = str(HEAD_STATIC_POSE[joint_name])


def add_object_attachment_plugin(model):
    """Attach the P3 grasp mechanism: a fixed joint, toggled at runtime, from
    the right-gripper palm to the graspable object.

    gz-sim-detachable-joint-system requires the parent link to live in the
    model that hosts the plugin (a tree-topology requirement -- kinematic
    loops aren't supported), so this must be a plugin on PR2's own model,
    naming the object only by its model/link name. It also starts attached
    by that system's own default (see tutorials/detachable_joints.md), so
    the grasp controller must send one detach on startup before the object
    is actually free -- this plugin block alone does not leave it free.
    """
    plugin = ET.SubElement(model, 'plugin', {
        'filename': 'gz-sim-detachable-joint-system',
        'name': 'gz::sim::systems::DetachableJoint',
    })
    ET.SubElement(plugin, 'parent_link').text = grasp_object.DETACHABLE_JOINT_PARENT_LINK
    ET.SubElement(plugin, 'child_model').text = grasp_object.OBJECT_MODEL_NAME
    ET.SubElement(plugin, 'child_link').text = grasp_object.OBJECT_LINK_NAME
    ET.SubElement(plugin, 'attach_topic').text = grasp_object.DETACHABLE_JOINT_ATTACH_TOPIC
    ET.SubElement(plugin, 'detach_topic').text = grasp_object.DETACHABLE_JOINT_DETACH_TOPIC
    ET.SubElement(plugin, 'output_topic').text = grasp_object.DETACHABLE_JOINT_STATE_TOPIC


def build_table_model_xml():
    """Static table the graspable object rests on (plan section 4, P3)."""
    x, y, z = grasp_object.TABLE_POSE_XYZ
    sx, sy, sz = grasp_object.TABLE_SIZE_M
    return f'''<model name="{grasp_object.TABLE_MODEL_NAME}">
  <static>true</static>
  <pose>{x} {y} {z} 0 0 0</pose>
  <link name="{grasp_object.TABLE_LINK_NAME}">
    <collision name="collision">
      <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      <material>
        <ambient>0.55 0.4 0.25 1</ambient>
        <diffuse>0.55 0.4 0.25 1</diffuse>
      </material>
    </visual>
  </link>
</model>'''


def build_graspable_object_xml():
    """The dynamic object P3 grasps -- collision, mass, and friction match
    exactly what planning_scene_sync.py adds to the MoveIt planning scene,
    since both import grasp_object.py's constants (plan section 4, P3
    item 2: one source of truth)."""
    x, y, z = grasp_object.OBJECT_INITIAL_POSE_XYZ
    sx, sy, sz = grasp_object.OBJECT_SIZE_M
    mass = grasp_object.OBJECT_MASS_KG
    friction = grasp_object.OBJECT_FRICTION
    # Solid-box inertia tensor: I_xx = m*(sy^2+sz^2)/12, etc.
    ixx = mass * (sy ** 2 + sz ** 2) / 12.0
    iyy = mass * (sx ** 2 + sz ** 2) / 12.0
    izz = mass * (sx ** 2 + sy ** 2) / 12.0
    return f'''<model name="{grasp_object.OBJECT_MODEL_NAME}">
  <pose>{x} {y} {z} 0 0 0</pose>
  <link name="{grasp_object.OBJECT_LINK_NAME}">
    <inertial>
      <mass>{mass}</mass>
      <inertia><ixx>{ixx}</ixx><iyy>{iyy}</iyy><izz>{izz}</izz>
        <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
    </inertial>
    <collision name="collision">
      <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      <surface>
        <friction><ode><mu>{friction}</mu><mu2>{friction}</mu2></ode></friction>
      </surface>
    </collision>
    <visual name="visual">
      <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
      <material>
        <ambient>0.9 0.6 0.1 1</ambient>
        <diffuse>0.9 0.6 0.1 1</diffuse>
      </material>
    </visual>
  </link>
  <plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">
    <publish_model_pose>true</publish_model_pose>
    <publish_link_pose>false</publish_link_pose>
    <publish_collision_pose>false</publish_collision_pose>
    <publish_visual_pose>false</publish_visual_pose>
    <publish_nested_model_pose>false</publish_nested_model_pose>
    <use_pose_vector_msg>true</use_pose_vector_msg>
  </plugin>
</model>'''


def populate_zero_effort_limits_from_urdf(model):
    """Restore controller-relevant effort and velocity limits absent from SDF17.

    PR2_SDF17 deliberately contains the canonical native SDF topology and
    geometry, but its exported axis effort entries are all zero. Gazebo treats
    those values as hard actuator caps, so merely adding controllers would
    create motors incapable of applying force. Copy only effort/velocity from
    the matching flattened URDF; all SDF kinematics and visual/collision data
    remain authoritative.
    """
    urdf_root = ET.parse(URDF_PATH).getroot()
    urdf_limits = {}
    for joint in urdf_root.findall('joint'):
        limit = joint.find('limit')
        if limit is not None:
            urdf_limits[joint.get('name')] = limit.attrib

    for joint in model.findall('joint'):
        limits = urdf_limits.get(joint.get('name'))
        axis_limit = joint.find('axis/limit')
        if limits is None or axis_limit is None:
            continue
        for key in ('effort', 'velocity'):
            value = limits.get(key)
            if value is None:
                continue
            target = axis_limit.find(key)
            if target is None:
                target = ET.SubElement(axis_limit, key)
            if key != 'effort' or target.text is None or float(target.text) == 0.0:
                target.text = value


def load_sdf17_model_block(with_drive_controllers=False,
                           with_manipulation_controllers=False,
                           with_graspable_object=False):
    """Return the native PR2 SDF 1.7 model fitted with fixture systems."""
    if not SDF17_MODEL_PATH.is_file():
        sys.exit(f'error: native PR2 SDF model is missing: {SDF17_MODEL_PATH}')

    root = ET.parse(SDF17_MODEL_PATH).getroot()
    model = root.find('model')
    if model is None:
        sys.exit(f'error: native PR2 SDF has no <model>: {SDF17_MODEL_PATH}')
    model.set('name', 'pr2')

    # The model is embedded in a generated world, so make all local model://
    # mesh resources independent of Gazebo's process-wide resource search path.
    for uri in model.iter('uri'):
        text = (uri.text or '').strip()
        if not text.startswith('model://pr2_sdf17/'):
            continue
        uri.text = str(SDF17_MODEL_DIR / text.removeprefix('model://pr2_sdf17/'))

    populate_zero_effort_limits_from_urdf(model)
    fix_orphaned_gripper_links(model)
    fix_wheel_friction(model)
    fix_wheel_effort(model)
    validate_right_arm_model(model)
    fix_right_arm_light_damping(model)
    if with_drive_controllers:
        add_caster_drive_controllers(model)
    if with_manipulation_controllers:
        add_right_arm_controllers(model)
        add_left_arm_static_pose(model)
        add_head_static_pose(model)
    if with_graspable_object:
        if not with_manipulation_controllers:
            sys.exit('error: with_graspable_object requires with_manipulation_controllers')
        add_object_attachment_plugin(model)

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

    # base_footprint (the URDF's actual root link, fixed 0.051m below
    # base_link -- see pr2.urdf's base_footprint_joint) sits at ground level
    # by convention, so no z-offset hack is needed here the way simple_robot
    # needed one for its own wheel radius.
    pose = model.find('pose')
    if pose is None:
        pose = ET.Element('pose')
        model.insert(0, pose)
    pose.text = '0 0 0 0 0 0'

    pose_publisher = ET.SubElement(model, 'plugin', {
        'filename': 'gz-sim-pose-publisher-system',
        'name': 'gz::sim::systems::PosePublisher',
    })
    # publish_link_pose=false: only the model root pose is needed (relayed as
    # odom->base_footprint by pr2_model_pose_root_relay.py). Publishing every
    # link's pose bridged a second, "pr2/"-prefixed copy of the whole robot's
    # TF tree alongside robot_state_publisher's real one (robot_state_publisher
    # already reconstructs every link's pose correctly via FK from the same
    # physics-solved /joint_states this fixture also bridges), which spammed
    # move_group's planning_scene_monitor with "two or more unconnected trees"
    # warnings for every single link. This alone does not make MoveIt planning
    # succeed in this fixture -- that still fails with MoveItErrorCodes::
    # FAILURE for a separate, unresolved reason -- but it is a real, verified
    # fix for the TF duplication itself (confirmed live: the per-link warnings
    # are gone; only the harmless top-level 'pr2'/'open_interior'/
    # 'graspable_object' root frames still warn, which are unrelated to the
    # robot's own kinematic chain).
    for tag, value in (('publish_link_pose', 'false'),
                        ('publish_collision_pose', 'false'),
                        ('publish_visual_pose', 'false'),
                        ('publish_nested_model_pose', 'false'),
                        ('publish_model_pose', 'true'),
                        ('use_pose_vector_msg', 'true')):
        ET.SubElement(pose_publisher, tag).text = value
    ET.SubElement(model, 'plugin', {
        'filename': 'gz-sim-joint-state-publisher-system',
        'name': 'gz::sim::systems::JointStatePublisher',
    })
    return model


def write_world_file(with_drive_controllers=False, with_manipulation_controllers=False,
                     with_graspable_object=False):
    model = load_sdf17_model_block(
        with_drive_controllers=with_drive_controllers,
        with_manipulation_controllers=with_manipulation_controllers,
        with_graspable_object=with_graspable_object,
    )
    model_block = ET.tostring(model, encoding='unicode')
    if with_graspable_object:
        model_block += '\n' + build_table_model_xml() + '\n' + build_graspable_object_xml()
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    GENERATED_WORLD_PATH.write_text(
        WORLD_TEMPLATE.format(model_block=model_block), encoding='utf-8'
    )


def write_warehouse_world(with_drive_controllers=False, with_manipulation_controllers=False,
                          with_graspable_object=False):
    """Use the shared open_interior world (the shared warehouse scene), with
    PR2 added."""
    world_path = WORLDS_DIR / 'open_interior' / 'open_interior.sdf'
    world_text = world_path.read_text(encoding='utf-8')
    # Keep this fixture offline; the panda fixture performs the same
    # replacement. This must run BEFORE the sensors-system strip below: that
    # regex's trailing `\s*` swallows the Sun block's leading indentation,
    # which would otherwise make this exact-match replace silently no-op (and
    # the online fuel <include> would remain in the world).
    world_text = world_text.replace(
        '''    <!-- Sun -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>''',
        '''    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>''')
    # The shared warehouse enables Gazebo's rendering-based Sensors system.
    # PR2's physics fixture does not use Gazebo sensor data, and that system
    # aborts a headless server when no X display is available. Keep all
    # physics/UserCommands/scene plugins while removing only this renderer.
    world_text = re.sub(
        r'<plugin\s+filename="gz-sim-sensors-system".*?</plugin>\s*', '', world_text,
        flags=re.DOTALL)
    # open_interior.sdf's floor collision has no <surface> element either --
    # contact friction is a function of BOTH surfaces, so even a high wheel
    # mu (see fix_wheel_friction) may be capped by the floor's own unset
    # default. Only patched in this fixture's own generated copy, not the
    # shared source: TurtleBot3's DiffDrive plugin drives fine on this same
    # world without it (that plugin integrates body velocity directly
    # rather than depending on realistic wheel/ground friction the way
    # PR2's from-scratch caster rig does), so there is no reason to risk
    # changing behavior for other fixtures that already work.
    floor_collision = '''    <collision name="collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>40 40</size>
        </plane>
      </geometry>
    </collision>'''
    floor_collision_with_friction = '''    <collision name="collision">
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>40 40</size>
        </plane>
      </geometry>
      <surface>
        <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
      </surface>
    </collision>'''
    if floor_collision not in world_text:
        sys.exit('error: open_interior.sdf floor collision block has changed '
                  'shape; update write_warehouse_world()\'s floor_collision '
                  'string to match')
    world_text = world_text.replace(floor_collision, floor_collision_with_friction)
    if '</world>' not in world_text:
        sys.exit(f'error: warehouse world has no closing world tag: {world_path}')
    model_xml = ET.tostring(
        load_sdf17_model_block(
            with_drive_controllers=with_drive_controllers,
            with_manipulation_controllers=with_manipulation_controllers,
            with_graspable_object=with_graspable_object,
        ), encoding='unicode')
    if with_graspable_object:
        model_xml += '\n' + build_table_model_xml() + '\n' + build_graspable_object_xml()
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    GENERATED_WORLD_PATH.write_text(
        world_text.replace('</world>', model_xml + '\n</world>', 1), encoding='utf-8')


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

    write_warehouse_world()
    print(f'generated world: {GENERATED_WORLD_PATH}')

    gz_args = ['gz', 'sim', str(GENERATED_WORLD_PATH), '-r']
    if args.headless:
        gz_args += ['-s', '--headless-rendering']

    print(f"starting gz sim (pr2{', headless)...' if args.headless else ', GUI)...'}")
    log_handle = open(LOG_FILE, 'w', encoding='utf-8')
    proc = subprocess.Popen(
        gz_args, env=gz_env(), stdout=log_handle, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    PID_FILE.write_text(str(proc.pid))
    bridge = subprocess.Popen(
        [sys.executable, str(EXAMPLE_DIR / 'gazebo_log_bridge.py'), str(LOG_FILE)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True,
    )
    LOG_BRIDGE_PID_FILE.write_text(str(bridge.pid))

    time.sleep(2)
    if proc.poll() is not None or pid_alive() is None:
        sys.exit(f'error: gz sim exited immediately; see {LOG_FILE}')
    print(f'OK: gz sim running (pid {proc.pid}). Log: {LOG_FILE}')
    print('No base/joint actuation -- PR2 will settle under gravity, unpowered '
          '(see this script\'s module docstring).')
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
    if LOG_BRIDGE_PID_FILE.exists():
        try:
            os.kill(int(LOG_BRIDGE_PID_FILE.read_text().strip()), signal.SIGTERM)
        except (OSError, ValueError):
            pass
        LOG_BRIDGE_PID_FILE.unlink(missing_ok=True)
    print(f'stopped (pid {pid}).')


def cmd_status(_args):
    pid = pid_alive()
    print(f"gz sim: {'running (pid ' + str(pid) + ')' if pid else 'not running'}")
    if LOG_FILE.exists():
        print(f'log: {LOG_FILE}')


def cmd_verify(_args):
    """Verify the native SDF arm/gripper baseline without starting Gazebo."""
    require_gz()
    model = load_sdf17_model_block()
    validate_right_arm_model(model)
    print('OK: native PR2 SDF17 has the validated right-arm chain:')
    for joint in RIGHT_ARM_JOINTS:
        print(f'  {joint}')
    print(f'OK: gripper command joint: {RIGHT_GRIPPER_COMMAND_JOINT}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    up_parser = sub.add_parser('up', help='start gz sim with PR2 spawned')
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
    sub.add_parser('verify', help='validate native SDF17 right-arm/gripper model')
    args = parser.parse_args()

    {'up': cmd_up, 'down': cmd_down, 'status': cmd_status, 'verify': cmd_verify}[args.action](args)


if __name__ == '__main__':
    main()
