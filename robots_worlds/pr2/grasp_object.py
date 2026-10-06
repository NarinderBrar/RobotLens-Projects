#!/usr/bin/env python3
"""Single source of truth for P3's graspable object and its table.

Both the Gazebo world generator (gazebo/run_pr2_gazebo.py) and the grasp/
planning-scene controller (pr2_grasp_controller.py) import these constants
so the spawned physics object and the planning-scene collision object never
disagree on shape, size, or nominal pose (plan section 4, P3 item 2).

Everything here is in the warehouse world frame, which for this fixture is
also PR2's base frame: PR2 spawns at the world origin with no offset.
"""

OBJECT_MODEL_NAME = 'graspable_object'
OBJECT_LINK_NAME = 'body'
# A small cube, sized to fit inside r_gripper_joint's ~0.09 m max finger
# gap with clearance, light and high-friction so a temporary fixed
# attachment (plan section 4, P3) reads as a plausible grasp.
OBJECT_SIZE_M = (0.05, 0.05, 0.05)
OBJECT_MASS_KG = 0.15
OBJECT_FRICTION = 0.9

TABLE_MODEL_NAME = 'graspable_object_table'
TABLE_LINK_NAME = 'link'
TABLE_SIZE_M = (0.5, 0.4, 0.42)
# Front-right of PR2's torso, within the validated right-arm's reach.
TABLE_POSE_XYZ = (0.55, -0.35, TABLE_SIZE_M[2] / 2.0)

OBJECT_INITIAL_POSE_XYZ = (
    TABLE_POSE_XYZ[0],
    TABLE_POSE_XYZ[1],
    TABLE_SIZE_M[2] + OBJECT_SIZE_M[2] / 2.0,
)

# The Gazebo-side attach mechanism: a gz-sim-detachable-joint-system plugin
# lives on the PR2 model (the parent, per gz-sim's tree-topology requirement
# -- see docs/IMPLEMENTATION_PLAN_PR2_MOVEIT_PICK_PLACE.md P3), fixed
# between the right-gripper palm and this object. It starts attached by
# gz-sim's own default (see tutorials/detachable_joints.md), so the grasp
# controller must detach it once at startup before the object is free.
#
# r_gripper_palm_link is NOT a separate SDF link: r_gripper_palm_joint is
# fixed, so `gz sdf -p`'s URDF->SDF conversion lumps its collision/visual/
# inertial into its parent, r_wrist_roll_link (confirmed empirically -- no
# link with "palm" in its name survives conversion). Use that link instead.
DETACHABLE_JOINT_PARENT_LINK = 'r_wrist_roll_link'
DETACHABLE_JOINT_ATTACH_TOPIC = f'/model/pr2/{OBJECT_MODEL_NAME}/attach'
DETACHABLE_JOINT_DETACH_TOPIC = f'/model/pr2/{OBJECT_MODEL_NAME}/detach'
DETACHABLE_JOINT_STATE_TOPIC = f'/model/pr2/{OBJECT_MODEL_NAME}/state'

# Palm-relative grasp envelope: how far the object's origin may sit from
# r_gripper_tool_frame (in that frame) and still count as "in the gripper"
# once the fingers report closed (plan section 4, P3 item 3).
# A real close-on-object attempt stops short of the fully-closed 0.0 rad
# (nothing to close around otherwise), so this is "closed enough to be
# gripping something small", not "fully closed".
GRASP_ENVELOPE_MAX_DISTANCE_M = 0.08
GRIPPER_CLOSED_MAX_POSITION_RAD = 0.35

# MoveIt loads the original pr2.urdf (not the Gazebo-lumped SDF), where
# r_gripper_palm_link still exists as its own link -- unlike
# DETACHABLE_JOINT_PARENT_LINK above, use it directly for the planning-scene
# attachment; it is the physically correct attach point even though Gazebo
# can't name it.
MOVEIT_ATTACH_LINK = 'r_gripper_palm_link'
MOVEIT_TOUCH_LINKS = (
    'r_gripper_palm_link',
    'r_gripper_l_finger_link',
    'r_gripper_r_finger_link',
    'r_gripper_l_finger_tip_link',
    'r_gripper_r_finger_tip_link',
)
