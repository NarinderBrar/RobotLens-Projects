#!/usr/bin/env python3
"""Send the right_arm group to an arbitrary joint-space pose via a real
MoveGroup goal (plans + executes through move_group -> P1 adapter -> Gazebo),
so you can watch collision avoidance/rejection live in RobotLens.

Usage:
  source /opt/ros/jazzy/setup.bash
  source install/setup.bash   # for the pr2_right_arm_moveit_config package
  python3 send_arm_pose.py <shoulder_pan> <shoulder_lift> <upper_arm_roll> \
      <elbow_flex> <forearm_roll> <wrist_flex> <wrist_roll>

Example -- home pose:
  python3 send_arm_pose.py 0 0 0 -0.2 0 -0.15 0

Example -- a deliberately aggressive fold, likely to self-collide or come
close to it (elbow tucked hard, upper arm rolled toward the torso):
  python3 send_arm_pose.py -1.5 1.0 -3.5 -2.0 0 -1.8 0

If the goal itself is in self-collision, MoveGroup reports
GOAL_IN_COLLISION (-12) immediately, no motion happens, and you'll see that
printed below -- that's the collision matrix from P2 doing its job. If the
goal is collision-free but the straight-line path isn't, OMPL either finds
a way around it (and you'll see the arm take a visibly indirect path in
RobotLens) or reports PLANNING_FAILED (-1) if it can't find one in the allowed
time.
"""
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration as MsgDuration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

RIGHT_ARM_JOINTS = (
    'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
    'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint',
    'r_wrist_roll_joint',
)
# A fresh Gazebo spawn defaults every joint's Gazebo controller target to
# 0.0, but pr2.urdf's soft limits forbid 0.0 for these two -- MoveGroup
# refuses to plan at all (START_STATE_INVALID) until they're out of that
# range. Matches the SRDF's "home" pose.
HOME_JOINTS = {
    'r_shoulder_pan_joint': 0.0, 'r_shoulder_lift_joint': 0.0,
    'r_upper_arm_roll_joint': 0.0, 'r_elbow_flex_joint': -0.2,
    'r_forearm_roll_joint': 0.0, 'r_wrist_flex_joint': -0.15,
    'r_wrist_roll_joint': 0.0,
}

MOVEIT_ERROR_NAMES = {
    1: 'SUCCESS', -1: 'PLANNING_FAILED', -2: 'INVALID_MOTION_PLAN',
    -4: 'CONTROL_FAILED', -10: 'START_STATE_IN_COLLISION',
    -12: 'GOAL_IN_COLLISION', -13: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
    -14: 'GOAL_CONSTRAINTS_VIOLATED', -15: 'INVALID_GROUP_NAME',
    -26: 'START_STATE_INVALID', -27: 'GOAL_STATE_INVALID',
}


def main():
    if len(sys.argv) != 8:
        sys.exit(f'usage: {sys.argv[0]} <7 joint values, radians, in RIGHT_ARM_JOINTS order>')
    values = [float(v) for v in sys.argv[1:]]

    rclpy.init()
    node = Node('send_arm_pose')
    client = ActionClient(node, MoveGroup, '/move_action')
    arm_client = ActionClient(
        node, FollowJointTrajectory, '/pr2_right_arm_controller/follow_joint_trajectory')
    joint_positions = {}
    node.create_subscription(
        JointState, '/joint_states', lambda m: joint_positions.update(zip(m.name, m.position)), 10)

    print('waiting for /move_action...')
    if not client.wait_for_server(timeout_sec=30.0):
        sys.exit('error: /move_action not available -- is move_group running?')
    if not arm_client.wait_for_server(timeout_sec=30.0):
        sys.exit('error: arm trajectory action not available -- is the fixture running?')
    deadline = time.monotonic() + 10.0
    while not joint_positions and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    elbow = joint_positions.get('r_elbow_flex_joint')
    wrist_flex = joint_positions.get('r_wrist_flex_joint')
    needs_priming = elbow is None or not (-2.1213 <= elbow <= -0.15) \
        or wrist_flex is None or not (-2.0 <= wrist_flex <= -0.1)
    if needs_priming:
        # A fresh spawn starts with every joint's Gazebo target at 0.0,
        # which MoveIt's soft limits reject for these two -- MoveGroup
        # would otherwise fail every goal with START_STATE_INVALID. Prime
        # once via the raw action (bypassing MoveGroup, which can't plan
        # yet) into the SRDF's "home" pose.
        print('priming out of the invalid fresh-spawn state (home pose) first...')
        prime_goal = FollowJointTrajectory.Goal()
        prime_goal.trajectory = JointTrajectory(
            joint_names=list(RIGHT_ARM_JOINTS),
            points=[JointTrajectoryPoint(
                positions=[HOME_JOINTS[j] for j in RIGHT_ARM_JOINTS],
                time_from_start=MsgDuration(sec=4))])
        send_future = arm_client.send_goal_async(prime_goal)
        rclpy.spin_until_future_complete(node, send_future, timeout_sec=10.0)
        prime_handle = send_future.result()
        if prime_handle is not None and prime_handle.accepted:
            result_future = prime_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=10.0)
        print('priming done (best-effort; proceeding regardless)')

    goal = MoveGroup.Goal()
    goal.request.group_name = 'right_arm'
    constraints = Constraints()
    for name, value in zip(RIGHT_ARM_JOINTS, values):
        constraints.joint_constraints.append(JointConstraint(
            joint_name=name, position=value,
            tolerance_above=0.02, tolerance_below=0.02, weight=1.0))
    goal.request.goal_constraints = [constraints]
    goal.request.allowed_planning_time = 10.0
    goal.request.num_planning_attempts = 5
    goal.request.max_velocity_scaling_factor = 0.3
    goal.request.max_acceleration_scaling_factor = 0.3
    goal.planning_options.plan_only = False

    print(f'sending goal: {dict(zip(RIGHT_ARM_JOINTS, values))}')
    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future, timeout_sec=15.0)
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        sys.exit('error: MoveGroup rejected the goal outright')

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=30.0)
    result = result_future.result()
    if result is None:
        sys.exit('error: timed out waiting for a result')

    code = result.result.error_code.val
    name = MOVEIT_ERROR_NAMES.get(code, str(code))
    print(f'result: {name} (error_code={code})')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
