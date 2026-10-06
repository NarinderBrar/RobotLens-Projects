#!/usr/bin/env python3
"""Right-arm idle -> approach demo, no picking.

Moves the right arm from its idle (home) pose to the grasp-ready approach
pose -- palm APPROACH_Z_OFFSET_M above the live object's center, gripper open
-- then stops. It never closes the gripper, never attaches, and never lifts.

Reuses pick_place_task's validated helpers directly (Pr2PickPlaceTask): the
motion is a move_group plan executed through the P1 arm adapter with the same
bounded retry semantics as the pick-place state machine, and the approach pose
is computed with the same multi-probe + joint-margin IK selection.

Run the fixture with `pick_place:=false` (this script does not need the
/pick_place server it otherwise owns), plus move_group, then:

  source install/setup.bash
  python3 robots_worlds/pr2/scripts/idle_to_approach.py
"""
import pathlib
import sys
import threading
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

import rclpy
from rclpy.executors import MultiThreadedExecutor

import pr2_arm_trajectory_controller as p1
from pick_place_task import (
    APPROACH_Z_OFFSET_M,
    DEFAULT_PLANNING_TIMEOUT_S,
    DEFAULT_VELOCITY_SCALING,
    HOME_JOINTS,
    RIGHT_GRIPPER_OPEN,
    Pr2PickPlaceTask,
)

WAIT_TIMEOUT_S = 45.0


def wait_for(task, predicate, what):
    deadline = time.monotonic() + WAIT_TIMEOUT_S
    while time.monotonic() < deadline:
        if predicate(task):
            return True
        time.sleep(0.25)
    print(f'ERROR: {what} not available within {WAIT_TIMEOUT_S}s')
    return False


def main():
    rclpy.init()
    task = Pr2PickPlaceTask()
    # The demo drives the task's helpers directly, so the /pick_place action
    # server this node would otherwise own is not needed (run the fixture
    # with `pick_place:=false` to avoid a duplicate-server warning).
    task._server.destroy()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(task)
    threading.Thread(target=executor.spin, daemon=True).start()

    checks = [
        (lambda t: t._move_group.server_is_ready(),
         'move_group /move_action'),
        (lambda t: t._compute_ik.service_is_ready(),
         'move_group /compute_ik'),
        (lambda t: t._fresh_right_arm_positions() is not None,
         'fresh /joint_states'),
        (lambda t: t._lookup_palm_pose() is not None,
         'r_gripper_palm_link TF'),
    ]
    for predicate, what in checks:
        if not wait_for(task, predicate, what):
            executor.shutdown()
            rclpy.shutdown()
            return 1

    task._gripper_goal(RIGHT_GRIPPER_OPEN, 'open gripper (ready for picking)')

    print('-- moving to idle (home) pose --')
    ok, code, message = task._move_joint_goal(
        dict(HOME_JOINTS), 'move to idle', DEFAULT_VELOCITY_SCALING,
        DEFAULT_PLANNING_TIMEOUT_S)
    if not ok:
        print(f'FAILED at idle: {message}')
        executor.shutdown()
        rclpy.shutdown()
        return 1

    candidates = task._default_grasp_candidates()
    if not candidates:
        print('FAILED: could not build a grasp candidate (palm/object TF)')
        executor.shutdown()
        rclpy.shutdown()
        return 1
    candidate = candidates[0]
    print(f'object center: ({candidate.position.x:.3f}, {candidate.position.y:.3f}, '
          f'{candidate.position.z:.3f})')
    print(f'approach target: palm {APPROACH_Z_OFFSET_M:.2f} m above object center')

    approach_joints, err = task._ik_for(candidate, APPROACH_Z_OFFSET_M)
    if approach_joints is None:
        print(f'FAILED: approach IK: {err}')
        executor.shutdown()
        rclpy.shutdown()
        return 1
    print('approach joint config:')
    for joint in p1.RIGHT_ARM_JOINTS:
        print(f'  {joint}: {approach_joints[joint]:+.3f}')

    print('-- moving to approach (ready for picking) --')
    ok, code, message = task._move_joint_goal(
        approach_joints, 'move to approach', DEFAULT_VELOCITY_SCALING,
        DEFAULT_PLANNING_TIMEOUT_S)
    if not ok:
        print(f'FAILED at approach: {message}')
        executor.shutdown()
        rclpy.shutdown()
        return 1

    print('SUCCESS: right arm is at the grasp-ready approach pose; stopped (no pick).')
    executor.shutdown()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
