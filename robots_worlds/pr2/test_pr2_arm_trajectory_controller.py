#!/usr/bin/env python3
"""Tests for pr2_arm_trajectory_controller.py's validation and action logic.

Covers the P1 controller-adapter verification items from
docs/IMPLEMENTATION_PLAN_PR2_MOVEIT_PICK_PLACE.md section 6: joint ordering,
limits, cancellation, timeout, and terminal result semantics. Run with:
  source /opt/ros/jazzy/setup.bash
  python3 -m unittest robots_worlds/pr2/test_pr2_arm_trajectory_controller.py

No Gazebo instance is required: the action tests call the controller's
execute callback directly (bypassing the real ActionServer/executor) and
seed /joint_states-equivalent state directly instead of subscribing, so only
rclpy and gz-transport's local publish path are exercised.
"""
import math
import time
import unittest

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pr2_arm_trajectory_controller import (
    RIGHT_ARM_JOINTS,
    RIGHT_ARM_LIMITS,
    Pr2ArmTrajectoryController,
    validate_arm_trajectory,
)


def _point(positions, seconds):
    return JointTrajectoryPoint(
        positions=list(positions),
        time_from_start=Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9)),
    )


def _valid_target():
    """A position for every right-arm joint that satisfies RIGHT_ARM_LIMITS."""
    return tuple(
        0.0 if RIGHT_ARM_LIMITS[joint] is None else sum(RIGHT_ARM_LIMITS[joint]) / 2
        for joint in RIGHT_ARM_JOINTS
    )


class TestValidateArmTrajectory(unittest.TestCase):
    def test_accepts_and_reorders_to_canonical_joint_order(self):
        target = _valid_target()
        shuffled_names = list(reversed(RIGHT_ARM_JOINTS))
        shuffled_positions = list(reversed(target))
        trajectory = JointTrajectory(
            joint_names=shuffled_names, points=[_point(shuffled_positions, 1.0)])

        canonical, error = validate_arm_trajectory(trajectory)

        self.assertIsNone(error)
        self.assertEqual(canonical, [(1.0, target)])

    def test_rejects_malformed_trajectories(self):
        target = list(_valid_target())
        cases = {
            'duplicate joint names': JointTrajectory(
                joint_names=list(RIGHT_ARM_JOINTS[:-1]) + [RIGHT_ARM_JOINTS[0]],
                points=[_point(target, 1.0)]),
            'wrong joint set': JointTrajectory(
                joint_names=list(RIGHT_ARM_JOINTS[:-1]) + ['bogus_joint'],
                points=[_point(target, 1.0)]),
            'non-monotonic point times': JointTrajectory(
                joint_names=list(RIGHT_ARM_JOINTS),
                points=[_point(target, 1.0), _point(target, 0.5)]),
            'limit violation': JointTrajectory(
                joint_names=list(RIGHT_ARM_JOINTS),
                points=[_point(
                    [RIGHT_ARM_LIMITS[RIGHT_ARM_JOINTS[0]][1] + 1.0] + target[1:], 1.0)]),
            'non-finite position': JointTrajectory(
                joint_names=list(RIGHT_ARM_JOINTS),
                points=[_point([math.nan] + target[1:], 1.0)]),
            'exceeds max trajectory duration': JointTrajectory(
                joint_names=list(RIGHT_ARM_JOINTS),
                points=[_point(target, 31.0)]),
            'no points': JointTrajectory(joint_names=list(RIGHT_ARM_JOINTS), points=[]),
        }
        for description, trajectory in cases.items():
            with self.subTest(description):
                canonical, error = validate_arm_trajectory(trajectory)
                self.assertIsNone(canonical)
                self.assertIsNotNone(error)


class FakeGoalHandle:
    """Stands in for rclpy's ServerGoalHandle so _execute_arm can be called
    directly, without running a real ActionServer/executor."""

    def __init__(self, trajectory):
        self.request = FollowJointTrajectory.Goal(trajectory=trajectory)
        self.is_cancel_requested = False
        self.canceled_called = False
        self.aborted_called = False
        self.succeeded_called = False

    def publish_feedback(self, _feedback):
        pass

    def canceled(self):
        self.canceled_called = True

    def abort(self):
        self.aborted_called = True

    def succeed(self):
        self.succeeded_called = True


class TestExecuteArm(unittest.TestCase):
    """Exercises Pr2ArmTrajectoryController._execute_arm's cancellation,
    timeout, and terminal result semantics against a real node instance."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Pr2ArmTrajectoryController()
        self.target = _valid_target()
        self.trajectory = JointTrajectory(
            joint_names=list(RIGHT_ARM_JOINTS), points=[_point(self.target, 0.0)])

    def tearDown(self):
        self.node.destroy_node()

    def _seed_joint_state(self, positions):
        self.node._joint_positions = dict(zip(RIGHT_ARM_JOINTS, positions))
        self.node._last_state_wall_time = time.monotonic()

    def _execute(self, goal_handle):
        # Mirrors what the real ActionServer does: _accept_arm_goal acquires
        # _action_lock before execute_callback (_execute_arm) ever runs.
        self.node._accept_arm_goal(goal_handle)
        return self.node._execute_arm(goal_handle)

    def test_cancel_requested_before_start_returns_canceled(self):
        self._seed_joint_state(self.target)
        goal_handle = FakeGoalHandle(self.trajectory)
        goal_handle.is_cancel_requested = True

        result = self._execute(goal_handle)

        self.assertTrue(goal_handle.canceled_called)
        self.assertFalse(goal_handle.aborted_called)
        self.assertEqual(result.error_string, 'trajectory canceled')

    def test_unreached_goal_times_out_with_tolerance_violation(self):
        # Bypass the STATE_MAX_AGE_SECONDS freshness clock so this exercises
        # the convergence deadline itself, not /joint_states going stale
        # mid-wait (both paths report GOAL_TOLERANCE_VIOLATED, so without
        # this the test would pass for the wrong reason).
        never_converges = tuple(p - 5.0 for p in self.target)
        self.node._fresh_positions = lambda _joint_names: never_converges
        goal_handle = FakeGoalHandle(self.trajectory)

        result = self._execute(goal_handle)

        self.assertTrue(goal_handle.aborted_called)
        self.assertFalse(goal_handle.succeeded_called)
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED)

    def test_goal_within_tolerance_succeeds(self):
        self._seed_joint_state(self.target)
        goal_handle = FakeGoalHandle(self.trajectory)

        result = self._execute(goal_handle)

        self.assertTrue(goal_handle.succeeded_called)
        self.assertEqual(result.error_code, 0)

    def test_stale_joint_state_aborts_before_executing(self):
        goal_handle = FakeGoalHandle(self.trajectory)  # no _seed_joint_state call

        result = self._execute(goal_handle)

        self.assertTrue(goal_handle.aborted_called)
        self.assertEqual(result.error_code, FollowJointTrajectory.Result.INVALID_GOAL)


if __name__ == '__main__':
    unittest.main()
