#!/usr/bin/env python3
"""Execute validated PR2 right-arm actions through Gazebo position controllers.

This is an example-side adapter, not RobotLens production code. It exposes the
standard FollowJointTrajectory and GripperCommand actions and is the only
component that publishes to the PR2 arm/gripper Gazebo transport topics.
"""
import math
import threading
import time

import rclpy
from control_msgs.action import FollowJointTrajectory, GripperCommand
from gz.msgs10.double_pb2 import Double
from gz.transport13 import Node as GzNode
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

RIGHT_ARM_JOINTS = (
    'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
    'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint',
    'r_wrist_roll_joint',
)
# pr2.urdf's <limit> tags are the hard mechanical stops; every non-continuous
# right-arm joint also carries a tighter <safety_controller soft_lower_limit
# soft_upper_limit>, which is what MoveIt's RobotModel actually enforces at
# planning time (see pr2_right_arm_moveit_config, a sibling package of this
# fixture). Execution limits
# must never be looser than planning limits, so this adapter validates
# against the soft bounds, not the hard ones.
RIGHT_ARM_LIMITS = {
    'r_shoulder_pan_joint': (-2.1353981633974484, 0.5646018366025517),
    'r_shoulder_lift_joint': (-0.3535999999999999, 1.2963),
    'r_upper_arm_roll_joint': (-3.7500000000000004, 0.65),
    'r_elbow_flex_joint': (-2.1212999999999997, -0.15),
    'r_forearm_roll_joint': None,
    'r_wrist_flex_joint': (-2.0, -0.1),
    'r_wrist_roll_joint': None,
}
GRIPPER_JOINT = 'r_gripper_l_finger_joint'
GRIPPER_LIMITS = (0.0, 0.548)
STATE_MAX_AGE_SECONDS = 0.5
ARM_GOAL_TOLERANCE = 0.05
GRIPPER_GOAL_TOLERANCE = 0.03
MAX_TRAJECTORY_SECONDS = 30.0


def duration_seconds(duration):
    return duration.sec + duration.nanosec / 1_000_000_000.0


def validate_arm_trajectory(trajectory):
    """Return canonical points or a human-readable validation failure."""
    if tuple(trajectory.joint_names) != RIGHT_ARM_JOINTS:
        if len(trajectory.joint_names) != len(set(trajectory.joint_names)):
            return None, 'trajectory contains duplicate joint names'
        if set(trajectory.joint_names) != set(RIGHT_ARM_JOINTS):
            return None, 'trajectory must contain exactly the seven right-arm joints'
    if not trajectory.points:
        return None, 'trajectory has no points'

    order = {name: index for index, name in enumerate(trajectory.joint_names)}
    canonical = []
    previous_time = -1.0
    for point in trajectory.points:
        if len(point.positions) != len(trajectory.joint_names):
            return None, 'each trajectory point must supply every joint position'
        if point.velocities and len(point.velocities) != len(trajectory.joint_names):
            return None, 'velocity fields must be empty or match joint_names'
        point_time = duration_seconds(point.time_from_start)
        if not math.isfinite(point_time) or point_time < 0.0 or point_time <= previous_time:
            return None, 'point times must be non-negative, finite, and strictly increasing'
        if point_time > MAX_TRAJECTORY_SECONDS:
            return None, f'trajectory exceeds {MAX_TRAJECTORY_SECONDS:.0f}s limit'
        positions = tuple(point.positions[order[name]] for name in RIGHT_ARM_JOINTS)
        for joint_name, position in zip(RIGHT_ARM_JOINTS, positions):
            if not math.isfinite(position):
                return None, f'{joint_name} target is not finite'
            limits = RIGHT_ARM_LIMITS[joint_name]
            if limits and not limits[0] <= position <= limits[1]:
                return None, f'{joint_name} target {position:.3f} exceeds [{limits[0]:.3f}, {limits[1]:.3f}]'
        canonical.append((point_time, positions))
        previous_time = point_time
    return canonical, None


class Pr2ArmTrajectoryController(Node):
    def __init__(self):
        super().__init__('pr2_arm_trajectory_controller')
        self._callback_group = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()
        self._action_lock = threading.Lock()
        self._joint_positions = {}
        self._last_state_wall_time = 0.0
        self._gz = GzNode()
        self._arm_publishers = {
            joint: self._gz.advertise(f'/pr2/right_arm/{joint}/position_cmd', Double)
            for joint in RIGHT_ARM_JOINTS
        }
        self._gripper_publisher = self._gz.advertise('/pr2/right_gripper/position_cmd', Double)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 20,
            callback_group=self._callback_group)
        self._arm_server = ActionServer(
            self, FollowJointTrajectory,
            '/pr2_right_arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_arm,
            goal_callback=self._accept_arm_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._callback_group)
        self._gripper_server = ActionServer(
            self, GripperCommand,
            '/pr2_right_gripper_controller/gripper_cmd',
            execute_callback=self._execute_gripper,
            goal_callback=self._accept_gripper_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._callback_group)

    def destroy_node(self):
        self._arm_server.destroy()
        self._gripper_server.destroy()
        super().destroy_node()

    def _on_joint_state(self, message):
        if len(message.name) != len(message.position):
            self.get_logger().warning('Ignoring malformed /joint_states message')
            return
        with self._state_lock:
            self._joint_positions.update(zip(message.name, message.position))
            self._last_state_wall_time = time.monotonic()

    def _fresh_positions(self, joint_names):
        with self._state_lock:
            if time.monotonic() - self._last_state_wall_time > STATE_MAX_AGE_SECONDS:
                return None
            values = [self._joint_positions.get(name) for name in joint_names]
        return None if any(value is None or not math.isfinite(value) for value in values) else tuple(values)

    def _accept_arm_goal(self, _goal):
        return GoalResponse.ACCEPT if self._action_lock.acquire(blocking=False) else GoalResponse.REJECT

    def _accept_gripper_goal(self, _goal):
        return GoalResponse.ACCEPT if self._action_lock.acquire(blocking=False) else GoalResponse.REJECT

    @staticmethod
    def _accept_cancel(_goal):
        return CancelResponse.ACCEPT

    def _publish_arm(self, positions):
        for publisher, position in zip(self._arm_publishers.values(), positions):
            publisher.publish(Double(data=position))

    def _publish_arm_feedback(self, goal_handle, desired, actual):
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(RIGHT_ARM_JOINTS)
        feedback.desired.positions = list(desired)
        feedback.actual.positions = list(actual)
        feedback.error.positions = [target - measured for target, measured in zip(desired, actual)]
        goal_handle.publish_feedback(feedback)

    def _abort_arm(self, goal_handle, code, message):
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = message
        goal_handle.abort()
        return result

    def _execute_arm(self, goal_handle):
        try:
            points, error = validate_arm_trajectory(goal_handle.request.trajectory)
            if error:
                return self._abort_arm(goal_handle, FollowJointTrajectory.Result.INVALID_GOAL, error)
            current = self._fresh_positions(RIGHT_ARM_JOINTS)
            if current is None:
                return self._abort_arm(goal_handle, FollowJointTrajectory.Result.INVALID_GOAL,
                                       'complete, fresh /joint_states is required before execution')

            started = time.monotonic()
            index = 0
            while index < len(points):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result(error_code=0, error_string='trajectory canceled')
                actual = self._fresh_positions(RIGHT_ARM_JOINTS)
                if actual is None:
                    return self._abort_arm(goal_handle, FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED,
                                           '/joint_states became stale or incomplete')
                elapsed = time.monotonic() - started
                while index + 1 < len(points) and elapsed >= points[index + 1][0]:
                    index += 1
                point_time, target = points[index]
                if index + 1 < len(points) and elapsed > point_time:
                    next_time, next_target = points[index + 1]
                    fraction = (elapsed - point_time) / (next_time - point_time)
                    target = tuple(a + fraction * (b - a) for a, b in zip(target, next_target))
                self._publish_arm(target)
                self._publish_arm_feedback(goal_handle, target, actual)
                if elapsed >= points[-1][0]:
                    break
                time.sleep(0.02)

            deadline = time.monotonic() + 1.0 + duration_seconds(goal_handle.request.goal_time_tolerance)
            target = points[-1][1]
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return FollowJointTrajectory.Result(error_code=0, error_string='trajectory canceled')
                actual = self._fresh_positions(RIGHT_ARM_JOINTS)
                if actual is None:
                    return self._abort_arm(goal_handle, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                                           '/joint_states became stale or incomplete')
                self._publish_arm(target)
                self._publish_arm_feedback(goal_handle, target, actual)
                if max(abs(a - b) for a, b in zip(target, actual)) <= ARM_GOAL_TOLERANCE:
                    goal_handle.succeed()
                    return FollowJointTrajectory.Result(error_code=0, error_string='')
                time.sleep(0.02)
            return self._abort_arm(goal_handle, FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED,
                                   f'final position error exceeded {ARM_GOAL_TOLERANCE} rad')
        finally:
            self._action_lock.release()

    def _execute_gripper(self, goal_handle):
        try:
            target = goal_handle.request.command.position
            if not math.isfinite(target) or not GRIPPER_LIMITS[0] <= target <= GRIPPER_LIMITS[1]:
                result = GripperCommand.Result()
                result.stalled = False
                result.reached_goal = False
                goal_handle.abort()
                return result
            if self._fresh_positions((GRIPPER_JOINT,)) is None:
                result = GripperCommand.Result()
                result.stalled = False
                result.reached_goal = False
                goal_handle.abort()
                return result
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return GripperCommand.Result(stalled=False, reached_goal=False)
                actual = self._fresh_positions((GRIPPER_JOINT,))
                if actual is None:
                    break
                self._gripper_publisher.publish(Double(data=target))
                feedback = GripperCommand.Feedback()
                feedback.position = actual[0]
                feedback.reached_goal = abs(target - actual[0]) <= GRIPPER_GOAL_TOLERANCE
                goal_handle.publish_feedback(feedback)
                if feedback.reached_goal:
                    goal_handle.succeed()
                    return GripperCommand.Result(position=actual[0], effort=0.0,
                                                 stalled=False, reached_goal=True)
                time.sleep(0.02)
            goal_handle.abort()
            return GripperCommand.Result(position=0.0, effort=0.0, stalled=False, reached_goal=False)
        finally:
            self._action_lock.release()


def main():
    rclpy.init()
    node = Pr2ArmTrajectoryController()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
