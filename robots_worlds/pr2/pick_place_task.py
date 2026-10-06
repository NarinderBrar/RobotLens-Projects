#!/usr/bin/env python3
"""P4 pick-and-place task node (docs/IMPLEMENTATION_PLAN_PR2_MOVEIT_PICK_PLACE.md).

Composes P1 (pr2_arm_trajectory_controller.py's FollowJointTrajectory/
GripperCommand actions), P2 (move_group), and P3 (pr2_grasp_controller.py's
~/attach and ~/detach) into one cancellable PickPlace action:

  Validate scene -> Open -> Pre-grasp -> Approach -> Close -> Confirm/attach
  -> Lift -> Pre-place -> Lower -> Open/detach -> Retreat -> Verify placement

Run after pr2_robotlens_gazebo.launch.py and
pr2_right_arm_moveit_config/launch/move_group.launch.py are both up:

  source /opt/ros/jazzy/setup.bash
  source install/setup.bash   # for pr2_pick_place_interfaces
  python3 robots_worlds/pr2/pick_place_task.py

Reach poses (grasp candidates, and by extension pre-grasp/approach/lift)
are solved live via /compute_ik and executed as real MoveGroup joint-space
goals -- not canned trajectories -- the same approach validated live this
session (see run_pr2_gazebo.py's add_right_arm_controllers docstring for
the controller-side fixes that made this reliable, and its UNRESOLVED note
for r_wrist_roll_joint's remaining physics-engine-level instability, which
this task's per-candidate retry and post-attach safe-stop behavior below
exist partly to absorb).

Grasp policy simplification for this first P4 delivery: only ONE object is
ever in scene (grasp_object.OBJECT_MODEL_NAME), so `object_id` is validated
against that constant rather than looked up; there is no perception or
multi-object selection here.
"""
import math
import os
import threading
import time

import diagnostic_msgs.msg
import diagnostic_updater
import rclpy
import rclpy.qos
from builtin_interfaces.msg import Duration as MsgDuration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from pr2_pick_place_interfaces.action import PickPlace

# Sibling-file imports (all three live in this same directory, matching
# pr2_grasp_controller.py's own `import grasp_object as go` -- Python adds
# a script's own directory to sys.path, so this works without any package
# install as long as this file is run the same way P1/P3 are). Reused
# directly rather than re-declared, so this task can never silently drift
# out of sync with the adapter/grasp-controller's own joint lists,
# tolerances, and frame convention.
import grasp_object as go  # noqa: E402
import pr2_arm_trajectory_controller as p1  # noqa: E402
import pr2_grasp_controller as p3  # noqa: E402

RIGHT_GRIPPER_OPEN = p1.GRIPPER_LIMITS[1]
RIGHT_GRIPPER_CLOSED = p1.GRIPPER_LIMITS[0]
IK_LINK_NAME = 'r_wrist_roll_link'
# Deliberately NOT the SRDF "home" group_state's literal values
# (elbow_flex=-0.2, wrist_flex=-0.15, see
# pr2_right_arm_moveit_config/config/pr2_right_arm.srdf) even though this
# plays the same "safe fallback pose" role: those values sit only 0.05 rad
# inside their own soft limits (elbow_flex's is -0.15, wrist_flex's is
# -0.1), so a partially-executed/aborted trajectory -- routine given
# r_wrist_roll_joint's known instability aborting otherwise-fine goals --
# can easily leave one of them back outside the valid range, making
# MoveGroup report START_STATE_INVALID again on the very next attempt.
# Confirmed live this session: HOME_JOINTS' priming fallback recursed into
# START_STATE_INVALID across all 3 retry attempts with the SRDF's literal
# values. -0.4/-0.3 keep the same posture with 4-6x the margin.
HOME_JOINTS = {
    # End/rest pose: pan the right arm 25 degrees outward while keeping the
    # remaining validated home posture unchanged.
    'r_shoulder_pan_joint': math.radians(25.0), 'r_shoulder_lift_joint': 0.0,
    'r_upper_arm_roll_joint': 0.0, 'r_elbow_flex_joint': -0.4,
    'r_forearm_roll_joint': 0.0, 'r_wrist_flex_joint': -0.3,
    'r_wrist_roll_joint': 0.0,
}
# Heights are relative to the object's own center (grasp_object.
# OBJECT_INITIAL_POSE_XYZ / a candidate's own position): a small approach
# clearance so the closing gripper doesn't collide with the object before
# CLOSE, a larger pre-grasp standoff to reduce collision risk on the way
# in, and a lift height clear of the table (grasp_object.TABLE_SIZE_M[2]).
#
# APPROACH_Z_OFFSET_M must clear the object's half-height plus the palm
# link's own geometry: the graspable object is a 5cm cube (half-height
# 0.025), and probing /compute_ik at +0.02 (the original value) returned
# NO SOLUTION at any orientation -- the palm target sat inside the cube.
# The minimum collision-free approach measured live is +0.08, which is also
# exactly grasp_object.GRASP_ENVELOPE_MAX_DISTANCE_M (the attach gate
# requires the palm within that distance of the object), so the two agree
# by design. Pre-grasp/lift remain offset above that same clearance.
APPROACH_Z_OFFSET_M = 0.08
PRE_GRASP_Z_OFFSET_M = APPROACH_Z_OFFSET_M + 0.10
LIFT_Z_OFFSET_M = APPROACH_Z_OFFSET_M + 0.15
PRE_PLACE_Z_OFFSET_M = 0.10
PLACEMENT_TOLERANCE_M = 0.05
MAX_GRASP_CANDIDATES = 5
MAX_MOTION_RETRIES = 6
DEFAULT_PLANNING_TIMEOUT_S = 10.0
DEFAULT_VELOCITY_SCALING = 0.3
IK_TIMEOUT_S = 0.5
# MultiThreadedExecutor's worker threads are non-daemon: if one is mid-
# retry against a controller/service that will never answer (e.g.
# move_group/Gazebo already gone), Python's interpreter-exit machinery
# will otherwise wait to join it forever, so a Ctrl+C'd process just sits
# there as an invisible orphan even though the main thread's spin() has
# already returned. Bound the shutdown wait and hard-exit past it.
SHUTDOWN_TIMEOUT_S = 5.0
# move_group's KDL IK restarts from a random seed per call and is
# non-deterministic between a comfortable solution and a "wound-up" one
# (see _ik_for); probe this many times and keep the solution with the
# best joint-limit margin. Each probe is one /compute_ik call, so the
# worst-case cost is this many IK_TIMEOUT_S round-trips.
IK_PROBE_COUNT = 4
FUTURE_POLL_S = 0.01


def _is_default_quaternion(q):
    return q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 1.0


def _joint_goal_constraints(joint_values, tolerance=0.02):
    constraints = Constraints()
    for name, value in joint_values.items():
        constraints.joint_constraints.append(JointConstraint(
            joint_name=name, position=value,
            tolerance_above=tolerance, tolerance_below=tolerance, weight=1.0))
    return constraints


class Pr2PickPlaceTask(Node):
    def __init__(self):
        super().__init__('pr2_pick_place_task')
        self._callback_group = ReentrantCallbackGroup()
        self._task_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._joint_positions = {}
        self._joint_state_wall_time = 0.0

        # P5: low-rate, bounded diagnostics on the standard /diagnostics
        # topic (diagnostic_msgs/DiagnosticArray) -- RobotLens's existing
        # DiagnosticsPanel already subscribes to and renders this; no new
        # UI or custom topic needed. Matches the pattern in the other
        # example fixtures' diagnostics nodes. State is plain instance
        # attributes updated by the state machine as it runs; the Updater's
        # own ~1 Hz timer (diagnostic_updater's default) reads them, so
        # this never publishes faster than that regardless of how often
        # the state machine transitions.
        self._diag_lock = threading.Lock()
        self._task_state = 'IDLE'
        self._active_plan_id = None
        self._last_outcome = 'no task run yet'
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID('pr2')
        self.updater.add('PickPlace Task', self._task_diagnostic)

        self._tf_buffer = Buffer()
        self.create_subscription(
            TFMessage, '/tf', self._on_tf, 20, callback_group=self._callback_group)
        self.create_subscription(
            TFMessage, '/tf_static', self._on_tf_static,
            rclpy.qos.QoSProfile(
                depth=100,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE),
            callback_group=self._callback_group)
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 20,
            callback_group=self._callback_group)

        self._move_group = ActionClient(
            self, MoveGroup, '/move_action', callback_group=self._callback_group)
        self._arm_trajectory = ActionClient(
            self, FollowJointTrajectory, '/pr2_right_arm_controller/follow_joint_trajectory',
            callback_group=self._callback_group)
        self._gripper = ActionClient(
            self, GripperCommand, '/pr2_right_gripper_controller/gripper_cmd',
            callback_group=self._callback_group)
        self._compute_ik = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self._callback_group)
        self._attach = self.create_client(
            Trigger, '/pr2_grasp_controller/attach', callback_group=self._callback_group)
        self._detach = self.create_client(
            Trigger, '/pr2_grasp_controller/detach', callback_group=self._callback_group)

        self._server = ActionServer(
            self, PickPlace, '/pick_place',
            execute_callback=self._execute,
            goal_callback=self._accept_goal,
            cancel_callback=self._accept_cancel,
            callback_group=self._callback_group)

    def destroy_node(self):
        self._server.destroy()
        super().destroy_node()

    # ---- P5 diagnostics --------------------------------------------------

    def _set_task_status(self, state=None, plan_id=None, outcome=None):
        """Single entry point every state-machine update goes through, so
        the diagnostic callback (running on the Updater's own timer, a
        different thread) never observes a torn/partial update."""
        with self._diag_lock:
            if state is not None:
                self._task_state = state
            if plan_id is not None or plan_id is False:  # False means "clear it"
                self._active_plan_id = None if plan_id is False else plan_id
            if outcome is not None:
                self._last_outcome = outcome

    def _task_diagnostic(self, stat):
        with self._diag_lock:
            state, plan_id, outcome = self._task_state, self._active_plan_id, self._last_outcome
        if plan_id is not None:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, f'running: {state}')
        elif outcome.startswith('FAILED') or outcome.startswith('CANCELED'):
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, f'idle -- last: {outcome}')
        else:
            stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, f'idle -- last: {outcome}')
        stat.add('State', state)
        stat.add('Active Plan ID', plan_id or 'none')
        stat.add('Last Outcome', outcome)
        return stat

    # ---- state intake ---------------------------------------------------

    def _on_joint_state(self, message):
        if len(message.name) != len(message.position):
            return
        with self._state_lock:
            self._joint_positions.update(zip(message.name, message.position))
            self._joint_state_wall_time = time.monotonic()

    def _on_tf(self, message):
        for transform in message.transforms:
            if transform.child_frame_id == go.OBJECT_MODEL_NAME:
                # Same graft as pr2_grasp_controller.py's _on_tf -- the
                # object's real pose is rooted under the disconnected
                # `open_interior` tree; see that module's docstring.
                # Duplicated here rather than shared because it is
                # explicitly local-buffer-only in both places (never
                # published to the shared /tf topic).
                grafted = type(transform)()
                grafted.header.stamp = transform.header.stamp
                grafted.header.frame_id = p3.PLANNING_FRAME
                grafted.child_frame_id = go.OBJECT_MODEL_NAME
                grafted.transform = transform.transform
                self._tf_buffer.set_transform(grafted, 'pr2_pick_place_task')
                continue
            self._tf_buffer.set_transform(transform, 'pr2_pick_place_task')

    def _on_tf_static(self, message):
        for transform in message.transforms:
            self._tf_buffer.set_transform_static(transform, 'pr2_pick_place_task')

    def _fresh_right_arm_positions(self):
        with self._state_lock:
            if time.monotonic() - self._joint_state_wall_time > p1.STATE_MAX_AGE_SECONDS:
                return None
            values = {j: self._joint_positions.get(j) for j in p1.RIGHT_ARM_JOINTS}
        if any(v is None or not math.isfinite(v) for v in values.values()):
            return None
        return values

    def _lookup(self, target_frame, source_frame):
        try:
            return self._tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None

    def _lookup_palm_pose(self):
        return self._lookup(p3.PLANNING_FRAME, go.MOVEIT_ATTACH_LINK)

    def _current_object_pose(self):
        transform = self._lookup(p3.PLANNING_FRAME, go.OBJECT_MODEL_NAME)
        if transform is None:
            return None
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    # ---- goal/cancel handling --------------------------------------------

    def _accept_goal(self, goal_request):
        if goal_request.object_id != go.OBJECT_MODEL_NAME:
            self.get_logger().warning(
                f'rejecting PickPlace goal: object_id {goal_request.object_id!r} != '
                f'{go.OBJECT_MODEL_NAME!r} (the only object this fixture owns)')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT if self._task_lock.acquire(blocking=False) else GoalResponse.REJECT

    @staticmethod
    def _accept_cancel(_goal):
        return CancelResponse.ACCEPT

    # ---- low-level motion/IO primitives ----------------------------------

    def _wait(self, future, timeout_sec):
        """Busy-wait for a future without spinning this node's own executor
        from inside one of its own callbacks (this method only ever runs
        inside _execute, itself an executor callback) -- matches
        pr2_grasp_controller.py's _apply_scene_sync: with a
        MultiThreadedExecutor, other worker threads complete the future in
        the background while this thread just polls, so nesting a real
        spin call here is both unnecessary and unsafe."""
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(FUTURE_POLL_S)
        return future.done()

    def _move_joint_goal(self, joint_values, label, velocity_scaling, planning_timeout):
        """Wraps _move_joint_goal_once with a bounded retry on two error
        codes specifically, confirmed live this session as the two ways an
        otherwise-valid goal fails transiently rather than describing the
        goal itself:

        CONTROL_FAILED (-4): execution ran but a joint missed its final
        tolerance -- exactly r_wrist_roll_joint's known, real
        physics-engine instability (see run_pr2_gazebo.py's UNRESOLVED
        docstring note). Retried as-is; sometimes converges on a second
        attempt.

        START_STATE_INVALID (-26): the *current* joint state violates a
        soft limit (elbow_flex/wrist_flex have very little margin -- see
        HOME_JOINTS' comment above). Routine after a CONTROL_FAILED abort
        partway through a previous trajectory. MoveGroup cannot plan at
        all from here, so first prime out of it via a raw
        FollowJointTrajectory goal (bypassing MoveGroup), then retry.

        Other error codes (GOAL_IN_COLLISION, PLANNING_FAILED,
        INVALID_GROUP_NAME, ...) are not retried -- they describe the goal
        itself and would just fail identically again."""
        ok, code, message = self._move_joint_goal_once(
            joint_values, label, velocity_scaling, planning_timeout)
        attempt = 1
        while not ok and code in (-4, -26) and attempt < MAX_MOTION_RETRIES:
            attempt += 1
            if code == -26:
                self.get_logger().info(f'{label}: START_STATE_INVALID, priming out of it '
                                       f'(attempt {attempt}/{MAX_MOTION_RETRIES})')
                self._prime_out_of_invalid_start_state()
            else:
                self.get_logger().info(f'{label}: CONTROL_FAILED, retrying (attempt {attempt}/'
                                       f'{MAX_MOTION_RETRIES})')
            ok, code, message = self._move_joint_goal_once(
                joint_values, f'{label} (retry {attempt})', velocity_scaling, planning_timeout)
        return ok, code, message

    def _move_joint_goal_once(self, joint_values, label, velocity_scaling, planning_timeout):
        """Real MoveGroup joint-space goal: planned (collision-aware, using
        the P2 self-collision matrix and P3's live-synced table/object) and
        executed through the P1 adapter. Returns (ok, moveit_error_code,
        message)."""
        goal = MoveGroup.Goal()
        goal.request.group_name = 'right_arm'
        goal.request.goal_constraints = [_joint_goal_constraints(joint_values)]
        goal.request.allowed_planning_time = planning_timeout
        goal.request.num_planning_attempts = 5
        goal.request.max_velocity_scaling_factor = velocity_scaling
        goal.request.max_acceleration_scaling_factor = velocity_scaling
        goal.planning_options.plan_only = False

        send_future = self._move_group.send_goal_async(goal)
        if not self._wait(send_future, 15.0) or send_future.result() is None:
            return False, 0, f'{label}: MoveGroup goal send timed out'
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return False, 0, f'{label}: MoveGroup rejected the goal'

        result_future = goal_handle.get_result_async()
        if not self._wait(result_future, planning_timeout + 20.0) or result_future.result() is None:
            return False, 0, f'{label}: MoveGroup result timed out'
        code = result_future.result().result.error_code.val
        if code != 1:
            return False, code, f'{label}: MoveGroup error_code={code}'
        return True, code, f'{label}: ok'

    def _prime_out_of_invalid_start_state(self):
        """Bypasses MoveGroup (which cannot plan from an invalid start
        state at all) via a raw FollowJointTrajectory goal to HOME_JOINTS.
        Only ever called as a fallback after MoveGroup itself reports
        START_STATE_INVALID -- never used for ordinary transitions, since a
        raw single-point trajectory is not collision-checked and could cut
        through the table if the arm is already near it."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=list(p1.RIGHT_ARM_JOINTS),
            points=[JointTrajectoryPoint(
                positions=[HOME_JOINTS[j] for j in p1.RIGHT_ARM_JOINTS],
                time_from_start=MsgDuration(sec=4))])
        send_future = self._arm_trajectory.send_goal_async(goal)
        if not self._wait(send_future, 10.0) or send_future.result() is None:
            return
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return
        result_future = goal_handle.get_result_async()
        self._wait(result_future, 10.0)

    def _move_home(self, label, velocity_scaling, planning_timeout):
        """Thin wrapper so call sites read as intent ("move home") rather
        than repeating the HOME_JOINTS dict everywhere -- _move_joint_goal
        itself already handles the START_STATE_INVALID priming fallback
        for every goal, not just this one."""
        ok, _code, message = self._move_joint_goal(
            HOME_JOINTS, label, velocity_scaling, planning_timeout)
        return ok, message

    def _gripper_goal(self, position, label):
        """Best-effort: a real close-on-object physically stops short of
        the commanded target (see grasp_object.py's
        GRIPPER_CLOSED_MAX_POSITION_RAD docstring), so reached_goal=False
        here is expected and not itself a failure -- ~/attach's own
        precondition check is the authoritative test, in _confirm_attach."""
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 0.0
        send_future = self._gripper.send_goal_async(goal)
        if not self._wait(send_future, 10.0) or send_future.result() is None:
            return
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return
        result_future = goal_handle.get_result_async()
        if not self._wait(result_future, 10.0):
            self.get_logger().info(f'{label}: gripper goal timed out (continuing)')

    def _call_trigger(self, client, label):
        future = client.call_async(Trigger.Request())
        if not self._wait(future, 10.0) or future.result() is None:
            return False, f'{label}: service call timed out'
        result = future.result()
        return result.success, f'{label}: {result.message}'

    def _ik_for(self, target_pose, z_offset):
        seed = self._fresh_right_arm_positions()
        if seed is None:
            return None, 'no fresh right-arm joint state to seed IK with'

        # IMPORTANT: only the group's own joints, never a raw /joint_states
        # dump -- feeding move_group a RobotState naming a joint outside
        # its SRDF model (e.g. a passive/screw joint) throws an uncaught
        # moveit::Exception and crashes the whole move_group process.
        # Confirmed empirically this session; see reach-and-grasp testing
        # notes in memory.
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'right_arm'
        request.ik_request.ik_link_name = IK_LINK_NAME
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = MsgDuration(
            sec=0, nanosec=int(IK_TIMEOUT_S * 1e9))
        request.ik_request.robot_state.joint_state.name = list(p1.RIGHT_ARM_JOINTS)
        request.ik_request.robot_state.joint_state.position = [
            seed[j] for j in p1.RIGHT_ARM_JOINTS]
        pose = PoseStamped()
        pose.header.frame_id = p3.PLANNING_FRAME
        pose.pose.position.x = target_pose.position.x
        pose.pose.position.y = target_pose.position.y
        pose.pose.position.z = target_pose.position.z + z_offset
        pose.pose.orientation = target_pose.orientation
        request.ik_request.pose_stamped = pose

        # move_group's KDL IK restarts from a random seed per call, so the
        # same request is non-deterministic: it can return either a
        # comfortable solution (all joints comfortably inside their soft
        # limits) or a "wound-up" one (e.g. r_upper_arm_roll_joint pinned
        # near its -3.75 soft limit, r_forearm_roll_joint far from zero).
        # The wound-up family is physically harder to hold and was the one
        # that made P4 pre-grasp goals fail tolerance. Probe several times
        # and keep the solution with the largest joint-limit margin, which
        # deterministically prefers the comfortable family. Confirmed live:
        # repeated identical /compute_ik calls return both families.
        best = None
        best_score = None
        last_error = 'no IK probe succeeded'
        for _ in range(IK_PROBE_COUNT):
            future = self._compute_ik.call_async(request)
            if not self._wait(future, IK_TIMEOUT_S + 5.0) or future.result() is None:
                last_error = 'compute_ik timed out'
                continue
            result = future.result()
            if result.error_code.val != 1:
                last_error = f'compute_ik failed: error_code={result.error_code.val}'
                continue
            solution = dict(zip(
                result.solution.joint_state.name, result.solution.joint_state.position))
            missing = [j for j in p1.RIGHT_ARM_JOINTS if j not in solution]
            if missing:
                last_error = f'IK solution missing joints: {missing}'
                continue
            ordered = {j: solution[j] for j in p1.RIGHT_ARM_JOINTS}
            score = self._ik_margin_score(ordered)
            if best_score is None or score > best_score:
                best, best_score = ordered, score
        if best is None:
            return None, last_error
        return best, 'ok'

    @staticmethod
    def _ik_margin_score(solution):
        """How comfortably a joint-space solution sits inside the soft
        limits. The score is the smallest soft-limit margin across the
        group, in radians -- a solution pinned against a limit (the
        wound-up IK family) scores ~0, a comfortable one scores well. The
        continuous roll joints have no soft bounds, so they score by how
        far they've wound away from zero, which makes the high-winding
        solutions that strain the physics rank low too."""
        margins = []
        for joint_name, value in solution.items():
            limits = p1.RIGHT_ARM_LIMITS[joint_name]
            if limits is None:
                margins.append(max(0.0, math.pi - abs(value)))
            else:
                lower, upper = limits
                margins.append(min(value - lower, upper - value))
        return min(margins)

    def _default_grasp_candidates(self):
        home_pose = self._lookup_palm_pose()
        if home_pose is None:
            return []
        # Prefer the live object pose over the nominal spawn constant: the
        # planning scene (and the attach envelope gate) track the object's
        # real pose, so IK must target the same pose or the collision check
        # rejects solutions that would otherwise reach the real object.
        candidate = Pose()
        live = self._current_object_pose()
        if live is not None:
            candidate.position.x = live.position.x
            candidate.position.y = live.position.y
            candidate.position.z = live.position.z
        else:
            candidate.position.x = go.OBJECT_INITIAL_POSE_XYZ[0]
            candidate.position.y = go.OBJECT_INITIAL_POSE_XYZ[1]
            candidate.position.z = go.OBJECT_INITIAL_POSE_XYZ[2]
        candidate.orientation = home_pose.transform.rotation
        return [candidate]

    def _safe_stop_after_attach(self, reason):
        """The object is physically held when this is called -- P3's own
        safety rule (plan section 4, item 5): stop, open the gripper only
        when safe, report the exact failure, never just abandon a rigidly
        attached object. Best-effort: this is a last-resort path, not
        expected to itself fail cleanly if the reason it's being called is
        already a deeper problem."""
        self.get_logger().warning(f'safe-stopping after attach: {reason}')
        self._gripper_goal(RIGHT_GRIPPER_OPEN, 'safe-stop open')
        self._call_trigger(self._detach, 'safe-stop detach')

    # ---- the state machine ------------------------------------------------

    def _execute(self, goal_handle):
        try:
            return self._run_state_machine(goal_handle)
        finally:
            self._task_lock.release()

    def _run_state_machine(self, goal_handle):
        request = goal_handle.request
        started = time.monotonic()
        plan_id = bytes(goal_handle.goal_id.uuid).hex()
        self._set_task_status(state='VALIDATE_SCENE', plan_id=plan_id)

        def feedback(state):
            fb = PickPlace.Feedback()
            fb.current_state = state
            fb.elapsed_seconds = time.monotonic() - started
            goal_handle.publish_feedback(fb)
            self.get_logger().info(f'PickPlace: {state}')
            self._set_task_status(state=state)

        def finish(success, message, failed_state):
            result = PickPlace.Result()
            result.success = success
            result.message = message
            result.failed_state = failed_state
            result.final_object_pose = self._current_object_pose() or Pose()
            outcome = 'SUCCEEDED' if success else f'FAILED at {failed_state}: {message}'
            self._set_task_status(state='IDLE', plan_id=False, outcome=outcome)
            if success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
                self.get_logger().error(f'PickPlace failed at {failed_state}: {message}')
            return result

        def canceled():
            result = PickPlace.Result()
            result.success = False
            result.message = 'task canceled'
            result.failed_state = 'CANCELED'
            result.final_object_pose = self._current_object_pose() or Pose()
            self._set_task_status(state='IDLE', plan_id=False, outcome='CANCELED')
            goal_handle.canceled()
            return result

        velocity_scaling = (request.velocity_scaling if request.velocity_scaling > 0.0
                            else DEFAULT_VELOCITY_SCALING)
        planning_timeout = (request.planning_timeout if request.planning_timeout > 0.0
                           else DEFAULT_PLANNING_TIMEOUT_S)

        feedback('VALIDATE_SCENE')
        if self._fresh_right_arm_positions() is None:
            return finish(False, 'no fresh /joint_states for the right arm', 'VALIDATE_SCENE')
        if self._lookup_palm_pose() is None:
            return finish(False, f'TF unavailable for {go.MOVEIT_ATTACH_LINK} in '
                          f'{p3.PLANNING_FRAME}', 'VALIDATE_SCENE')
        if self._current_object_pose() is None:
            return finish(False, 'TF unavailable for the graspable object -- is '
                          'pr2_grasp_controller.py running?', 'VALIDATE_SCENE')
        if goal_handle.is_cancel_requested:
            return canceled()

        feedback('OPEN')
        ok, message = self._move_home(
            'move to home before opening', velocity_scaling, planning_timeout)
        if not ok:
            return finish(False, f'could not reach a safe starting pose: {message}', 'OPEN')
        self._gripper_goal(RIGHT_GRIPPER_OPEN, 'open')
        if goal_handle.is_cancel_requested:
            return canceled()

        candidates = list(request.grasp_candidates) or self._default_grasp_candidates()
        if not candidates:
            return finish(False, 'no grasp candidate available (TF unavailable for the '
                          'gripper or object)', 'PRE_GRASP')
        candidates = candidates[:MAX_GRASP_CANDIDATES]

        attached = False
        grasp_used = None
        last_error = 'no grasp candidates were provided or computed'
        for index, candidate in enumerate(candidates):
            if goal_handle.is_cancel_requested:
                return canceled()
            label = f'candidate {index + 1}/{len(candidates)}'

            feedback('PRE_GRASP')
            pre_grasp_joints, err = self._ik_for(candidate, PRE_GRASP_Z_OFFSET_M)
            if pre_grasp_joints is None:
                last_error = f'PRE_GRASP IK failed ({label}): {err}'
                continue
            ok, _code, err = self._move_joint_goal(
                pre_grasp_joints, f'pre-grasp ({label})', velocity_scaling, planning_timeout)
            if not ok:
                last_error = f'PRE_GRASP failed ({label}): {err}'
                continue
            if goal_handle.is_cancel_requested:
                return canceled()

            feedback('APPROACH')
            grasp_joints, err = self._ik_for(candidate, APPROACH_Z_OFFSET_M)
            if grasp_joints is None:
                last_error = f'APPROACH IK failed ({label}): {err}'
                continue
            ok, _code, err = self._move_joint_goal(
                grasp_joints, f'approach ({label})', velocity_scaling, planning_timeout)
            if not ok:
                last_error = f'APPROACH failed ({label}): {err}'
                continue
            if goal_handle.is_cancel_requested:
                return canceled()

            feedback('CLOSE')
            self._gripper_goal(RIGHT_GRIPPER_CLOSED, f'close ({label})')

            feedback('CONFIRM_ATTACH')
            ok, err = self._call_trigger(self._attach, f'attach ({label})')
            if ok:
                attached = True
                grasp_used = candidate
                break
            last_error = f'CONFIRM_ATTACH failed ({label}): {err}'
            self._gripper_goal(RIGHT_GRIPPER_OPEN, f're-open after failed attach ({label})')

        if not attached:
            self._move_home(
                'retreat after exhausting grasp candidates', velocity_scaling, planning_timeout)
            return finish(False, last_error, 'CONFIRM_ATTACH')

        if goal_handle.is_cancel_requested:
            self._safe_stop_after_attach('canceled after attach')
            return canceled()

        feedback('LIFT')
        lift_joints, err = self._ik_for(grasp_used, LIFT_Z_OFFSET_M)
        ok = lift_joints is not None
        if ok:
            ok, _code, err = self._move_joint_goal(
                lift_joints, 'lift', velocity_scaling, planning_timeout)
        if not ok:
            self._safe_stop_after_attach(f'LIFT failed: {err}')
            return finish(False, f'LIFT failed: {err}', 'LIFT')
        if goal_handle.is_cancel_requested:
            self._safe_stop_after_attach('canceled during lift')
            return canceled()

        placement_orientation = (
            grasp_used.orientation if _is_default_quaternion(request.placement_pose.orientation)
            else request.placement_pose.orientation)
        placement_target = Pose()
        placement_target.position.x = request.placement_pose.position.x
        placement_target.position.y = request.placement_pose.position.y
        placement_target.position.z = request.placement_pose.position.z
        placement_target.orientation = placement_orientation

        feedback('PRE_PLACE')
        pre_place_joints, err = self._ik_for(placement_target, PRE_PLACE_Z_OFFSET_M)
        ok = pre_place_joints is not None
        if ok:
            ok, _code, err = self._move_joint_goal(
                pre_place_joints, 'pre-place', velocity_scaling, planning_timeout)
        if not ok:
            self._safe_stop_after_attach(f'PRE_PLACE failed: {err}')
            return finish(False, f'PRE_PLACE failed: {err}', 'PRE_PLACE')
        if goal_handle.is_cancel_requested:
            self._safe_stop_after_attach('canceled during pre-place')
            return canceled()

        feedback('LOWER')
        lower_joints, err = self._ik_for(placement_target, APPROACH_Z_OFFSET_M)
        ok = lower_joints is not None
        if ok:
            ok, _code, err = self._move_joint_goal(
                lower_joints, 'lower', velocity_scaling, planning_timeout)
        if not ok:
            self._safe_stop_after_attach(f'LOWER failed: {err}')
            return finish(False, f'LOWER failed: {err}', 'LOWER')

        feedback('OPEN_DETACH')
        self._gripper_goal(RIGHT_GRIPPER_OPEN, 'open before detach')
        ok, err = self._call_trigger(self._detach, 'detach')
        if not ok:
            return finish(False, f'OPEN_DETACH failed: {err}', 'OPEN_DETACH')

        feedback('RETREAT')
        self._move_home('retreat', velocity_scaling, planning_timeout)

        feedback('VERIFY_PLACEMENT')
        final_pose = self._current_object_pose()
        if final_pose is None:
            return finish(False, 'could not verify final object pose (TF unavailable)',
                         'VERIFY_PLACEMENT')
        dx = final_pose.position.x - request.placement_pose.position.x
        dy = final_pose.position.y - request.placement_pose.position.y
        dz = final_pose.position.z - request.placement_pose.position.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance > PLACEMENT_TOLERANCE_M:
            return finish(
                False, f'object landed {distance:.3f} m from the placement target '
                f'(tolerance {PLACEMENT_TOLERANCE_M} m)', 'VERIFY_PLACEMENT')
        return finish(True, 'pick-and-place complete', 'VERIFY_PLACEMENT')


def main():
    rclpy.init()
    node = Pr2PickPlaceTask()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # See SHUTDOWN_TIMEOUT_S: don't let a wedged worker thread (stuck
        # talking to a controller/service that's gone) keep this process
        # alive after Ctrl+C. shutdown(timeout_sec=...) gives up waiting
        # instead of blocking forever; the watchdog then forces the actual
        # process exit, since a still-running non-daemon worker thread
        # would otherwise keep the interpreter alive even after main()
        # returns.
        watchdog = threading.Timer(SHUTDOWN_TIMEOUT_S + 2.0, lambda: os._exit(1))
        watchdog.daemon = True
        watchdog.start()
        executor.shutdown(timeout_sec=SHUTDOWN_TIMEOUT_S)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        watchdog.cancel()


if __name__ == '__main__':
    main()
