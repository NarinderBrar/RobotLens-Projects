#!/usr/bin/env python3
"""P3 grasp and object-ownership controller
(docs/IMPLEMENTATION_PLAN_PR2_MOVEIT_PICK_PLACE.md, section 4).

Owns the one graspable object defined in grasp_object.py end to end:

- Seeds the MoveIt planning scene with the table and object (same source of
  truth as the Gazebo spawn in gazebo/run_pr2_gazebo.py) and keeps the
  object's world CollisionObject pose synchronized with its live Gazebo
  pose while it is not attached.
- Detaches the Gazebo DetachableJoint at startup: that system starts welded
  to the wrist by its own default (see run_pr2_gazebo.py's
  add_object_attachment_plugin), and a single startup detach message races the
  plugin's load and is silently dropped if it loses. The controller therefore
  republishes detach until a DetachableJoint state transition (or the
  object's TF height) confirms the object is free, so the object can never be
  observed or acted on while still welded to the arm.
- Exposes `~/attach` and `~/detach` (std_srvs/Trigger) as the grasp
  primitives a future P4 task node composes into the full pick-place state
  machine. `~/attach` requires both a closed-enough gripper and the object
  sitting within a small palm-relative envelope (plan section 4, P3 item 3)
  before creating the Gazebo fixed joint and the MoveIt
  AttachedCollisionObject together; `~/detach` reverses both and restores
  the object as a world CollisionObject. Every failure names the exact
  check or call that failed rather than a generic error (plan section 5).

Frame-graph caveat: the object's Gazebo pose bridges into /tf rooted under
`open_interior` (the shared warehouse world's name), a tree the robot's own
`base_footprint` tree (rooted at `odom` via pr2_model_pose_root_relay.py) has
no TF edge to -- the same "two or more unconnected trees" condition
move_group's planning scene monitor already logs for other Gazebo-sourced
frames. Since PR2 spawns at the world origin with identity pose and this
fixture's plan explicitly excludes moving the base while manipulating
(mobile-base planning is a listed non-goal), this node treats `open_interior`
as coincident with `base_footprint` and manually grafts the object's
transform onto that frame in its own local tf2 buffer -- it does not publish
this graft to the shared /tf topic, so it cannot mislead other consumers.
"""
import math
import threading
import time

import diagnostic_msgs.msg
import diagnostic_updater
import rclpy
import rclpy.qos
from geometry_msgs.msg import Pose, TransformStamped
from gz.msgs10.empty_pb2 import Empty as GzEmpty
from gz.msgs10.stringmsg_pb2 import StringMsg as GzStringMsg
from gz.transport13 import Node as GzNode
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, LookupException, ConnectivityException, ExtrapolationException

import grasp_object as go

PLANNING_FRAME = 'base_footprint'
GRIPPER_JOINT = 'r_gripper_l_finger_joint'
STATE_MAX_AGE_SECONDS = 1.0
SCENE_SYNC_PERIOD_SECONDS = 1.0

# gz-sim's DetachableJoint auto-welds the object to the wrist on its first
# simulation tick (its attachRequested flag starts true), and a single detach
# publish at node start races the plugin's load -- if it lands before the
# plugin subscribes, gz-transport drops it and the object stays welded to the
# arm. Detach is therefore retried on a timer until a state transition (or the
# object's TF height) proves the plugin processed it; see
# _startup_detach_confirmed. A welded object would otherwise be dragged by the
# arm before any grasp attempt.
STARTUP_DETACH_PERIOD_SECONDS = 0.2
STARTUP_DETACH_TIMEOUT_SECONDS = 60.0
# Free-on-table puts the object center at TABLE top (0.42) + half the cube
# (0.025); welded to the wrist at home it sits near 0.88. 0.6 separates the
# two unambiguously.
STARTUP_FREE_Z_MAX_M = 0.6


def box_collision_object(object_id, frame_id, size_xyz, pose, operation):
    obj = CollisionObject()
    obj.header.frame_id = frame_id
    obj.id = object_id
    obj.operation = operation
    if operation != CollisionObject.REMOVE:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(size_xyz)
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
    return obj


def identity_pose(xyz):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = xyz
    pose.orientation.w = 1.0
    return pose


class Pr2GraspController(Node):
    def __init__(self):
        super().__init__('pr2_grasp_controller')
        self._callback_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._gripper_position = None
        self._gripper_state_wall_time = 0.0
        self._object_stamp_wall_time = 0.0
        self._attached = False
        self._scene_seeded = False

        self._tf_buffer = Buffer()

        self._gz = GzNode()
        self._attach_pub = self._gz.advertise(go.DETACHABLE_JOINT_ATTACH_TOPIC, GzEmpty)
        self._detach_pub = self._gz.advertise(go.DETACHABLE_JOINT_DETACH_TOPIC, GzEmpty)
        self._detachable_joint_state = None
        self._gz.subscribe(
            GzStringMsg, go.DETACHABLE_JOINT_STATE_TOPIC, self._on_detachable_state)

        self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 20,
            callback_group=self._callback_group)
        self.create_subscription(
            TFMessage, '/tf', self._on_tf, 20, callback_group=self._callback_group)
        # robot_state_publisher publishes fixed URDF joints (e.g.
        # r_gripper_palm_joint, so r_gripper_palm_link itself) on the
        # latched /tf_static, not /tf -- without this, lookups through any
        # fixed-joint link never resolve even though the rest of the tree
        # looks connected.
        self.create_subscription(
            TFMessage, '/tf_static', self._on_tf_static,
            rclpy.qos.QoSProfile(
                depth=100,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE),
            callback_group=self._callback_group)

        self._apply_scene = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene', callback_group=self._callback_group)

        # P5: object ownership state on the standard /diagnostics topic
        # (diagnostic_msgs/DiagnosticArray) -- RobotLens's existing
        # DiagnosticsPanel already renders this, same mechanism
        # pick_place_task.py uses for task state/plan id/outcome. Reads
        # self._attached directly under self._lock each tick (~1 Hz,
        # diagnostic_updater's own default), not pushed on every
        # attach/detach -- deliberately low-rate, matching the plan's
        # "low-rate, bounded diagnostics" wording.
        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID('pr2')
        self.updater.add('Object Ownership', self._ownership_diagnostic)

        self.create_service(
            Trigger, '~/attach', self._on_attach, callback_group=self._callback_group)
        self.create_service(
            Trigger, '~/detach', self._on_detach, callback_group=self._callback_group)

        # DetachableJoint starts welded to the wrist (attachRequested starts
        # true); free the object before anything else can observe or act on
        # it. Publish immediately and keep retrying on a timer until a state
        # transition proves the plugin processed a detach -- see
        # _startup_detach.
        self._startup_detach_attempts = 0
        self._startup_detach_timer = self.create_timer(
            STARTUP_DETACH_PERIOD_SECONDS, self._startup_detach,
            callback_group=self._callback_group)
        self._startup_detach()

        self._seed_timer = self.create_timer(
            1.0, self._try_seed_scene, callback_group=self._callback_group)
        self._sync_timer = self.create_timer(
            SCENE_SYNC_PERIOD_SECONDS, self._sync_object_pose, callback_group=self._callback_group)

    # ---- state intake -----------------------------------------------

    def _on_detachable_state(self, message):
        # The DetachableJoint plugin publishes "attached"/"detached" on every
        # attach/detach transition (gz-sim DetachableJoint::PublishJointState);
        # transitions only, so a node that starts after a state settled may
        # legitimately never hear one.
        with self._lock:
            self._detachable_joint_state = message.data

    def _startup_detach_confirmed(self):
        with self._lock:
            state = self._detachable_joint_state
        if state == 'detached':
            return True
        if not self._object_pose_fresh():
            return False
        transform = self._lookup(PLANNING_FRAME, go.OBJECT_MODEL_NAME)
        return (transform is not None
                and transform.transform.translation.z <= STARTUP_FREE_Z_MAX_M)

    def _startup_detach(self):
        self._detach_pub.publish(GzEmpty())
        if self._startup_detach_confirmed():
            self._startup_detach_timer.cancel()
            self.get_logger().info(
                f'startup detach confirmed (state={self._detachable_joint_state!r}); '
                'object is free')
            return
        self._startup_detach_attempts += 1
        elapsed = self._startup_detach_attempts * STARTUP_DETACH_PERIOD_SECONDS
        if elapsed >= STARTUP_DETACH_TIMEOUT_SECONDS:
            self._startup_detach_timer.cancel()
            self.get_logger().error(
                f'object not confirmed free after {STARTUP_DETACH_TIMEOUT_SECONDS}s of '
                'detach publishes; it is likely welded to the wrist and grasps will drag it')

    def _on_joint_state(self, message):
        if GRIPPER_JOINT not in message.name:
            return
        index = message.name.index(GRIPPER_JOINT)
        with self._lock:
            self._gripper_position = message.position[index]
            self._gripper_state_wall_time = time.monotonic()

    def _on_tf(self, message):
        for transform in message.transforms:
            if transform.child_frame_id == go.OBJECT_MODEL_NAME:
                # Graft the object's pose (rooted under the disconnected
                # `open_interior` tree) onto PLANNING_FRAME directly --
                # see the module docstring for why this is a documented
                # simplification, not a bug. Setting only the graft (and
                # not also the raw open_interior-rooted edge) avoids a
                # multiple-parent conflict for the same child frame in
                # this buffer. Local to this buffer only; never published
                # to the shared /tf topic.
                grafted = TransformStamped()
                grafted.header.stamp = transform.header.stamp
                grafted.header.frame_id = PLANNING_FRAME
                grafted.child_frame_id = go.OBJECT_MODEL_NAME
                grafted.transform = transform.transform
                self._tf_buffer.set_transform(grafted, 'pr2_grasp_controller')
                with self._lock:
                    self._object_stamp_wall_time = time.monotonic()
                continue
            # The robot's own tree (base_footprint -> ... ->
            # r_gripper_palm_link), fed in unmodified so lookups spanning
            # both it and the graft above can chain through one buffer.
            self._tf_buffer.set_transform(transform, 'pr2_grasp_controller')

    def _on_tf_static(self, message):
        for transform in message.transforms:
            self._tf_buffer.set_transform_static(transform, 'pr2_grasp_controller')

    def _fresh_gripper_position(self):
        with self._lock:
            if time.monotonic() - self._gripper_state_wall_time > STATE_MAX_AGE_SECONDS:
                return None
            return self._gripper_position

    def _object_pose_fresh(self):
        with self._lock:
            return time.monotonic() - self._object_stamp_wall_time <= STATE_MAX_AGE_SECONDS

    def _lookup(self, target_frame, source_frame):
        try:
            return self._tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warning(f'TF lookup {source_frame}->{target_frame} failed: {exc}')
            return None

    # ---- planning-scene seeding and sync ------------------------------

    def _try_seed_scene(self):
        if self._scene_seeded:
            self._seed_timer.cancel()
            return
        if not self._apply_scene.service_is_ready():
            return
        table = box_collision_object(
            go.TABLE_MODEL_NAME, PLANNING_FRAME, go.TABLE_SIZE_M,
            identity_pose(go.TABLE_POSE_XYZ), CollisionObject.ADD)
        obj = box_collision_object(
            go.OBJECT_MODEL_NAME, PLANNING_FRAME, go.OBJECT_SIZE_M,
            identity_pose(go.OBJECT_INITIAL_POSE_XYZ), CollisionObject.ADD)
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [table, obj]
        if self._apply_scene_sync(scene):
            self._scene_seeded = True
            self._seed_timer.cancel()
            self.get_logger().info('seeded planning scene with table and object')
        else:
            self.get_logger().warning('ApplyPlanningScene rejected the initial table/object seed; retrying')

    def _sync_object_pose(self):
        if not self._scene_seeded or self._attached:
            return
        if not self._object_pose_fresh():
            return
        transform = self._lookup(PLANNING_FRAME, go.OBJECT_MODEL_NAME)
        if transform is None:
            return
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        obj = box_collision_object(
            go.OBJECT_MODEL_NAME, PLANNING_FRAME, go.OBJECT_SIZE_M, pose, CollisionObject.ADD)
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        self._apply_scene_sync(scene)

    def _apply_scene_sync(self, scene):
        if not self._apply_scene.service_is_ready():
            return False
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self._apply_scene.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        return future.done() and future.result() is not None and future.result().success

    # ---- P5 diagnostics -------------------------------------------------

    def _ownership_diagnostic(self, stat):
        with self._lock:
            attached = self._attached
        stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK,
                     'attached to gripper' if attached else 'free in world')
        stat.add('Object ID', go.OBJECT_MODEL_NAME)
        stat.add('Attached', str(attached))
        return stat

    # ---- attach / detach services -------------------------------------

    def _on_attach(self, _request, response):
        with self._lock:
            if self._attached:
                response.success = False
                response.message = 'object is already attached'
                return response

        gripper = self._fresh_gripper_position()
        if gripper is None:
            response.success = False
            response.message = 'gripper joint state is stale or unavailable'
            return response
        if gripper > go.GRIPPER_CLOSED_MAX_POSITION_RAD:
            response.success = False
            response.message = (
                f'gripper not closed enough to grasp: {gripper:.3f} rad exceeds the '
                f'{go.GRIPPER_CLOSED_MAX_POSITION_RAD:.3f} rad closed threshold')
            return response

        object_in_palm = self._lookup(go.MOVEIT_ATTACH_LINK, go.OBJECT_MODEL_NAME)
        if object_in_palm is None:
            response.success = False
            response.message = 'could not resolve object pose relative to the gripper (TF unavailable)'
            return response
        t = object_in_palm.transform.translation
        distance = math.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2)
        if distance > go.GRASP_ENVELOPE_MAX_DISTANCE_M:
            response.success = False
            response.message = (
                f'object is {distance:.3f} m from the gripper, outside the '
                f'{go.GRASP_ENVELOPE_MAX_DISTANCE_M:.3f} m grasp envelope')
            return response

        self._attach_pub.publish(GzEmpty())

        # Confirmed empirically: applying an attached_collision_objects ADD
        # for an id that currently exists as a world object moves it out of
        # world.collision_objects as part of the same call -- a *separate*
        # world REMOVE call afterwards fails (nothing left to remove by
        # then), and a *combined* single diff doing both also fails. One
        # call, attach only.
        attach_pose = Pose()
        attach_pose.position.x = t.x
        attach_pose.position.y = t.y
        attach_pose.position.z = t.z
        attach_pose.orientation = object_in_palm.transform.rotation
        attached = AttachedCollisionObject()
        attached.link_name = go.MOVEIT_ATTACH_LINK
        attached.object = box_collision_object(
            go.OBJECT_MODEL_NAME, go.MOVEIT_ATTACH_LINK, go.OBJECT_SIZE_M,
            attach_pose, CollisionObject.ADD)
        attached.touch_links = list(go.MOVEIT_TOUCH_LINKS)

        attach_scene = PlanningScene()
        attach_scene.is_diff = True
        attach_scene.robot_state.is_diff = True
        attach_scene.robot_state.attached_collision_objects = [attached]

        if not self._apply_scene_sync(attach_scene):
            self._detach_pub.publish(GzEmpty())
            response.success = False
            response.message = 'MoveIt ApplyPlanningScene rejected the attach; rolled back the Gazebo attach'
            return response

        with self._lock:
            self._attached = True
        response.success = True
        response.message = 'attached'
        return response

    def _on_detach(self, _request, response):
        with self._lock:
            if not self._attached:
                response.success = False
                response.message = 'object is not currently attached'
                return response

        self._detach_pub.publish(GzEmpty())

        # Confirmed empirically (mirroring _on_attach): removing an
        # attached_collision_objects entry restores it to
        # world.collision_objects automatically, at MoveIt's own FK-derived
        # pose for the link it was attached to. _sync_object_pose's normal
        # 1 Hz loop (guarded on `not self._attached`, now true) corrects
        # that to the live Gazebo pose shortly after, so no separate
        # restore call is needed here.
        detach_attached = AttachedCollisionObject()
        detach_attached.link_name = go.MOVEIT_ATTACH_LINK
        detach_attached.object = box_collision_object(
            go.OBJECT_MODEL_NAME, go.MOVEIT_ATTACH_LINK, go.OBJECT_SIZE_M,
            identity_pose((0.0, 0.0, 0.0)), CollisionObject.REMOVE)
        detach_scene = PlanningScene()
        detach_scene.is_diff = True
        detach_scene.robot_state.is_diff = True
        detach_scene.robot_state.attached_collision_objects = [detach_attached]

        if not self._apply_scene_sync(detach_scene):
            response.success = False
            response.message = (
                'Gazebo detach was sent, but MoveIt ApplyPlanningScene failed to detach the '
                'object from the robot state -- planning-scene state may now disagree with Gazebo')
            return response

        with self._lock:
            self._attached = False
        response.success = True
        response.message = 'detached'
        return response


def main():
    rclpy.init()
    node = Pr2GraspController()
    executor = MultiThreadedExecutor(num_threads=4)
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
