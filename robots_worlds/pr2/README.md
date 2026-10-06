# RobotLens Example 2: PR2, static

A working RobotLens fixture for the real PR2 robot (Willow Garage's
`pr2_common`/`pr2_description`), built on top of `../pr2`'s existing
flattened URDF. Unlike `../pr2` (which auto-animates the arms/head/torso/
grippers with a canned sine wave), this example deliberately has **no
animation at all** — every joint sits at its own zero/rest position and the
base never moves — so link-to-link transforms can be inspected without
motion as a confound. Use `pr2_rviz.launch.py` (below) if you want
to pose joints by hand with sliders instead.

## Design: kinematic, not Gazebo-physics-driven ("Option A")

PR2's real base is 4 independently-steered casters (8 wheels total) — a
holonomic drive, not a simple two-wheel differential drive. gz-sim has no
built-in plugin for that shape of base (unlike `../example1`'s `simple_robot`
fixture, which uses gz-sim's `DiffDrive` system on a real two-wheel rover).
Making Gazebo's own physics actually track a real PR2 base and all ~19 of
its actuated joints would need a per-joint `JointPositionController` plugin
block for each one, plus a matching `ros_gz_bridge` entry and a relay node —
a lot of moving parts, none of which can be visually verified without a live
Gazebo GUI session. (That full physics version is implemented anyway — see
"Full Gazebo-physics version" below.)

So this example stays **fully kinematic**, the same shape as `../pr2` and
`../unified`:

- `robot_state_publisher` + `joint_state_publisher` + `base_node.py` publish
  `/tf`, `/joint_states`, and `/odom` directly — no Gazebo involvement at
  all. RobotLens (and RViz) render PR2 straight from those topics.
- The launch starts PR2's `scene_description_publisher`, so RobotLens loads the
  shared static warehouse plus PR2-local static visuals for the reference table
  and object alongside PR2. These renderer-only SDF scenes are not Gazebo
  physics worlds; the table/object do not live-update in RobotLens.
- A companion Gazebo world (`gazebo/pr2_world.sdf`) is available if you want
  RobotLens's Simulation source to have something to **Attach** to (a `/clock`
  heartbeat) for a Gazebo-integration demo — but it's an empty ground plane;
  PR2 itself is never spawned into it, and nothing about the robot's motion
  depends on it running.

## Files

| File | Publishes | Message type |
|---|---|---|
| `pr2.urdf` | — | `robot_description` (copied from `../pr2/pr2.urdf`; see that example's README for how it was generated and its external-mesh-path caveat) |
| `base_node.py` | `/odom`, `/wheel_joint_states` (the 8 wheel-spin joints only), TF (`map`→`odom` static, `odom`→`base_footprint` dynamic) | `nav_msgs/Odometry`, `sensor_msgs/JointState` |
| `cmd_vel_commander.py` | `/cmd_vel` (bounded random walk, identical to `../pr2`'s/`../unified`'s — robot-agnostic). **Not run** by `pr2.launch.py` (see Running, below) — kept here only for anyone who wants to add driving back in. | `geometry_msgs/Twist` |

Plain `joint_state_publisher` (a stock ROS 2 package, not a file in this
repo — no `_gui` suffix, no window, no sliders) provides every other joint:
torso lift, head pan/tilt, both 7-DOF arms, and both grippers' one real
`*_gripper_l_finger_joint` per side (the other 3 `<mimic>` finger joints per
gripper are resolved automatically by `robot_state_publisher`), each at its
own zero/rest position. It's launched with
`source_list:=['wheel_joint_states']`, which merges `base_node.py`'s wheel
values into its output instead of also defaulting those to zero (cosmetic
only — with no `cmd_vel_commander.py` running, the wheels stay at 0 anyway).

## Running

```bash
source /opt/ros/jazzy/setup.bash
python3 robots_worlds/pr2/launch/pr2.launch.py
```

Nothing moves: `cmd_vel_commander.py` isn't part of this launch, so the base
stays parked at the origin, and every joint sits at its zero/rest position
with no GUI/sliders involved. This is meant as a clean baseline for
inspecting whether each link's transform looks right — no animation to
second-guess. Then launch RobotLens and connect — PR2 appears under Hierarchy →
Robots and the shared warehouse appears under Scene Models, fully static.

If you want it to drive around again, run `cmd_vel_commander.py` yourself
alongside the launch (`python3 robots_worlds/pr2/cmd_vel_commander.py`),
or publish directly:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.4}}" -r 10
```

## Viewing in RViz

A standalone viewer (robot_state_publisher + joint_state_publisher_gui +
rviz2 with a saved config), independent of the fixture above — for posing
the model by hand without any base motion:

```bash
source /opt/ros/jazzy/setup.bash
python3 robots_worlds/pr2/launch/pr2_rviz.launch.py
```

Fixed Frame is `base_footprint`. Since no base node runs here, every joint
(including the 8 wheels) gets a slider — there's no `source_list` split.

## Viewing PR2 directly in Gazebo

`gazebo/run_pr2_gazebo.py` is the PR2 equivalent of
`../example1/launch/example1_gazebo.py`'s `simple_robot` case: it loads the
native `PR2_SDF17/model.sdf` model and spawns it, with real physics/collision
enabled, into its own gz-sim world so you can look at it in Gazebo's own
renderer. `pr2_robotlens_gazebo.launch.py` uses the same native SDF model.
The flattened `pr2.urdf` remains the `pr2_tf_publisher` source for the ROS TF
tree only. RobotLens loads `PR2_SDF17` through the fixture's `scene_models`
parameter; its effort/velocity limits are also used to fill the SDF export's
zero actuator-effort entries so Gazebo's drive and arm controllers can apply
force.

```bash
source /opt/ros/jazzy/setup.bash
robots_worlds/pr2/gazebo/run_pr2_gazebo.py up          # GUI
robots_worlds/pr2/gazebo/run_pr2_gazebo.py up --headless
robots_worlds/pr2/gazebo/run_pr2_gazebo.py status
robots_worlds/pr2/gazebo/run_pr2_gazebo.py down
```

This is a **separate, standalone** viewer — it doesn't talk to ROS or
RobotLens at all, and isn't meant to run alongside `pr2.launch.py`
expecting them to show "the same robot" (they're two independent PR2s in
two independent renderers).

Two things worth knowing about what you'll see:

- **No base or joint actuation.** PR2's real base is 4 independently-steered
  casters (8 wheels, a holonomic drive) — gz-sim has no built-in plugin for
  that shape of base the way it has `DiffDrive` for a two-wheel rover, so
  nothing drives it. Every joint is similarly unpowered. PR2 will settle
  under gravity wherever physics puts it — arms/head/torso sag from their
  authored zero pose rather than holding it, same as any real robot with
  its motors off.
- **The gripper's four-bar linkage isn't fully constrained.** Each gripper's
  "parallel link" pair is a closed kinematic loop in the real mechanism;
  gz-sim's rigid-body physics can only represent trees, so several of the
  mechanism's coupling joints get silently dropped at load time (harmless
  — you'll see `[Err] ... child link already has a parent joint` lines in
  the log). One consequence *was* fatal: both parallel links per gripper
  had every one of their joints dropped, leaving them fully unconstrained
  and in free fall (confirmed empirically: one measured 629m below the
  ground after 11s of headless sim). `run_pr2_gazebo.py` fixes that
  specific case by welding each of those 4 links to its wrist with a
  synthesized fixed joint — see `fix_orphaned_gripper_links()`'s comment
  for the full story. The gripper will look basically right but won't
  visually open/close in physics (nothing drives it anyway, per the point
  above).

## Optional: an empty Gazebo world to Attach to

`gazebo/pr2_world.sdf` is a ground-plane-only world (no robot model — see
its own header comment and the Design note above). It exists purely so
RobotLens's Simulation panel has a session to **Attach** to if you want to
demo/exercise that workflow alongside PR2; it has no effect on the robot
itself, which keeps moving via `/tf`/`/joint_states` from
`pr2.launch.py` regardless of whether this is running.

```bash
# terminal 1
source /opt/ros/jazzy/setup.bash
robots_worlds/pr2/gazebo/run_pr2_gazebo.sh up

# terminal 2
source /opt/ros/jazzy/setup.bash
python3 robots_worlds/pr2/launch/pr2.launch.py

# terminal 3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robotlens_desktop robotlens_desktop
```

In RobotLens, open `View > Simulation` and click `Attach` — the panel should
reach `Running` (the `/clock` heartbeat from `pr2_world.sdf`). PR2 itself
was already visible before Attaching (it doesn't depend on Gazebo) and
won't change behavior after.

```bash
robots_worlds/pr2/gazebo/run_pr2_gazebo.sh status
robots_worlds/pr2/gazebo/run_pr2_gazebo.sh down
```

## Full Gazebo-physics version (real joints under physics)

The kinematic fixture above is the deliberate default. The **single-command
Gazebo-physics version** exists too, as the PR2 equivalent of
`../example1/launch/example1_robotlens_gazebo.launch.py`:

```bash
source /opt/ros/jazzy/setup.bash
python3 robots_worlds/pr2/launch/pr2_robotlens_gazebo.launch.py
```

One command owns everything: Gazebo runs the shared warehouse world with a
physics PR2 whose casters are driven by real `gz-sim` `JointController`
plugin blocks, one per wheel (built by `run_pr2_gazebo.py`'s
`with_drive_controllers=True`); `ros_gz_bridge` bridges `/clock`, `/tf`, and
`/joint_states`; `pr2_physics_controller.py` converts ROS `/cmd_vel` into
per-wheel velocity commands over gz-transport; `pr2_model_pose_root_relay.py`
relays the model pose as `odom→base_footprint` (there is no `/odom` topic —
PR2's caster base has no DiffDrive-style odometry plugin); and example1's
`scene_description_node.py` exposes the warehouse scene. Then in RobotLens
use `View > Simulation > Attach` and drive PR2 with the Simulation panel's
Teleop (Play Mode).

**Caster-base history and current state (2026-08-15):** getting this base
actually drivable took three separate fixes, in order:

1. **Chaotic instability.** The wheel `JointController` plugins originally
   ran in gz-sim's default kinematic `SetVelocity` mode (no
   `use_force_commands`), which made all four (then still actively steered)
   casters oscillate chaotically from the moment physics started --
   steering velocities up to +/-10 rad/s, even with zero `/cmd_vel` ever
   published. Fixed by switching to `use_force_commands` with a real PID
   gain, so DART treats the wheel as an ordinary torque-limited actuator.
2. **Steering couldn't hold heading under load.** Even stabilized, the
   independently-steered casters drifted several to tens of degrees off
   heading under real wheel reaction torque, canceling most of the net
   drive force across the four corners. Rather than keep chasing
   zero-lag four-corner steering control, `add_caster_drive_controllers()`
   now welds all four casters permanently forward-facing (a `<fixed>`
   joint, no steering DOF at all) and drives them as a skid-steer base
   instead -- `pr2_physics_controller.py`'s kinematics changed to match
   (differential wheel speed per side, no steer angle). This trades away
   holonomic strafing (already a Shift+Left/Right bonus in RobotLens's own
   teleop, not the primary interaction) for a much simpler, more robust
   drive model.
3. **The graspable object was silently anchoring the base.** Even with
   steering removed, the *real* fixture (this launch, arm + graspable
   object + all) still barely translated, while an isolated test world
   with just the wheels worked fine. A/B tested live with everything else
   identical: `with_graspable_object=True` covered <2 cm in a 2 s full
   forward command, `=False` covered ~0.56 m. Root cause:
   `add_object_attachment_plugin`'s `DetachableJoint` starts attached
   (gz-sim's own default), and `pr2_grasp_controller.py`'s startup-detach
   retry logic doesn't reliably win that race before a drive command
   arrives -- with the gripper still rigidly attached to an object resting
   on a static table, the whole robot behaves like it's anchored. Fixed:
   `prepare_world()` now only spawns the graspable object (and
   `pr2_grasp_controller.py`, which has nothing to do without it) when
   `pick_place:=true`, since the object is only ever meaningful for the P3
   pick-place task anyway.

**Current state, confirmed live:** with `pick_place:=false`, forward and
backward driving both work end to end through the real `/cmd_vel` ->
`pr2_physics_controller.py` -> gz-transport path (roughly 0.13 m/s net,
some loss from acceleration ramp-up versus the ~0.28 m/s a full-speed wheel
command would give with zero slip). **Turning does not** -- tested at both
a modest 0.5 rad/s and a strong 2.0 rad/s `angular.z`, and at both normal
(mu=1.0) and elevated (mu=3.0) floor friction, the base's measured
orientation shows zero change every time, confirmed via both `/tf` and raw
gz-transport `/model/pr2/pose`. Wheel joint state during a turn command
shows an inconsistent, partial pattern (some wheels tracking their
differential target, others staying near zero) rather than a clean
explanation -- the four-corner, 8-wheel rectangular-footprint skid-steer
geometry may need a different kinematic model or per-wheel tuning than
plain differential left/right speed. Not yet investigated further.

The same physics launch now also installs conservative position controllers for
the seven validated right-arm joints and `r_gripper_l_finger_joint`. They are
connected through `pr2_arm_trajectory_controller.py`, which provides standard
ROS 2 actions: `/pr2_right_arm_controller/follow_joint_trajectory`
(`control_msgs/action/FollowJointTrajectory`) and
`/pr2_right_gripper_controller/gripper_cmd`
(`control_msgs/action/GripperCommand`). The adapter rejects malformed,
incomplete, duplicate, stale, or out-of-limit commands and makes no direct
command topic a user-facing control path.

Install the standard action definitions before running this launch:

```bash
sudo apt-get install ros-jazzy-control-msgs
```

**Known limitation:** `r_shoulder_lift_joint`'s simulated Gazebo actuation
does not reliably track commanded targets or hold position -- it drifts
slowly even with no command in flight, and increasing the controller's
`p_gain` makes its error worse rather than better, which rules out ordinary
gravity droop or actuator saturation as the cause. The other six right-arm
joints (and the gripper) converge correctly. See
`gazebo/run_pr2_gazebo.py`'s `add_right_arm_controllers` docstring for what
has been ruled out; root cause not yet identified. Any pose that requires
moving `r_shoulder_lift_joint` away from wherever it is currently resting
is not reliably reachable in this fixture until this is fixed.

### MoveIt 2 configuration (P2)

`pr2_right_arm_moveit_config` (a subdirectory of this fixture, not a
standalone example -- it depends on `pr2.urdf` here) is a standalone ROS 2
*package* (SRDF, kinematics, joint limits, OMPL planning, and
controller-manager config for the `right_arm`/`right_gripper` groups) that
lets `move_group` plan and execute against the P1 controllers above. Build
and run it separately from this fixture's own launch:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --paths robots_worlds/pr2/pr2_right_arm_moveit_config
source install/setup.bash
python3 robots_worlds/pr2/pr2_right_arm_moveit_config/launch/move_group.launch.py
```

`move_group` loads cleanly and plans and executes real `right_arm` goals,
including a full home&harr;ready round trip and IK-solved reach poses at
the P3 graspable object (not just canned joint targets). One right-arm
joint, `r_wrist_roll_joint`, still has an open, real physics-engine
instability (not a controller-tuning issue -- confirmed by disabling its
controller entirely, see `gazebo/run_pr2_gazebo.py`'s
`add_right_arm_controllers` docstring) that intermittently fails an
otherwise-correct goal's tolerance check; P4's task node retries around it
but hasn't yet beaten every occurrence. See
[`docs/IMPLEMENTATION_PLAN_PR2_MOVEIT_PICK_PLACE.md`](../../docs/IMPLEMENTATION_PLAN_PR2_MOVEIT_PICK_PLACE.md)
for the full plan and current status.

**Note on RobotLens's own Motion panel against this fixture:** the validation
above is through `pick_place_task.py`/`idle_to_approach.py`, which use
MoveIt's own official Python bindings (correct service names built in).
RobotLens's own C++ `MoveItAdapter` is a separate integration with its own
history: it hardcoded the wrong `/move_group/...`-prefixed service names
until 2026-08-15 (see `docs/PROJECT_STATUS.md`'s Phase 5 update). After that
fix, planning still failed with a generic `MoveItErrorCodes::FAILURE` for
every request, root-caused the same day: with `pick_place:=false` (no P1
adapter to move the arm first), the right arm's Gazebo controllers held it
at their implicit 0.0 rad spawn setpoint, which is inside
`r_elbow_flex_joint`/`r_wrist_flex_joint`'s hard URDF limits but outside
their `<safety_controller>` soft limits -- MoveIt's `CheckStartStateBounds`
adapter enforces the soft limits and rejected every plan before OMPL ever
ran. Fixed in `run_pr2_gazebo.py` by giving the right arm's controllers an
`initial_position` seeded from `pr2_right_arm.srdf`'s own `home` group
state, the same technique the left arm/head already used. Discovery and
planning are now both confirmed live through this exact fixture. The
panel's own Plan/Execute/confirm click-through is now also confirmed live
(2026-08-15), driven interactively through the real app: discover ->
Plan an end-effector pose goal (`SUCCESS`, 24-point trajectory) -> Execute
-> confirm through the shared confirmation modal -> execution `succeeded
in 0.3s`, max tracking error `0.0000 rad`. That run surfaced and fixed two
more bugs (see `docs/PROJECT_STATUS.md`'s later 2026-08-15 update): `move_
group.launch.py` never actually loaded the URDF into its own
`robot_description` parameter (missing `publish_robot_description=True`,
so it endlessly retried fetching it from a `robot_state_publisher` node
that doesn't exist in this fixture), and `MoveItPanel.cpp` sent the end
effector's SRDF *name* (`"right_eef"`) instead of its `parent_link`
(`"r_wrist_roll_link"`) as the pose-goal link, which `move_group` rejected
with "Link 'right_eef' not found in model 'pr2'".

### Grasp and object ownership (P3)

The physics launch also spawns a single graspable box on a small table in
front of the right arm and starts `pr2_grasp_controller.py`, both sourced
from one shared module, `grasp_object.py` (dimensions, mass, friction, and
nominal poses), so the Gazebo model and the MoveIt planning-scene
`CollisionObject` never disagree. The controller:

- seeds the planning scene with the table and object at startup, and keeps
  the object's world-frame pose synchronized with its live Gazebo pose
  while it isn't attached;
- frees the object at startup (Gazebo's `gz-sim-detachable-joint-system`
  plugin starts attached by its own default -- see
  `gazebo/run_pr2_gazebo.py`'s `add_object_attachment_plugin` docstring), and
  republishes the detach on a timer until a DetachableJoint state transition
  or the object's TF height confirms the plugin actually processed it -- a
  single startup message can race the plugin's load and be silently dropped,
  leaving the object welded to the arm and dragged by it; and
- exposes `/pr2_grasp_controller/attach` and `/pr2_grasp_controller/detach`
  (`std_srvs/srv/Trigger`) as the grasp primitives a later pick-and-place
  task node (P4) composes into a full state machine. `attach` requires both
  a closed-enough gripper and the object sitting within a small
  palm-relative envelope before creating a fixed joint in Gazebo and an
  `AttachedCollisionObject` in MoveIt together; `detach` reverses both and
  restores the object as a world collision object. Every rejection names
  the exact failed check (stale state, gripper not closed, object out of
  envelope, or a MoveIt/Gazebo call failure) rather than a generic error.

This was validated live end to end -- precondition rejections, a successful
attach/detach cycle, correct planning-scene state on both sides, and
double-attach/double-detach rejection -- both by teleporting the object to
the gripper via Gazebo's `set_pose` service (the original P3 validation)
and, in a later session, by a real MoveIt-planned reach to an IK-solved
grasp pose (not a teleport). See the P4 section below and the plan doc for
the current reliability caveat on the reach path.

### Pick-and-place task (P4)

`pr2_pick_place_interfaces` (a subdirectory of this fixture) defines the
`PickPlace` action
(`object_id`, `grasp_candidates`, `placement_pose`, `planning_timeout`,
`velocity_scaling` / `success`, `message`, `failed_state`,
`final_object_pose` / `current_state`, `elapsed_seconds` feedback), and
`pick_place_task.py` (started automatically by this fixture's launch file)
implements the full state machine from the plan doc:

```text
Validate scene -> Open -> Pre-grasp -> Approach -> Close -> Confirm/attach
-> Lift -> Pre-place -> Lower -> Open/detach -> Retreat -> Verify placement
```

It composes P1's actions, P2's `move_group` (grasp/lift/place poses are
solved live via `/compute_ik`, never a canned trajectory), and P3's
attach/detach -- and gates success on the object's real final TF pose
matching the requested placement within 5 cm, not on the mechanical steps
alone completing. Build the interfaces package once, same pattern as P2:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --paths robots_worlds/pr2/pr2_pick_place_interfaces
source install/setup.bash
```

Then with the fixture and `move_group` both running, send a goal -- either
with a client of your own, or with the bundled gate runner:

Open three terminals from the workspace root. Source ROS 2 in each terminal;
after building the two packages above, also source the workspace overlay in the
second and third terminals.

```bash
# terminal 1: Gazebo PR2 fixture
source /opt/ros/jazzy/setup.bash
python3 robots_worlds/pr2/launch/pr2_robotlens_gazebo.launch.py

# terminal 2: MoveIt 2
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 robots_worlds/pr2/pr2_right_arm_moveit_config/launch/move_group.launch.py

# terminal 3: send one task goal
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 robots_worlds/pr2/scripts/pick_place_client.py 1
```

Wait for the first two processes to finish their startup checks before running
the client. The `1` requests one run; increase it only when intentionally
running repeated validation.

```bash
python3 robots_worlds/pr2/scripts/pick_place_client.py [num_runs]
```

which loops the pick-and-place (default 20 runs) and exits reporting each
run's outcome. See `docs/IMPLEMENTATION_PLAN_PR2_MOVEIT_PICK_PLACE.md`'s P4
status paragraph for exactly what's been confirmed live so far, and the open
`r_wrist_roll_joint` caveat that can fail an otherwise-correct run.

#### Idle -> approach demo (no picking)

To move the right arm from its idle (home) pose to the grasp-ready approach
pose -- palm `APPROACH_Z_OFFSET_M` above the live object, gripper open, and
stopped there, with no closing/attaching/lifting -- run the fixture with the
pick-place task disabled (this demo owns that node's machinery itself), plus
`move_group`, then:

```bash
python3 robots_worlds/pr2/launch/pr2_robotlens_gazebo.launch.py pick_place:=false
python3 robots_worlds/pr2/pr2_right_arm_moveit_config/launch/move_group.launch.py
python3 robots_worlds/pr2/scripts/idle_to_approach.py
```

`idle_to_approach.py` instantiates `pick_place_task`'s own `Pr2PickPlaceTask`
(so it reuses the exact IK probe/margin selection and the `move_group` retry
semantics) but drives the helpers directly and never sends a `/pick_place`
goal. While it runs, the left arm stays put at the fixture's mirrored natural
resting pose (`LEFT_ARM_STATIC_POSE` in `gazebo/run_pr2_gazebo.py`, held by
per-joint `JointPositionController` plugins -- it mirrors the right arm's
home so both arms read as one balanced stance).

### Observation: task/plan/outcome/ownership diagnostics (P5)

`pick_place_task.py` and `pr2_grasp_controller.py` both publish low-rate,
bounded status on the standard `/diagnostics` topic
(`diagnostic_msgs/DiagnosticArray`, via `diagnostic_updater` -- same
dependency and pattern as `robots_worlds/example1/diagnostics_node.py`; if not
already installed: `sudo apt install ros-jazzy-diagnostic-updater`). No new
RobotLens UI or custom topic is involved -- its existing `DiagnosticsPanel`
already subscribes to and renders this:

- **PickPlace Task** (`pick_place_task.py`): current state-machine state,
  the active goal's UUID as its plan ID, and the last completed run's
  outcome. `OK` while idle or after a successful run, `WARN` after a
  failed or canceled one, with the exact failure message attached.
- **Object Ownership** (`pr2_grasp_controller.py`): whether the graspable
  object is currently attached to the gripper.

### Manipulation baseline validation

Before enabling PR2 arm controllers or MoveIt, verify the converted SDF still
has the expected right-arm chain, limits, collision geometry, and one
commandable gripper joint:

```bash
robots_worlds/pr2/gazebo/run_pr2_gazebo.py verify
```

This command does not start Gazebo. It fails rather than generating a model if
the URDF-to-SDF conversion has changed the seven-joint right-arm chain or the
`r_gripper_l_finger_joint` command joint. The full MoveIt implementation plan
is in [`docs/IMPLEMENTATION_PLAN_PR2_MOVEIT_PICK_PLACE.md`](../../docs/IMPLEMENTATION_PLAN_PR2_MOVEIT_PICK_PLACE.md).
