# RobotLens example: pr2_classic — the classic ROS tool stack, one launch

A single-command demo that brings up the whole old-school ROS tool stack around
the same physics PR2 the `../pr2` fixture drives, all talking to one ROS graph.
It exists to exercise the traditional tools the way a classic ROS 1
`roslaunch` demo did — not to add anything RobotLens-specific.

Run it (each command in its own terminal as needed):

```bash
# terminal 1: everything
source /opt/ros/jazzy/setup.bash
python3 robots_worlds/pr2_classic/launch/pr2_classic.launch.py
```

One launch file opens, on the same simulation:

| Tool | What you get |
|---|---|
| **Gazebo** (`gz sim`, visible GUI) | The shared `open_interior` warehouse world with the physics PR2: driven caster base, right-arm/gripper position controllers, graspable object |
| **RViz** (`rviz2`) | `../pr2/rviz/pr2.rviz` with the live RobotModel, TF tree, fixed frame `base_footprint` |
| **joint_state_publisher_gui** | The classic joint sliders (see "Sliders vs physics" below) |
| **rqt** | Opens a saved perspective with **Console** and **Topic Monitor** docked, so it shows live content immediately (not a blank window) |
| **rqt_graph** | The ROS computation-graph viewer |
| **MoveIt 2** | `move_group` from `../pr2/pr2_right_arm_moveit_config` |
| **Nav2** | Full `bringup_launch.py`: map_server, AMCL, planner, controller, BT navigator |

Everything is fed by the pr2 fixture's machinery, reused as-is (never copied):
the world comes from `../pr2/gazebo/run_pr2_gazebo.py`, `/cmd_vel`→caster
conversion from `../pr2/pr2_physics_controller.py`, the trajectory/gripper
actions from `../pr2/pr2_arm_trajectory_controller.py`, the root-pose TF from
`../pr2/pr2_model_pose_root_relay.py`, the grasp controller from
`../pr2/pr2_grasp_controller.py`, and the warehouse scene publisher from
`../pr2/scripts/scene_description_node.py`. The only new code here is glue:
`pr2_classic_odom.py` (Nav2 needs an `/odom` topic, which PR2's holonomic
caster base has no plugin for) plus this launch file and the Nav2 config.

With everything up, RobotLens can Attach to the same simulation
(`View > Simulation > Attach`) and render the identical robot from the same
`/tf`/`/joint_states` the classic tools are showing.

## What actually runs, and what's skipped automatically

The launch checks the ament index and **skips MoveIt and/or Nav2 with a warning
on stderr when their packages aren't available**, rather than failing the whole
launch. Force either back on with `moveit:=true` / `nav2:=true` (only useful
after installing the prerequisites below).

- **MoveIt 2** needs the pr2 fixture's MoveIt package built *and* the workspace
  overlay sourced, exactly like `../pr2`:

  ```bash
  source /opt/ros/jazzy/setup.bash
  colcon build --paths robots_worlds/pr2/pr2_right_arm_moveit_config
  source install/setup.bash
  ```

  then re-run the pr2_classic launch from a shell that sources that overlay.
  `move_group` plans/executes through the same P1 trajectory/gripper actions as
  the pr2 fixture (the fixture's `r_shoulder_lift_joint` /
  `r_wrist_roll_joint` caveats in `../pr2/README.md` still apply).

- **Nav2** needs the standard packages:

  ```bash
  sudo apt install ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-amcl ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-planner ros-jazzy-nav2-controller \
    ros-jazzy-nav2-bt-navigator ros-jazzy-nav2-behavior-tree \
    ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-dwb-controller \
    ros-jazzy-nav2-velocity-smoother ros-jazzy-nav2-waypoint-follower \
    ros-jazzy-nav2-collision-monitor ros-jazzy-nav2-navfn-planner \
    ros-jazzy-nav2-voxel-grid
  ```

## Nav2 notes — best-effort by design

PR2's physics world bridges no lidar, so this demo ships a deliberately blank
20×20 m map (`nav2/pr2_classic_map.pgm`) and a standard params file
(`nav2/nav2_params.yaml`). Nav2 comes up fully lifecycle-active with empty
costmaps: you can watch the servers in `rqt_graph` and `rqt_console`, send
goals, and see the BT navigator plan across free space. Two consequences:

- **No localizing AMCL.** The launch adds a static `map→odom` identity
  transform so the TF tree is complete without laser scans. If you later bridge
  a `gz-sim` lidar as `/scan`, remove that static transform so AMCL can own
  `map→odom` instead.
- **The controller can actually drive the robot.** Nav2's `/cmd_vel` flows into
  the pr2 fixture's `pr2_physics_controller.py` → per-caster commands, so the
  PR2 moves in Gazebo/RViz. With no costmap data it will happily drive into
  walls — don't leave Nav2 goals running unattended against the warehouse
  geometry.

## Sliders vs physics (two publishers on purpose)

The PR2's arm/gripper joints are *continuous* (no URDF limits), so
`joint_state_publisher_gui` runs **without** `source_list` and is the single
authoritative `/joint_states` publisher. Dragging a slider really moves the
model in RViz (robot_state_publisher consumes that topic) and nothing fights
back — the classic joint-publisher demo.

The physics world does *not* use that topic. Gazebo's real joint states are
bridged to a separate `/gz_joint_states` topic, and only the pr2 fixture's
controllers consume them (`pr2_arm_trajectory_controller.py` and
`pr2_grasp_controller.py` are launched with
`-r /joint_states:=/gz_joint_states`). So MoveIt execution and grasp logic keep
seeing true physics while the sliders control the display model.

Two consequences worth knowing:

- The RViz model shows whatever the sliders say; if you leave them at their
  initial (all-zero) pose while physics settles, RViz and Gazebo can disagree —
  that's the point of the sliders, not a bug.
- `move_group`'s planning-scene "current state" follows `/joint_states` (the
  sliders) while its controller feedback comes from `/gz_joint_states` (physics).
  Keep the sliders near the physics pose before sending a MoveIt goal to avoid a
  jump at execution start.

## Known limitations

- The missing-mesh/texture warnings in RViz for
  `/home/.../pr2_common/pr2_description/materials/textures/*` are the inherited
  pr2 URDF external-mesh caveat (`../pr2/README.md`); textures render blank,
  geometry and transforms are correct.
- `gz sim` logs `stopKd`/`stopKp`/`provide_feedback` SDF warnings for the PR2
  model — same world the pr2 physics fixture generates, harmless.
- All of `../pr2/README.md`'s physics/MoveIt caveats apply unchanged.
- Nav2 params follow the standard Jazzy layout; if your installed Nav2 version
  differs, `nav2/nav2_params.yaml` may need a small adjustment.
- rqt's Topic Monitor prints benign `Malformed msg message_type: ..._FeedbackMessage`
  warnings for the trajectory/gripper action topics (a known rqt_topic limitation
  with action types); the console and topic views work normally.
- `rqt --perspective-file rqt/pr2_classic.perspective` imports that perspective
  into `~/.config/ros.org/rqt_gui.ini` (a hidden `@pr2_classic.perspective`
  becomes rqt's current perspective). A later bare `rqt` reopens it; delete that
  config file to reset.

## Launch arguments

```bash
python3 robots_worlds/pr2_classic/launch/pr2_classic.launch.py \
  moveit:=false nav2:=false use_sim_time:=false
```

- `use_sim_time` — follow the bridged Gazebo `/clock` (default `true`).
- `moveit` — default auto (on only when `pr2_right_arm_moveit_config` is built
  and sourced).
- `nav2` — default auto (on only when `nav2_bringup` is installed).
