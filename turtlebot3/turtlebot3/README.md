# RobotLens Example: TurtleBot3 Burger, Gazebo physics + Nav2

A physics-driven TurtleBot3 Burger fixture (Gazebo Harmonic `DiffDrive`,
same launch shape as `../pr2`'s "Full Gazebo-physics version") with Nav2
bringup, so RobotLens's Navigation panel
(`docs/FUTURE_IMPROVEMENTS.md` Phase M) has a live
`nav2_msgs` stack to discover, plan against, and navigate with.

## Files

| Path | Publishes | Purpose |
|---|---|---|
| `robots/turtlebot3_burger.urdf` | -- | Robot description (standard ROBOTIS TurtleBot3 Burger URDF: base, two driven wheels, caster, IMU, and a `base_scan` lidar mount -- see "Why no AMCL" below for why nothing publishes `/scan` from it) |
| `scripts/gazebo_runner.py` | -- | Builds the physics world (spawns the URDF as gz model `simple_robot` -- an internal name carried over from `../example1`'s DiffDrive plugin config, not a naming bug) |
| `scripts/model_pose_root_relay.py` | `odom`->`base_footprint` TF | Relays Gazebo's true 3D world pose as the odom->base_footprint transform (the bridged `/odom` topic is a flattened 2D DiffDrive estimate; see the script's own docstring) |
| `scripts/diagnostics_node.py` | `/diagnostics` | Frequency/health checks so the Diagnostics panel has something real to show |
| `scripts/scene_description_node.py` | `scene_models` param | Shares the `open_interior` warehouse SDF scene with RobotLens |
| `nav2/maps/open_interior.{yaml,pgm}` | -- | Synthetic static map for Nav2 (see "Why a synthetic map" below) |
| `nav2/nav2_params.yaml` | -- | map_server/planner_server/controller_server/behavior_server/bt_navigator config |
| `launch/turtlebot3.launch.py` | -- | Single launch: Gazebo + bridge + robot_state_publisher + the three scripts above, plus the static `map`->`odom` transform and Nav2's five servers under one `lifecycle_manager` |

## Running

Nav2 (`ros-jazzy-navigation2` and `ros-jazzy-nav2-bringup`, which pull in
`nav2-map-server`, `nav2-planner`, `nav2-controller`, `nav2-behaviors`,
`nav2-bt-navigator`, and `nav2-lifecycle-manager`) is not part of the
RobotLens package's own dependencies (`package.xml`/`CMakeLists.txt`) --
only `nav2_msgs` (the message/action definitions the C++ adapter links
against) is. Install the servers separately:

```bash
sudo apt-get update
sudo apt-get install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

Then:

```bash
source /opt/ros/jazzy/setup.bash
python3 examples/turtlebot3/launch/turtlebot3.launch.py
```

Wait for the Nav2 nodes to report all five reaching the `active` lifecycle
state, then launch RobotLens (`ros2 run robotlens_desktop robotlens_desktop`),
open `View > Navigation`, and check the panel's off-by-default "Enable
navigation (Nav2)" checkbox. The status line should reach `connected`, the
current pose should populate from `/odom` (no `/amcl_pose` on this fixture),
and Plan/Navigate/Cancel/Clear Costmap should all work against goals inside
roughly `[-4.5, 4.5]` on both axes (the map's free interior). **Live-verified
end to end (2026-08-15):** discover -> plan (a real multi-point path) ->
confirm (the shared confirmation modal shows the correct robot/action/goal)
-> navigate (the real Gazebo robot drove to the goal and arrived) -> clear
costmap. See `docs/PROJECT_STATUS.md`'s 2026-08-15 update for the two bugs
this surfaced and fixed (`lifecycle_manager`'s node-name mismatch and two
`/`-vs-`::` pluginlib class names).

## Behavior Tree status

Nav2 publishes per-node status changes on the stock `/behavior_tree_log`
topic. RobotLens consumes that topic directly, so the fixture requires no
RobotLens-specific plugin or tree instrumentation.

## Why no AMCL

Real localization needs a real sensor. This URDF has a `base_scan` lidar
mount, but nothing publishes `/scan`: adding a working Gazebo lidar sensor
plugin + `ros_gz_bridge` entry hits the same host-specific blocker recorded
in `docs/PROJECT_STATUS.md` -- `gz sim`'s sensor rendering (ogre1/ogre2)
aborts in EGL init under `--headless-rendering` on this class of host, so a
live lidar can't be verified here (same reason the PR2 fixture's own
sensor-facing Phase 8 consumer is unverified). Rather than ship an AMCL
config nobody can run, `turtlebot3.launch.py` publishes a static
identity `map`->`odom` transform instead: `RobotLens`'s `Nav2Adapter`
already discovers pose via `/amcl_pose` with an `/odom` fallback
(`docs/PROJECT_STATUS.md`, Nav2 core section), so it works unmodified
against this fixture's ground-truth `/odom`->`base_footprint` chain --
there is just no independent correction of drift against a real map. If a
host with working headless sensor rendering becomes available, `nav2_params.yaml`
already carries a placeholder `amcl` block documenting the swap.

## Why a synthetic map

`nav2/maps/open_interior.{yaml,pgm}` is a plain 10m x 10m bordered room
(4px/0.2m wall ring, 0.05 m/cell), not a survey of the Gazebo
`open_interior` world's actual geometry -- see the comment at the top of
`open_interior.yaml` for the exact generator. It exists to give the
planner/costmaps/`bt_navigator` a valid `map` frame to plan in, not to model
real obstacles: since there is no lidar (above), nothing in this fixture
checks a goal or path against the real Gazebo scene's walls either way.
Keep navigation goals within the free interior (`[-4.5, 4.5]` on both axes,
origin at the robot's Gazebo spawn point) so the planner has room to work.

## Known limitations

- No dynamic obstacle avoidance: both costmaps use only `static_layer` +
  `inflation_layer` (no `voxel_layer`/`obstacle_layer`, since there is no
  `/scan` to feed one). A path that is clear in the static map is planned
  and followed even if something is actually in the way in Gazebo.
- No real localization (see "Why no AMCL"): `map`->`odom` never corrects
  for drift, so long/looping navigation runs can accumulate error between
  where Nav2 thinks the robot is and where it actually is in Gazebo.

## MuJoCo backend (lightweight alternative)

The same TurtleBot3 Burger is also available as a MuJoCo-native fixture that
runs in-process (no Gazebo, no `ros_gz_bridge`). This is useful for
fast iteration without the overhead of an external physics server.

### Files

| Path | Purpose |
|---|---|
| `mujoco/turtlebot3_burger.xml` | MuJoCo MJCF model: base, two driven wheels, caster, lidar mount. Uses the same STL meshes as the URDF. |
| `scripts/run_mujoco_turtlebot3.py` | Convenience script: creates a RobotLens project file with MuJoCo backend configured and launches RobotLens with `--project`. |

### Prerequisites

MuJoCo must be installed (headers + shared library). The build system looks
for MuJoCo at `/tmp/mujoco/mujoco-3.3.0/` by default. If installed elsewhere,
set `MUJOCO_INSTALL_DIR` in CMake or the `MUJOCO_ROOT_DIR` environment
variable.

### Running

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 examples/turtlebot3/scripts/run_mujoco_turtlebot3.py
```

This creates a RobotLens project file at `~/.robotlens/mujoco_turtlebot3/project.json`
with the MuJoCo backend and world file pre-configured, then launches RobotLens
with `--project`.

After RobotLens opens:
1. The project loads with MuJoCo backend pre-configured
2. Open the **Simulation** panel (View > Simulation)
3. Press **Launch** to start MuJoCo physics
4. Use arrow keys or WASD for teleop

### What's different from Gazebo

- **In-process**: `mj_step()` runs on the main thread at ~0.5ms per step — no subprocess, no bridge
- **Direct teleop**: `injectTeleop()` writes wheel velocities directly to `mjData->ctrl`, bypassing ROS `/cmd_vel`
- **No ROS bridge**: joint states and poses are read directly from `mjData->xpos`/`mjData->xquat` after each step
- **No Nav2**: the MuJoCo fixture is physics-only; Nav2 bringup requires the Gazebo version
