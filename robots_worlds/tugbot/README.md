# RobotLens tugbot example

This example uses the MOV.AI tugbot AMR (differential-drive base with a
linear gripper rail), vendored as a Fuel-style SDF model
(`tugbot/model.config` + `tugbot/model.sdf`) with front/back cameras,
front/back/omni gpu_lidars, and an IMU. Unlike TurtleBot3, tugbot's own
`model.sdf` already ships the DiffDrive, PosePublisher, and
JointStatePublisher plugins it needs for Gazebo physics -- no URDF->SDF
conversion step -- so Gazebo spawns `tugbot/` directly via `<include>`.

Run it after sourcing ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
python3 robots_worlds/tugbot/launch/tugbot.launch.py
```

This starts Gazebo (headless), the ROS-Gazebo bridge, robot_state_publisher,
the scene description node, `model_pose_root_relay.py`, and
`diagnostics_node.py`. Run the RobotLens desktop UI separately, use
`View > Simulation > Attach` to connect to it, and set `tugbot`'s source to
`Simulation`. Drive it with RobotLens's own in-app teleop (Simulation panel >
Teleop (Play Mode) > Enter Play Mode) using arrow keys + numpad -- no
external terminal or teleop process needed.

The fixture publishes:

- `/tf` (`odom -> base_link` via `model_pose_root_relay.py`'s relay of
  Gazebo's true physics-solved model pose, plus `base_link -> ...` for every
  other link from the bridged PosePublisher pose)
- `/odom` (`nav_msgs/msg/Odometry`, from DiffDrive)
- `/joint_states` (`sensor_msgs/msg/JointState` -- currently just the
  gripper rail/hand joints; tugbot's own `model.sdf` JointStatePublisher
  plugin only lists those two)
- `/diagnostics` (`diagnostic_msgs/msg/DiagnosticArray`, see
  `diagnostics_node.py`'s docstring)

`/cmd_vel` (`geometry_msgs/msg/Twist`) drives the base; stop the in-app
teleop if you want to drive it manually, for example:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \\
  "{linear: {x: 0.2}, angular: {z: 0.4}}" -r 10
```

`robot_description` is auto-published from `tugbot/generated/tugbot.urdf`
(generated from `tugbot/model.sdf` by `tools/sdf_to_urdf` -- regenerate
after changing `model.sdf`:

```bash
build/robotlens_desktop/sdf_to_urdf robots_worlds/tugbot/tugbot robots_worlds/tugbot/tugbot/generated/tugbot.urdf
```

), and the static `warehouse` scene environment is auto-loaded too via
`scene_description_node.py`'s `scene_models` parameter, so just running the
launch file shows the AMR and the environment together in RobotLens out of the
box.

`robots_worlds/worlds/warehouse` is a Fuel-style model folder (a static building
shell -- floor + walls, one link, no plugins of its own), not a `<world>`,
so `gazebo_runner.py` generates a minimal world (physics/user-commands/
scene-broadcaster/sensors system plugins + a sun light, named
`tugbot_view`) and `<include>`s both it and tugbot into it.

**Not yet wired up:** tugbot's cameras, lidars, and IMU are present in
`model.sdf` (and will render/simulate fine in Gazebo's own GUI) but have no
`ros_gz_bridge` entries here yet -- this fixture is spawn+drive+view+
`/joint_states`+`/odom` only, matching TurtleBot3's and Panda's own
Gazebo-physics fixtures. Add bridge entries in `launch/tugbot.launch.py`'s
`bridge_config()` if you need those topics on the ROS graph.

## Viewing in Gazebo directly

`tugbot.launch.py` always runs `gz sim` headless (`-s --headless-rendering`).
To instead watch the physics in Gazebo's own GUI, generate the same world
and point `gz sim` at it yourself:

```bash
source /opt/ros/jazzy/setup.bash
python3 -c "
import sys
sys.path.insert(0, 'robots_worlds/tugbot/scripts')
import gazebo_runner
print(gazebo_runner.write_warehouse_world())
"
# prints the generated world path; then:
GZ_SIM_RESOURCE_PATH=robots_worlds/tugbot:robots_worlds/worlds gz sim <printed path> -r
```

Do not run this at the same time as `tugbot.launch.py` -- `gz-transport` is
unpartitioned by default, so two independent `gz sim` processes both
simulating a model named `tugbot` will cross-talk on identically-named
topics.

## Model provenance and Harmonic port

`tugbot/model.sdf` is a MOV.AI Fuel model originally authored for
Ignition/Gazebo Citadel (`ignition-gazebo-*-system` plugin filenames,
`ignition::gazebo::systems::*` class names). This repo's Gazebo install is
Harmonic (`gz-sim8`), which ships no `libignition-gazebo-*` aliases, so the
plugin filenames/class names in `model.sdf` were rewritten to Harmonic's own
`gz-sim-*-system` / `gz::sim::systems::*` naming -- see the comment above
model.sdf's `<!-- CONTROLERS PLUGINS -->` block. Its `wheel_front`/
`wheel_back` caster joints are SDF `<joint type="ball">`, which URDF has no
equivalent for; `SdfModelParser::jointTypeToString` falls back to `"fixed"`
for any SDF joint type it doesn't otherwise recognize, so both casters come
out as fixed joints in `tugbot.urdf` (they render frozen in RViz/RobotLens's
URDF view; Gazebo physics, which reads `model.sdf` directly, still treats
them as real ball joints). If you regenerate `tugbot.urdf` after an SDF
change, re-run `check_urdf` on the result.

## Mesh `<collision>` geometry doesn't work under dartsim

gz-sim Harmonic's default physics engine, dartsim, does not implement
constructing collision shapes directly from an SDF `<mesh>` element --
confirmed via `gz sim --verbose 4`:

```
[Dbg] Mesh construction from an SDF has not been implemented yet for dartsim.
[Dbg] The geometry element of collision [warehouse_collision] couldn't be created
```

This silently produces **no collision at all** for that link -- nothing
crashes or errors at the default log level, it just falls through. Both
`robots_worlds/worlds/warehouse/model.sdf`'s floor (`warehouse_collision`) and
`tugbot/model.sdf`'s own chassis (`base_link_collision`) originally used
`<mesh>` collisions for exactly this reason and were replaced with `<box>`
primitives sized from each mesh's own vertex bounding box (visuals are
untouched, still the full detailed mesh). If you add a new mesh-based
`<collision>` anywhere in either file, replace it with a primitive
(`box`/`cylinder`/`sphere`) the same way, or physics will silently pass
through it.
