# ROS 2 two Panda arms

Two Franka Pandas on the ground plane, 1.6 m apart. MuJoCo runs inside RobotLens, and
communication uses ROS 2. Each arm is commanded through its own namespaced MuJoCo bridge topic.

## Quick start

From this project folder:

```bash
python3 scripts/quickstart.py setup   # check prerequisites and prepare the project
python3 scripts/quickstart.py test    # smoke test in RobotLens
python3 scripts/quickstart.py run     # smoke test plus the two-arm scenario
python3 scripts/quickstart.py         # setup, then run
```

`test` uses the running RobotLens session if there is one. Otherwise it starts RobotLens from
`../../RobotLens` (build it first with `./build.sh`) and closes it at the end, keeping its log
in `evidence/quickstart/robotlens.log`. The shared code is in `../tools/quickstart.py`.

`run` opens the project, closes the splash screen, starts ROS 2 and the simulation, enables the
simulation bridge, and then moves both arms at once to the Panda ready pose `[0, -0.785, 0, -2.356, 0, 1.571, 0.785]`, then
turns `arm1`'s base to +1.2 rad and `arm2`'s to -1.2 rad (`[±1.2, -0.3, 0.4, -1.8, 0.3, 1.8, 0]`). Each pose is published as a `sensor_msgs/JointState` on
`/arm1/cmd_joints` and `/arm2/cmd_joints` before either arm is checked. Every joint of each arm must
end within 0.05 rad of its target and holding, measured on `/armN/joint_states`. It then stops the
simulation and communication and writes `evidence/quickstart/report.json`.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/ros-mujoco-2panda-ground-plane.xml` | Project with both Pandas |
| `robots/panda.urdf`, `robots/meshes/` | Panda model prepared for MuJoCo |
| `scripts/build_project.py` | Builds `.robotlens/` over MCP |

## Namespaces

| Robot | Position | Joint Control state endpoint | Namespace |
| --- | --- | --- | --- |
| `panda` | `(0, 0.8)` | `/arm1/joint_states` | `arm1` |
| `panda_copy` | `(0, -0.8)` | `/arm2/joint_states` | `arm2` |

Each Panda has a Joint Control capability, set with `robot.joint_control.configure`. RobotLens
takes the simulation namespace from the folder of its state endpoint, so each arm's MuJoCo bridge
uses its own `armN/joint_states` and `armN/cmd_joints`. The copy's internal `panda_copy_`
joint prefix is removed from the published names, so both arms report `panda_joint1` to
`panda_joint7`.

The Panda model is the one from `../zenoh-mujoco-panda-ground-plane`; see its README for the
inertias, actuators, and damping.

## Rebuilding the project

`.robotlens/` was created entirely by `scripts/build_project.py` over MCP: it creates the
project, imports both Pandas, places them, configures their Joint Control endpoints, and saves.
To rebuild, remove `.robotlens/` and run it from an empty RobotLens session (or with RobotLens
closed). The shared builder is `../tools/new_project.py`.

## Verification

On 2026-10-21 the project was built with `scripts/build_project.py`, and
`python3 scripts/quickstart.py run` passed three times in a row: both arms reached both poses,
every joint within 0.011 rad.
