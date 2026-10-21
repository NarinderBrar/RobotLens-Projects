# ROS 2 Gazebo two Panda arms

Two Franka Pandas on the ground plane, 1.6 m apart, simulated by Gazebo and connected over ROS 2.

## Quick start

From this project folder:

```bash
python3 scripts/quickstart.py setup   # check prerequisites and prepare the project
python3 scripts/quickstart.py test    # smoke test in RobotLens
python3 scripts/quickstart.py run     # smoke test plus the arm scenario
python3 scripts/quickstart.py         # setup, then run
```

`test` uses the running RobotLens session if there is one. Otherwise it starts RobotLens from
`../../RobotLens` (build it first with `./build.sh`) and closes it at the end, keeping its log
in `evidence/quickstart/robotlens.log`. The shared code is in `../tools/quickstart.py`.

`run` opens the project, closes the splash screen, starts ROS 2 and Gazebo, then moves both arms
at once to the Panda ready pose, then turns `arm1`'s base to +1.2 rad and `arm2`'s to -1.2 rad
(`[±1.2, -0.3, 0.4, -1.8, 0.3, 1.8, 0]`). Commands go to each arm's Gazebo controller topics
`armN/<joint>/cmd_pos`. Every joint of each arm must end within 0.05 rad of its target and
holding, measured on ROS 2 `/armN/joint_states`. It then stops the simulation and communication
and writes `evidence/quickstart/report.json`.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/ros-gazebo-2panda-ground-plane.xml` | Project with both Pandas |
| `robots/panda_arm{1,2}.urdf`, `robots/meshes/` | One Gazebo Panda per arm, differing only in name and controller topics |
| `scripts/build_project.py` | Builds `.robotlens/` over MCP |

| Robot | Position | Controller topics | Joint states |
| --- | --- | --- | --- |
| `panda_arm1` | `(0, 0.8)` | `arm1/<joint>/cmd_pos` | `/arm1/joint_states` |
| `panda_arm2` | `(0, -0.8)` | `arm2/<joint>/cmd_pos` | `/arm2/joint_states` |

Each Panda has a Joint Control capability (`robot.joint_control.configure`) whose state endpoint
gives it its namespace, so RobotLens publishes each arm's joint states separately.

## The Gazebo Panda model

The URDF is the inertial Panda from `../zenoh-mujoco-panda-ground-plane` without its MuJoCo
`robotlens_sim` block, plus what Gazebo needs. RobotLens converts it with `gz sdf -p`, which
keeps standard URDF extensions:

- a `world` link and a fixed `world_fixed` joint to `panda_link0`, so the base does not fall
- a `<gazebo>` block with a `JointPositionController` per arm and finger joint, on Gazebo
  topics `armN/<joint>/cmd_pos`. PD gains match the MuJoCo servos (p 2400/2400/1800/1800/450/450/300),
  and output is limited to the Panda's rated torques (87 and 12 N m).

RobotLens adds the joint-state publisher and bridges it to ROS 2. Under gravity joint 4
settles about 0.011 rad short of its target.

## Rebuilding the project

`.robotlens/` was created entirely by `scripts/build_project.py` over MCP. To rebuild, remove
`.robotlens/` and run it from an empty RobotLens session (or with RobotLens closed). The shared
builder is `../tools/new_project.py`.

## Verification

On 2026-10-21 the project was built with `scripts/build_project.py`, and
`python3 scripts/quickstart.py run` passed three times in a row: both arms reached both poses,
every joint within 0.011 rad.
