# ROS 2 Gazebo Panda arm

A Franka Panda on the ground plane, simulated by Gazebo and connected over ROS 2.

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

`run` opens the project, closes the splash screen, starts ROS 2 and Gazebo, then commands the
Panda ready pose `[0, -0.785, 0, -2.356, 0, 1.571, 0.785]`, then a pose that turns the base
1.2 rad `[1.2, -0.3, 0.4, -1.8, 0.3, 1.8, 0]`, by publishing each joint's position on its Gazebo
controller topic `panda/<joint>/cmd_pos`. After each, every joint must be within 0.05 rad of its
target and holding, measured on ROS 2 `/joint_states`. Screenshots of both poses are saved in
`evidence/quickstart/`. It then stops the simulation and communication and writes
`evidence/quickstart/report.json`.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/ros-gazebo-panda-ground-plane.xml` | Project with the Panda |
| `robots/panda.urdf`, `robots/meshes/` | Panda model prepared for Gazebo |
| `scripts/build_project.py` | Builds `.robotlens/` over MCP |

## The Gazebo Panda model

The URDF is the inertial Panda from `../zenoh-mujoco-panda-ground-plane` without its MuJoCo
`robotlens_sim` block, plus what Gazebo needs. RobotLens converts it with `gz sdf -p`, which
keeps standard URDF extensions:

- a `world` link and a fixed `world_fixed` joint to `panda_link0`, so the base does not fall
- a `<gazebo>` block with a `JointPositionController` per arm and finger joint, on Gazebo
  topics `panda/<joint>/cmd_pos`. PD gains match the MuJoCo servos (p 2400/2400/1800/1800/450/450/300),
  and output is limited to the Panda's rated torques (87 and 12 N m).

RobotLens adds the joint-state publisher and bridges it to ROS 2. Under gravity joint 4
settles about 0.011 rad short of its target.

## Rebuilding the project

`.robotlens/` was created entirely by `scripts/build_project.py` over MCP. To rebuild, remove
`.robotlens/` and run it from an empty RobotLens session (or with RobotLens closed). The shared
builder is `../tools/new_project.py`.

## Verification

On 2026-10-21 the project was built with `scripts/build_project.py`, and
`python3 scripts/quickstart.py run` passed three times in a row; the RobotLens log shows the
poses coming from Gazebo.
