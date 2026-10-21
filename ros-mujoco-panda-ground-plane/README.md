# ROS 2 Panda arm

A Franka Panda on the ground plane. MuJoCo runs inside RobotLens, and communication uses ROS 2.
The arm is commanded through the MuJoCo bridge's joint command topic.

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

`run` opens the project, closes the splash screen, starts ROS 2 and the simulation, enables the
simulation bridge, then publishes two `sensor_msgs/JointState` commands on `/cmd_joints`: the
Panda ready pose `[0, -0.785, 0, -2.356, 0, 1.571, 0.785]`, then a pose that turns the base
1.2 rad `[1.2, -0.3, 0.4, -1.8, 0.3, 1.8, 0]`. After each, every joint must be within 0.05 rad
of its target and holding, measured on `/joint_states`. Screenshots of both poses are saved in
`evidence/quickstart/`. It then stops the simulation and communication and writes
`evidence/quickstart/report.json`.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/ros-mujoco-panda-ground-plane.xml` | Project with the Panda |
| `robots/panda.urdf`, `robots/meshes/` | Panda model prepared for MuJoCo |
| `scripts/build_project.py` | Builds `.robotlens/` over MCP |

## The Panda model

`robots/panda.urdf` is the Franka Panda with relative mesh paths (`meshes/visual`,
`meshes/collision`) and the link masses and inertias from
`../../RobotLens-Automation/Zenoh-Mujoco-Manipulation`. The copy in `../robots_worlds/panda` has
no inertia, which MuJoCo rejects ("mass and inertia of moving bodies must be larger than
mjMINVAL"). For simulation the project adds:

- a `robotlens_sim` block with a fixed base and position actuators on the seven arm joints
  (gain 800/800/600/600/150/150/100, effort 87 N m on joints 1-4 and 12 N m on 5-7, the Panda's
  rated torques) and on both fingers
- joint damping (12 to 2) so the position servos settle without ringing
- the Panda's real velocity limits (2.175 and 2.61 rad/s) instead of 100 rad/s

Joint 4 settles about 0.034 rad short of its target under gravity with these gains.

## Rebuilding the project

`.robotlens/` was created entirely by `scripts/build_project.py` over MCP: it creates the
project, imports the Panda, and saves. To rebuild, remove `.robotlens/` and run it from an
empty RobotLens session (or with RobotLens closed). The shared builder is
`../tools/new_project.py`.

## Verification

On 2026-10-21 the project was built with `scripts/build_project.py`, and
`python3 scripts/quickstart.py run` passed three times in a row. It needs the RobotLens fix
that gives a fixed-base robot its joints on the MuJoCo bridge.
