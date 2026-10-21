# ROS 2 Gazebo Parrot Bebop 2

A Parrot Bebop 2 quadcopter on the ground plane, simulated by Gazebo and connected over ROS 2.

## Quick start

From this project folder:

```bash
python3 scripts/quickstart.py setup   # check prerequisites and prepare the project
python3 scripts/quickstart.py test    # smoke test in RobotLens
python3 scripts/quickstart.py run     # smoke test plus the flight scenario
python3 scripts/quickstart.py         # setup, then run
```

`test` uses the running RobotLens session if there is one. Otherwise it starts RobotLens from
`../../RobotLens` (build it first with `./build.sh`) and closes it at the end, keeping its log
in `evidence/quickstart/robotlens.log`. The shared code is in `../tools/quickstart.py`.

`run` opens the project, closes the splash screen, starts ROS 2 and Gazebo, then flies:

1. arm and climb at 0.6 m/s, then hover; the drone must be above 1.5 m
2. fly a square: +x, +y, -x, -y at 0.8 m/s for 2.5 s each; every leg must move at least 1.5 m
   and stay above 1.0 m
3. descend at 0.6 m/s and disarm; the drone must end below 0.3 m

It then stops the simulation and communication and writes `evidence/quickstart/report.json`.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/ros-gazebo-bebop2-open-field.xml` | Project with the drone |
| `robots/bebop2.urdf`, `robots/meshes/` | Bebop 2 with the multicopter plugins, namespace `drone` |
| `scripts/make_drone_urdf.py` | Regenerates the URDF |
| `scripts/build_project.py` | Builds `.robotlens/` over MCP |

## The drone model

`scripts/make_drone_urdf.py` writes the drone URDF from
`../robots_worlds/parrot_bebop_2/robots/parrot_bebop_2.urdf` (0.503 kg, box collision). It adds
the gz-sim multicopter stack in `<gazebo>` blocks, which RobotLens keeps when it converts the
URDF with `gz sdf -p`: four `MulticopterMotorModel` rotors and a `MulticopterVelocityControl`
that turns a body-frame twist on `/<namespace>/gazebo/command/twist` into rotor speeds. The
controller starts disarmed; `true` on `/<namespace>/enable` arms it. Constants match the Bebop
fixture's `gazebo_runner.py`.

Commands go through `../tools/gz_publish.py`, which sends a whole batch from one process. Every
check reads the drone's world pose from Gazebo with `gz model -p`. During each flight step the
viewport camera glides to keep the drones, and where they started, in view.

## Verification

On 2026-10-21 `python3 scripts/quickstart.py run` passed more than three times in a row: the
drone climbed to 2.2 m, flew a 1.7 m square back over its start point, and landed.
