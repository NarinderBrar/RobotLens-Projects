# ROS 2 Gazebo three Parrot Bebop 2s

Three Parrot Bebop 2 quadcopters on the ground plane, 2 m apart, simulated by Gazebo and connected
over ROS 2. Each has its own namespace (`drone1`, `drone2`, `drone3`).

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

`run` opens the project, closes the splash screen, starts ROS 2 and Gazebo, then flies all three
at once:

1. arm and climb; every drone must be above 1.5 m
2. each flies to its own hover point (`drone1` -x, `drone2` +y, `drone3` +x); each must move at
   least 1 m and stay above 1.0 m
3. descend and disarm; every drone must end below 0.3 m

It then stops the simulation and communication and writes `evidence/quickstart/report.json`.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/ros-gazebo-3bebop2-open-field.xml` | Project with the three drones |
| `robots/bebop2_{1,2,3}.urdf`, `robots/meshes/` | One Bebop 2 per namespace |
| `scripts/make_drone_urdf.py` | Regenerates the URDFs |
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

On 2026-10-21 `python3 scripts/quickstart.py run` passed three times in a row after batch
publishing was added: all three climbed to 2.6 m, moved 1.6 m to separate hover points
(-3.6, 0), (0, 1.6), and (3.6, 0), and landed there. Before that, with one `gz topic` call per
message, the commands landed about a second apart and the drones overshot to 6-9 m.
