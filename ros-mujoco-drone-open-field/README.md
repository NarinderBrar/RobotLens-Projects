# ROS 2 MuJoCo Parrot Bebop 2

A Parrot Bebop 2 quadcopter on the ground plane. MuJoCo runs inside RobotLens, and communication
uses ROS 2.

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

`run` enables the simulation bridge, then flies:

1. climb at 0.6 m/s; the drone must be above 1.5 m
2. a square: +x, +y, -x, -y at 0.8 m/s for 2.5 s each; every leg must move at least 1.5 m and
   stay above 1.0 m
3. descend at 0.6 m/s; the drone must end below 0.3 m

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/ros-mujoco-drone-open-field.xml` | Project with the drone |
| `robots/bebop2.urdf`, `robots/meshes/` | Bebop 2 with the multirotor hints |
| `scripts/make_drone_urdf.py` | Regenerates the URDF |
| `scripts/build_project.py` | Builds `.robotlens/` over MCP |

## How the drone flies

MuJoCo runs inside RobotLens. `robots/bebop2.urdf` is the Parrot Bebop 2 with a `robotlens_sim`
block written by `scripts/make_drone_urdf.py`: a floating base and four rotors (thrust coefficient
2.5e-6, moment coefficient 0.004, 1200 rad/s, the Gazebo Bebop fixture's constants). RobotLens
adds one thrust actuator per rotor and flies the drone with its built-in multirotor controller
(ADR 0061): a body-frame velocity and yaw-rate command on `cmd_vel` becomes a tilt and heading, and
then per-rotor thrust. The drone arms on its first climb command and disarms after resting on the
ground for 1 s. Odometry on `odom` carries altitude, roll, pitch, and climb rate.

The generator also gives `base_footprint` a negligible inertial. Without one, MuJoCo merges the
fixed `base_link` into it and computes the body's mass from the collision box (9.4 kg instead of
0.503 kg), and the drone cannot lift off.

Commands are streamed over ROS 2 (`/cmd_vel`, `/odom`) for the length of each step, then set to zero; every check
reads the drone's `odom`. The camera glides to follow the drones during each step.

## Rebuilding the project

`.robotlens/` was created entirely by `scripts/build_project.py` over MCP. To rebuild, remove
`.robotlens/` and run it from an empty RobotLens session (or with RobotLens closed). The shared
builder is `../tools/new_project.py`.

## Verification

On 2026-10-21 `python3 scripts/quickstart.py run` passed three times in a row: the drone climbed to
about 2 m, flew a 1.9 m square back to within a few centimetres of its start with altitude held
within 1 cm, and landed.
