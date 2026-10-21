# ROS 2 MuJoCo three Parrot Bebop 2s

Three Parrot Bebop 2 quadcopters on the ground plane, 2 m apart. MuJoCo runs inside RobotLens, and
communication uses ROS 2. Each drone has a Joint Control state endpoint `/droneN/joint_states`,
which gives it the namespace `droneN`, so it has its own `droneN/cmd_vel` and `droneN/odom`.

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

`run` enables the simulation bridge, then flies all three at once: climb (each above 1.5 m), fly
to separate hover points (`drone1` -x, `drone2` +y, `drone3` +x; each at least 1 m and above
1.0 m), and land (each below 0.3 m).

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/ros-mujoco-3drone-open-field.xml` | Project with the three drones |
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

Commands are streamed over ROS 2 for the length of each step, then set to zero; every check
reads the drone's `odom`. The camera glides to follow the drones during each step.

## Rebuilding the project

`.robotlens/` was created entirely by `scripts/build_project.py` over MCP. To rebuild, remove
`.robotlens/` and run it from an empty RobotLens session (or with RobotLens closed). The shared
builder is `../tools/new_project.py`.

## Verification

On 2026-10-21 `python3 scripts/quickstart.py run` passed three times in a row: all three climbed
to about 2 m, moved 1.4-1.5 m to separate hover points, and landed there.
