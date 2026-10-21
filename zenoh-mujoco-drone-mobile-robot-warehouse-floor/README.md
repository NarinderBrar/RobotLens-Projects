# Zenoh MuJoCo drone escorting a four-wheel robot

A Parrot Bebop 2 escorts a four-wheel `Mobile_robot` along a warehouse aisle. MuJoCo runs inside
RobotLens, and communication uses native Zenoh. The robot navigates with the external Zenoh navigation stack under `robot1`; the drone, namespaced
`drone` by its Joint Control state endpoint, flies above it.

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

`run`:

1. starts `stacks/run_navigation.sh` and attaches to the robot's navigation
2. climbs the drone to about 3 m, above the racks; it must be above 2 m
3. drives the robot from `(-6.5, 0)` to `(-3.5, 0, 0)` while `../tools/zenoh_escort.py` holds the
   drone over it: at 10 Hz it reads both odometries and steers the drone toward the point 3 m above
   the robot, then keeps steering for 8 s after the robot arrives
4. checks the robot reached its goal within 0.35 m and the drone is within 1 m (horizontally) of
   the robot and above 2 m, both from odometry
5. flies the drone back along the aisle, clear of the robot, and lands it (it must end below
   0.3 m; directly above the robot it would land on top of it)

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/zenoh-mujoco-drone-mobile-robot-warehouse-floor.xml` | Project with the robot, its LiDAR, the drone, and the warehouse |
| `robots/Mobile_robot.urdf` | Four-wheel skid-steer base |
| `robots/bebop2.urdf`, `robots/meshes/` | Bebop 2 with the multirotor hints |
| `worlds/warehouse.xml` | Warehouse world |
| `stacks/`, `maps/`, `scripts/generate_map.py`, `scripts/publish_map.py` | Zenoh navigation stack, map, and publisher (from `../zenoh-mujoco-mobile-robot-warehouse-floor`) |
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

Commands are streamed over Zenoh (an unmatched command is retried once and the retry is recorded) for the length of each step, then set to zero; every check
reads the drone's `odom`. The camera glides to follow the drones during each step.

## Rebuilding the project

`.robotlens/` was created entirely by `scripts/build_project.py` over MCP. To rebuild, remove
`.robotlens/` and run it from an empty RobotLens session (or with RobotLens closed). The shared
builder is `../tools/new_project.py`.

## Verification

On 2026-10-21 `python3 scripts/quickstart.py run` passed three times in a row: the robot drove
about 2.8 m to its goal, the drone ended directly over it (0.00 m) at 3 m, then moved clear and
landed.
