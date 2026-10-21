# ROS 2 Gazebo three four-wheel mobile robots

Three four-wheel skid-steer `Mobile_robot`s in the Gazebo open interior, connected over ROS 2.
Each robot navigates with its own Nav2 in `/robot1`, `/robot2`, and `/robot3`.

## Quick start

From this project folder:

```bash
python3 scripts/quickstart.py setup   # check prerequisites and prepare the project
python3 scripts/quickstart.py test    # smoke test in RobotLens
python3 scripts/quickstart.py run     # smoke test plus the navigation scenario
python3 scripts/quickstart.py         # setup, then run
```

`test` uses the running RobotLens session if there is one. Otherwise it starts RobotLens from
`../../RobotLens` (build it first with `./build.sh`) and closes it at the end, keeping its log
in `evidence/quickstart/robotlens.log`. The shared code is in `../tools/quickstart.py`.

`run` opens the project, closes the splash screen, starts ROS 2 and Gazebo, starts the three
Nav2 stacks and waits until each lifecycle manager reports every node active, then sends the
three goals at once: each robot drives 6 m east across the lower-left room. Every goal must
succeed, and each robot must end within 0.35 m of its goal, measured from
`map -> robotN/base_footprint`. It then stops navigation, the simulation, and communication, in
that order, and writes `evidence/quickstart/report.json`.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/ros-gazebo-3mobile-robot-open-interior.xml` | Project with three robots, their LiDARs, and the world |
| `robots/Mobile_robot.urdf` | Four-wheel skid-steer base with a LiDAR mount |
| `worlds/open_interior.sdf` | Gazebo world with walls, blockers, and shelves |
| `nav2_config/` | Namespaced Nav2 launch, parameters, and the open interior map |
| `scripts/build_project.py` | Builds `.robotlens/` over MCP |

| Robot | Start | Goal | Namespace |
| --- | --- | --- | --- |
| `Mobile_robot` | `(-12, -2.5)` | `(-6, -2.5)` | `/robot1` |
| `Mobile_robot_copy` | `(-12, -4.5)` | `(-6, -4.5)` | `/robot2` |
| `Mobile_robot_copy_2` | `(-12, -6.75)` | `(-6, -6.75)` | `/robot3` |

## How it runs

Gazebo runs the `open_interior` world: four 13 x 11.5 m rooms joined by doorways. When the
simulation starts, RobotLens adds a DiffDrive plugin to each robot on `/robotN/cmd_vel` and an
odometry publisher that reports world poses. So `map -> robotN/odom` is identity, and TF
matches Gazebo's own model pose (checked with `gz model -p`).

Nav2 runs per robot in its own namespace, launched and managed by RobotLens from
`nav2_config/nav2_robot_launch.py` with `nav2_params.yaml` and the static `nav2_map.yaml` of the
open interior. The launch takes `robot_radius`; these robots pass 0.5 m.

Starting several Nav2 stacks at once occasionally loses a lifecycle `change_state` reply in
Fast DDS. Nav2's lifecycle manager then waits forever ("failed to send response ... (timeout)"
in `controller_server`'s log, and RobotLens reports "timed out waiting for Nav2 action
servers"). `run` therefore treats a stack that is not active within 60 s as stuck, restarts it
up to three times, and records any restart in the report.

## Rebuilding the project

`.robotlens/` was created entirely by `scripts/build_project.py` over MCP: it creates the
project, imports the world and robots, places them, attaches and enables each 2D LiDAR 4 cm
above `lidar_link`, configures managed Nav2, and saves. To rebuild, remove `.robotlens/` and run
it from an empty RobotLens session (or with RobotLens closed). The shared builder is
`../tools/new_project.py`.

## Verification

On 2026-10-21 the project was built with `scripts/build_project.py`. Before the restart was
added, 4 of 6 cold runs passed and 2 stalled on the lost lifecycle reply described above. With
it, `python3 scripts/quickstart.py run` passed five times in a row: all three goals succeeded
concurrently, and each robot drove 5.77 m and ended 0.23 m from its goal. The race did not
occur in those five runs, so the restart path has not yet been exercised live.
