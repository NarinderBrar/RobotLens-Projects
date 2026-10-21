# ROS 2 Gazebo four-wheel mobile robot

A four-wheel skid-steer `Mobile_robot` in the Gazebo open interior, connected over ROS 2. The
robot navigates with Nav2 in the `/robot1` namespace.

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

`run` opens the project, closes the splash screen, starts ROS 2 and Gazebo, starts Nav2 and
waits until its lifecycle manager reports every node active, then sends one goal across the
lower-left room from `(-12, -3)` to `(-6, -3, 0)`. The goal must succeed, and the robot must end
within 0.35 m of it, measured from `map -> robot1/base_footprint`. It then stops navigation,
the simulation, and communication, in that order, and writes `evidence/quickstart/report.json`.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/ros-gazebo-mobile-robot-open-interior.xml` | Project with the robot, its LiDAR, and the world |
| `robots/Mobile_robot.urdf` | Four-wheel skid-steer base with a LiDAR mount |
| `worlds/open_interior.sdf` | Gazebo world with walls, blockers, and shelves |
| `nav2_config/` | Namespaced Nav2 launch, parameters, and the open interior map |
| `scripts/build_project.py` | Builds `.robotlens/` over MCP |

The 2D LiDAR publishes `/robot1/scan` (360°, 0.12-12 m, ray-based).

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

On 2026-10-21 the project was built with `scripts/build_project.py`, and
`python3 scripts/quickstart.py run` passed four times in a row, each from a cold RobotLens
start: the robot drove 5.76 m and ended 0.24 m from its goal. Gazebo's model pose matched TF
at `(-6.24, -3.00)`.
