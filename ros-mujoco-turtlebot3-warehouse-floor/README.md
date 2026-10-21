# ROS 2 TurtleBot3

A TurtleBot3 Burger in the warehouse. MuJoCo runs inside RobotLens, and communication uses
ROS 2. The robot navigates with Nav2 in the `/robot1` namespace.

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

`run` opens the project, closes the splash screen, starts ROS 2 and the simulation, enables the
robot's simulation bridge, starts Nav2 and waits until its lifecycle manager reports every node
active, then sends one goal from `(-6.5, 0)` to `(-3.5, 0, 0)`. The goal must succeed, and the
robot must end within 0.35 m of it, measured from `map -> robot1/base_footprint`. It then stops navigation,
the simulation, and communication, in that order, and writes `evidence/quickstart/report.json`.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/ros-mujoco-turtlebot3-warehouse-floor.xml` | Project with the robot, its LiDAR, and the warehouse |
| `robots/turtlebot3_burger.urdf`, `meshes/` | ROBOTIS TurtleBot3 Burger; MuJoCo drives `wheel_left_joint` and `wheel_right_joint` |
| `worlds/warehouse.xml` | Warehouse world |
| `nav2_config/` | Namespaced Nav2 launch, parameters, and static map |
| `scripts/generate_warehouse_map.py` | Regenerates the static map from the world |
| `scripts/build_project.py` | Builds `.robotlens/` over MCP |

The 2D LiDAR publishes `/robot1/scan` (360°, 0.12-12 m, ray-based).

Nav2 runs per robot in its own namespace, launched and managed by RobotLens from
`nav2_config/nav2_robot_launch.py` with `nav2_params.yaml` and the static
`warehouse_navigation_map.yaml` (0.05 m cells, from `scripts/generate_warehouse_map.py`). MuJoCo
publishes odometry in world coordinates, so `map -> robotN/odom` is identity. The launch
and parameters come from `../ros-mujoco-5mobile-manipulator-warehouse-fleet`; this copy of the
launch takes a `robot_radius` argument (default 0.65 m), and the project passes 0.12 m for the
Burger.

## Rebuilding the project

`.robotlens/` was created entirely by `scripts/build_project.py` over MCP: it creates the
project, imports the world and robots, places them, attaches and enables each 2D LiDAR 2 cm
above `base_scan`, configures managed Nav2, and saves. To rebuild, remove `.robotlens/` and run
it from an empty RobotLens session (or with RobotLens closed). The shared builder is
`../tools/new_project.py`.

## Verification

On 2026-10-21 the project was built with `scripts/build_project.py`, and
`python3 scripts/quickstart.py run` passed three times in a row, each from a cold RobotLens
start: the robot drove 2.84 m and ended 0.25 m from its goal. Nav2 stops as soon as the robot is inside its 0.25 m `xy_goal_tolerance`.
