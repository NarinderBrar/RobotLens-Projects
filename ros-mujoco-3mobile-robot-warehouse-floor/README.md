# ROS 2 three four-wheel mobile robots

Three four-wheel skid-steer `Mobile_robot`s in the warehouse, each in its own aisle. MuJoCo runs
inside RobotLens, and communication uses ROS 2. Each robot navigates with its own Nav2 in
`/robot1`, `/robot2`, and `/robot3`.

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

`run` opens the project, closes the splash screen, starts ROS 2 and the simulation, enables each
robot's simulation bridge, starts the three Nav2 stacks and waits until each lifecycle manager
reports every node active, then sends the three goals at once: each robot drives from
`x = -6.5` to `x = -3.5` in its aisle. Every goal must succeed, and each robot must end within
0.35 m of its goal, measured from `map -> robotN/base_footprint`. It then stops navigation,
the simulation, and communication, in that order, and writes `evidence/quickstart/report.json`.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/ros-mujoco-3mobile-robot-warehouse-floor.xml` | Project with three robots, their LiDARs, and the warehouse |
| `robots/Mobile_robot.urdf` | Mobile base with four velocity wheel actuators and a LiDAR mount |
| `worlds/warehouse.xml` | Warehouse world |
| `nav2_config/` | Namespaced Nav2 launch, parameters, and static map |
| `scripts/generate_warehouse_map.py` | Regenerates the static map from the world |
| `scripts/build_project.py` | Builds `.robotlens/` over MCP |

| Robot | Start | Namespace | LiDAR topic |
| --- | --- | --- | --- |
| `Mobile_robot` | `(-6.5, 0)` | `/robot1` | `/robot1/scan` |
| `Mobile_robot_copy` | `(-6.5, 4.5)` | `/robot2` | `/robot2/scan` |
| `Mobile_robot_copy_2` | `(-6.5, -4.5)` | `/robot3` | `/robot3/scan` |

Nav2 runs per robot in its own namespace, launched and managed by RobotLens from
`nav2_config/nav2_robot_launch.py` with `nav2_params.yaml` and the static
`warehouse_navigation_map.yaml` (0.05 m cells, from `scripts/generate_warehouse_map.py`). MuJoCo
publishes odometry in world coordinates, so `map -> robotN/odom` is identity. The launch
and parameters are the ones used by `../ros-mujoco-5mobile-manipulator-warehouse-fleet`, whose
mobile base and 0.65 m radius are the same.

## Rebuilding the project

`.robotlens/` was created entirely by `scripts/build_project.py` over MCP: it creates the
project, imports the world and robots, places them, attaches and enables each 2D LiDAR 4 cm
above `lidar_link`, configures managed Nav2, and saves. To rebuild, remove `.robotlens/` and run
it from an empty RobotLens session (or with RobotLens closed). The shared builder is
`../tools/new_project.py`.

## Verification

On 2026-10-21 the project was built with `scripts/build_project.py`, and
`python3 scripts/quickstart.py run` passed three times in a row, each from a cold RobotLens
start: all three goals succeeded concurrently, and each robot drove 2.77 m and ended
0.23 m from its goal.
