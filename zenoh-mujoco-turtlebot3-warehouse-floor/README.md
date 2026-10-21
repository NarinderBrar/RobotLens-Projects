# Zenoh TurtleBot3

A TurtleBot3 Burger in the warehouse. MuJoCo runs inside RobotLens, and communication uses
native Zenoh. The robot runs the external Zenoh navigation stack under the prefix `robot1`.

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

`run` opens the project, closes the splash screen, starts Zenoh and the simulation, starts
`stacks/run_navigation.sh`, enables the robot's simulation bridge, and sends one goal from
`(-6.5, 0)` to `(-4, 0)`. The goal must succeed, and the robot must end within 0.25 m of it,
measured on `robot1/odom`. It then stops navigation, the stack, the simulation, and
communication, in that order, and writes `evidence/quickstart/report.json`.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/zenoh-mujoco-turtlebot3-warehouse-floor.xml` | Project with the robot, its LiDAR, and the warehouse |
| `robots/turtlebot3_burger.urdf`, `meshes/` | ROBOTIS TurtleBot3 Burger; MuJoCo drives `wheel_left_joint` and `wheel_right_joint` |
| `worlds/warehouse.xml` | Warehouse world |
| `stacks/navigation.json` | Navigation stack configuration sized for the Burger |
| `stacks/run_navigation.sh` | Generates the map and owns navigation plus the map publisher |
| `scripts/generate_map.py`, `scripts/publish_map.py` | Occupancy grid from the world, published on `robot1/map` |
| `scripts/build_project.py` | Builds `.robotlens/` over MCP |

The 2D LiDAR sits 2 cm above `base_scan`, scans 360° from 0.12 to 12 m with ray-based
simulation, and publishes `robot1/scan`. The stack uses a 0.12 m footprint, 0.2 m/s and
1.0 rad/s limits, the LiDAR's -0.032 m offset, and ignores returns within 0.15 m.

## Rebuilding the project

`.robotlens/` was created entirely by `scripts/build_project.py` over MCP: it creates the
project, imports the world and robots, places them, attaches and enables each 2D LiDAR on
`base_scan`, configures navigation, and saves. To rebuild, remove `.robotlens/` and run it from
an empty RobotLens session (or with RobotLens closed). The shared builder is
`../tools/new_project.py`.

## Verification

On 2026-10-21 the project was built with `scripts/build_project.py`, and
`python3 scripts/quickstart.py run` passed three times in a row: the robot drove 2.43 m and
ended 0.10 m from its goal each time.
