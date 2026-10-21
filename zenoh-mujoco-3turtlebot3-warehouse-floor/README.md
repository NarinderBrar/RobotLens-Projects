# Zenoh three TurtleBot3s

Three TurtleBot3 Burgers in the warehouse, each in its own aisle. MuJoCo runs inside RobotLens,
and communication uses native Zenoh. Each robot runs its own external Zenoh navigation stack
under `robot1`, `robot2`, and `robot3`.

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
`stacks/run_fleet.sh` (three navigation stacks and map publishers), enables each robot's
simulation bridge, and sends the three goals at once: each robot drives from `x = -6.5` to
`x = -4` in its aisle. Every goal must succeed, and each robot must end within 0.25 m of its
goal, measured on `robotN/odom`. It then stops navigation, the stacks, the simulation, and
communication, in that order, and writes `evidence/quickstart/report.json`.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/zenoh-mujoco-3turtlebot3-warehouse-floor.xml` | Project with three robots, their LiDARs, and the warehouse |
| `robots/turtlebot3_burger.urdf`, `meshes/` | ROBOTIS TurtleBot3 Burger |
| `worlds/warehouse.xml` | Warehouse world |
| `stacks/navigation-robot{1,2,3}.json` | One navigation stack configuration per prefix |
| `stacks/run_fleet.sh` | Generates the map and owns all navigation stacks and map publishers |
| `scripts/generate_map.py`, `scripts/publish_map.py` | Occupancy grid from the world, published on `<prefix>/map` |
| `scripts/build_project.py` | Builds `.robotlens/` over MCP |

| Robot | Start | Prefix | LiDAR topic |
| --- | --- | --- | --- |
| `turtlebot3_burger` | `(-6.5, 0)` | `robot1` | `robot1/scan` |
| `turtlebot3_burger_copy` | `(-6.5, 4.5)` | `robot2` | `robot2/scan` |
| `turtlebot3_burger_copy_2` | `(-6.5, -4.5)` | `robot3` | `robot3/scan` |

## Rebuilding the project

`.robotlens/` was created entirely by `scripts/build_project.py` over MCP: it creates the
project, imports the world and robots, places them, attaches and enables each 2D LiDAR on
`base_scan`, configures navigation, and saves. To rebuild, remove `.robotlens/` and run it from
an empty RobotLens session (or with RobotLens closed). The shared builder is
`../tools/new_project.py`.

## Verification

On 2026-10-21 the project was built with `scripts/build_project.py`, and
`python3 scripts/quickstart.py run` passed three times in a row: all three goals succeeded
concurrently, and each robot drove 2.43 m and ended 0.10 m from its goal.
