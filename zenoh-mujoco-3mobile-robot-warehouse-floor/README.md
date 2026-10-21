# Zenoh Three Mobile Robots

Three four-wheel skid-steer mobile robots in the warehouse, each in its own
aisle. MuJoCo runs inside RobotLens, and communication uses native Zenoh. Each
robot runs its own external Zenoh navigation stack under the prefixes `robot1`,
`robot2`, and `robot3`. No external model assets are required.

## Quick start

From this project folder:

```bash
python3 scripts/quickstart.py setup   # check prerequisites and prepare the project
python3 scripts/quickstart.py test    # smoke test plus the navigation scenario
python3 scripts/quickstart.py run     # test and write evidence/quickstart/report.json
python3 scripts/quickstart.py         # setup, then run
```

`test` uses the running RobotLens session if there is one. Otherwise it starts RobotLens from
`../../RobotLens` (build it first with `./build.sh`) and closes it at the end, keeping its log
in `evidence/quickstart/robotlens.log`. The script:

1. opens the project and closes the splash screen
2. starts Zenoh and the MuJoCo simulation
3. checks that the three robots are loaded
4. starts `stacks/run_fleet.sh` (three navigation stacks and map publishers)
5. enables each robot's simulation bridge and waits for its navigation stack
6. sends the three goals at once: each robot drives 4 m east in its aisle to
   `(-2.5, 0)`, `(-2.5, 4.5)`, and `(-2.5, -4.5)`
7. checks that every goal succeeds and, from independent `robotN/odom`
   readings, that each robot ends within 0.25 m of its goal
8. saves screenshots before and after the goals in `evidence/quickstart/`

It then stops navigation, the stacks, the simulation, and communication, in that order.
`--keep-running` leaves the simulation and RobotLens running for a look. The shared code is in
`../tools/quickstart.py`.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/zenoh-mujoco-3mobile-robot-warehouse-floor.xml` | Saved project with three robots, their LiDARs, and the warehouse |
| `robots/Mobile_robot.urdf` | Mobile base, four velocity actuators, and LiDAR mount |
| `worlds/warehouse.xml` | Warehouse world |
| `stacks/navigation-robot{1,2,3}.json` | One navigation stack configuration per robot prefix |
| `stacks/run_fleet.sh` | Generates the map and owns all navigation stacks and map publishers |
| `scripts/generate_map.py` | Converts static warehouse collision boxes into an occupancy grid |
| `scripts/publish_map.py` | Publishes the global grid on `<prefix>/map` |
| `maps/warehouse.json` | Generated full-world map in the stack's JSON contract |

| Robot | Start | Prefix | LiDAR |
| --- | --- | --- | --- |
| `Mobile_robot` | `(-6.5, 0)` | `robot1` | `2D LiDAR 1` on `robot1/scan` |
| `Mobile_robot_copy` | `(-6.5, 4.5)` | `robot2` | `2D LiDAR 2` on `robot2/scan` |
| `Mobile_robot_copy_2` | `(-6.5, -4.5)` | `robot3` | `2D LiDAR 3` on `robot3/scan` |

All robots face positive X. Each 2D LiDAR is mounted 4 cm above its robot's
`lidar_link`, scans 360° from 0.1 to 12 m with ray-based simulation, and is
enabled in the saved project. The project was built entirely over MCP,
including the LiDAR topics.

## Run by hand

1. Open `.robotlens/zenoh-mujoco-3mobile-robot-warehouse-floor.xml` in a RobotLens build with
   Zenoh and MuJoCo support.
2. Start the Zenoh connection, then set up and play the simulation. Enable each
   robot's simulation bridge state publishing and command acceptance; bridge
   switches must be enabled each session.
3. From this project folder, run `./stacks/run_fleet.sh`. It finds
   `zms_navigation` through `$ZMS_BIN`, then `PATH`, then the workspace's
   `Zenoh-motion-stacks/build/system` or `motion-stacks/build/system`. The map
   publishers need Python with `eclipse-zenoh`: `$ZMS_PYTHON`, then `python3`,
   then the workspace automation virtual environment. Ctrl+C stops everything;
   if one component exits, the launcher stops the others.
4. Select a robot and pick a viewport goal. Run only one stack per prefix on the
   connected Zenoh network; the single-robot warehouse project also uses `robot1`.
5. Stop the simulation and the launcher when finished.

The saved navigation mode is **AttachOnly** for all three robots.

## Verification

On 2026-10-21, `python3 scripts/quickstart.py run` passed three times in a row
against a live RobotLens session. In each run all three goals succeeded
concurrently in about 17 s, and each robot moved 3.86 m and ended 0.14 m from
its goal, measured on `robotN/odom`.
