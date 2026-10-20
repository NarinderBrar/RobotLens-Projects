# ROS 2 MuJoCo five-robot warehouse fleet

ROS 2 five-robot warehouse experiment using in-process MuJoCo. Earlier names of this project
were `ros2-multi-robot` and `ros2-mujoco-multi-robot`.

This project prepares the same five-robot warehouse experiment for a ROS 2 and Nav2 comparison with `../zenoh-mujoco-5mobile-manipulator-warehouse-fleet`. It uses the RobotLens in-process MuJoCo backend, one ROS 2 namespace per robot, and a five-instance Nav2 launch file.

## Quick start

From this project folder:

```bash
python3 scripts/quickstart.py setup   # check prerequisites and prepare the project
python3 scripts/quickstart.py test    # smoke test in RobotLens
python3 scripts/quickstart.py         # setup, then test
```

`test` uses the running RobotLens session if there is one. Otherwise it starts RobotLens from
`../../RobotLens` (build it first with `./build.sh`) and closes it at the end, keeping its log
in `evidence/quickstart/robotlens.log`. The test checks that:

- the project opens
- communication starts and connects
- `scripts/setup_fleet.sh` prepares the scene and starts the simulation
- the robots are loaded: `Mobile_manipulator`, `Mobile_manipulator_copy`, `…_copy_4`
- a viewport screenshot is saved to `evidence/quickstart/quickstart.png`

It then stops navigation, the simulation, and communication, in that order. `--keep-running` leaves the simulation and RobotLens running for a look.
The shared code is in `../tools/quickstart.py`.

## Contents

- [Requirements](#requirements)
- [Layout](#layout)
- [Fleet](#fleet)
- [Run guide](#run-guide)
- [Comparison](#comparison)
- [Known limits and blockers](#known-limits-and-blockers)

## Requirements

| Requirement | Purpose |
| --- | --- |
| RobotLens with ROS 2 support | Editor, MuJoCo backend, ROS 2 bridge, and Nav2 adapters |
| ROS 2 Jazzy Nav2 packages | Five namespaced Nav2 server sets |
| Python 3 and RobotLens automation client | `scripts/mcp.py` MCP wrapper |
| A running RobotLens session | Required before `setup_fleet.sh` and `send_goals.sh` |

## Layout

| Path | Purpose |
| --- | --- |
| `.robotlens/ros-mujoco-5mobile-manipulator-warehouse-fleet.xml` | ROS 2/MuJoCo workspace with the primary Nav2 capability |
| `worlds/warehouse.xml` | Same warehouse asset as the Zenoh project |
| `robots/Mobile_manipulator.urdf` | Same mobile manipulator asset as the Zenoh project |
| `nav2_config/nav2_robot_launch.py` | Starts one managed namespaced Nav2 server set and map-to-odom transform |
| `nav2_config/nav2_params.yaml` | Fleet Nav2 costmap, scan, controller, planner, and footprint settings |
| `nav2_config/warehouse_map.*` | Original coarse starter map, retained for reference |
| `nav2_config/warehouse_navigation_map.*` | Generated 0.05 m occupancy map used for navigation |
| `scripts/generate_warehouse_map.py` | Rasterizes static warehouse collision boxes into the navigation map |
| `scripts/mcp.py` | Project-local MCP helper |
| `scripts/setup_fleet.sh` | Imports, places, namespaces, configures LiDARs, starts comms/simulation, and enables bridges |
| `scripts/start_nav2_fleet.sh` | Starts each robot's managed Nav2 session through MCP |
| `scripts/send_goals.sh` | Sends five target-scoped goals and reports their MCP status |

## Fleet

| Robot | Scene name | ROS 2 namespace | Start `(x, y, yaw)` | Goal `(x, y)` |
| --- | --- | --- | --- | --- |
| robot1 | `Mobile_manipulator` | `/robot1` | `(-6.5, 0, 0°)` | `(6.5, 0)` |
| robot2 | `Mobile_manipulator_copy` | `/robot2` | `(-3.6, -4.5, 90°)` | `(-3.6, 4.5)` |
| robot3 | `Mobile_manipulator_copy_2` | `/robot3` | `(3.6, 4.5, -90°)` | `(3.6, -4.5)` |
| robot4 | `Mobile_manipulator_copy_3` | `/robot4` | `(-6.5, 4.5, 0°)` | `(6.5, 4.5)` |
| robot5 | `Mobile_manipulator_copy_4` | `/robot5` | `(6.5, -4.5, 180°)` | `(-6.5, -4.5)` |

The intended per-robot endpoints are `/robotN/cmd_vel`, `/robotN/odom`, `/robotN/scan`, `/robotN/joint_states`, and `/robotN/navigate_to_pose`. LiDAR settings are ray-based, range minimum 0.1 m, mount z 0.04 m, and zero rotation.

## Run guide

1. Launch RobotLens yourself and run:

   ```bash
   ./scripts/setup_fleet.sh
   ```

2. Setup reuses the five saved robots, importing only missing instances. It enables each 2D LiDAR through MCP and explicitly assigns `/robotN/scan`. Repeating setup does not import another four robots. These script behaviors have mocked regression coverage; live fleet navigation still needs verification.

   Setup also regenerates the navigation map from `worlds/warehouse.xml`. Its bounds follow the floor, and collision boxes within robot height remain occupied. The generator rejects unsupported or dynamic geometry instead of silently producing an incomplete map. The original 1.65 m/cell starter map is not used because it blocks valid warehouse corridors.

3. Start the five managed Nav2 sessions:

   ```bash
   ./scripts/start_nav2_fleet.sh
   ```

4. Send the fleet goals and inspect each status:

   ```bash
   ./scripts/send_goals.sh
   ```

   `robot.navigation.goal`, `route`, `cancel`, and `status` resolve the target owner's Nav2 adapter and namespace. `start_nav2_fleet.sh` uses `robot.navigation.start` once per robot, so each managed process is independently owned and can be stopped with `robot.navigation.stop` for that target.

5. Collect the same performance evidence as the Zenoh experiment with `robotlens.collect_performance_snapshot`, the top-down `view.set_camera` configuration, and `robotlens.capture_viewport_screenshot`.

The setup script gives `project.open` its required runtime-resolved absolute path; all stored assets and project settings use project-relative paths.

## Comparison

| Measure | Zenoh five-robot run | ROS 2 five-robot run |
| --- | --- | --- |
| FPS idle | 59.8 | To measure |
| FPS while driving | 59.9 | To measure |
| Time to reach all goals | About 30 s | To measure |
| CPU/RAM | Not recorded | To measure |
| Notes | 60 FPS cap; LiDAR avoidance disabled | Awaiting live validation |

## Measuring FPS

Use the same display cap for both projects before comparing results: Settings → Appearance → VSync, enabled for a 60 FPS cap or disabled for uncapped measurement. After the scene is idle for 10 seconds, call `robotlens.collect_performance_snapshot`; repeat while all five goals are executing. Record the reported FPS, frame-time statistics, cap setting, and whether the sample was idle or driving.

## Known limits and blockers

- The current saved project contains five robot instances; unsaved live edits still require saving in RobotLens.
- Reload can create duplicate 2D LiDAR sensors; setup removes duplicates.
- LiDAR enable/topic assignment is now part of MCP setup; inspect live sensor/TF readiness before sending goals.
- The generated map has regression checks for start/goal clearance and rack occupancy. Live robot1 planning and arrival have been observed, but the complete fleet and reliable lifecycle startup still need validation before benchmark numbers are comparable.
