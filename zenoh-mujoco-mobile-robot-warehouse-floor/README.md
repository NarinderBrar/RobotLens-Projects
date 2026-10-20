# Zenoh Single Mobile Robot

A single four-wheel skid-steer mobile robot in the warehouse from
`../zenoh-mujoco-5mobile-manipulator-warehouse-fleet`. MuJoCo runs inside RobotLens, and communication uses
native Zenoh. The mobile base is derived from the reference robot with its
arm, gripper, camera, and IMU removed. No external model assets are required.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/zenoh-mujoco-mobile-robot-warehouse-floor.xml` | Saved project with one robot and the warehouse |
| `robots/Mobile_robot.urdf` | Mobile base, four velocity actuators, and LiDAR mount |
| `worlds/warehouse.xml` | Warehouse snapshot copied from the reference project |
| `stacks/navigation.json` | Single external navigation stack configuration |
| `stacks/run_navigation.sh` | Generates the map and owns navigation plus the map publisher |
| `scripts/generate_map.py` | Converts static warehouse collision boxes into an occupancy grid |
| `scripts/publish_map.py` | Publishes the global grid on native Zenoh |
| `maps/warehouse.json` | Generated full-world map in the stack's JSON contract |

The robot starts at `(-6.5, 0, 0)`, facing along positive X. Its enabled 2D
LiDAR uses ray-based simulation and publishes `robot1/scan` at 10 Hz. Navigation
uses `robot1/cmd_vel`, `robot1/odom`, and `robot1/navigation/*`. The stack keeps
the reference robot's conservative speed, footprint, and avoidance settings.

## Run

1. Open `.robotlens/zenoh-mujoco-mobile-robot-warehouse-floor.xml` in a RobotLens build with
   Zenoh and MuJoCo support. Confirm the scene contains `Mobile_robot` and
   `warehouse`.
2. Start the Zenoh connection and use the Simulation panel to set up and play
   the scene. Enable the robot's simulation bridge state publishing and
   command acceptance. Bridge switches must be enabled each session.
3. From this project folder, run:

   ```bash
   ./stacks/run_navigation.sh
   ```

   The launcher checks `$ZMS_BIN`, then `PATH`, then the workspace's
   `Zenoh-motion-stacks/build/system` and `motion-stacks/build/system` folders.
   It validates the configuration and regenerates the global map before starting
   navigation and its map publisher. Stop both with Ctrl+C. If either process
   exits unexpectedly, the launcher stops the other one too.

   The publisher needs Python with `eclipse-zenoh`. The launcher uses
   `$ZMS_PYTHON`, then `python3`, with the existing workspace automation virtual
   environment as a fallback. For a separate environment, set
   `ZMS_PYTHON=/path/to/venv/bin/python`.
4. Select `Mobile_robot` and confirm its navigation stack is attached and
   available. Pick a viewport goal in the clear central aisle, for example
   `(6.5, 0)`. Check odometry, LiDAR, and the reported navigation result.
5. Stop simulation and the navigation process when finished.

The saved navigation mode is **AttachOnly**; start the external stack with the
launcher above. Run only one stack using the `robot1` prefix on the connected
Zenoh network; the reference fleet also uses that prefix.

## Global map

The launcher generates a 334 × 256 grid at 0.05 m per cell, covering the full
16.7 × 12.8 m warehouse from origin `(-8.35, -6.4)`. Collision geometry within
the mobile robot's 0.5 m clearance height becomes occupied; overhead geometry
is excluded. Cells are conservative at box edges, and robot-radius inflation
is applied by the navigation stack. Unsupported geometry and dynamic bodies
are rejected. Regenerate by restarting the launcher after editing the world.

The publisher repeats `robot1/map` every second so navigation and RobotLens
can subscribe later or reconnect. Confirm `map_available: true` before sending
goals. The simulation's world-frame odometry aligns with this map; particle
localization remains disabled.

Select `robot1/map` and enable its **Map** visualizer to see the global occupancy
map. The saved project enables that display automatically. During navigation,
`robot1/navigation/costmap` shows the whole map with inflated obstacles and
remembered LiDAR hits. This display is downsampled to 167 × 128 cells at 0.1 m;
planning still uses the original 5 cm map. Obstacle avoidance keeps its local
planning window. RobotLens hides navigation overlays when the goal finishes;
the `robot1/map` display stays available.

The stack must include the periodic costmap-delivery fix from
`Zenoh-motion-stacks` commit `5c8943e` on branch `fix/global-costmap-delivery`. It republishes the
current costmap at the configured rate even when cells are unchanged, so a
late visualizer does not retain an old grid. The source change was built and
verified during this project setup.

## Verification

Project XML, robot links and joints, relative asset paths, navigation JSON, and
shell syntax were checked statically.

Live MCP verification on 2026-10-20 used the existing RobotLens session. Both
central-aisle navigation goals succeeded. The 6 m rolling costmap (100 × 100
cells at 0.06 m) translated and rotated with the robot, with LiDAR obstacle
regions visible and the Map visualizer reporting `Ok`. See the
[recorded observations](evidence/costmap-refresh.json),
[eastbound screenshot](evidence/costmap-eastbound.png), and
[westbound screenshot](evidence/costmap-westbound.png). The test navigation
process, simulation, and connection were stopped; the original robot pose and
camera were restored. The desktop was left open and the project was not saved.
That initial acceptance did not include an agent build or unit-test run.

Global-map acceptance on 2026-10-20 subsequently passed the rack-avoidance goal
`(-5.37, 4.5, pi/2)` and the return goal `(-6.5, 0, 0)`. The rack-avoidance goal
passed again after rebuilding the stack with periodic costmap delivery. The
direct line intersects occupied cells, while the captured 24-pose planned route
has no occupied-cell intersections at 2.5 cm sample spacing. Both raw global
map and global costmap visualizers reported `Ok`. This is static-scene global
mapping from project geometry, not online map reconstruction.

The map generator's four tests, the stack's visualization tests, and the new
late-subscriber integration regression passed. See
[global-map observations](evidence/global-map-delivery.json),
[global-map screenshot](evidence/global-map.png), and
[global-costmap screenshot](evidence/global-costmap.png). All test-owned
processes were stopped; the original robot pose was restored and RobotLens
was left open.

Run the project geometry tests with:

```bash
python3 -m unittest discover -s scripts -p 'test_*.py' -v
```
