# ROS 2 Gazebo drone escorting a four-wheel robot

A Parrot Bebop 2 escorts a four-wheel `Mobile_robot` across the Gazebo open interior, connected
over ROS 2. The robot navigates with Nav2 in `/robot1`; the drone flies above it.

## Quick start

From this project folder:

```bash
python3 scripts/quickstart.py setup   # check prerequisites and prepare the project
python3 scripts/quickstart.py test    # smoke test in RobotLens
python3 scripts/quickstart.py run     # smoke test plus the escort scenario
python3 scripts/quickstart.py         # setup, then run
```

`test` uses the running RobotLens session if there is one. Otherwise it starts RobotLens from
`../../RobotLens` (build it first with `./build.sh`) and closes it at the end, keeping its log
in `evidence/quickstart/robotlens.log`. The shared code is in `../tools/quickstart.py`.

`run`:

1. starts the robot's Nav2 and waits until its lifecycle manager reports every node active
2. arms the drone and climbs; it must be above 1.5 m
3. sends the robot from `(-12, -3)` to `(-6, -3, 0)`. While it drives, the script reads both
   Gazebo poses about once a second and steers the drone toward the point 2.5 m above the
   robot, with a proportional velocity command (gain 0.5, at most 0.6 m/s). It keeps
   steering for 12 s after the robot arrives so the drone settles over it
4. checks that the robot reached its goal within 0.35 m (TF) and that the drone is within
   1 m (horizontally) of the robot and still above 1.5 m, both from Gazebo's model poses
5. lands and disarms the drone; it must end below 0.3 m

The camera frames the route before the drive and both vehicles at the end.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/ros-gazebo-bebop2-mobile-robot-open-interior.xml` | Project with the robot, its LiDAR, the drone, and the world |
| `robots/Mobile_robot.urdf` | Four-wheel skid-steer base |
| `robots/bebop2.urdf`, `robots/meshes/` | Bebop 2 with the gz-sim multicopter plugins, namespace `drone` (see `../ros-gazebo-bebop2-open-field`) |
| `worlds/open_interior.sdf` | Gazebo world |
| `nav2_config/` | Namespaced Nav2 launch, parameters, and map (see `../ros-gazebo-mobile-robot-open-interior`) |
| `scripts/build_project.py` | Builds `.robotlens/` over MCP |

## Verification

On 2026-10-21 the project was built with `scripts/build_project.py`, and
`python3 scripts/quickstart.py run` passed four times in a row: the robot drove 5.77 m to within
0.23 m of its goal, and the drone ended 0.56-0.86 m from the robot at 2.5-2.8 m.
