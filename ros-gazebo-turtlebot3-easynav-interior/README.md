# TurtleBot3 EasyNavigation in the open interior

One TurtleBot3 Burger in the `open_interior` warehouse world, simulated by Gazebo and connected over ROS 2. Its navigation stack is EasyNavigation, configured by `easynav/file_urdf_turtlebot3_burger.params.yaml`.

## Quick start

From this project folder:

```bash
python3 scripts/quickstart.py setup   # check prerequisites and prepare the project
python3 scripts/quickstart.py test    # smoke test in RobotLens
python3 scripts/quickstart.py         # setup, then test
```

`test` uses the running RobotLens session if there is one. Otherwise it starts RobotLens from
`../../RobotLens` (build it first with `./build.sh`) and closes it at the end, keeping its log
in `evidence/quickstart-robotlens.log`. The test checks that:

- the project opens
- communication starts and connects
- the simulation starts and reaches Running
- the robots are loaded: `turtlebot3_burger`
- RobotLens resolves the TF frames `base_link`
- with `--navigation`, the robot's navigation servers start
- a viewport screenshot is saved to `evidence/quickstart.png`

It then stops navigation, the simulation, and communication, in that order. `--keep-running` leaves the simulation and RobotLens running for a look. `--navigation` also starts the navigation servers.
The shared code is in `../tools/quickstart.py`.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/ros-gazebo-turtlebot3-easynav-interior.xml` | RobotLens project |
| `worlds/open_interior.sdf` | Warehouse interior world (walls, blockers, shelves) |
| `turtlebot3/` | TurtleBot3 Burger models, Gazebo runner, launch files and Nav2 assets; see `turtlebot3/README.md` |
| `nav2_config/` | Nav2 launch, parameters and map used by RobotLens's managed Nav2 session |
| `easynav/` | EasyNavigation parameters (older versions are kept as `.bak-*`) |
| `scripts/quickstart.py` | Setup check and smoke test |
