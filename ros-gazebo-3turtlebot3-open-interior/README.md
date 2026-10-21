# Three TurtleBot3 robots in the open interior

Three TurtleBot3 Burgers (`tb1`, `tb2`, `tb3`) in the `open_interior` warehouse world, simulated by Gazebo and connected over ROS 2, each in its own namespace. `tb1` uses Nav2 (`nav2_config/nav2_namespaced_launch.py`), `tb2` uses EasyNavigation, and `tb3` is driven by keyboard control.

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
in `evidence/quickstart/robotlens.log`. The test checks that:

- the project opens
- communication starts and connects
- the simulation starts and reaches Running
- the robots are loaded: `tb1`, `tb2`, `tb3`
- RobotLens resolves the TF frames `tb1/base_link`, `tb2/base_link`, `tb3/base_link`
- a viewport screenshot is saved to `evidence/quickstart/quickstart.png`

`run` then launches Nav2 for `tb1` in `/tb1` (`nav2_config/nav2_namespaced_launch.py` with
`nav2_params_tb1.yaml`), starts `tb2`'s EasyNavigation, and sends both goals at once: `tb1` to
`(-1, 2.2, pi/2)` and `tb2` to `(-1, -2.2, -pi/2)`. Each goal must succeed, and each robot must end
within 0.3 m of its goal, measured from TF. `tb3` is not commanded. The report is
`evidence/quickstart/report.json`.

Gazebo publishes each robot's odometry in world coordinates, so the launch keeps
`map -> tb1/odom` at identity (`x:=0.0 y:=0.0`). A spawn offset there would be applied twice and
put Nav2's pose about 1.1 m away from the real robot.

It then stops navigation (`navigation.stop_servers` for Nav2 and EasyNavigation), the simulation,
and communication, in that order. `--keep-running` leaves the simulation and RobotLens running for a look.
The shared code is in `../tools/quickstart.py`.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/ros-gazebo-3turtlebot3-open-interior.xml` | RobotLens project |
| `worlds/open_interior.sdf` | Warehouse interior world (walls, blockers, shelves) |
| `turtlebot3/` | TurtleBot3 Burger models, Gazebo runner, launch files and Nav2 assets; see `turtlebot3/README.md` |
| `nav2_config/` | Nav2 launch, parameters and map used by RobotLens's managed Nav2 session |
| `scripts/quickstart.py` | Setup check, smoke test, and navigation scenario |

## Verification

On 2026-10-21, `python3 scripts/quickstart.py run` passed three times in a row. `tb1` ended
0.15 m and `tb2` 0.03 m from their goals each time, and no navigation process outlived the run.
