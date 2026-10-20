# Gazebo robot finish test

RobotLens project: `.robotlens/ros-gazebo-procedural-robot-ground-plane.xml`. It uses ROS 2 communication and the
Gazebo simulator with RobotLens's default ground-plane world.

It checks the built-in procedural robot workflow with Gazebo: add a robot from the Robots panel
(`robot.add`), change its settings (`robot.configure`), and press **Finish robot** (`robot.finish`).
Finishing writes an SDF 1.9 model to `robots/<name>.sdf`, imports it as a normal robot, and the
simulator setup merges it into the world. See ADR 0057 in RobotLens-Docs.

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
- the robots are loaded: `Mobile_manipulator`
- a viewport screenshot is saved to `evidence/quickstart.png`

It then stops navigation, the simulation, and communication, in that order. `--keep-running` leaves the simulation and RobotLens running for a look.
The shared code is in `../tools/quickstart.py`.
