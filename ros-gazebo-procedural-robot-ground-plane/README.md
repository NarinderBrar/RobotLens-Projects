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
python3 scripts/quickstart.py run     # smoke test plus the motion scenario
python3 scripts/quickstart.py         # setup, then run
```

`test` uses the running RobotLens session if there is one. Otherwise it starts RobotLens from
`../../RobotLens` (build it first with `./build.sh`) and closes it at the end, keeping its log
in `evidence/quickstart/robotlens.log`. The test checks that:

- the project opens
- communication starts and connects
- the simulation starts and reaches Running
- the robots are loaded: `Autonomous_mobile_robot`, `Mobile_manipulator`
- a viewport screenshot is saved to `evidence/quickstart/quickstart.png`

`run` then commands both robots through their Gazebo controllers and measures the result from
Gazebo's own poses (`gz model`):

- `Autonomous_mobile_robot`, the finished procedural robot, drives both wheels at 4 rad/s for 4 s
  and must move at least 0.3 m
- `Mobile_manipulator` moves `arm_base_joint` to 1.2 rad and `upper_arm_joint` to 0.6 rad; the
  `arm_base` link must yaw by at least 1.1 rad and `upper_arm` must pitch by at least 0.5 rad

It then stops navigation, the simulation, and communication, in that order. `--keep-running` leaves the simulation and RobotLens running for a look.
The shared code is in `../tools/quickstart.py`.

## Verification

On 2026-10-21, `python3 scripts/quickstart.py run` passed three times in a row: the procedural
robot drove 1.68 m, `arm_base` reached 1.20 rad of yaw and `upper_arm` 0.61 rad of pitch.
