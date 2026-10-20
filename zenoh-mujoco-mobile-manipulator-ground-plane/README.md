# Motion stacks mobile manipulator

RobotLens project `.robotlens/zenoh-mujoco-mobile-manipulator-ground-plane.xml` (Zenoh, in-process MuJoCo). It tests the standalone `motion-stacks` navigation and manipulation processes against a RobotLens-simulated robot over native Zenoh.

## Quick start

From this project folder:

```bash
python3 scripts/quickstart.py setup   # check prerequisites and prepare the project
python3 scripts/quickstart.py test    # smoke test in RobotLens
python3 scripts/quickstart.py         # setup, then test
```

The test does not start the motion stacks; use the steps below for that.

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

## Contents

| Path | Purpose |
| --- | --- |
| `extensions/robot-gallery` | Robot builder package, copied from `../zenoh-mujoco-procedural-robot-generated-atrium` |
| `robots/Mobile_manipulator.urdf` | Generated `mobile_manipulator` robot (seed 20), finished to URDF: 4 wheels, 6-joint arm, 2 fingers |
| `stacks/navigation.json`, `stacks/manipulation.json` | Stack configurations; empty `robot_prefix` matches RobotLens's unprefixed keys |
| `scripts/mcp.py` | Runs a RobotLens MCP command or tool against the newest live session |
| `scripts/fk.py` | Forward kinematics from the URDF: prints the tool pose for given joint positions, for building pose targets |
| `scripts/run_stacks.sh` | Validates both configurations, then runs both stacks; logs go to `evidence/logs/` |
| `tools/zenoh_probe` | Lists live Zenoh keys (`zenoh_probe <seconds> [key]`), prints every sample with a timestamp (`zenoh_probe watch <seconds> <key>`), or publishes (`zenoh_probe put <key> <json> <seconds> <hz>`) |
| `evidence/` | Screenshots and logs from the last run |

Build the probe with `cmake -S tools/zenoh_probe -B tools/zenoh_probe/build -G Ninja && cmake --build tools/zenoh_probe/build`. Build `Zenoh-motion-stacks` with its `system` preset; `run_stacks.sh` looks for it at `../../Zenoh-motion-stacks/build/system` unless `ZMS_BIN` is set.

## Running the test

1. Open the project in RobotLens, start communication, and play the simulation.
2. Select the robot, add the Joint Control capability, and tick **Publish states** and **Accept commands** in its Simulation Bridge section. Zenoh Joint Control needs the RobotLens change from 2026-10-14.
3. Check `tools/zenoh_probe/build/zenoh_probe 2` lists `joint_states` and `odom`.
4. Run `scripts/run_stacks.sh`.
5. Drive with `../../Zenoh-motion-stacks/build/system/zms_cli "" <client-id> goal|joint-target|status|watch ...`.

## Results, 2026-10-14

First run, before fixes:

| Check | Result |
| --- | --- |
| Navigation goal (1.5, 0, 0) | Succeeded |
| Navigation goal (1.5, 1.0, 90°) | Reached the position, never finished the final turn |
| Turn in place, 0.3 to 1.5 rad/s | 5 to 11 % of the command |
| Arm joint target | Arm did not move; failed with `tracking_error` |

Final run, after the fixes listed below:

| Check | Result |
| --- | --- |
| Turn in place, 0.3 rad/s for 4 s | 1.27 rad turned (1.20 commanded) |
| Navigation goal (+1.5, 0, 0°) | Succeeded in 6 s |
| Navigation goal (+1.5, +1.0, 90°) | Succeeded in 8 s |
| Navigation goal back to start, 180° | Succeeded in 12 s |
| Arm joint target `0.8,0.3,0.5,0.5,0.4,0.3` (6 joints) | Succeeded in about 2 s; `joint_states` also carried wheel and finger joints |

Fixes behind the final run:

- RobotLens: Joint Control capability and joint sliders available on Zenoh.
- RobotLens: `cmd_joints` entries addressed by joint name reach that joint's position actuator.
- RobotLens: every wheel on each side is driven, and skid-steer bases get a bounded yaw-rate correction.
- RobotLens: merging a URDF robot into an MJCF world keeps its joint limits in the world's angle unit; limits were previously read as degrees.
- motion-stacks: joint states are matched by name, joints brake within the acceleration limit, the final turn has hysteresis, and `progress_timeout_s` ends stuck commands with `no_progress`.

The arm target `upper_arm_joint -0.6, forearm_joint 1.0` puts the forearm into the chassis and cannot be reached; use a collision-free pose.

Second live run (motion-stacks `2e47c14`, manipulation 0.2.0):

| Check | Result |
| --- | --- |
| Pose target from `scripts/fk.py` for joints `0.8,0.3,0.5,0.5,0.4,0.3` | IK chose `0.8,0.3,0.501,0.5,0.4,0.299`; succeeded in 2 s; reached tool pose within 1.0 cm and 0.03 rad |
| Joint target `0,-1.9,-2.0,0,0,0` | Rejected `invalid_target`: base_link collides with forearm |
| Pose inside the configured table | Rejected `unreachable` |
| Pose 3 m away | Rejected `unreachable` |
| Upper arm beyond its limit | Rejected `invalid_target` |
| Navigation goals out to (1.2, 0.8, 90°) and back | Both succeeded (8 s, 12 s); the return ended 0.15 m from the goal, within the final-turn hysteresis of twice `position_tolerance` |

Third live run (RobotLens Zenoh `scan` and motion-stack commands, motion-stacks avoidance and Cartesian fixes):

The 2D LiDAR `2D LiDAR 1` on `lidar_link` publishes `scan` over Zenoh. It is mounted 4 cm above the link origin so its rays clear the chassis top face, with a level orientation. `stacks/navigation.json` sets `scan_offset` to the LiDAR position, enables avoidance, and ignores returns within 0.3 m of the base centre (the arm base). `stacks/manipulation.json` configures the two finger joints as the gripper.

| Check | Result |
| --- | --- |
| `scan` with a 0.5 m box 2 m ahead, then 1.2 m to the left | 0° at 1.42 m, then +77° to +106° at 0.95 m, for both LiDAR simulation backends |
| Goal (3.5, 0) with the box at (2, 0) | Detoured to y = -1.06 m, passed the box, succeeded in 17 s at (3.44, -0.08) |
| Gripper open, close, open from `motion_stack.gripper` | Fingers 0.035, 0.0, 0.035 m; all succeeded |
| Cartesian move up 7 cm | Succeeded; 1.0 cm from the goal, at most 0.35 cm off the straight line |
| Cartesian move forward 6 cm | Succeeded; 1.3 cm from the goal, at most 1.5 cm off the line |
| Cartesian sideways move with fixed orientation | Rejected `unreachable`; the arm needs base yaw to move sideways |
| `motion_stack.status`, `goal`, `cancel`, `target`, `initial_pose`, `find` | Work; status lists both stacks at 0.2.0 with their features |

Fixes behind this run: RobotLens Zenoh laser scans, depth LiDAR bearings, ray-cast LiDAR geometry and minimum-range start; motion-stacks self-hit filtering, obstacle memory, real detours, and per-shape collision padding for Cartesian paths.

## Open findings

- The simulation bridge switches (Publish states, Accept commands) are not saved with the project and must be ticked after every launch.
- The 2D LiDAR and IMU virtual sensors appear twice on the robot links after reopening the project.
- Only the 2D LiDAR publishes on Zenoh; other virtual sensors stay ROS 2 only.
- A primitive added while the simulation plays has no collision; the robot drove through one.
- Localization was not checked live; it needs a map on `map`.
