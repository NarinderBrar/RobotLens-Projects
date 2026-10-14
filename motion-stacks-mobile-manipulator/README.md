# Motion stacks mobile manipulator

RobotLens project `.robotlens/motion-stacks-mobile-manipulator.xml` (Zenoh, in-process MuJoCo). It tests the standalone `motion-stacks` navigation and manipulation processes against a RobotLens-simulated robot over native Zenoh.

## Contents

| Path | Purpose |
| --- | --- |
| `extensions/robot-gallery` | Robot builder package, copied from `procedural-atlas` |
| `robots/Mobile_manipulator.urdf` | Generated `mobile_manipulator` robot (seed 20), finished to URDF: 4 wheels, 6-joint arm, 2 fingers |
| `stacks/navigation.json`, `stacks/manipulation.json` | Stack configurations; empty `robot_prefix` matches RobotLens's unprefixed keys |
| `scripts/mcp.py` | Runs a RobotLens MCP command or tool against the newest live session |
| `scripts/run_stacks.sh` | Validates both configurations, then runs both stacks; logs go to `evidence/logs/` |
| `tools/zenoh_probe` | Lists live Zenoh keys (`zenoh_probe <seconds> [key]`) or publishes (`zenoh_probe put <key> <json> <seconds> <hz>`) |
| `evidence/` | Screenshots and logs from the last run |

Build the probe with `cmake -S tools/zenoh_probe -B tools/zenoh_probe/build -G Ninja && cmake --build tools/zenoh_probe/build`. Build `motion-stacks` with its `system` preset; `run_stacks.sh` looks for it at `../../motion-stacks/build/system` unless `ZMS_BIN` is set.

## Running the test

1. Open the project in RobotLens, start communication, and play the simulation.
2. Select the robot, add the Joint Control capability, and tick **Publish states** and **Accept commands** in its Simulation Bridge section. Zenoh Joint Control needs the RobotLens change from 2026-10-14.
3. Check `tools/zenoh_probe/build/zenoh_probe 2` lists `joint_states` and `odom`.
4. Run `scripts/run_stacks.sh`.
5. Drive with `../../motion-stacks/build/system/zms_cli "" <client-id> goal|joint-target|status|watch ...`.

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

## Open findings

- The simulation bridge switches (Publish states, Accept commands) are not saved with the project and must be ticked after every launch.
- The 2D LiDAR and IMU virtual sensors appear twice on the robot links after reopening the project.
