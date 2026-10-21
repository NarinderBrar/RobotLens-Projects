# Zenoh Panda arm

A Franka Panda on the ground plane. MuJoCo runs inside RobotLens, and communication uses native
Zenoh. The arm is moved by the external Zenoh manipulation stack (`zms_manipulation`).

## Quick start

From this project folder:

```bash
python3 scripts/quickstart.py setup   # check prerequisites and prepare the project
python3 scripts/quickstart.py test    # smoke test in RobotLens
python3 scripts/quickstart.py run     # smoke test plus the arm scenario
python3 scripts/quickstart.py         # setup, then run
```

`test` uses the running RobotLens session if there is one. Otherwise it starts RobotLens from
`../../RobotLens` (build it first with `./build.sh`) and closes it at the end, keeping its log
in `evidence/quickstart/robotlens.log`. The shared code is in `../tools/quickstart.py`.

`run` opens the project, closes the splash screen, starts Zenoh and the simulation, starts
`stacks/run_manipulation.sh`, enables the simulation bridge, waits for the manipulation stack,
then sends two joint targets with `motion_stack.target`: the Panda ready pose
`[0, -0.785, 0, -2.356, 0, 1.571, 0.785]`, then a pose that turns the base 1.2 rad
`[1.2, -0.3, 0.4, -1.8, 0.3, 1.8, 0]`. After each, every joint must be within 0.05 rad of its
target and holding, measured on the `joint_states` key, and the stack's own result for that
command must be `succeeded`. Screenshots of both poses are saved in
`evidence/quickstart/`. It then stops the stack, the simulation, and communication, in that
order, and writes `evidence/quickstart/report.json`.

## Files

| Path | Purpose |
| --- | --- |
| `.robotlens/zenoh-mujoco-panda-ground-plane.xml` | Project with the Panda |
| `robots/panda.urdf`, `robots/meshes/` | Panda model prepared for MuJoCo |
| `stacks/manipulation.json` | `zms_manipulation` configuration: chain `panda_link0` to `panda_link8`, gripper fingers |
| `stacks/run_manipulation.sh` | Validates the configuration and runs `zms_manipulation` |
| `scripts/build_project.py` | Builds `.robotlens/` over MCP |

The launcher finds `zms_manipulation` through `$ZMS_BIN`, then `PATH`, then the workspace's
`Zenoh-motion-stacks/build/system` or `motion-stacks/build/system`. The Panda URDF has no
collision primitives, so the stack plans without self-collision checks.

## The Panda model

`robots/panda.urdf` is the Franka Panda with relative mesh paths (`meshes/visual`,
`meshes/collision`) and the link masses and inertias from
`../../RobotLens-Automation/Zenoh-Mujoco-Manipulation`. The copy in `../robots_worlds/panda` has
no inertia, which MuJoCo rejects ("mass and inertia of moving bodies must be larger than
mjMINVAL"). For simulation the project adds:

- a `robotlens_sim` block with a fixed base and position actuators on the seven arm joints
  (gain 2400/2400/1800/1800/450/450/300, effort 87 N m on joints 1-4 and 12 N m on 5-7, the
  Panda's rated torques) and on both fingers
- joint damping (20 to 3) so the position servos settle without ringing
- the Panda's real velocity limits (2.175 and 2.61 rad/s) instead of 100 rad/s

Under gravity joint 4 settles about 0.023 rad short of a stack trajectory's end, and 0.011 rad
short of a direct command; both are inside `zms_manipulation`'s 0.03 rad `position_tolerance`.
With a third of these gains the sag was 0.034 rad, the stack reported "joints did not settle",
and on that failure it held the arm where it was.

## Rebuilding the project

`.robotlens/` was created entirely by `scripts/build_project.py` over MCP: it creates the
project, imports the Panda, and saves. To rebuild, remove `.robotlens/` and run it from an
empty RobotLens session (or with RobotLens closed). The shared builder is
`../tools/new_project.py`.

## Verification

On 2026-10-21 the project was built with `scripts/build_project.py`, and
`python3 scripts/quickstart.py run` passed three times in a row. It needs the RobotLens fix
that gives a fixed-base robot its joints on the MuJoCo bridge; without it `joint_states` stays
empty.
