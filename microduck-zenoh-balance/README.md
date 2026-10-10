# Microduck Zenoh balance training

This project trains a policy to keep the Microduck standing in RobotLens' in-process MuJoCo simulation. RobotLens owns the CPU/GPU physics environments and viewport. The project owns the task, PPO policy, optimizer, resets, rewards, and checkpoints. The trainer connects to RobotLens through native Zenoh schema 2; it does not use RobotLens' embedded Python runtime.

## Environment

Create a project-local environment and install the dependencies:

```bash
cd /home/narinder/Documents/RobotLens-Workspace/RobotLens-Projects/microduck-zenoh-balance
python3 -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r training/requirements.txt
```

Install a CUDA-enabled PyTorch build matching the machine's NVIDIA driver if the default PyTorch wheel does not include a compatible CUDA runtime. The trainer requires CUDA and stops with an error instead of silently training on CPU.

## Create and prepare the RobotLens project

The `.robotlens` project file was created and opened through RobotLens MCP. Keep that file MCP-owned; do not hand-edit it. The project includes the Microduck CAD meshes and uses `src/microduck/mujoco/real/microduck_hybrid.xml`: CAD meshes provide the robot's detailed appearance while the model retains the 14-joint training layout and simplified contact geometry. The source `microduck_real.xml` is also included for reference, but RobotLens did not import it successfully in this project.

In the open RobotLens session, use `robotlens.run` with these commands in order:

```json
{"command":"comms.start"}
{"command":"training.load_task","args":{"path":"/home/narinder/Documents/RobotLens-Workspace/RobotLens-Projects/microduck-zenoh-balance/training/microduck/external.task.json"}}
```

Use `model.list` to confirm that the saved hybrid model is loaded, and `comms.status` to confirm Zenoh is connected. Run `training.preflight`, then inspect `robotlens.inspect_learning_runtime`. The previous full-training setup used external Zenoh execution, a 4 by 4 grid, GPU physics, 500 transitions per environment per iteration, and 100 iterations.

For single-robot balance training, set the Training panel's Robot to `world-main`, Grid Side to `1`, Execution to `external`, Device to `gpu`, and Viewport Preview to `grid`. The project's `world-main` asset points to `microduck_hybrid.xml`; confirm that exact path in `robotlens.inspect_learning_runtime.grid_preview.model_path` before starting. The current `robot-main` asset points to `microduck_real.xml`, whose joint and actuator order places the head before the right leg. The trainer expects left leg, right leg, then head, so using `robot-main` sends some controls to the wrong joints. Training runs their own MuJoCo physics pool; the interactive Simulation panel can remain stopped.

## Start training over MCP

Create the checkpoint and readiness paths inside this project, then launch the trainer before starting training:

```json
{"command":"process.launch","args":{"path":"training/microduck/run_external_trainer.sh","name":"microduck_balance_real","args":["--steps","50000","--rollout-steps","500","--checkpoint","training/runs/microduck-real-gpu/policy.pt","--ready-file","training/runs/microduck-real-gpu/ready"]}}
```

Wait for `process.status` to show the trainer running and the readiness file to exist, then issue `training.start`. Monitor `robotlens.inspect_learning_runtime` and `process.status`. The trainer writes `policy.pt` and `training.json` under `training/runs/microduck-real-gpu/`. The current model uses the same actuator order and joint layout as the primitive model, so a compatible prior checkpoint can be resumed.

The step count and rollout size must both match the Training panel's effective transition count: `iterations * max(transitions, episodeStepLimit)` and `max(transitions, episodeStepLimit)`, respectively. Stop the run with `training.stop`; the managed project process is stopped with `process.stop` when needed.

## Continue training in stages

Each training run can add more transitions to a saved policy. Set `--resume-from` to the previous checkpoint and write the next checkpoint to a new path so you can keep each stage. For example, with 500 transitions per iteration and a 500-step episode limit, run a 10-iteration stage by setting the Training panel to 10 iterations and launch:

```json
{"command":"process.launch","args":{"path":"training/microduck/run_external_trainer.sh","name":"microduck_balance_real_stage_02","args":["--steps","5000","--rollout-steps","500","--resume-from","training/runs/microduck-real-gpu/policy.pt","--checkpoint","training/runs/microduck-real-stage-02/policy.pt","--ready-file","training/runs/microduck-real-stage-02/ready"]}}
```

Wait for the process to be ready, then start training. For another stage, use the stage 02 checkpoint as `--resume-from` and choose a new output directory. Checkpoints retain the policy, optimizer, random generator state, cumulative transition count, and training history. `--steps` is the number of additional transitions for that stage. The new checkpoint's `steps` field and training history show the cumulative total.

Keep `--steps` equal to the Training panel's iterations multiplied by its effective transitions per iteration. For example, 20 iterations at 500 transitions means `--steps 10000` and `--rollout-steps 500`.

Use `--disturbance-scale 1` for the original reset difficulty or up to `--disturbance-scale 3` to increase starting body tilt, angular velocity, and hip-roll offsets. The training log records this scale, mean completed episode length, and success rate. An episode succeeds by remaining upright for the 500-step limit. Compare those measures and mean reward across new checkpoints instead of judging a single fall in the viewport.

## Evaluate on new disturbances

After training, configure the external task for 10 iterations and 500 transitions, keep the 4 by 4 GPU grid, run preflight, then launch the same managed trainer in deterministic evaluation mode before `training.start`:

```json
{"command":"process.launch","args":{"path":"training/microduck/run_external_trainer.sh","name":"microduck_balance_real_eval","args":["--steps","5000","--rollout-steps","500","--seed","31415","--checkpoint","training/runs/microduck-real-gpu/policy.pt","--evaluation-report","training/runs/microduck-real-gpu/evaluation.json","--evaluate"]}}
```

The evaluation report records the success rate, mean episode return, and mean episode length over the new seeded disturbances. The hybrid-model checkpoint is written under `training/runs/microduck-real-gpu/`. It is a simulator policy checkpoint; a hardware controller still needs matching sensor, joint-order, and safety-limit integration before use on the physical robot.

## Task

The policy observes the trunk orientation and motion plus all 14 joint positions and velocities. It controls the 14 position servos around the standing pose. Resets include small, randomized body and joint disturbances. The reward favors upright posture, trunk height, and low joint motion; falls end an episode. PPO optimizes the policy using CUDA.

## Balance on a rolling ball

`src/microduck/mujoco/real/microduck_ball_balance.xml` places both feet on a freely rolling 7.2 cm radius ball, 40% smaller in diameter than the initial ball model. The ball has its own free joint and collides with the floor and feet. `training/microduck/ball_balance.task.json` selects that model and keeps the 14 actuator order of the hybrid robot. The ball trainer observes its position and velocity relative to the robot and rewards upright posture, staying centered over the ball, and completing 500-step episodes. This is a separate task and requires a separate checkpoint.

In RobotLens, import the ball model, load the ball task, then select `world-main` as the Training panel's Robot and World. Confirm `robotlens.inspect_learning_runtime.grid_preview.model_path` ends in `microduck_ball_balance.xml`. Configure external Zenoh execution, GPU, grid side 1, 500 transitions, and a short iteration count. Keep viewport preview off while idle; turn on grid preview after training starts. Run preflight before starting.

Launch `training/microduck/run_external_trainer.sh` through `process.launch` with `--ball-balance`, `--steps` equal to iterations times 500, `--rollout-steps 500`, and a new `--checkpoint` path. To start from the learned flat-ground actor, add `--initialize-from` with a compatible standing checkpoint such as `training/runs/microduck-balance-grid1-live-stage-02/policy.pt`. That copies the actor and starts a new critic and optimizer; later ball stages use `--resume-from` with the preceding ball checkpoint. Wait for the trainer's readiness file before `training.start`.

The saved training metrics include mean reward, completed episode length, and success rate. A full ball-balance success requires 500 transitions. The smaller ball has both feet in contact at the starting pose, and RobotLens GPU preflight admits the task. The first 10,000-transition stage averaged about 37 transitions per episode; the next 20,000-transition stage, after stiffening ball contact, averaged about 39. Neither stage completed a 500-transition episode. Their checkpoints are in `training/runs/microduck-ball-grid1-stage-01/` and `training/runs/microduck-ball-grid1-stage-02/`. The next stage can resume from stage 02 on a 2 by 2 GPU grid.

## Seesaw balance

`src/microduck/mujoco/real/microduck_seesaw_balance.xml` stands the robot on a wooden plank resting on a free wooden ball, like a balance board. The ball has a 4 cm radius and 80 g mass. The plank is 30 cm long across the robot's left-right axis, 9 cm deep, and 1 cm thick, weighing 120 g. Ball and plank are separate free bodies, so the plank can tip in any direction and the ball can roll on the floor. Without control, the robot tips the plank and falls in about half a second.

`training/microduck/seesaw_balance.task.json` selects that model with the same 14 actuators. The trainer's `--seesaw-balance` mode observes the plank's pose and motion relative to the robot and the ball's position under the plank. It rewards an upright robot, a level plank, staying centred on the plank, and keeping the ball under the plank's centre. An episode ends when the plank tilts more than 0.35 rad, the robot leaves the plank or falls, or the ball reaches the plank's end. Starting disturbances are half the flat-ground size.

In RobotLens, import the seesaw model and load the seesaw task with project-relative paths:

```json
{"command":"model.import","args":{"path":"src/microduck/mujoco/real/microduck_seesaw_balance.xml"}}
{"command":"training.load_task","args":{"path":"training/microduck/seesaw_balance.task.json"}}
```

Select `world-main` for Robot and World, external execution, GPU, 500 transitions, and grid preview, then run preflight. Start from the flat-ground actor:

```json
{"command":"process.launch","args":{"path":"training/microduck/run_external_trainer.sh","name":"microduck_seesaw_stage_01","args":["--seesaw-balance","--steps","10000","--rollout-steps","500","--initialize-from","training/runs/microduck-balance-grid1-live-stage-02/policy.pt","--checkpoint","training/runs/microduck-seesaw-grid2-stage-01/policy.pt","--ready-file","training/runs/microduck-seesaw-grid2-stage-01/ready"]}}
```

That example is 20 iterations. Wait for the readiness file, then call `training.start`. Later stages use `--resume-from` with the previous seesaw checkpoint. Judge progress by completed episode length and success rate, not reward alone.

## Seesaw on a fixed roller

The free-ball seesaw is too unstable to learn from the standing policy directly, so balance is first learned on a roller fixed to the floor. `src/microduck/mujoco/real/microduck_seesaw_roller.xml` rests the same plank on a 5 cm radius capsule roller whose axis points forward, so the plank tips only left and right. The roller is a free body held by a weld constraint, which keeps it visible in the training grid. `microduck_seesaw_roller_r25.xml` through `microduck_seesaw_roller_r08.xml` are the same scene with larger rollers, with matching `seesaw_roller_rNN.task.json` manifests.

The trainer's `--seesaw-roller` mode uses the same 56 observations as the ball seesaw, and `--roller-radius` must match the selected model. A larger roller is easier: the standing policy survives about 450 of 500 transitions on a 30 cm roller but only about 40 on the 5 cm roller. Train from 25 cm down to 5 cm, resuming each stage from the previous one:

1. Start at 25 cm with `--initialize-from training/runs/microduck-balance-grid1-live-stage-02/policy.pt --critic-warmup 5`.
2. For 20, 16, 12, 8, and 5 cm, use `--resume-from` with the previous stage's `policy.pt`.
3. Move on once the last five updates each report `recent_success_rate` of at least 0.95.

Each update records `recent_episode_length` and `recent_success_rate` for the episodes completed since the previous update. A 4 by 4 GPU grid needed about 60 updates at 25 cm and 10 to 75 updates for each later stage. The final 5 cm checkpoint is `training/runs/microduck-roller-r05/policy.pt`; every environment balanced for the full 500 transitions over its last updates.

## Seesaw on a ball

After the roller, the robot learns the ball seesaw in two more curricula, resuming each stage from the previous one:

1. A fixed ball welded to the floor, so the plank can tip in any direction but the ball cannot roll: `microduck_seesaw_dome_r25.xml`, `_r16`, `_r10`, and `_r05`, with matching `seesaw_dome_rNN.task.json` manifests. These use `--seesaw-balance` with `--ball-radius` matching the model.
2. A free rolling ball: `microduck_seesaw_ball_r25.xml`, `_r16`, `_r10`, and `_r06`, then the 4 cm `microduck_seesaw_balance.xml`.

On a 4 by 4 GPU grid, the dome stages took 30 to 190 updates each and the ball stages 130 to 390. The final 4 cm checkpoint is `training/runs/microduck-seesaw-ball-r04/policy.pt`. In deterministic evaluation with a new seed it balanced in all 160 episodes for the full 500 transitions.

## Seesaw tools

`training/microduck/tools/` holds scripts that drive a live RobotLens session over MCP. Start RobotLens with this project open, start Zenoh communications, and run the scripts from the project root with the system `python3`. They only use the standard library and resolve every project file relative to the project root.

Train the whole curriculum from the flat-ground policy:

```bash
python3 training/microduck/tools/seesaw_curriculum.py
```

Each stage is `kind:radius_cm`, where kind is `roller`, `dome`, or `ball`. Pass stages to run part of the curriculum, and use `--resume` to continue the first stage from an existing checkpoint:

```bash
python3 training/microduck/tools/seesaw_curriculum.py --resume --initialize-from training/runs/microduck-seesaw-ball-r10/policy.pt ball:10 ball:06 ball:04
```

A stage passes when, after at least 10 updates, its last 5 updates each report `recent_success_rate` of at least 0.95. With only 16 environments, one fall drops an update to 0.94, so a stage that reaches its 300 iterations without passing is resumed once more by default (`--retries`). Use `--grid` to change the grid side.

Run the trained 4 cm policy in a loop on an 8 by 8 grid, with deterministic actions:

```bash
python3 training/microduck/tools/run_policy.py
```

Press Ctrl+C to stop it, or stop it from another terminal:

```bash
python3 training/microduck/tools/run_policy.py --stop
```

`--grid` changes the number of robots; 16 by 16 is noticeably slower to draw. `--checkpoint`, `--model`, `--task`, `--mode`, and `--radius` run an earlier stage's policy, for example `--mode roller --radius 0.05 --model src/microduck/mujoco/real/microduck_seesaw_roller.xml --task training/microduck/seesaw_roller.task.json --checkpoint training/runs/microduck-roller-r05/policy.pt`. `--stop` stops every running managed process in the session.
