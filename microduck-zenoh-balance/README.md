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
{"command":"model.import","args":{"path":"src/microduck/mujoco/real/microduck_hybrid.xml"}}
{"command":"comms.start"}
{"command":"training.load_task","args":{"path":"/home/narinder/Documents/RobotLens-Workspace/RobotLens-Projects/microduck-zenoh-balance/training/microduck/external.task.json"}}
```

Wait for `model.status` to report the hybrid model loaded and `comms.status` to report Zenoh connected. Import replaces the previous MJCF source. Load the task manifest again so it selects the hybrid model. Run `training.preflight`, then inspect `robotlens.inspect_learning_runtime`. The recommended starting configuration is external Zenoh execution, a 4 by 4 grid, GPU physics, 500 transitions per environment per iteration, and 100 iterations. The 4 by 4 GPU preflight was admitted in the current RobotLens session at an estimated 243 MiB for learning environments.

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

## Evaluate on new disturbances

After training, configure the external task for 10 iterations and 500 transitions, keep the 4 by 4 GPU grid, run preflight, then launch the same managed trainer in deterministic evaluation mode before `training.start`:

```json
{"command":"process.launch","args":{"path":"training/microduck/run_external_trainer.sh","name":"microduck_balance_real_eval","args":["--steps","5000","--rollout-steps","500","--seed","31415","--checkpoint","training/runs/microduck-real-gpu/policy.pt","--evaluation-report","training/runs/microduck-real-gpu/evaluation.json","--evaluate"]}}
```

The evaluation report records the success rate, mean episode return, and mean episode length over the new seeded disturbances. The hybrid-model checkpoint is written under `training/runs/microduck-real-gpu/`. It is a simulator policy checkpoint; a hardware controller still needs matching sensor, joint-order, and safety-limit integration before use on the physical robot.

## Task

The policy observes the trunk orientation and motion plus all 14 joint positions and velocities. It controls the 14 position servos around the standing pose. Resets include small, randomized body and joint disturbances. The reward favors upright posture, trunk height, and low joint motion; falls end an episode. PPO optimizes the policy using CUDA.
