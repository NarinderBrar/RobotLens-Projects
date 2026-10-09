#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import random
import sys
import time
from pathlib import Path

import torch
from torch import nn
from torch.distributions import Normal
import zenoh


ACTUATORS = (
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll",
)
HOME = (
    0.0, -0.0872665, -0.457924, -0.004940, 0.452984,
    0.0, 0.0872665, 0.457924, 0.004940, -0.452984,
    0.3490659, 0.3490659, 0.0, 0.0,
)
ACTUATOR_INDEX = {name: index for index, name in enumerate(ACTUATORS)}
ACTION_SCALE = 0.25
EPISODE_LIMIT = 500
STAND_HEIGHT = 0.12
OBSERVATION_SCALE = torch.tensor(
    [0.25] * 14 + [5.0] * 14 + [1.0] * 4 + [10.0] * 3 + [10.0] * 3,
    dtype=torch.float32,
)


class ActorCritic(nn.Module):
    def __init__(self, observation_dim: int, action_dim: int):
        super().__init__()
        self.actor = nn.Sequential(
            nn.Linear(observation_dim, 256), nn.Tanh(),
            nn.Linear(256, 256), nn.Tanh(), nn.Linear(256, action_dim),
        )
        self.critic = nn.Sequential(
            nn.Linear(observation_dim, 256), nn.Tanh(),
            nn.Linear(256, 256), nn.Tanh(), nn.Linear(256, 1),
        )
        self.log_std = nn.Parameter(torch.full((action_dim,), -0.5))
        for module in self.modules():
            if isinstance(module, nn.Linear):
                nn.init.orthogonal_(module.weight, gain=math.sqrt(2))
                nn.init.zeros_(module.bias)
        nn.init.orthogonal_(self.actor[-1].weight, gain=0.01)
        nn.init.orthogonal_(self.critic[-1].weight, gain=1.0)

    def distribution(self, observations: torch.Tensor) -> Normal:
        return Normal(self.actor(observations), self.log_std.clamp(-3.0, 1.0).exp())

    def value(self, observations: torch.Tensor) -> torch.Tensor:
        return self.critic(observations).squeeze(-1)


def payload_text(sample) -> str:
    payload = sample.payload
    if hasattr(payload, "to_bytes"):
        payload = payload.to_bytes()
    return bytes(payload).decode("utf-8")


def observe(environment: dict) -> list[float]:
    positions = environment["positions"]
    velocities = environment["velocities"]
    if len(positions) != 21 or len(velocities) != 20:
        raise ValueError(
            f"Microduck state dimensions are {len(positions)} positions and "
            f"{len(velocities)} velocities; expected 21 and 20"
        )
    trunk = next(
        (body for body in environment["bodyPoses"] if body["name"] == "trunk_base"), None
    )
    if trunk is None:
        raise ValueError("physics state is missing the trunk_base body pose")
    quaternion = trunk["quaternion"]
    values = (
        [positions[index + 7] - HOME[index] for index in range(14)]
        + list(velocities[6:20])
        + list(quaternion)
        + list(trunk["angularVelocity"])
        + list(trunk["linearVelocity"])
    )
    scales = OBSERVATION_SCALE.tolist()
    return [max(-10.0, min(10.0, value / scale)) for value, scale in zip(values, scales)]


def reset_state(environment: dict, seed: int, episode: int) -> tuple[list[float], list[float]]:
    rng = random.Random(seed + int(environment["environmentIndex"]) * 1000003 + episode * 9176)
    curriculum = min(1.0, 0.35 + episode / 200.0)
    roll = rng.uniform(-0.12, 0.12) * curriculum
    pitch = rng.uniform(-0.12, 0.12) * curriculum
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    quaternion = [cr * cp, sr * cp, cr * sp, -sr * sp]
    positions = [0.0, 0.0, STAND_HEIGHT, *quaternion, *HOME]
    positions[7 + ACTUATOR_INDEX["left_hip_roll"]] += rng.uniform(-0.04, 0.04) * curriculum
    positions[7 + ACTUATOR_INDEX["right_hip_roll"]] -= rng.uniform(-0.04, 0.04) * curriculum
    velocities = [0.0] * 20
    velocities[4] = rng.uniform(-0.15, 0.15) * curriculum
    velocities[5] = rng.uniform(-0.15, 0.15) * curriculum
    return positions, velocities


def outcome(environment: dict, controls: list[float], episode_steps: int) -> dict:
    trunk = next(body for body in environment["bodyPoses"] if body["name"] == "trunk_base")
    x, y = float(trunk["quaternion"][1]), float(trunk["quaternion"][2])
    upright = 1.0 - 2.0 * (x * x + y * y)
    height = float(trunk["position"][2])
    joint_error = sum(
        (position - home) ** 2
        for position, home in zip(environment["positions"][7:], HOME)
    ) / len(HOME)
    joint_speed = sum(value * value for value in environment["velocities"][6:20]) / len(HOME)
    body_speed = sum(value * value for value in trunk["angularVelocity"])
    body_speed += sum(value * value for value in trunk["linearVelocity"])
    effort = sum((value - home) ** 2 for value, home in zip(controls, HOME)) / len(HOME)
    finite = all(
        math.isfinite(float(value))
        for value in environment["positions"] + environment["velocities"] + controls
    )
    fallen = not finite or height < 0.075 or upright < 0.45
    truncated = episode_steps >= EPISODE_LIMIT
    reward = 2.0 * upright + 2.0 * min(height / 0.12, 1.0)
    reward -= 0.35 * joint_error + 0.015 * joint_speed + 0.04 * body_speed + 0.01 * effort
    if fallen:
        reward -= 5.0
    return {
        "reward": reward if finite else -10.0,
        "terminated": fallen,
        "truncated": truncated and not fallen,
        "success": truncated and not fallen,
    }


def normalize_observations(values, device: torch.device) -> torch.Tensor:
    return torch.as_tensor(values, dtype=torch.float32, device=device)


def action_log_prob(distribution: Normal, latent: torch.Tensor) -> torch.Tensor:
    squashed = torch.tanh(latent)
    return (distribution.log_prob(latent) - torch.log(1.0 - squashed.square() + 1e-6)).sum(-1)


def update_policy(
    model: ActorCritic,
    optimizer: torch.optim.Optimizer,
    observations: list,
    latents: list,
    old_log_probs: list,
    values: list,
    rewards: list,
    next_values: list,
    terminated: list,
    episode_done: list,
    device: torch.device,
    update_epochs: int,
    minibatch_size: int,
) -> dict:
    if not rewards:
        return {"mean_reward": 0.0, "mean_episode_return": 0.0, "policy_loss": 0.0, "value_loss": 0.0}
    obs = torch.as_tensor(observations, dtype=torch.float32, device=device)
    actions = torch.as_tensor(latents, dtype=torch.float32, device=device)
    old_log = torch.as_tensor(old_log_probs, dtype=torch.float32, device=device)
    old_value = torch.as_tensor(values, dtype=torch.float32, device=device)
    reward_tensor = torch.as_tensor(rewards, dtype=torch.float32, device=device)
    next_value_tensor = torch.as_tensor(next_values, dtype=torch.float32, device=device)
    terminal_tensor = torch.as_tensor(terminated, dtype=torch.float32, device=device)
    done_tensor = torch.as_tensor(episode_done, dtype=torch.float32, device=device)
    advantages = torch.zeros_like(reward_tensor)
    accumulator = torch.zeros(reward_tensor.shape[1], dtype=torch.float32, device=device)
    for step in reversed(range(reward_tensor.shape[0])):
        delta = reward_tensor[step] + 0.99 * next_value_tensor[step] * (1.0 - terminal_tensor[step]) - old_value[step]
        accumulator = delta + 0.99 * 0.95 * (1.0 - done_tensor[step]) * accumulator
        advantages[step] = accumulator
    returns = advantages + old_value
    flat_obs = obs.reshape(-1, obs.shape[-1])
    flat_actions = actions.reshape(-1, actions.shape[-1])
    flat_old_log = old_log.reshape(-1)
    flat_advantages = advantages.reshape(-1)
    flat_returns = returns.reshape(-1)
    flat_advantages = (flat_advantages - flat_advantages.mean()) / (flat_advantages.std(unbiased=False) + 1e-8)
    losses = []
    count = flat_obs.shape[0]
    for _ in range(update_epochs):
        for indices in torch.randperm(count, device=device).split(minibatch_size):
            distribution = model.distribution(flat_obs[indices])
            new_log = action_log_prob(distribution, flat_actions[indices])
            ratio = (new_log - flat_old_log[indices]).exp()
            unclipped = ratio * flat_advantages[indices]
            clipped = ratio.clamp(0.8, 1.2) * flat_advantages[indices]
            policy_loss = -torch.minimum(unclipped, clipped).mean()
            value_loss = 0.5 * (model.value(flat_obs[indices]) - flat_returns[indices]).square().mean()
            entropy = distribution.entropy().sum(-1).mean()
            loss = policy_loss + value_loss - 0.01 * entropy
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(), 0.5)
            optimizer.step()
            losses.append((policy_loss.detach(), value_loss.detach()))
    return {
        "mean_reward": float(reward_tensor.mean().item()),
        "mean_episode_return": float(returns.mean().item()),
        "policy_loss": float(torch.stack([item[0] for item in losses]).mean().item()),
        "value_loss": float(torch.stack([item[1] for item in losses]).mean().item()),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, required=True)
    parser.add_argument("--rollout-steps", type=int, default=256)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--resume-from", type=Path)
    parser.add_argument("--ready-file", type=Path)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--timeout", type=float, default=170.0)
    parser.add_argument("--update-epochs", type=int, default=8)
    parser.add_argument("--minibatch-size", type=int, default=256)
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--evaluation-report", type=Path)
    args = parser.parse_args()
    if args.steps <= 0 or args.rollout_steps <= 0 or args.timeout <= 0:
        parser.error("steps, rollout steps, and timeout must be positive")
    if not torch.cuda.is_available():
        print("CUDA is required for Microduck PPO training; no CPU fallback is enabled.", file=sys.stderr)
        return 2
    random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)
    device = torch.device("cuda")
    model = ActorCritic(38, len(ACTUATORS)).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=3e-4, eps=1e-5)
    project_root = Path(__file__).resolve().parents[2]
    checkpoint_path = args.checkpoint if args.checkpoint.is_absolute() else project_root / args.checkpoint
    checkpoint_path = checkpoint_path.resolve()
    resume_path = args.resume_from
    if resume_path is not None:
        if args.evaluate:
            parser.error("--resume-from cannot be combined with --evaluate")
        if not resume_path.is_absolute():
            resume_path = project_root / resume_path
        resume_path = resume_path.resolve()
    global_step_offset = 0
    previous_metrics = None
    if resume_path is not None:
        checkpoint = torch.load(resume_path, map_location=device, weights_only=False)
        if checkpoint.get("task") != "microduck-standing-balance-zenoh":
            raise ValueError("checkpoint does not belong to the Microduck standing-balance task")
        if (checkpoint.get("observation_dim") != 38
                or checkpoint.get("action_dim") != len(ACTUATORS)
                or checkpoint.get("actuators") != list(ACTUATORS)
                or checkpoint.get("home_controls") != list(HOME)
                or checkpoint.get("action_scale") != ACTION_SCALE):
            raise ValueError("checkpoint is incompatible with the current Microduck policy")
        model.load_state_dict(checkpoint["model"])
        optimizer.load_state_dict(checkpoint["optimizer"])
        global_step_offset = int(checkpoint.get("steps", 0))
        previous_metrics = checkpoint.get("training_metrics")
        if previous_metrics is None:
            source_metrics_path = resume_path.parent / "training.json"
            if source_metrics_path.is_file():
                previous_metrics = json.loads(source_metrics_path.read_text())
        if "python_rng_state" in checkpoint:
            random.setstate(checkpoint["python_rng_state"])
        if "torch_rng_state" in checkpoint:
            torch.set_rng_state(checkpoint["torch_rng_state"])
        if "cuda_rng_state" in checkpoint:
            torch.cuda.set_rng_state_all(checkpoint["cuda_rng_state"])
    if args.evaluate:
        checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
        if checkpoint.get("task") != "microduck-standing-balance-zenoh":
            raise ValueError("checkpoint does not belong to the Microduck standing-balance task")
        model.load_state_dict(checkpoint["model"])
        model.eval()
    metrics_path = checkpoint_path.parent / "training.json"
    checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
    metrics = previous_metrics or {
        "task": "microduck-standing-balance-zenoh", "device": str(device), "seed": args.seed, "updates": []
    }
    metrics["device"] = str(device)
    metrics["last_seed"] = args.seed
    metrics.setdefault("updates", [])
    if resume_path is not None:
        metrics.setdefault("stages", []).append({
            "resume_from": str(resume_path), "additional_steps": args.steps, "seed": args.seed
        })
    states: queue.Queue = queue.Queue(maxsize=2)
    callback_errors: queue.Queue = queue.Queue(maxsize=1)
    session = zenoh.open(zenoh.Config())
    publisher = None

    def receive(sample) -> None:
        try:
            payload = json.loads(payload_text(sample))
            states.put_nowait(payload)
        except queue.Full:
            try:
                callback_errors.put_nowait("RobotLens published state faster than the trainer consumed it")
            except queue.Full:
                pass
        except Exception as error:
            try:
                callback_errors.put_nowait(str(error))
            except queue.Full:
                pass

    subscriber = session.declare_subscriber("training/*/external/physics/state", receive)
    expected_sequence = 0
    run_id = None

    def next_state() -> dict:
        try:
            error = callback_errors.get_nowait()
            raise RuntimeError(error)
        except queue.Empty:
            pass
        try:
            state = states.get(timeout=args.timeout)
        except queue.Empty as error:
            raise TimeoutError("timed out waiting for RobotLens physics state") from error
        if state.get("schema") != 2 or state.get("kind") != "state":
            raise RuntimeError("RobotLens sent an unsupported physics state message")
        if run_id is not None and state.get("runId") != run_id:
            raise RuntimeError("received a physics state from a different training run")
        if state.get("sequence") != expected_sequence:
            raise RuntimeError(
                f"physics sequence mismatch: got {state.get('sequence')}, expected {expected_sequence}"
            )
        return state

    def send(command: dict) -> None:
        nonlocal expected_sequence
        command.update({"schema": 2, "runId": run_id, "sequence": expected_sequence})
        publisher.put(json.dumps(command, separators=(",", ":")))
        expected_sequence += 1

    def save_checkpoint(step_count: int, update_record: dict) -> None:
        temporary = checkpoint_path.with_suffix(checkpoint_path.suffix + ".tmp")
        metrics["steps"] = global_step_offset + step_count
        metrics["updates"].append(update_record)
        torch.save({
            "schema": 1,
            "task": "microduck-standing-balance-zenoh",
            "observation_dim": 38,
            "action_dim": len(ACTUATORS),
            "actuators": list(ACTUATORS),
            "home_controls": list(HOME),
            "action_scale": ACTION_SCALE,
            "steps": global_step_offset + step_count,
            "model": model.state_dict(),
            "optimizer": optimizer.state_dict(),
            "metrics": update_record,
            "training_metrics": metrics,
            "python_rng_state": random.getstate(),
            "torch_rng_state": torch.get_rng_state(),
            "cuda_rng_state": torch.cuda.get_rng_state_all(),
        }, temporary)
        os.replace(temporary, checkpoint_path)
        temporary_metrics = metrics_path.with_suffix(".json.tmp")
        temporary_metrics.write_text(json.dumps(metrics, indent=2) + "\n")
        os.replace(temporary_metrics, metrics_path)

    try:
        if args.ready_file:
            ready_path = args.ready_file if args.ready_file.is_absolute() else project_root / args.ready_file
            ready_path.parent.mkdir(parents=True, exist_ok=True)
            ready_path.write_text("ready\n")
        state = next_state()
        run_id = state["runId"]
        publisher = session.declare_publisher(f"training/{run_id}/external/physics/command")
        environments = state["environments"]
        resets = []
        for environment in environments:
            positions, velocities = reset_state(environment, args.seed, int(environment["episodeIndex"]))
            resets.append({
                "environmentIndex": environment["environmentIndex"],
                "positions": positions,
                "velocities": velocities,
            })
        send({"kind": "reset", "resets": resets})
        state = next_state()
        count = len(state["environments"])
        if count == 0:
            raise RuntimeError("RobotLens reported no physics environments")
        if state.get("controlDimension") != len(ACTUATORS):
            raise RuntimeError(f"expected 14 controls, got {state.get('controlDimension')}")
        episode_steps = [0] * count
        episode_returns = [0.0] * count
        completed_episodes: list[float] = []
        completed_episode_lengths: list[int] = []
        successful_episodes = 0
        rollout = {name: [] for name in (
            "observations", "latents", "log_probs", "values", "rewards",
            "next_values", "terminated", "done",
        )}
        controls = None
        pending_outcomes = None
        step_count = 0
        started = time.monotonic()

        while step_count < args.steps:
            observations = [observe(environment) for environment in state["environments"]]
            observation_tensor = normalize_observations(observations, device)
            with torch.no_grad():
                distribution = model.distribution(observation_tensor)
                latent = distribution.mean if args.evaluate else distribution.sample()
                actions = torch.tanh(latent)
                log_probs = action_log_prob(distribution, latent)
                values = model.value(observation_tensor)
            action_values = actions.clamp(-1.0, 1.0).cpu().tolist()
            controls = [
                [home + ACTION_SCALE * action for home, action in zip(HOME, row)]
                for row in action_values
            ]
            command = {"kind": "step", "controls": controls}
            if pending_outcomes is not None:
                command["outcomes"] = pending_outcomes
                pending_outcomes = None
            send(command)
            next_physics_state = next_state()
            next_environments = next_physics_state["environments"]
            outcomes = []
            reset_rows = []
            for index, environment in enumerate(next_environments):
                episode_steps[index] += 1
                result = outcome(environment, controls[index], episode_steps[index])
                if not environment.get("stepOk", True):
                    result = {"reward": -10.0, "terminated": True, "truncated": False, "success": False}
                outcomes.append(result)
                episode_returns[index] += result["reward"]
            if not args.evaluate:
                next_observations = [observe(environment) for environment in next_environments]
                next_tensor = normalize_observations(next_observations, device)
                with torch.no_grad():
                    next_values = model.value(next_tensor)
                rollout["observations"].append(observations)
                rollout["latents"].append(latent.cpu().tolist())
                rollout["log_probs"].append(log_probs.cpu().tolist())
                rollout["values"].append(values.cpu().tolist())
                rollout["rewards"].append([item["reward"] for item in outcomes])
                rollout["next_values"].append(next_values.cpu().tolist())
                rollout["terminated"].append([float(item["terminated"]) for item in outcomes])
                rollout["done"].append([float(item["terminated"] or item["truncated"]) for item in outcomes])
            step_count += 1

            for index, result in enumerate(outcomes):
                if result["terminated"] or result["truncated"]:
                    completed_episodes.append(episode_returns[index])
                    completed_episode_lengths.append(episode_steps[index])
                    successful_episodes += int(result["success"])
                    episode_returns[index] = 0.0
                    episode_steps[index] = 0
                    positions, velocities = reset_state(
                        next_environments[index], args.seed, int(next_environments[index]["episodeIndex"]) + 1
                    )
                    reset_rows.append({
                        "environmentIndex": index,
                        "positions": positions,
                        "velocities": velocities,
                    })

            final_step = step_count >= args.steps
            rollout_full = len(rollout["rewards"]) >= args.rollout_steps
            if not args.evaluate and (rollout_full or final_step):
                update_record = update_policy(
                    model, optimizer, rollout["observations"], rollout["latents"],
                    rollout["log_probs"], rollout["values"], rollout["rewards"],
                    rollout["next_values"], rollout["terminated"], rollout["done"],
                    device, args.update_epochs, args.minibatch_size,
                )
                update_record.update({
                    "step": global_step_offset + step_count,
                    "completed_episodes": len(completed_episodes),
                    "mean_completed_episode_return": (
                        sum(completed_episodes) / len(completed_episodes) if completed_episodes else None
                    ),
                    "elapsed_seconds": time.monotonic() - started,
                })
                save_checkpoint(step_count, update_record)
                print(json.dumps(update_record), flush=True)
                rollout = {name: [] for name in rollout}

            if final_step:
                completion_path = str(checkpoint_path)
                if args.evaluate:
                    report_path = args.evaluation_report
                    if report_path is None:
                        report_path = checkpoint_path.parent / "evaluation.json"
                    elif not report_path.is_absolute():
                        report_path = project_root / report_path
                    report_path = report_path.resolve()
                    report_path.parent.mkdir(parents=True, exist_ok=True)
                    report = {
                        "task": "microduck-standing-balance-zenoh",
                        "checkpoint": str(checkpoint_path),
                        "device": str(device),
                        "seed": args.seed,
                        "physics_steps": step_count,
                        "episodes": len(completed_episodes),
                        "successful_episodes": successful_episodes,
                        "success_rate": (
                            successful_episodes / len(completed_episodes)
                            if completed_episodes else 0.0
                        ),
                        "mean_episode_return": (
                            sum(completed_episodes) / len(completed_episodes)
                            if completed_episodes else 0.0
                        ),
                        "mean_episode_length": (
                            sum(completed_episode_lengths) / len(completed_episode_lengths)
                            if completed_episode_lengths else 0.0
                        ),
                    }
                    report_path.write_text(json.dumps(report, indent=2) + "\n")
                    print(json.dumps(report), flush=True)
                send({
                    "kind": "complete",
                    "checkpointPath": completion_path,
                    "outcomes": outcomes,
                })
                break
            if reset_rows:
                send({"kind": "reset", "resets": reset_rows, "outcomes": outcomes})
                state = next_state()
                pending_outcomes = None
            else:
                state = next_physics_state
                pending_outcomes = outcomes
    except Exception as error:
        print(f"Microduck trainer failed: {error}", file=sys.stderr, flush=True)
        return 1
    finally:
        subscriber.undeclare()
        if publisher is not None:
            publisher.undeclare()
        session.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
