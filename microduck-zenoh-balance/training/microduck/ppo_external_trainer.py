#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import random
import struct
import sys
import time
from pathlib import Path

import numpy as np
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
BALL_RADIUS = 0.072
BALL_STAND_HEIGHT = 0.258
FLAT_TASK = "microduck-standing-balance-zenoh"
BALL_TASK = "microduck-ball-balance-zenoh"
SEESAW_TASK = "microduck-seesaw-balance-zenoh"
ROLLER_TASK = "microduck-seesaw-roller-zenoh"
SEESAW_BALL_RADIUS = 0.04
SEESAW_MODES = {
    "roller": {
        "plank_height": 0.105, "stand_height": 0.230, "support": "seesaw_roller",
        "support_pose": (0.0, 0.0, 0.05, 1.0, 0.0, 0.0, 0.0),
    },
    "seesaw": {
        "plank_height": 0.085, "stand_height": 0.207, "support": "seesaw_ball",
        "support_pose": (0.0, 0.0, SEESAW_BALL_RADIUS, 1.0, 0.0, 0.0, 0.0),
    },
}
TASKS = {
    "flat": {"name": FLAT_TASK, "positions": 21, "velocities": 20, "observations": 38},
    "ball": {"name": BALL_TASK, "positions": 28, "velocities": 26, "observations": 47},
    "seesaw": {"name": SEESAW_TASK, "positions": 35, "velocities": 32, "observations": 56},
    "roller": {"name": ROLLER_TASK, "positions": 35, "velocities": 32, "observations": 56},
}
OBSERVATION_SCALE = torch.tensor(
    [0.25] * 14 + [5.0] * 14 + [1.0] * 4 + [10.0] * 3 + [10.0] * 3,
    dtype=torch.float32,
)
BALL_OBSERVATION_SCALE = torch.cat((
    OBSERVATION_SCALE,
    torch.tensor([0.072, 0.072, 0.072, 1.0, 1.0, 1.0, 5.0, 5.0, 5.0]),
))
SEESAW_OBSERVATION_SCALE = torch.cat((
    OBSERVATION_SCALE,
    torch.tensor([0.05, 0.05, 0.05] + [1.0] * 4 + [5.0] * 3 + [1.0] * 3 + [0.05, 0.05] + [1.0] * 3),
))


def body_pose(environment: dict, name: str) -> dict:
    body = next((body for body in environment["bodyPoses"] if body["name"] == name), None)
    if body is None:
        raise ValueError(f"physics state is missing the {name} body pose")
    return body


def tilt_cosine(quaternion) -> float:
    x, y = float(quaternion[1]), float(quaternion[2])
    return 1.0 - 2.0 * (x * x + y * y)


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


BINARY_STATE_MAGIC = b"RLPS"
BINARY_COMMAND_MAGIC = b"RLPC"
BINARY_VERSION = 1
BODY_POSE_VALUES = 13


def payload_bytes(sample) -> bytes:
    payload = sample.payload
    if hasattr(payload, "to_bytes"):
        payload = payload.to_bytes()
    return bytes(payload)


def decode_binary_state(data: bytes) -> dict:
    header = struct.Struct("<4sIQIIIIIII")
    (_, version, sequence, count, control_dimension, position_dimension, velocity_dimension,
     physics_steps, finished, body_count) = header.unpack_from(data, 0)
    if version != BINARY_VERSION:
        raise RuntimeError(f"unsupported binary physics state version {version}")
    offset = header.size

    def text() -> str:
        nonlocal offset
        (length,) = struct.unpack_from("<I", data, offset)
        offset += 4
        value = data[offset:offset + length].decode("utf-8")
        offset += length
        return value

    run_id = text()
    names = [text() for _ in range(body_count)]
    offset = (offset + 7) // 8 * 8

    def array(dtype, length):
        nonlocal offset
        values = np.frombuffer(data, dtype=dtype, count=length, offset=offset)
        offset += values.nbytes
        return values

    episodes = array("<u8", count).tolist()
    transitions = array("<u8", count).tolist()
    times = array("<f8", count).tolist()
    positions = array("<f4", count * position_dimension).reshape(count, position_dimension)
    velocities = array("<f4", count * velocity_dimension).reshape(count, velocity_dimension)
    bodies = array("<f4", count * body_count * BODY_POSE_VALUES).reshape(
        count, body_count, BODY_POSE_VALUES).astype(np.float64)
    step_ok = array("u1", count).astype(bool)
    if offset != len(data):
        raise RuntimeError("binary physics state has an unexpected size")
    arrays = {
        "positions": positions.astype(np.float64),
        "velocities": velocities.astype(np.float64),
        "bodies": {name: bodies[:, slot, :] for slot, name in enumerate(names)},
        "stepOk": step_ok,
    }
    step_ok_list = step_ok.tolist()
    environments = [
        {"environmentIndex": index, "episodeIndex": episodes[index],
         "transitionIndex": transitions[index], "simulationTime": times[index],
         "stepOk": step_ok_list[index]}
        for index in range(count)
    ]
    return {
        "schema": 2, "kind": "state", "binary": True, "runId": run_id, "sequence": sequence,
        "controlDimension": control_dimension, "positionDimension": position_dimension,
        "velocityDimension": velocity_dimension, "physicsSteps": physics_steps,
        "finished": bool(finished), "environments": environments, "arrays": arrays,
    }


HOME_ARRAY = np.asarray(HOME, dtype=np.float64)


def required_body(arrays: dict, name: str) -> np.ndarray:
    body = arrays["bodies"].get(name)
    if body is None:
        raise ValueError(f"physics state is missing the {name} body pose")
    return body


def observe_batch(arrays: dict, mode: str = "flat") -> np.ndarray:
    positions = arrays["positions"]
    velocities = arrays["velocities"]
    if (positions.shape[1] != TASKS[mode]["positions"]
            or velocities.shape[1] != TASKS[mode]["velocities"]):
        raise ValueError(
            f"Microduck state dimensions are {positions.shape[1]} positions and "
            f"{velocities.shape[1]} velocities; expected {TASKS[mode]['positions']} and "
            f"{TASKS[mode]['velocities']}"
        )
    trunk = required_body(arrays, "trunk_base")
    columns = [positions[:, 7:21] - HOME_ARRAY, velocities[:, 6:20], trunk[:, 3:13]]
    if mode in SEESAW_MODES:
        config = SEESAW_MODES[mode]
        plank = required_body(arrays, "seesaw_plank")
        support = required_body(arrays, config["support"])
        offset = trunk[:, 0:3] - plank[:, 0:3]
        offset[:, 2] -= config["stand_height"] - config["plank_height"]
        columns += [offset, plank[:, 3:13], support[:, 0:2] - plank[:, 0:2], support[:, 10:13]]
    if mode == "ball":
        ball = required_body(arrays, "balance_ball")
        offset = trunk[:, 0:3] - ball[:, 0:3]
        offset[:, 2] -= BALL_STAND_HEIGHT - BALL_RADIUS
        columns += [offset, ball[:, 10:13], ball[:, 7:10]]
    scales = {
        "flat": OBSERVATION_SCALE, "ball": BALL_OBSERVATION_SCALE, "seesaw": SEESAW_OBSERVATION_SCALE,
        "roller": SEESAW_OBSERVATION_SCALE,
    }[mode].numpy().astype(np.float64)
    values = np.concatenate(columns, axis=1)
    return np.clip(values / scales, -10.0, 10.0).astype(np.float32)


def outcome_batch(arrays: dict, controls: np.ndarray, episode_steps: np.ndarray,
                  mode: str = "flat") -> dict:
    positions = arrays["positions"]
    velocities = arrays["velocities"]
    trunk = required_body(arrays, "trunk_base")
    upright = 1.0 - 2.0 * (trunk[:, 4] ** 2 + trunk[:, 5] ** 2)
    height = trunk[:, 2]
    with np.errstate(invalid="ignore", over="ignore"):
        joint_error = np.mean((positions[:, 7:21] - HOME_ARRAY) ** 2, axis=1)
        joint_speed = np.sum(velocities[:, 6:20] ** 2, axis=1) / len(HOME)
        body_speed = np.sum(trunk[:, 7:13] ** 2, axis=1)
        effort = np.mean((controls - HOME_ARRAY) ** 2, axis=1)
        finite = (np.isfinite(positions).all(axis=1) & np.isfinite(velocities).all(axis=1)
                  & np.isfinite(controls).all(axis=1))
        fallen = ~finite | (height < 0.075) | (upright < 0.45)
        truncated = episode_steps >= EPISODE_LIMIT
        reward = 2.0 * upright + 2.0 * np.minimum(height / 0.12, 1.0)
        reward -= 0.35 * joint_error + 0.015 * joint_speed + 0.04 * body_speed + 0.01 * effort
        if mode in SEESAW_MODES:
            config = SEESAW_MODES[mode]
            trunk_above_plank = config["stand_height"] - config["plank_height"]
            plank = required_body(arrays, "seesaw_plank")
            support = required_body(arrays, config["support"])
            ball_speed = np.sum(support[:, 10:13] ** 2, axis=1)
            plank_level = 1.0 - 2.0 * (plank[:, 4] ** 2 + plank[:, 5] ** 2)
            plank_tilt = np.arccos(np.clip(plank_level, -1.0, 1.0))
            dx = trunk[:, 0] - plank[:, 0]
            dy = trunk[:, 1] - plank[:, 1]
            trunk_offset = np.hypot(dx, dy)
            relative_height = height - plank[:, 2]
            ball_offset = np.hypot(support[:, 0] - plank[:, 0], support[:, 1] - plank[:, 1])
            plank_spin = np.sum(plank[:, 7:10] ** 2, axis=1)
            fallen = (fallen | (relative_height < 0.09) | (plank_tilt > 0.35)
                      | (np.abs(dx) > 0.06) | (np.abs(dy) > 0.1) | (ball_offset > 0.1))
            reward -= 2.0 * np.minimum(height / STAND_HEIGHT, 1.0)
            reward += 2.0 * np.clip(relative_height / trunk_above_plank, 0.0, 1.0)
            reward += 2.0 * np.exp(-((plank_tilt / 0.1) ** 2))
            reward += 1.0 * np.exp(-((trunk_offset / 0.06) ** 2))
            reward += 1.5 * np.exp(-((ball_offset / 0.05) ** 2))
            reward -= 0.2 * ball_speed + 0.05 * plank_spin
        if mode == "ball":
            ball = required_body(arrays, "balance_ball")
            dx = trunk[:, 0] - ball[:, 0]
            dy = trunk[:, 1] - ball[:, 1]
            radial = np.hypot(dx, dy)
            relative_height = height - ball[:, 2]
            ball_speed = np.sum(ball[:, 10:13] ** 2, axis=1)
            ball_speed += 0.01 * np.sum(ball[:, 7:10] ** 2, axis=1)
            fallen = fallen | (relative_height < 0.12) | (radial > 0.11)
            reward -= 2.0 * np.minimum(height / STAND_HEIGHT, 1.0)
            reward += 2.0 * np.clip(relative_height / (BALL_STAND_HEIGHT - BALL_RADIUS), 0.0, 1.0)
            reward += 1.5 * np.exp(-((radial / 0.05) ** 2))
            reward -= 0.2 * ball_speed
        reward = np.where(fallen, reward - 5.0, reward)
        reward = np.where(finite, reward, -10.0)
    step_ok = arrays["stepOk"]
    reward = np.where(step_ok, reward, -10.0)
    terminated = fallen | ~step_ok
    succeeded = truncated & ~terminated
    return {"reward": reward, "terminated": terminated, "truncated": succeeded,
            "success": succeeded}


def encode_binary_step(run_id: str, sequence: int, controls, outcomes) -> bytes:
    control_array = np.asarray(controls, dtype="<f4")
    count, control_dimension = control_array.shape
    run_bytes = run_id.encode("utf-8")
    parts = [struct.pack("<4sIQIII", BINARY_COMMAND_MAGIC, BINARY_VERSION, sequence, count,
                         control_dimension, 1 if outcomes is not None else 0),
             struct.pack("<I", len(run_bytes)), run_bytes]
    header_length = sum(len(part) for part in parts)
    parts.append(b"\0" * ((4 - header_length % 4) % 4))
    parts.append(control_array.tobytes())
    if outcomes is not None:
        parts.append(np.asarray([item["reward"] for item in outcomes], dtype="<f4").tobytes())
        flags = [
            (1 if item["terminated"] else 0) | (2 if item["truncated"] else 0)
            | (4 if item["success"] else 0)
            for item in outcomes
        ]
        parts.append(np.asarray(flags, dtype="u1").tobytes())
    return b"".join(parts)


def observe(environment: dict, mode: str = "flat") -> list[float]:
    positions = environment["positions"]
    velocities = environment["velocities"]
    expected_positions = TASKS[mode]["positions"]
    expected_velocities = TASKS[mode]["velocities"]
    if len(positions) != expected_positions or len(velocities) != expected_velocities:
        raise ValueError(
            f"Microduck state dimensions are {len(positions)} positions and "
            f"{len(velocities)} velocities; expected {expected_positions} and {expected_velocities}"
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
    if mode in SEESAW_MODES:
        config = SEESAW_MODES[mode]
        plank = body_pose(environment, "seesaw_plank")
        support = body_pose(environment, config["support"])
        support_x, support_y = support["position"][0], support["position"][1]
        support_velocity = support["linearVelocity"]
        values.extend([
            trunk["position"][0] - plank["position"][0],
            trunk["position"][1] - plank["position"][1],
            trunk["position"][2] - plank["position"][2]
            - (config["stand_height"] - config["plank_height"]),
            *plank["quaternion"],
            *plank["angularVelocity"],
            *plank["linearVelocity"],
            support_x - plank["position"][0],
            support_y - plank["position"][1],
            *support_velocity,
        ])
    if mode == "ball":
        ball = body_pose(environment, "balance_ball")
        values.extend([
            trunk["position"][0] - ball["position"][0],
            trunk["position"][1] - ball["position"][1],
            trunk["position"][2] - ball["position"][2] - (BALL_STAND_HEIGHT - BALL_RADIUS),
            *ball["linearVelocity"],
            *ball["angularVelocity"],
        ])
    scales = {
        "flat": OBSERVATION_SCALE, "ball": BALL_OBSERVATION_SCALE, "seesaw": SEESAW_OBSERVATION_SCALE,
        "roller": SEESAW_OBSERVATION_SCALE,
    }[mode].tolist()
    return [max(-10.0, min(10.0, value / scale)) for value, scale in zip(values, scales)]


def reset_state(
    environment: dict, seed: int, episode: int, disturbance_scale: float,
    mode: str = "flat",
) -> tuple[list[float], list[float]]:
    rng = random.Random(seed + int(environment["environmentIndex"]) * 1000003 + episode * 9176)
    curriculum = min(1.0, 0.35 + episode / 200.0) * disturbance_scale
    if mode in SEESAW_MODES:
        curriculum *= 0.5
    roll = rng.uniform(-0.12, 0.12) * curriculum
    pitch = rng.uniform(-0.12, 0.12) * curriculum
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    quaternion = [cr * cp, sr * cp, cr * sp, -sr * sp]
    height = (
        SEESAW_MODES[mode]["stand_height"] if mode in SEESAW_MODES
        else BALL_STAND_HEIGHT if mode == "ball" else STAND_HEIGHT
    )
    positions = [0.0, 0.0, height, *quaternion, *HOME]
    positions[7 + ACTUATOR_INDEX["left_hip_roll"]] += rng.uniform(-0.04, 0.04) * curriculum
    positions[7 + ACTUATOR_INDEX["right_hip_roll"]] -= rng.uniform(-0.04, 0.04) * curriculum
    velocities = [0.0] * 20
    velocities[4] = rng.uniform(-0.15, 0.15) * curriculum
    velocities[5] = rng.uniform(-0.15, 0.15) * curriculum
    if mode in SEESAW_MODES:
        config = SEESAW_MODES[mode]
        positions[0] = rng.uniform(-0.006, 0.006) * curriculum
        positions[1] = rng.uniform(-0.006, 0.006) * curriculum
        positions.extend(config["support_pose"])
        velocities.extend([0.0] * 6)
        positions.extend([0.0, 0.0, config["plank_height"], 1.0, 0.0, 0.0, 0.0])
        velocities.extend([
            0.0, 0.0, 0.0,
            rng.uniform(-0.1, 0.1) * curriculum,
            rng.uniform(-0.1, 0.1) * curriculum,
            0.0,
        ])
    if mode == "ball":
        positions[0] = rng.uniform(-0.006, 0.006) * curriculum
        positions[1] = rng.uniform(-0.006, 0.006) * curriculum
        positions.extend([0.0, 0.0, BALL_RADIUS, 1.0, 0.0, 0.0, 0.0])
        velocities.extend([
            rng.uniform(-0.02, 0.02) * curriculum,
            rng.uniform(-0.02, 0.02) * curriculum,
            0.0,
            rng.uniform(-0.12, 0.12) * curriculum,
            rng.uniform(-0.12, 0.12) * curriculum,
            0.0,
        ])
    return positions, velocities


def outcome(
    environment: dict, controls: list[float], episode_steps: int,
    mode: str = "flat",
) -> dict:
    trunk = next(body for body in environment["bodyPoses"] if body["name"] == "trunk_base")
    x, y = float(trunk["quaternion"][1]), float(trunk["quaternion"][2])
    upright = 1.0 - 2.0 * (x * x + y * y)
    height = float(trunk["position"][2])
    joint_error = sum(
        (position - home) ** 2
        for position, home in zip(environment["positions"][7:21], HOME)
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
    if mode in SEESAW_MODES:
        config = SEESAW_MODES[mode]
        trunk_above_plank = config["stand_height"] - config["plank_height"]
        plank = body_pose(environment, "seesaw_plank")
        support = body_pose(environment, config["support"])
        support_x, support_y = float(support["position"][0]), float(support["position"][1])
        ball_speed = sum(float(value) ** 2 for value in support["linearVelocity"])
        plank_level = tilt_cosine(plank["quaternion"])
        plank_tilt = math.acos(max(-1.0, min(1.0, plank_level)))
        dx = float(trunk["position"][0]) - float(plank["position"][0])
        dy = float(trunk["position"][1]) - float(plank["position"][1])
        trunk_offset = math.hypot(dx, dy)
        relative_height = height - float(plank["position"][2])
        ball_offset = math.hypot(
            support_x - float(plank["position"][0]), support_y - float(plank["position"][1])
        )
        plank_spin = sum(float(value) ** 2 for value in plank["angularVelocity"])
        fallen = (
            fallen or relative_height < 0.09 or plank_tilt > 0.35
            or abs(dx) > 0.06 or abs(dy) > 0.1 or ball_offset > 0.1
        )
        reward -= 2.0 * min(height / STAND_HEIGHT, 1.0)
        reward += 2.0 * max(0.0, min(relative_height / trunk_above_plank, 1.0))
        reward += 2.0 * math.exp(-((plank_tilt / 0.1) ** 2))
        reward += 1.0 * math.exp(-((trunk_offset / 0.06) ** 2))
        reward += 1.5 * math.exp(-((ball_offset / 0.05) ** 2))
        reward -= 0.2 * ball_speed + 0.05 * plank_spin
    if mode == "ball":
        ball = body_pose(environment, "balance_ball")
        dx = float(trunk["position"][0]) - float(ball["position"][0])
        dy = float(trunk["position"][1]) - float(ball["position"][1])
        radial = math.hypot(dx, dy)
        relative_height = height - float(ball["position"][2])
        ball_speed = sum(float(value) ** 2 for value in ball["linearVelocity"])
        ball_speed += 0.01 * sum(float(value) ** 2 for value in ball["angularVelocity"])
        fallen = fallen or relative_height < 0.12 or radial > 0.11
        reward -= 2.0 * min(height / STAND_HEIGHT, 1.0)
        reward += 2.0 * max(0.0, min(relative_height / (BALL_STAND_HEIGHT - BALL_RADIUS), 1.0))
        reward += 1.5 * math.exp(-((radial / 0.05) ** 2))
        reward -= 0.2 * ball_speed
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
    train_actor: bool = True,
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
            loss = policy_loss + value_loss - 0.01 * entropy if train_actor else value_loss
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            nn.utils.clip_grad_norm_([*model.actor.parameters(), model.log_std], 0.5)
            nn.utils.clip_grad_norm_(model.critic.parameters(), 0.5)
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
    parser.add_argument("--initialize-from", type=Path)
    parser.add_argument("--ball-balance", action="store_true")
    parser.add_argument("--seesaw-balance", action="store_true")
    parser.add_argument("--seesaw-roller", action="store_true")
    parser.add_argument("--roller-radius", type=float, default=0.05)
    parser.add_argument("--ball-radius", type=float, default=SEESAW_BALL_RADIUS)
    parser.add_argument("--critic-warmup", type=int, default=0)
    parser.add_argument("--ready-file", type=Path)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--disturbance-scale", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=170.0)
    parser.add_argument("--update-epochs", type=int, default=8)
    parser.add_argument("--minibatch-size", type=int, default=256)
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--evaluation-report", type=Path)
    args = parser.parse_args()
    if args.steps <= 0 or args.rollout_steps <= 0 or args.timeout <= 0:
        parser.error("steps, rollout steps, and timeout must be positive")
    if not 0.1 <= args.disturbance_scale <= 3.0:
        parser.error("disturbance scale must be between 0.1 and 3.0")
    if args.resume_from and args.initialize_from:
        parser.error("--resume-from and --initialize-from cannot be combined")
    if sum((args.ball_balance, args.seesaw_balance, args.seesaw_roller)) > 1:
        parser.error("--ball-balance, --seesaw-balance, and --seesaw-roller cannot be combined")
    mode = (
        "seesaw" if args.seesaw_balance else "roller" if args.seesaw_roller
        else "ball" if args.ball_balance else "flat"
    )
    if not 0.02 <= args.roller_radius <= 0.3:
        parser.error("roller radius must be between 0.02 and 0.3 meters")
    if not 0.02 <= args.ball_radius <= 0.3:
        parser.error("ball radius must be between 0.02 and 0.3 meters")
    if mode == "seesaw":
        SEESAW_MODES["seesaw"].update({
            "plank_height": 2.0 * args.ball_radius + 0.005,
            "stand_height": 2.0 * args.ball_radius + 0.127,
            "support_pose": (0.0, 0.0, args.ball_radius, 1.0, 0.0, 0.0, 0.0),
        })
    if mode == "roller":
        SEESAW_MODES["roller"].update({
            "plank_height": 2.0 * args.roller_radius + 0.005,
            "stand_height": 2.0 * args.roller_radius + 0.130,
            "support_pose": (0.0, 0.0, args.roller_radius, 1.0, 0.0, 0.0, 0.0),
        })
    if args.initialize_from and mode == "flat":
        parser.error("--initialize-from requires a ball, seesaw, or roller task")
    if not torch.cuda.is_available():
        print("CUDA is required for Microduck PPO training; no CPU fallback is enabled.", file=sys.stderr)
        return 2
    random.seed(args.seed)
    torch.manual_seed(args.seed)
    torch.cuda.manual_seed_all(args.seed)
    device = torch.device("cuda")
    task_name = TASKS[mode]["name"]
    observation_dim = TASKS[mode]["observations"]
    model = ActorCritic(observation_dim, len(ACTUATORS)).to(device)
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
    initialize_path = args.initialize_from
    if initialize_path is not None:
        if args.evaluate:
            parser.error("--initialize-from cannot be combined with --evaluate")
        if not initialize_path.is_absolute():
            initialize_path = project_root / initialize_path
        initialize_path = initialize_path.resolve()
        source = torch.load(initialize_path, map_location=device, weights_only=False)
        from_flat = source.get("task") == FLAT_TASK and source.get("observation_dim") == 38
        from_seesaw = (
            mode in SEESAW_MODES and source.get("task") in (SEESAW_TASK, ROLLER_TASK)
            and source.get("observation_dim") == observation_dim
        )
        if (not (from_flat or from_seesaw)
                or source.get("action_dim") != len(ACTUATORS)
                or source.get("actuators") != list(ACTUATORS)
                or source.get("home_controls") != list(HOME)
                or source.get("action_scale") != ACTION_SCALE):
            raise ValueError(f"initial policy is incompatible with the Microduck {mode}-balance task")
        initialized = model.state_dict()
        for name, target in initialized.items():
            if name == "actor.0.weight" and from_seesaw:
                target.copy_(source["model"][name])
            elif name == "actor.0.weight":
                target[:, :38] = source["model"][name]
                target[:, 38:] = 0
            elif name.startswith("actor.") or name == "log_std":
                target.copy_(source["model"][name])
        model.load_state_dict(initialized)
    global_step_offset = 0
    previous_metrics = None
    if resume_path is not None:
        checkpoint = torch.load(resume_path, map_location=device, weights_only=False)
        if checkpoint.get("task") != task_name:
            raise ValueError("checkpoint does not belong to the selected Microduck task")
        if (checkpoint.get("observation_dim") != observation_dim
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
            torch.set_rng_state(torch.as_tensor(
                checkpoint["torch_rng_state"], dtype=torch.uint8, device="cpu"
            ).contiguous())
        if "cuda_rng_state" in checkpoint:
            torch.cuda.set_rng_state_all([
                torch.as_tensor(state, dtype=torch.uint8, device="cpu").contiguous()
                for state in checkpoint["cuda_rng_state"]
            ])
    if args.evaluate:
        checkpoint = torch.load(checkpoint_path, map_location=device, weights_only=False)
        if checkpoint.get("task") != task_name:
            raise ValueError("checkpoint does not belong to the selected Microduck task")
        model.load_state_dict(checkpoint["model"])
        model.eval()
    metrics_path = checkpoint_path.parent / "training.json"
    checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
    metrics = previous_metrics or {
        "task": task_name, "device": str(device), "seed": args.seed, "updates": []
    }
    metrics["device"] = str(device)
    metrics["last_seed"] = args.seed
    metrics["last_disturbance_scale"] = args.disturbance_scale
    if initialize_path is not None:
        metrics["initialized_from"] = str(initialize_path)
    metrics.setdefault("updates", [])
    if resume_path is not None:
        metrics.setdefault("stages", []).append({
            "resume_from": str(resume_path), "additional_steps": args.steps,
            "seed": args.seed, "disturbance_scale": args.disturbance_scale
        })
    states: queue.Queue = queue.Queue(maxsize=2)
    callback_errors: queue.Queue = queue.Queue(maxsize=1)
    session = zenoh.open(zenoh.Config())
    publisher = None

    def receive(sample) -> None:
        try:
            raw = payload_bytes(sample)
            if raw[:4] == BINARY_STATE_MAGIC:
                payload = decode_binary_state(raw)
            else:
                payload = json.loads(raw.decode("utf-8"))
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
    binary_commands = False

    def next_state() -> dict:
        nonlocal binary_commands
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
        binary_commands = bool(state.get("binary"))
        return state

    def send(command: dict) -> None:
        nonlocal expected_sequence
        if binary_commands and command.get("kind") == "step":
            publisher.put(encode_binary_step(run_id, expected_sequence, command["controls"],
                                             command.get("outcomes")))
        else:
            if isinstance(command.get("controls"), np.ndarray):
                command["controls"] = command["controls"].tolist()
            command.update({"schema": 2, "runId": run_id, "sequence": expected_sequence})
            publisher.put(json.dumps(command, separators=(",", ":")))
        expected_sequence += 1

    def save_checkpoint(step_count: int, update_record: dict) -> None:
        temporary = checkpoint_path.with_suffix(checkpoint_path.suffix + ".tmp")
        metrics["steps"] = global_step_offset + step_count
        metrics["updates"].append(update_record)
        torch.save({
            "schema": 1,
            "task": task_name,
            "observation_dim": observation_dim,
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
        expected_positions = TASKS[mode]["positions"]
        expected_velocities = TASKS[mode]["velocities"]
        if (state.get("positionDimension") != expected_positions
                or state.get("velocityDimension") != expected_velocities):
            raise ValueError(
                f"selected MuJoCo model has {state.get('positionDimension')} positions and "
                f"{state.get('velocityDimension')} velocities; expected "
                f"{expected_positions} and {expected_velocities}"
            )
        run_id = state["runId"]
        publisher = session.declare_publisher(f"training/{run_id}/external/physics/command")
        environments = state["environments"]
        resets = []
        for environment in environments:
            positions, velocities = reset_state(
                environment, args.seed, int(environment["episodeIndex"]),
                args.disturbance_scale, mode,
            )
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
        recent_lengths: list[int] = []
        update_count = 0
        recent_successes = 0
        rollout = {name: [] for name in (
            "observations", "latents", "log_probs", "values", "rewards",
            "next_values", "terminated", "done",
        )}
        controls = None
        pending_outcomes = None
        step_count = 0
        started = time.monotonic()

        while step_count < args.steps:
            arrays = state.get("arrays")
            if arrays is not None:
                observation_array = observe_batch(arrays, mode)
                observation_tensor = torch.as_tensor(observation_array, device=device)
                observations = None if args.evaluate else observation_array.tolist()
            else:
                observations = [
                    observe(environment, mode) for environment in state["environments"]
                ]
                observation_tensor = normalize_observations(observations, device)
            with torch.no_grad():
                distribution = model.distribution(observation_tensor)
                latent = distribution.mean if args.evaluate else distribution.sample()
                actions = torch.tanh(latent)
                log_probs = action_log_prob(distribution, latent)
                values = model.value(observation_tensor)
            controls = HOME_ARRAY + ACTION_SCALE * actions.clamp(-1.0, 1.0).cpu().numpy().astype(
                np.float64)
            command = {"kind": "step", "controls": controls}
            if pending_outcomes is not None:
                command["outcomes"] = pending_outcomes
                pending_outcomes = None
            send(command)
            next_physics_state = next_state()
            next_environments = next_physics_state["environments"]
            next_arrays = next_physics_state.get("arrays")
            outcomes = []
            reset_rows = []
            if next_arrays is not None:
                episode_steps = [steps + 1 for steps in episode_steps]
                batch = outcome_batch(next_arrays, controls, np.asarray(episode_steps), mode)
                outcomes = [
                    {"reward": float(reward), "terminated": bool(terminated),
                     "truncated": bool(truncated), "success": bool(success)}
                    for reward, terminated, truncated, success in zip(
                        batch["reward"].tolist(), batch["terminated"].tolist(),
                        batch["truncated"].tolist(), batch["success"].tolist())
                ]
                for index, result in enumerate(outcomes):
                    episode_returns[index] += result["reward"]
            else:
                control_rows = controls.tolist()
                for index, environment in enumerate(next_environments):
                    episode_steps[index] += 1
                    result = outcome(
                        environment, control_rows[index], episode_steps[index], mode
                    )
                    if not environment.get("stepOk", True):
                        result = {"reward": -10.0, "terminated": True, "truncated": False,
                                  "success": False}
                    outcomes.append(result)
                    episode_returns[index] += result["reward"]
            if not args.evaluate:
                if next_arrays is not None:
                    next_tensor = torch.as_tensor(observe_batch(next_arrays, mode), device=device)
                else:
                    next_observations = [
                        observe(environment, mode) for environment in next_environments
                    ]
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
                    recent_lengths.append(episode_steps[index])
                    recent_successes += int(result["success"])
                    episode_returns[index] = 0.0
                    episode_steps[index] = 0
                    positions, velocities = reset_state(
                        next_environments[index], args.seed,
                        int(next_environments[index]["episodeIndex"]) + 1,
                        args.disturbance_scale, mode,
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
                    train_actor=update_count >= args.critic_warmup,
                )
                update_count += 1
                update_record.update({
                    "step": global_step_offset + step_count,
                    "completed_episodes": len(completed_episodes),
                    "mean_completed_episode_return": (
                        sum(completed_episodes) / len(completed_episodes) if completed_episodes else None
                    ),
                    "mean_completed_episode_length": (
                        sum(completed_episode_lengths) / len(completed_episode_lengths)
                        if completed_episode_lengths else None
                    ),
                    "success_rate": (
                        successful_episodes / len(completed_episodes) if completed_episodes else None
                    ),
                    "recent_episodes": len(recent_lengths),
                    "recent_episode_length": (
                        sum(recent_lengths) / len(recent_lengths) if recent_lengths else None
                    ),
                    "recent_success_rate": (
                        recent_successes / len(recent_lengths) if recent_lengths else None
                    ),
                    "elapsed_seconds": time.monotonic() - started,
                })
                recent_lengths = []
                recent_successes = 0
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
                        "task": task_name,
                        "checkpoint": (
                            checkpoint_path.relative_to(project_root).as_posix()
                            if checkpoint_path.is_relative_to(project_root) else str(checkpoint_path)
                        ),
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
