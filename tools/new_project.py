"""Builds a RobotLens project from a spec over MCP; each project's scripts/build_project.py calls build().

The spec names the communication backend and simulator, the files to copy from this repository,
an optional world, and each robot with its pose, 2D LiDAR, and navigation settings. RobotLens
creates the project, imports the models, places the robots, attaches and configures the LiDARs,
configures navigation, and saves the project. Nothing in .robotlens/ is written by hand.

The build needs an empty RobotLens session: it uses the running one, or starts RobotLens
without a project and stops it at the end. It refuses to overwrite an existing project.
"""

from __future__ import annotations

import argparse
import shutil
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

from quickstart import (ROBOTLENS, SHUTDOWN_TIMEOUT_S, SOCKET_TIMEOUT_S, McpClient,
                        RobotLensInstance, RunContext, live_sessions, run_command, wait_until)

REPOSITORY = Path(__file__).resolve().parents[1]
LIDAR_DEFAULTS = {"lidar_simulation": "ray_based", "lidar_horizontal_fov_deg": 360.0,
                  "range_min": 0.12, "range_max": 12.0}


@dataclass
class Lidar:
    link: str
    topic: str
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    properties: dict = field(default_factory=dict)


@dataclass
class Robot:
    file: str
    pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
    lidar: Lidar | None = None
    navigation: dict | None = None


@dataclass
class Spec:
    root: Path
    communication: str
    simulator: str
    robots: tuple[Robot, ...]
    world: str = ""
    copy: dict[str, str] = field(default_factory=dict)


class BuildError(RuntimeError):
    pass


def step(label: str, result: dict) -> dict:
    if not result.get("ok", False):
        raise BuildError(f"{label}: {result.get('error')} {result.get('message', '')}".strip())
    print(f"  {label}")
    return result


def copy_files(spec: Spec) -> None:
    for target, source in spec.copy.items():
        destination, origin = spec.root / target, REPOSITORY / source
        if destination.exists():
            continue
        destination.parent.mkdir(parents=True, exist_ok=True)
        if origin.is_dir():
            shutil.copytree(origin, destination)
        else:
            shutil.copy2(origin, destination)
        print(f"  copied {source} -> {target}")


def import_model(ctx: RunContext, path: str) -> dict:
    step(f"import {path}", run_command(ctx, "model.import", {"path": path}))
    status = wait_until(lambda: (lambda s: s if s.get("state") == "idle" and
                                 s.get("last_request") == path else None)(
        run_command(ctx, "model.status")), 120.0, 0.5)
    if not status or not status.get("imported"):
        raise BuildError(f"import {path} did not finish: {status}")
    return status["imported"][-1]


def find_alias(ctx: RunContext, path: str) -> str:
    found = run_command(ctx, "scene.find", {"target": path, "limit": 100})
    for item in found.get("items") or found.get("candidates") or []:
        if item.get("path") == path:
            return item["alias"]
    raise BuildError(f"{path} not found in the scene")


def add_lidar(ctx: RunContext, robot: str, lidar: Lidar) -> None:
    created = step(f"{robot} 2D LiDAR created",
                   run_command(ctx, "create.object", {"action": "sensors.2d_lidar"}))["created"][0]
    alias = created["alias"]
    link = find_alias(ctx, f"Robots/{robot}/{lidar.link}")
    step(f"{created['name']} attached to {lidar.link}",
         run_command(ctx, "scene.set_parent", {"target": alias, "parent": link}))
    x, y, z = lidar.xyz
    step(f"{created['name']} at {lidar.xyz} on {lidar.link}",
         run_command(ctx, "scene.move", {"target": alias, "x": x, "y": y, "z": z}))
    properties = {**LIDAR_DEFAULTS, **lidar.properties, "topic": lidar.topic, "enabled": True}
    step(f"{created['name']} publishes {lidar.topic}",
         run_command(ctx, "scene.set_properties", {"target": alias, "properties": properties}))


def build_project(ctx: RunContext, spec: Spec) -> None:
    step("project created", run_command(ctx, "project.create", {
        "path": str(spec.root), "communication": spec.communication,
        "simulator": spec.simulator}))
    if not wait_until(lambda: (lambda s: s if not s.get("loading") and s.get("project_path")
                               else None)(run_command(ctx, "project.status")), 60.0, 0.5):
        raise BuildError("the new project did not open")
    run_command(ctx, "ui.splash", {"visible": False})
    if spec.world:
        import_model(ctx, spec.world)
    for robot in spec.robots:
        model = import_model(ctx, robot.file)
        name, alias = model["name"], model["alias"]
        x, y, yaw = robot.pose
        step(f"{name} at ({x}, {y})", run_command(ctx, "scene.move",
                                                  {"target": alias, "x": x, "y": y}))
        if yaw:
            step(f"{name} yaw {yaw} deg", run_command(ctx, "scene.rotate",
                                                      {"target": alias, "z": yaw}))
        if robot.lidar:
            add_lidar(ctx, name, robot.lidar)
        if robot.navigation:
            step(f"{name} navigation configured", run_command(
                ctx, "robot.navigation.configure", {"target": alias, **robot.navigation}))
    step("project saved", run_command(ctx, "project.save"))


def build(spec: Spec) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--keep-running", action="store_true",
                        help="leave RobotLens running with the new project open")
    args = parser.parse_args()
    if (spec.root / ".robotlens").exists():
        print(f"{spec.root.name} already has a project; remove .robotlens/ to rebuild it.")
        return 1
    print(f"Building {spec.root.name}")
    copy_files(spec)
    sessions = live_sessions()
    instance = None
    if sessions:
        socket = sessions[0].socket
        print(f"Using live RobotLens session {sessions[0].pid}")
    else:
        log = spec.root / "evidence" / "build" / "robotlens.log"
        instance = RobotLensInstance(ROBOTLENS, None, log)
        if not instance.wait_for_socket(SOCKET_TIMEOUT_S):
            print("RobotLens MCP session did not start")
            return 1
        socket = instance.socket
        print(f"Started RobotLens pid {instance.pid}")
    try:
        with McpClient(socket_path=str(socket), timeout=60.0) as client:
            build_project(RunContext(client, spec.root.name, 16.6), spec)
    except BuildError as error:
        print(f"FAIL {error}")
        return 1
    finally:
        if instance and not args.keep_running:
            time.sleep(1.0)
            instance.stop(SHUTDOWN_TIMEOUT_S)
    print(f"{spec.root.name}: built")
    return 0


if __name__ == "__main__":
    sys.exit("Run a project's scripts/build_project.py.")
