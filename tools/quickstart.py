"""Shared setup and smoke test for RobotLens projects; each project's scripts/quickstart.py calls run().

setup  checks RobotLens, ROS 2/Gazebo or Zenoh prerequisites and project files, then runs the
       project's own preparation steps.
test   uses the live RobotLens session (or starts one), opens the project, closes the splash
       screen, starts communication, plays the simulation, checks the robots and their TF, saves
       evidence/quickstart/quickstart.png, and stops everything in order: navigation, simulation,
       communication.
run    does what test does, then starts the project's stacks and runs its scenario (navigation
       goals, arm targets) with pass checks measured from ROS 2 or Zenoh, and writes
       evidence/quickstart/report.json.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[2]
ROBOTLENS = WORKSPACE / "RobotLens"
sys.path.insert(0, str(WORKSPACE / "RobotLens-Automation" / "scripts"))

from harness_lifecycle import ordered_stop  # noqa: E402
from mcp_client import McpClient, live_sessions  # noqa: E402
from orchestrate import (BINARY, ROS_SETUP, SHUTDOWN_TIMEOUT_S, SOCKET_TIMEOUT_S,  # noqa: E402
                         RobotLensInstance)
from run_mcp_tests import RunContext, SuiteResult, print_results, run_command, wait_until  # noqa: E402

PNG_MAGIC = b"\x89PNG\r\n\x1a\n"
TOOLS = Path(__file__).resolve().parent
GOAL_PROBES = {"easynav": WORKSPACE / "RobotLens-Automation" / "scripts" / "easynav_ros_probe.py",
               "nav2": WORKSPACE / "RobotLens-Automation" / "scripts" / "nav2_ros_probe.py"}
ZENOH_PYTHON = WORKSPACE / "RobotLens-Automation" / ".venv" / "bin" / "python"
TERMINAL = ("succeeded", "failed", "cancelled")


@dataclass
class Project:
    root: Path
    comms: str
    simulator: str
    robots: tuple[str, ...] = ()
    frames: tuple[str, ...] = ()
    requires: tuple[str, ...] = ()
    commands: tuple[str, ...] = ()
    setup_steps: tuple[tuple[str, str], ...] = ()
    prepare: str = ""
    navigation: bool = False
    stacks: str = ""
    before_simulation: tuple[dict, ...] = ()
    scenario: tuple[dict, ...] = ()

    @property
    def project_file(self) -> Path:
        return next((self.root / ".robotlens").glob("*.xml"))


def shell(command: str, cwd: Path, ros: bool, timeout: float) -> subprocess.CompletedProcess:
    if ros:
        command = f'source "{ROS_SETUP}" && {command}'
    return subprocess.run(["bash", "-c", command], cwd=cwd, capture_output=True, text=True,
                          timeout=timeout)


def setup(project: Project) -> SuiteResult:
    out = SuiteResult("setup")
    start = time.monotonic()
    out.expect("RobotLens is built", (ROBOTLENS / BINARY).exists(),
               f"run ./build.sh in {ROBOTLENS}")
    out.expect("project file present", project.project_file.exists(),
               str(project.project_file.relative_to(project.root)))
    if project.comms == "ros2" or project.simulator == "gazebo":
        out.expect("ROS 2 Jazzy installed", ROS_SETUP.exists(), str(ROS_SETUP))
    if project.simulator == "gazebo":
        out.expect("Gazebo (gz) installed", shutil.which("gz", path="/usr/bin:/bin") is not None,
                   "install Gazebo Harmonic")
    for path in project.requires:
        out.expect(f"{path} present", (project.root / path).exists(), str(project.root / path))
    for command in project.commands:
        found = shutil.which(command) is not None
        out.expect(f"{command} available", found, "not on PATH", required=False)
    if not out.failed:
        for label, command in project.setup_steps:
            done = shell(command, project.root, project.comms == "ros2", 3600)
            out.expect(label, done.returncode == 0,
                       (done.stdout + done.stderr).strip()[-600:])
    out.seconds = time.monotonic() - start
    return out


def open_project(ctx: RunContext, project: Project, out: SuiteResult) -> bool:
    status = wait_until(lambda: (lambda s: s if s.get("ok", True) and not s.get("loading")
                                 else None)(run_command(ctx, "project.status")), 120.0, 0.5) or {}
    current = status.get("project_path") or ""
    if not current.startswith(str(project.root)):
        opened = run_command(ctx, "project.open", {"path": str(project.root)})
        if not out.expect("project.open accepted", opened.get("ok", False), str(opened)):
            return False
    status = wait_until(lambda: (lambda s: s if not s.get("loading") and str(
        s.get("project_path") or "").startswith(str(project.root)) else None)(
            run_command(ctx, "project.status")), 120.0, 0.5) or {}
    if not out.expect("project open", bool(status), str(status.get("project_path"))):
        return False
    splash = run_command(ctx, "ui.splash", {"visible": False})
    return out.expect("splash screen closed",
                      splash.get("ok", False) and splash.get("visible") is False, str(splash))


def start_comms(ctx: RunContext, out: SuiteResult) -> bool:
    started = run_command(ctx, "comms.start")
    out.expect("comms.start accepted",
               started.get("ok", False) or started.get("error") == "already_running", str(started))
    status = wait_until(lambda: (lambda s: s if s.get("started") and s.get("connected") else None)(
        run_command(ctx, "comms.status")), 60.0, 0.5) or run_command(ctx, "comms.status")
    return out.expect("communication connected", bool(status.get("connected")),
                      f"{status.get('backend')} started={status.get('started')}")


def robot_alias(ctx: RunContext, robot: str) -> str:
    found = run_command(ctx, "scene.find", {"target": f"Robots/{robot}"})
    for item in found.get("candidates") or found.get("items") or []:
        if str(item.get("path", "")).rstrip("/").split("/")[-1] == robot:
            return item["alias"]
    return ""


def place_robots(ctx: RunContext, project: Project, out: SuiteResult) -> bool:
    ok = True
    for move in project.before_simulation:
        alias = robot_alias(ctx, move["move"])
        moved = run_command(ctx, "scene.move", {"target": alias, "x": move["x"], "y": move["y"]}) \
            if alias else {"ok": False, "error": "not_found"}
        ok &= out.expect(f"{move['move']} placed at ({move['x']}, {move['y']})",
                         moved.get("ok", False), str(moved))
    return ok


def play_simulation(ctx: RunContext, out: SuiteResult) -> bool:
    played = run_command(ctx, "simulation.play")
    out.expect("simulation.play accepted",
               played.get("ok", False) or played.get("error") == "already_running", str(played))
    last_play = time.monotonic()

    def running():
        nonlocal last_play
        status = run_command(ctx, "simulation.status")
        if status.get("status") in ("Running", "Attached") and not status.get("setup_running"):
            return status
        if status.get("status") == "Stopped" and not status.get("setup_running") and \
                time.monotonic() - last_play >= 4.0:
            run_command(ctx, "simulation.play")
            last_play = time.monotonic()
        return None

    status = wait_until(running, 180.0, 1.0) or run_command(ctx, "simulation.status")
    return out.expect("simulation running", status.get("status") in ("Running", "Attached"),
                      f"{status.get('status')} {status.get('last_error', '')}")


def check_scene(ctx: RunContext, project: Project, out: SuiteResult) -> None:
    models = {m.get("name") for m in run_command(ctx, "model.list").get("models", [])}
    for robot in project.robots:
        out.expect(f"robot {robot} loaded", robot in models, f"models: {sorted(models)}")
    for frame in project.frames:
        found = wait_until(lambda f=frame: (lambda r: r if r.get("found") else None)(
            run_command(ctx, "tf.lookup", {"frame": f})), 30.0, 1.0) or {}
        out.expect(f"TF frame {frame} resolved", bool(found), str(found.get("position_m")))


def start_navigation(ctx: RunContext, out: SuiteResult) -> None:
    started = run_command(ctx, "navigation.start_servers")
    if not out.expect("navigation.start_servers accepted", started.get("ok", False), str(started)):
        return
    status = wait_until(lambda: (lambda s: s if s.get("session_status") == "Running" else None)(
        run_command(ctx, "navigation.status")), 180.0, 1.0) or run_command(ctx, "navigation.status")
    out.expect("navigation servers running", status.get("session_status") == "Running",
               f"{status.get('stack')} {status.get('session_status')}")


def screenshot(ctx: RunContext, project: Project, out: SuiteResult, name: str = "quickstart",
               camera: dict | None = None) -> None:
    if camera:
        run_command(ctx, "view.set_camera", camera)
        time.sleep(0.5)
    shot = ctx.client.call("robotlens.capture_viewport_screenshot")
    png = shot.images[0] if shot.images else b""
    path = project.root / "evidence" / "quickstart" / f"{name}.png"
    if out.expect("viewport screenshot saved", png.startswith(PNG_MAGIC), str(shot.limitations)):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(png)


def probe_json(command: list[str], ros: bool, timeout: float) -> dict:
    line = " ".join(f"'{c}'" for c in command)
    if ros:
        line = f'source "{ROS_SETUP}" && exec {line}'
    try:
        done = subprocess.run(["bash", "-c", line], capture_output=True, text=True,
                              timeout=timeout + 20)
    except subprocess.TimeoutExpired:
        return {}
    for row in reversed(done.stdout.splitlines()):
        try:
            return json.loads(row)
        except json.JSONDecodeError:
            continue
    return {}


def truth_poses(project: Project, targets: list[str], timeout: float = 10.0) -> dict:
    if project.comms == "zenoh":
        report = probe_json([str(ZENOH_PYTHON), str(TOOLS / "zenoh_probe.py"), *targets,
                             "--timeout", str(timeout)], False, timeout)
        return {k: [v["x"], v["y"], 0.0, v.get("yaw", 0.0)] if v else None
                for k, v in report.items()}
    report = probe_json(["python3", str(TOOLS / "ros_probe.py"), "pose", "--parent", "map",
                         "--frames", *targets, "--timeout", str(timeout)], True, timeout)
    return report.get("frames", {})


def truth_joints(project: Project, source: str, timeout: float = 5.0) -> dict:
    if project.comms == "zenoh":
        report = probe_json([str(ZENOH_PYTHON), str(TOOLS / "zenoh_probe.py"), source,
                             "--timeout", str(timeout)], False, timeout)
        return report.get(source) or {}
    return probe_json(["python3", str(TOOLS / "ros_probe.py"), "joints", "--topic", source,
                       "--timeout", str(timeout)], True, timeout)


def gz(args: list[str], timeout: float = 15.0) -> str:
    line = f'source "{ROS_SETUP}" && exec gz ' + " ".join(f"'{a}'" for a in args)
    try:
        return subprocess.run(["bash", "-c", line], capture_output=True, text=True,
                              timeout=timeout).stdout
    except subprocess.TimeoutExpired:
        return ""


def gz_pose(model: str, link: str = "") -> list[float] | None:
    rows = gz(["model", "-m", model] + (["-l", link] if link else ["-p"])).splitlines()
    for index, row in enumerate(rows):
        if row.strip().startswith("- Pose [") and index + 2 < len(rows):
            try:
                xyz = [float(v) for v in rows[index + 1].strip(" []").split()]
                rpy = [float(v) for v in rows[index + 2].strip(" []").split()]
            except ValueError:
                return None
            return xyz + rpy
    return None


def gz_publish(command: dict) -> None:
    gz(["topic", "-t", command["topic"], "-m", command.get("type", "gz.msgs.Double"),
        "-p", command["payload"]])


def step_gz_motion(step: dict, out: SuiteResult) -> bool:
    checks = step["check"]
    before = [gz_pose(c["model"], c.get("link", "")) for c in checks]
    for command in step["publish"]:
        gz_publish(command)
    time.sleep(step.get("hold", 3.0))
    after = [gz_pose(c["model"], c.get("link", "")) for c in checks]
    for command in step.get("stop", []):
        gz_publish(command)
    ok = True
    for check, start, end in zip(checks, before, after):
        name = f"{check['model']}{'/' + check['link'] if check.get('link') else ''}"
        if not start or not end:
            ok &= out.expect(f"{name} pose read from Gazebo", False, f"{start} {end}")
            continue
        moved = math.dist(start[:3], end[:3])
        changes = [abs(math.remainder(b - a, math.tau)) for a, b in zip(start[3:], end[3:])]
        axis = ("roll", "pitch", "yaw").index(check["axis"]) if "axis" in check else None
        turned = changes[axis] if axis is not None else max(changes)
        passed = moved >= check.get("min_move", 0.0) and turned >= check.get("min_turn", 0.0)
        ok &= out.expect(f"{name} {check.get('label', 'moved')} (measured in Gazebo)", passed,
                         f"moved {moved:.3f} m, turned {turned:.3f} rad, "
                         f"at {[round(v, 2) for v in end]}")
    return ok


def nav_status(ctx: RunContext, robot: str) -> dict:
    return run_command(ctx, "robot.navigation.status", {"target": robot})


def stack_status(ctx: RunContext, prefix: str, stack: str) -> dict:
    entry = run_command(ctx, "motion_stack.status", {"prefix": prefix}).get(stack) or {}
    return entry.get("status") if entry.get("running") and isinstance(entry.get("status"), dict) \
        else {}


def find_result(value, command_id: str) -> str | None:
    if isinstance(value, dict):
        if value.get("command_id") == command_id and value.get("state") in TERMINAL:
            return value["state"]
        value = list(value.values())
    if isinstance(value, list):
        for item in value:
            found = find_result(item, command_id)
            if found:
                return found
    return None


def goal_state(ctx: RunContext, goal: dict) -> str:
    if "prefix" in goal:
        latest = run_command(ctx, "motion_stack.status", {"prefix": goal["prefix"]}).get("latest")
        return find_result(latest, goal.get("command_id", "")) or \
            stack_status(ctx, goal["prefix"], "navigation").get("state", "unknown") + " (no result)"
    return nav_status(ctx, goal["robot"]).get("state", "unknown")


def step_bridge(ctx: RunContext, step: dict, out: SuiteResult) -> bool:
    ok = True
    for robot in step["robots"]:
        result = run_command(ctx, "robot.simulation_bridge", {
            "target": robot, "publish_states": True, "accept_commands": True})
        ok &= out.expect(f"{robot} simulation bridge publishes states and accepts commands",
                         result.get("ok", False), str(result))
    return ok


def step_wait_stacks(ctx: RunContext, step: dict, out: SuiteResult) -> bool:
    ok = True
    for stack in step["stacks"]:
        ready = wait_until(lambda s=stack: stack_status(ctx, step.get("prefix", ""), s),
                           step.get("timeout", 60.0), 1.0)
        ok &= out.expect(f"{stack} stack alive for prefix '{step.get('prefix', '')}'",
                         bool(ready), str(ready))
    return ok


def select_robot(ctx: RunContext, robot: str) -> bool:
    found = run_command(ctx, "scene.find", {"target": {"name": robot}, "limit": 20})
    for alias in [item.get("alias") for item in found.get("items", []) if item.get("alias")]:
        run_command(ctx, "selection.set", {"target": {"alias": alias}})
        if str(run_command(ctx, "navigation.status").get("owner", "")).endswith(f"/{robot}"):
            return True
    return False


def step_launch(ctx: RunContext, step: dict, out: SuiteResult) -> bool:
    launched = run_command(ctx, "process.launch", {"path": step["path"], "name": step["name"],
                                                   "args": step.get("args", [])})
    if not out.expect(f"{step['name']} launched", launched.get("ok", False), str(launched)):
        return False
    time.sleep(step.get("settle", 5.0))
    items = run_command(ctx, "process.status", {"name": step["name"]}).get("processes", [])
    running = [item for item in items if item.get("state") == "running"]
    return out.expect(f"{step['name']} running", bool(running),
                      str(items[-1].get("output_tail", ""))[-400:] if items else "not found")


def step_start_servers(ctx: RunContext, step: dict, out: SuiteResult) -> bool:
    robot = step["robot"]
    if not out.expect(f"{robot} selected", select_robot(ctx, robot)):
        return False
    if step.get("generate_easynav_config"):
        generated = run_command(ctx, "navigation.generate_easynav_config")
        out.expect(f"{robot} EasyNav config generated", generated.get("ok", False), str(generated))
    started = {}
    for _ in range(3):
        started = run_command(ctx, "navigation.start_servers")
        if started.get("ok") or started.get("error") != "no_navigation_robot":
            break
        time.sleep(1.0)
        select_robot(ctx, robot)
    out.expect(f"{robot} navigation servers start accepted", started.get("ok", False), str(started))
    status = wait_until(lambda: (lambda s: s if s.get("session_status") in ("Running", "Attached")
                                 and (s.get("stack") != "EasyNavigation"
                                      or s.get("easynav_transport_available")) else None)(
        run_command(ctx, "navigation.status")), step.get("timeout", 180.0), 1.0) or \
        run_command(ctx, "navigation.status")
    return out.expect(f"{robot} navigation servers ready",
                      status.get("session_status") in ("Running", "Attached"),
                      f"{status.get('stack')} {status.get('session_status')} "
                      f"{status.get('last_error', '')}")


def nav_ready(status: dict) -> bool:
    return bool(status.get("connected")) or status.get("session_state") == "Running"


def lifecycle_active(manager: str, timeout: float) -> bool:
    return probe_json(["python3", str(TOOLS / "ros_probe.py"), "active", "--manager", manager,
                       "--timeout", str(timeout)], True, timeout).get("active", False)


def start_robot_navigation(ctx: RunContext, step: dict, robot: str, manager: str) -> tuple:
    if step.get("managed", True):
        started = run_command(ctx, "robot.navigation.start", {"target": robot})
        if not (started.get("ok", False) or started.get("error") == "already_running"):
            return False, str(started)
    status = wait_until(lambda: (lambda s: s if nav_ready(s) or s.get("session_state") == "Lost"
                                 else None)(nav_status(ctx, robot)),
                        step.get("timeout", 180.0), 1.0) or nav_status(ctx, robot)
    detail = (f"{status.get('stack_kind')} session={status.get('session_state')} "
              f"{status.get('session_error') or status.get('error', '')}".strip())
    if not nav_ready(status):
        return False, detail
    if manager and not lifecycle_active(manager, step.get("activation_timeout", 60.0)):
        return False, f"{manager} did not report all Nav2 nodes active"
    return True, detail


def step_start_navigation(ctx: RunContext, step: dict, out: SuiteResult) -> bool:
    managers = step.get("lifecycle_managers", [])
    for index, robot in enumerate(step["robots"]):
        manager = managers[index] if index < len(managers) else ""
        attempts = step.get("attempts", 3) if step.get("managed", True) else 1
        failures = []
        for attempt in range(1, attempts + 1):
            ready, detail = start_robot_navigation(ctx, step, robot, manager)
            if ready:
                break
            failures.append(detail)
            if attempt < attempts:
                run_command(ctx, "robot.navigation.stop", {"target": robot})
                wait_until(lambda: not nav_ready(nav_status(ctx, robot)), 20.0, 1.0)
                time.sleep(2.0)
        retried = f" after {len(failures)} restart(s): {failures}" if failures and ready else ""
        if not out.expect(f"{robot} navigation stack ready"
                          f"{' and active' if manager else ''}", ready,
                          (detail + retried) if ready else str(failures)):
            return False
    return True


def step_navigate(ctx: RunContext, project: Project, step: dict, out: SuiteResult) -> bool:
    goals = step["goals"]
    tolerance = step.get("tolerance", 0.25)
    before = truth_poses(project, [g["truth"] for g in goals])
    probes = {}
    for goal in goals:
        if goal.get("via") in GOAL_PROBES:
            probe = GOAL_PROBES[goal["via"]]
            ns = goal.get("ns", "")
            frame = f"{ns}/base_footprint" if ns else "base_footprint"
            args = ["python3", str(probe), "navigate", f"--x={goal['x']}",
                    f"--y={goal['y']}", f"--yaw={goal.get('yaw', 0.0)}", "--base-frame", frame,
                    "--frames", frame, "--timeout", str(step.get("timeout", 240.0))]
            if ns:
                args += ["--ns", ns, "--scan-topic", f"/{ns}/scan"]
            line = f'source "{ROS_SETUP}" && exec ' + " ".join(f"'{a}'" for a in args)
            probes[goal["robot"]] = subprocess.Popen(
                ["bash", "-c", line], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
                start_new_session=True, cwd=str(probe.parent))
            out.expect(f"{goal['robot']} {goal['via']} goal ({goal['x']}, {goal['y']}) sent", True)
            continue
        if "prefix" in goal:
            command = "motion_stack.goal"
            arguments = {"prefix": goal["prefix"], "x": goal["x"], "y": goal["y"],
                         "yaw": goal.get("yaw", 0.0)}
        else:
            command = "robot.navigation.goal"
            arguments = {"target": goal["robot"], "x": goal["x"], "y": goal["y"],
                         "yaw": goal.get("yaw", 0.0)}
        sent = wait_until(lambda c=command, a=arguments: (lambda r: r if r.get("ok") else None)(
            run_command(ctx, c, a)), 30.0, 2.0) or run_command(ctx, command, arguments)
        goal["command_id"] = sent.get("command_id", "")
        out.expect(f"{goal['robot']} goal ({goal['x']}, {goal['y']}) accepted",
                   sent.get("ok", False), str(sent))
    states: dict[str, str] = {}

    time.sleep(1.0)

    reports: dict[str, dict] = {}

    def finished():
        for goal in goals:
            process = probes.get(goal["robot"])
            if process is None:
                states[goal["robot"]] = goal_state(ctx, goal)
            elif process.poll() is not None and goal["robot"] not in reports:
                reports[goal["robot"]] = next((json.loads(row) for row in reversed(
                    process.stdout.read().splitlines()) if row.startswith("{")), {})
                states[goal["robot"]] = "succeeded" if reports[goal["robot"]].get("succeeded") \
                    else "failed"
            elif process.poll() is None:
                states[goal["robot"]] = "navigating"
        return all(state in TERMINAL for state in states.values())


    wait_until(finished, step.get("timeout", 240.0) + 30.0, 1.0)
    for process in probes.values():
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
    after = truth_poses(project, [g["truth"] for g in goals])
    ok = True
    for goal in goals:
        start, end = before.get(goal["truth"]), after.get(goal["truth"])
        error = math.hypot(end[0] - goal["x"], end[1] - goal["y"]) if end else float("inf")
        moved = math.hypot(end[0] - start[0], end[1] - start[1]) if start and end else 0.0
        ok &= out.expect(f"{goal['robot']} navigation succeeded",
                         states.get(goal["robot"]) == "succeeded", str(states.get(goal["robot"])))
        ok &= out.expect(f"{goal['robot']} within {tolerance} m of the goal (measured)",
                         error <= tolerance, f"error {error:.3f} m, moved {moved:.2f} m, "
                         f"at {[round(v, 2) for v in end] if end else None}")
    return ok


def step_joint_target(ctx: RunContext, project: Project, step: dict, out: SuiteResult) -> bool:
    joints = step["joints"]
    tolerance = step.get("tolerance", 0.05)
    sent = run_command(ctx, "motion_stack.target", {"prefix": step.get("prefix", ""),
                                                    "joints": joints})
    if not out.expect("joint target accepted", sent.get("ok", False), str(sent)):
        return False

    def errors():
        state = truth_joints(project, step["truth"])
        positions = dict(zip(state.get("name", []), state.get("position", [])))
        return {n: abs(positions[n] - p) if n in positions else float("inf")
                for n, p in zip(joints["name"], joints["position"])}

    reached = wait_until(lambda: (lambda e: e if max(e.values()) <= tolerance else None)(
        errors()), step.get("timeout", 60.0), 1.0)
    time.sleep(1.0)
    final = errors()
    return out.expect(f"joints within {tolerance} rad and holding (measured)",
                      bool(reached) and max(final.values()) <= tolerance,
                      ", ".join(f"{n} {e:.3f}" for n, e in final.items()))


def start_stacks(project: Project, out: SuiteResult) -> subprocess.Popen | None:
    if not project.stacks:
        return None
    log_path = project.root / "evidence" / "quickstart" / "stacks.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log = log_path.open("w")
    process = subprocess.Popen(["bash", "-c", project.stacks], cwd=project.root, stdout=log,
                               stderr=subprocess.STDOUT, start_new_session=True)
    time.sleep(3.0)
    out.expect(f"{project.stacks} running", process.poll() is None,
               "see evidence/quickstart/stacks.log")
    return process


def stop_stacks(process: subprocess.Popen | None, out: SuiteResult) -> None:
    if process is None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()
    except ProcessLookupError:
        pass
    out.expect("project stacks stopped", process.poll() is not None)


def stop_project_navigation(ctx: RunContext, project: Project, out: SuiteResult) -> None:
    for step in project.scenario:
        for goal in step.get("goals", []):
            if "prefix" not in goal and \
                    nav_status(ctx, goal["robot"]).get("state") not in TERMINAL + ("idle",):
                run_command(ctx, "robot.navigation.cancel", {"target": goal["robot"]})
    for step in project.scenario:
        if step["step"] == "start_navigation" and step.get("managed", True):
            for robot in step["robots"]:
                stopped = run_command(ctx, "robot.navigation.stop", {"target": robot})
                out.expect(f"{robot} navigation stopped",
                           stopped.get("ok", False) or stopped.get("error") == "not_running",
                           str(stopped))
    for step in project.scenario:
        if step["step"] == "start_servers" and select_robot(ctx, step["robot"]):
            stopped = run_command(ctx, "navigation.stop_servers")
            out.expect(f"{step['robot']} navigation servers stopped", stopped.get("ok", False),
                       str(stopped))
            wait_until(lambda: run_command(ctx, "navigation.status").get("session_status")
                       not in ("Running", "Attached", "Launching"), 20.0, 1.0)


def run_scenario(ctx: RunContext, project: Project, stage) -> None:
    for index, step in enumerate(project.scenario, 1):
        kind = step["step"]
        name = f"{index}.{kind}"
        if kind == "launch":
            ok = stage(name, lambda out, s=step: step_launch(ctx, s, out))
        elif kind == "start_servers":
            ok = stage(name, lambda out, s=step: step_start_servers(ctx, s, out))
        elif kind == "bridge":
            ok = stage(name, lambda out, s=step: step_bridge(ctx, s, out))
        elif kind == "wait_stacks":
            ok = stage(name, lambda out, s=step: step_wait_stacks(ctx, s, out))
        elif kind == "start_navigation":
            ok = stage(name, lambda out, s=step: step_start_navigation(ctx, s, out))
        elif kind == "navigate":
            ok = stage(name, lambda out, s=step: step_navigate(ctx, project, s, out))
        elif kind == "joint_target":
            ok = stage(name, lambda out, s=step: step_joint_target(ctx, project, s, out))
        elif kind == "gz_motion":
            ok = stage(name, lambda out, s=step: step_gz_motion(s, out))
        elif kind == "screenshot":
            ok = stage(name, lambda out, s=step: screenshot(ctx, project, out, s["name"],
                                                             s.get("camera")))
        else:
            ok = stage(name, lambda out, k=kind: out.expect(f"known step {k}", False))
        if not ok:
            return


def smoke_test(client: McpClient, project: Project, args) -> list[SuiteResult]:
    ctx = RunContext(client, project.root.name, 16.6)
    results: list[SuiteResult] = []

    def stage(name: str, fn) -> bool:
        out = SuiteResult(name)
        start = time.monotonic()
        try:
            ok = fn(out) is not False and not out.failed
        except Exception as error:  # noqa: BLE001 - always reach teardown
            out.error = str(error)
            ok = False
        out.seconds = time.monotonic() - start
        results.append(out)
        print_results([out])
        return ok

    stacks = None
    try:
        ok = stage("open", lambda out: open_project(ctx, project, out))
        ok = ok and stage("comms", lambda out: start_comms(ctx, out))
        if ok and project.prepare:
            def prepare(out: SuiteResult) -> bool:
                done = shell(project.prepare, project.root, False, 900)
                log = project.root / "evidence" / "quickstart" / "prepare.log"
                log.parent.mkdir(parents=True, exist_ok=True)
                log.write_text(done.stdout + done.stderr)
                return out.expect(project.prepare, done.returncode == 0,
                                  f"exit {done.returncode}: {(done.stdout + done.stderr).strip()[-600:]}")
            ok = stage("prepare", prepare)
        if ok and project.before_simulation:
            ok = stage("placement", lambda out: place_robots(ctx, project, out))
        if ok:
            ok = stage("simulation", lambda out: play_simulation(ctx, out))
        if ok:
            stage("scene", lambda out: check_scene(ctx, project, out))
            if args.navigation and project.navigation:
                stage("navigation", lambda out: start_navigation(ctx, out))
            stage("screenshot", lambda out: screenshot(ctx, project, out))
            if args.action in ("run", "all") and project.scenario:
                holder = {}
                ok = stage("stacks", lambda out: holder.update(process=start_stacks(project, out)))
                stacks = holder.get("process")
                if ok:
                    run_scenario(ctx, project, stage)
    finally:
        if not args.keep_running:
            def teardown(out: SuiteResult) -> None:
                stop_project_navigation(ctx, project, out)
                stop_stacks(stacks, out)
                ordered_stop(ctx, out)
            stage("teardown", teardown)
    return results


def test(project: Project, args) -> list[SuiteResult]:
    sessions = live_sessions()
    if sessions:
        print(f"Using live RobotLens session {sessions[0].pid}")
        with McpClient(socket_path=str(sessions[0].socket), timeout=60.0) as client:
            return smoke_test(client, project, args)
    log = project.root / "evidence" / "quickstart" / "robotlens.log"
    instance = RobotLensInstance(ROBOTLENS, project.project_file, log)
    print(f"Started RobotLens pid {instance.pid}; log {log.relative_to(project.root)}")
    shutdown = SuiteResult("shutdown")
    results: list[SuiteResult] = []
    try:
        if not shutdown.expect("RobotLens MCP session ready",
                               instance.wait_for_socket(SOCKET_TIMEOUT_S), str(instance.socket)):
            return [shutdown]
        with McpClient(socket_path=str(instance.socket), timeout=60.0) as client:
            results = smoke_test(client, project, args)
    finally:
        if args.keep_running:
            print(f"RobotLens left running (pid {instance.pid})")
        else:
            shutdown.expect("RobotLens exited on SIGTERM", instance.stop(SHUTDOWN_TIMEOUT_S),
                            instance.exit_description())
            print_results([shutdown])
    return results + [shutdown]


def run(project: Project) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=("setup", "test", "run", "all"), nargs="?",
                        default="all")
    parser.add_argument("--keep-running", action="store_true",
                        help="leave the simulation and RobotLens running after the test")
    if project.navigation:
        parser.add_argument("--navigation", action="store_true",
                            help="also start the robot's navigation servers")
    args = parser.parse_args()
    args.navigation = getattr(args, "navigation", False)
    results: list[SuiteResult] = []
    if args.action in ("setup", "all"):
        results.append(setup(project))
        print_results(results[-1:])
    if args.action in ("test", "run", "all") and not any(r.failed for r in results):
        results += test(project, args)
    failed = any(r.failed for r in results)
    if args.action in ("run", "all"):
        report = project.root / "evidence" / "quickstart" / "report.json"
        report.parent.mkdir(parents=True, exist_ok=True)
        report.write_text(json.dumps({
            "project": project.root.name, "passed": not failed,
            "stages": [{"name": r.name, "failed": r.failed, "seconds": round(r.seconds, 1),
                        "error": r.error,
                        "checks": [{"name": c.name, "ok": c.passed, "required": c.required,
                                    "detail": c.detail} for c in r.checks]}
                       for r in results]}, indent=2))
    print(f"\n{project.root.name}: {'FAIL' if failed else 'PASS'}")
    return 1 if failed else 0
