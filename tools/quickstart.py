"""Shared setup and smoke test for RobotLens projects; each project's scripts/quickstart.py calls run().

setup  checks RobotLens, ROS 2/Gazebo or Zenoh prerequisites and project files, then runs the
       project's own preparation steps.
test   uses the live RobotLens session (or starts one), opens the project, starts communication,
       plays the simulation, checks the robots and their TF, saves evidence/quickstart.png, and
       stops everything in order: navigation, simulation, communication.
"""

from __future__ import annotations

import argparse
import shutil
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
    return out.expect("project open", bool(status), str(status.get("project_path")))


def start_comms(ctx: RunContext, out: SuiteResult) -> bool:
    started = run_command(ctx, "comms.start")
    out.expect("comms.start accepted",
               started.get("ok", False) or started.get("error") == "already_running", str(started))
    status = wait_until(lambda: (lambda s: s if s.get("started") and s.get("connected") else None)(
        run_command(ctx, "comms.status")), 60.0, 0.5) or run_command(ctx, "comms.status")
    return out.expect("communication connected", bool(status.get("connected")),
                      f"{status.get('backend')} started={status.get('started')}")


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


def screenshot(ctx: RunContext, project: Project, out: SuiteResult) -> None:
    shot = ctx.client.call("robotlens.capture_viewport_screenshot")
    png = shot.images[0] if shot.images else b""
    path = project.root / "evidence" / "quickstart.png"
    if out.expect("viewport screenshot saved", png.startswith(PNG_MAGIC), str(shot.limitations)):
        path.parent.mkdir(exist_ok=True)
        path.write_bytes(png)


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

    try:
        ok = stage("open", lambda out: open_project(ctx, project, out))
        ok = ok and stage("comms", lambda out: start_comms(ctx, out))
        if ok and project.prepare:
            def prepare(out: SuiteResult) -> bool:
                done = shell(project.prepare, project.root, False, 900)
                return out.expect(project.prepare, done.returncode == 0,
                                  f"exit {done.returncode}: {(done.stdout + done.stderr).strip()[-600:]}")
            ok = stage("prepare", prepare)
        if ok:
            ok = stage("simulation", lambda out: play_simulation(ctx, out))
        if ok:
            stage("scene", lambda out: check_scene(ctx, project, out))
            if args.navigation and project.navigation:
                stage("navigation", lambda out: start_navigation(ctx, out))
            stage("screenshot", lambda out: screenshot(ctx, project, out))
    finally:
        if not args.keep_running:
            stage("teardown", lambda out: ordered_stop(ctx, out))
    return results


def test(project: Project, args) -> list[SuiteResult]:
    sessions = live_sessions()
    if sessions:
        print(f"Using live RobotLens session {sessions[0].pid}")
        with McpClient(socket_path=str(sessions[0].socket), timeout=60.0) as client:
            return smoke_test(client, project, args)
    log = project.root / "evidence" / "quickstart-robotlens.log"
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
    parser.add_argument("action", choices=("setup", "test", "all"), nargs="?", default="all")
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
    if args.action in ("test", "all") and not any(r.failed for r in results):
        results += test(project, args)
    failed = any(r.failed for r in results)
    print(f"\n{project.root.name}: {'FAIL' if failed else 'PASS'}")
    return 1 if failed else 0
