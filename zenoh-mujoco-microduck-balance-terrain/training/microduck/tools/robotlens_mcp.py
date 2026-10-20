import json
import os
import socket
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
NO_RETRY = ("training.start", "process.launch")


def log(*parts):
    print(time.strftime("%H:%M:%S"), *parts, flush=True)


def session_directory():
    runtime = os.environ.get("XDG_RUNTIME_DIR") or f"/run/user/{os.getuid()}"
    return Path(runtime) / "robotlens-sessions"


def live_socket():
    sessions = []
    for path in session_directory().glob("session-*.json"):
        try:
            record = json.loads(path.read_text())
            os.kill(int(record["pid"]), 0)
        except (OSError, ValueError, KeyError):
            continue
        if Path(record["socket"]).exists():
            sessions.append((int(record.get("started_at_ms", 0)), record["socket"]))
    if not sessions:
        raise RuntimeError("no live RobotLens session; start RobotLens and its MCP server")
    return max(sessions)[1]


def _request(sock, reader, request_id, method, params):
    sock.sendall((json.dumps({"jsonrpc": "2.0", "id": request_id, "method": method, "params": params}) + "\n").encode())
    while True:
        line = reader.readline()
        if not line:
            raise RuntimeError(f"connection closed during {method}")
        message = json.loads(line)
        if message.get("id") != request_id:
            continue
        if "error" in message:
            raise RuntimeError(f"{method} failed: {message['error']}")
        return message.get("result", {})


def _call_once(tool, arguments):
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.settimeout(120)
        sock.connect(live_socket())
        with sock.makefile("r", encoding="utf-8") as reader:
            _request(sock, reader, 1, "initialize", {"protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "microduck-tools", "version": "1"}})
            sock.sendall((json.dumps({"jsonrpc": "2.0", "method": "notifications/initialized"}) + "\n").encode())
            raw = _request(sock, reader, 2, "tools/call", {"name": tool, "arguments": arguments})
    for item in raw.get("content", []):
        if item.get("type") == "text":
            return json.loads(item["text"]).get("result", {})
    return {}


def call(tool, arguments):
    for _ in range(10):
        try:
            return _call_once(tool, arguments)
        except (OSError, RuntimeError) as error:
            if tool == "robotlens.run" and arguments.get("command") in NO_RETRY:
                raise
            log("MCP retry", arguments.get("command", tool), error)
            time.sleep(10)
    raise RuntimeError(f"MCP call failed: {tool}")


def run(command, args=None):
    return call("robotlens.run", {"command": command, "args": args or {}})


def runtime():
    return call("robotlens.inspect_learning_runtime", {})


def wait(predicate, timeout, step=2):
    end = time.time() + timeout
    while time.time() < end:
        value = predicate()
        if value:
            return value
        time.sleep(step)
    return None


def project_path(relative):
    return PROJECT_ROOT / relative


def process_state(name):
    return run("process.status", {"name": name})["processes"][0]


def prepare_task(model, task, grid, iterations, device="gpu"):
    run("model.import", {"path": model})
    wait(lambda: run("model.status").get("state") != "loading", 300, 3)
    log("load", run("training.load_task", {"path": task}))
    run("training.configure", {"robot": "world-main", "world": "world-main", "execution": "external", "device": device, "grid_side": grid, "transitions": 500, "iterations": iterations, "viewport_preview": "grid", "preview": True})
    run("training.preflight")

    def ready():
        eligibility = runtime()["start_eligibility"]
        if eligibility["preflight"]["running"] or any(b["code"] == "preflight_pending" for b in eligibility["blockers"]):
            return None
        return eligibility

    eligibility = wait(ready, 180)
    admitted = bool(eligibility and (eligibility["preflight"]["gpu_admitted"] if device == "gpu" else not eligibility["blockers"]))
    log("preflight", admitted, eligibility["blockers"] if eligibility else "timed out")
    return admitted


def launch_trainer(name, args, ready_file):
    ready = project_path(ready_file)
    ready.unlink(missing_ok=True)
    log("launch", run("process.launch", {"path": "training/microduck/run_external_trainer.sh", "name": name, "args": args + ["--ready-file", ready_file]}))
    if not wait(ready.exists, 180):
        log("trainer not ready", process_state(name))
        return False
    log("start", run("training.start"))
    return True


def stop_all():
    run("training.stop")
    time.sleep(5)
    for process in run("process.status").get("processes", []):
        if process["state"] == "running":
            log("stop", run("process.stop", {"name": process["name"]}))
    wait(lambda: runtime()["run"]["state"] != "running", 60)
