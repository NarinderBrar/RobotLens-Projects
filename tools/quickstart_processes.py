import json
import os
from pathlib import Path
import signal
import socket
import time


def process_snapshot():
    found = {}
    for path in Path("/proc").glob("[0-9]*/stat"):
        try:
            fields = path.read_text().rsplit(")", 1)[1].split()
            found[int(path.parent.name)] = (int(fields[1]), int(fields[19]), fields[0])
        except (OSError, ValueError, IndexError):
            continue
    return found


class OwnedProcesses:
    def __init__(self):
        self.identities = {}

    def refresh(self):
        found = process_snapshot()
        owned = {os.getpid()}
        while True:
            more = {pid for pid, (parent, _, _) in found.items()
                    if parent in owned and pid not in owned}
            if not more:
                break
            owned.update(more)
        owned.discard(os.getpid())
        self.identities.update({pid: found[pid][1] for pid in owned})

    def survivors(self):
        found = process_snapshot()
        return [pid for pid, start in self.identities.items()
                if pid in found and found[pid][1] == start and found[pid][2] != "Z"]

    def signal(self, pids, value):
        alive = set(self.survivors())
        for pid in pids:
            if pid in alive:
                try:
                    os.kill(pid, value)
                except ProcessLookupError:
                    pass

    def cleanup(self):
        self.refresh()
        signalled = set()
        deadline = time.monotonic() + 10
        while self.survivors() and time.monotonic() < deadline:
            self.refresh()
            current = set(self.survivors())
            self.signal(current - signalled, signal.SIGTERM)
            signalled.update(current)
            time.sleep(0.2)
        forced = self.survivors()
        self.signal(forced, signal.SIGKILL)
        deadline = time.monotonic() + 5
        while self.survivors() and time.monotonic() < deadline:
            time.sleep(0.2)
        return {"forced_pids": forced, "surviving_pids": self.survivors()}


def reap_children():
    while True:
        try:
            pid, _ = os.waitpid(-1, os.WNOHANG)
            if not pid:
                return
        except ChildProcessError:
            return


def clean_session_records(identities, directory):
    removed, retained = [], []
    for pid in identities:
        if Path(f"/proc/{pid}").exists():
            continue
        record = directory / f"session-{pid}.json"
        endpoint = directory / f"session-{pid}.sock"
        if not record.exists() and not endpoint.exists():
            continue
        try:
            if record.exists() and json.loads(record.read_text()).get("pid") != pid:
                retained.append(str(record))
                continue
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.settimeout(0.25)
                try:
                    client.connect(str(endpoint))
                    retained.append(str(endpoint))
                    continue
                except (ConnectionRefusedError, FileNotFoundError):
                    pass
            for path in (record, endpoint):
                if path.exists():
                    path.unlink()
                    removed.append(str(path))
        except (OSError, ValueError, TypeError):
            retained.append(str(record))
    return {"removed": removed, "retained": retained}
