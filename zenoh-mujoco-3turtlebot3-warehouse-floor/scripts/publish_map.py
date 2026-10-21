import argparse
import json
from pathlib import Path
import signal
import threading

import zenoh


PROJECT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=Path, default=PROJECT / "maps/warehouse.json")
    parser.add_argument("--config", type=Path, default=PROJECT / "stacks/navigation.json")
    args = parser.parse_args()
    grid = json.loads(args.map.read_text())
    if grid.get("contract") != 1 or len(grid["data"]) != grid["width"] * grid["height"]:
        raise ValueError("Invalid native occupancy grid")
    prefix = json.loads(args.config.read_text())["robot_prefix"].strip("/")
    key = f"{prefix}/map" if prefix else "map"
    payload = json.dumps(grid, separators=(",", ":"))
    stopping = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopping.set())
    with zenoh.open(zenoh.Config()) as session:
        print(f"Publishing global map on {key} every second", flush=True)
        while not stopping.is_set():
            session.put(key, payload, encoding=zenoh.Encoding.APPLICATION_JSON)
            stopping.wait(1)


if __name__ == "__main__":
    main()
