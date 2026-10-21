"""Streams velocity commands on Zenoh cmd_vel keys, then zero commands; prints one JSON line.

usage: zenoh_twist.py --command robot1/cmd_vel 0 0 0.5 0 [--command ...] --seconds 3

All commands share one Zenoh session, and the stream starts once every key has a matching
subscriber (up to --wait seconds), so no command is lost to discovery. The payload is
{"linear": {"x", "y", "z"}, "angular": {"z"}}, which RobotLens's MuJoCo bridge reads for drones
(ADR 0061).
"""

import argparse
import json
import time

import zenoh


def payload(x: float, y: float, z: float, yaw_rate: float) -> str:
    return json.dumps({"linear": {"x": x, "y": y, "z": z}, "angular": {"z": yaw_rate}})


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--command", nargs=5, action="append", required=True,
                        metavar=("KEY", "X", "Y", "Z", "YAW_RATE"))
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--wait", type=float, default=3.0)
    args = parser.parse_args()
    session = zenoh.open(zenoh.Config())
    streams = [(session.declare_publisher(key), payload(*map(float, twist)))
               for key, *twist in args.command]
    deadline = time.monotonic() + args.wait
    while time.monotonic() < deadline and not all(
            publisher.matching_status.matching for publisher, _ in streams):
        time.sleep(0.05)
    matched = [publisher.matching_status.matching for publisher, _ in streams]
    deadline = time.monotonic() + args.seconds
    sent = 0
    while time.monotonic() < deadline:
        for publisher, message in streams:
            publisher.put(message)
        sent += 1
        time.sleep(0.05)
    for _ in range(3):
        for publisher, _ in streams:
            publisher.put(payload(0.0, 0.0, 0.0, 0.0))
        time.sleep(0.05)
    for publisher, _ in streams:
        publisher.undeclare()
    session.close()
    print(json.dumps({"sent": sent, "matched": matched}))


if __name__ == "__main__":
    main()
