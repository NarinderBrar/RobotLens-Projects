"""Zenoh ground truth for quickstart scenarios; needs eclipse-zenoh. Prints one JSON line.

Subscribes to each key (for example robot1/odom or robot1/joint_states) and reports the latest
JSON payload received within the timeout.
"""

import argparse
import json
import time

import zenoh


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("keys", nargs="+")
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()
    latest: dict = {}
    session = zenoh.open(zenoh.Config())
    subscribers = []
    for key in args.keys:
        def handler(sample, key=key):
            try:
                latest[key] = json.loads(sample.payload.to_bytes())
            except (ValueError, AttributeError):
                pass
        subscribers.append(session.declare_subscriber(key, handler))
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and len(latest) < len(args.keys):
        time.sleep(0.05)
    time.sleep(0.2)
    for subscriber in subscribers:
        subscriber.undeclare()
    session.close()
    print(json.dumps({key: latest.get(key) for key in args.keys}))


if __name__ == "__main__":
    main()
