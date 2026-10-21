"""Publishes a batch of Gazebo messages from one process; prints one JSON line.

usage: gz_publish.py '[{"topic": "/drone1/enable", "type": "gz.msgs.Boolean",
                         "payload": "data: true"}]'

Each payload is protobuf text format. The script waits up to --wait seconds for each topic to
have a subscriber, then sends the whole batch --repeat times 0.1 s apart, because a new
publisher can drop its first message even after it reports a connection. Every message here is a
level command (a velocity, a position, an enable flag), so repeating it is harmless.
"""

import argparse
import importlib
import json
import time

from google.protobuf import text_format
from gz.transport13 import Node

MODULES = {"Boolean": "boolean_pb2", "Double": "double_pb2", "Twist": "twist_pb2"}


def message_class(type_name: str):
    short = type_name.rsplit(".", 1)[-1]
    module = importlib.import_module(f"gz.msgs10.{MODULES[short]}")
    return getattr(module, short)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("commands")
    parser.add_argument("--wait", type=float, default=3.0)
    parser.add_argument("--repeat", type=int, default=3)
    args = parser.parse_args()
    node = Node()
    publishers = []
    for command in json.loads(args.commands):
        cls = message_class(command.get("type", "gz.msgs.Double"))
        message = text_format.Parse(command["payload"], cls())
        publishers.append((command["topic"], node.advertise(command["topic"], cls), message))
    deadline = time.monotonic() + args.wait
    while time.monotonic() < deadline and not all(p.has_connections() for _, p, _ in publishers):
        time.sleep(0.02)
    connected = {topic: publisher.has_connections() for topic, publisher, _ in publishers}
    for _ in range(args.repeat):
        for _, publisher, message in publishers:
            publisher.publish(message)
        time.sleep(0.1)
    print(json.dumps({"published": len(publishers), "connected": connected}))


if __name__ == "__main__":
    main()
