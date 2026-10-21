"""Keeps a Zenoh drone above a ground robot until stopped; needs eclipse-zenoh.

usage: zenoh_escort.py --drone-odom drone/odom --robot-odom robot1/odom --cmd drone/cmd_vel

Same control as ros_escort.py over Zenoh JSON odometry and the drone cmd_vel payload of ADR 0061.
SIGTERM sends a zero command.
"""

import argparse
import json
import math
import signal
import time

import zenoh


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--drone-odom", required=True)
    parser.add_argument("--robot-odom", required=True)
    parser.add_argument("--cmd", required=True)
    parser.add_argument("--altitude", type=float, default=2.5)
    parser.add_argument("--gain", type=float, default=0.8)
    parser.add_argument("--max-speed", type=float, default=1.0)
    args = parser.parse_args()
    poses = {}

    def store(key):
        def callback(sample):
            try:
                poses[key] = json.loads(sample.payload.to_bytes())
            except ValueError:
                pass
        return callback

    session = zenoh.open(zenoh.Config())
    subscribers = [session.declare_subscriber(args.drone_odom, store("drone")),
                   session.declare_subscriber(args.robot_odom, store("robot"))]
    publisher = session.declare_publisher(args.cmd)
    stopping = []
    signal.signal(signal.SIGTERM, lambda *_: stopping.append(True))
    signal.signal(signal.SIGINT, lambda *_: stopping.append(True))

    def clamp(value):
        return max(-args.max_speed, min(args.max_speed, args.gain * value))

    def send(x, y, z):
        publisher.put(json.dumps({"linear": {"x": x, "y": y, "z": z}, "angular": {"z": 0.0}}))

    while not stopping:
        time.sleep(0.1)
        if "drone" not in poses or "robot" not in poses:
            continue
        drone, robot = poses["drone"], poses["robot"]
        yaw = drone.get("yaw", 0.0)
        wx, wy = clamp(robot["x"] - drone["x"]), clamp(robot["y"] - drone["y"])
        send(math.cos(yaw) * wx + math.sin(yaw) * wy, -math.sin(yaw) * wx + math.cos(yaw) * wy,
             clamp(args.altitude - drone.get("z", 0.0)))
    for _ in range(3):
        send(0.0, 0.0, 0.0)
        time.sleep(0.05)
    for subscriber in subscribers:
        subscriber.undeclare()
    session.close()


if __name__ == "__main__":
    main()
