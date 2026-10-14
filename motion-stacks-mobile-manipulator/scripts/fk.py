#!/usr/bin/env python3
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def rotation(rpy):
    r, p, y = rpy
    rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle(axis, angle):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * k + (1 - np.cos(angle)) * k @ k


def numbers(text, default):
    return [float(value) for value in text.split()] if text else default


def main():
    if len(sys.argv) != 5:
        print("usage: fk.py <urdf> <base> <tip> <comma-separated joint positions>", file=sys.stderr)
        return 2
    urdf, base, tip, values = sys.argv[1:]
    positions = [float(value) for value in values.split(",")]
    joints = {joint.find("child").get("link"): joint for joint in ET.parse(urdf).getroot().findall("joint")}
    chain = []
    link = tip
    while link != base:
        joint = joints[link]
        chain.append(joint)
        link = joint.find("parent").get("link")
    chain.reverse()
    transform = np.eye(4)
    index = 0
    for joint in chain:
        origin = joint.find("origin")
        local = np.eye(4)
        local[:3, :3] = rotation(numbers(origin.get("rpy") if origin is not None else "", [0, 0, 0]))
        local[:3, 3] = numbers(origin.get("xyz") if origin is not None else "", [0, 0, 0])
        transform = transform @ local
        if joint.get("type") in ("revolute", "continuous", "prismatic"):
            axis = numbers(joint.find("axis").get("xyz"), [1, 0, 0])
            motion = np.eye(4)
            if joint.get("type") == "prismatic":
                motion[:3, 3] = np.asarray(axis) * positions[index]
            else:
                motion[:3, :3] = axis_angle(axis, positions[index])
            transform = transform @ motion
            index += 1
    m = transform[:3, :3]
    w = np.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = np.copysign(np.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    y = np.copysign(np.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    z = np.copysign(np.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    print(json.dumps({"x": transform[0, 3], "y": transform[1, 3], "z": transform[2, 3], "qx": x, "qy": y, "qz": z, "qw": w}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
