#!/usr/bin/env python3
"""
Launch file for TurtleBot3 Burger with MuJoCo physics backend.

Starts RobotLens with a pre-configured MuJoCo project (in-process physics)
and optionally brings up Nav2. Mirrors the Gazebo turtlebot3.launch.py
pattern: one launch owns everything.

Usage:
    python3 examples/turtlebot3/launch/turtlebot3_mujoco.launch.py [--nav2]
"""

import argparse
import json
import os
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TURTLEBOT3_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "..", "..", ".."))
MJCF_PATH = os.path.join(TURTLEBOT3_DIR, "mujoco", "turtlebot3_burger.xml")
DEFAULT_PROJECT_DIR = os.path.join(os.path.expanduser("~"), ".robotlens", "mujoco_turtlebot3")


def find_robotlens_binary():
    """Find the robotlens_desktop binary."""
    for candidate in [
        os.path.join(PROJECT_ROOT, "install", "robotlens_desktop", "bin", "robotlens_desktop"),
        os.path.join(PROJECT_ROOT, "build", "robotlens_desktop", "robotlens_desktop"),
    ]:
        if os.path.isfile(candidate):
            return candidate
    import shutil
    result = shutil.which("robotlens_desktop")
    if result:
        return result
    return None


def create_project(project_dir):
    """Create a RobotLens project with MuJoCo backend configured."""
    os.makedirs(project_dir, exist_ok=True)
    project_file = os.path.join(project_dir, "project.json")
    project = {
        "schemaVersion": 7,
        "simulation": {
            "backend": "mujoco",
            "worldFile": MJCF_PATH,
            "worldName": "turtlebot3",
            "robotModelName": "",
            "executable": "",
            "args": [],
            "environment": {},
            "attachOnly": False,
        },
        "scene": {"primitives": []},
        "navigation": {"enabled": False},
    }
    with open(project_file, "w") as f:
        json.dump(project, f, indent=2)
    return project_file


def main():
    parser = argparse.ArgumentParser(
        description="Launch RobotLens with MuJoCo TurtleBot3 physics")
    parser.add_argument("--nav2", action="store_true",
                        help="Also bring up Nav2 (requires ros-jazzy-navigation2)")
    args, app_args = parser.parse_known_args()

    if not os.path.isfile(MJCF_PATH):
        print(f"[mujoco] ERROR: MJCF not found: {MJCF_PATH}")
        sys.exit(1)

    binary = find_robotlens_binary()
    if not binary:
        print("[mujoco] ERROR: robotlens_desktop not found. Build first:")
        print("  colcon build --packages-select robotlens_desktop")
        sys.exit(1)

    project_file = create_project(DEFAULT_PROJECT_DIR)

    print(f"[mujoco] MJCF:   {MJCF_PATH}")
    print(f"[mujoco] Project: {project_file}")
    print(f"[mujoco] Binary:  {binary}")
    print()
    print("  MuJoCo physics runs in-process. After RobotLens opens:")
    print("  1. Open View > Simulation")
    print("  2. Press Launch to start MuJoCo physics")
    print("  3. Arrow keys / WASD for teleop")
    print()

    cmd = [binary, "--project", project_file] + app_args
    try:
        proc = subprocess.run(cmd)
        sys.exit(proc.returncode)
    except KeyboardInterrupt:
        print("\n[mujoco] Shut down.")


if __name__ == "__main__":
    main()
