#!/usr/bin/env python3
"""
Convenience wrapper: launches RobotLens with MuJoCo TurtleBot3 physics.

Equivalent to:
    python3 examples/turtlebot3/launch/turtlebot3_mujoco.launch.py

See that file for full options (--nav2, etc.).
"""
import os
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LAUNCH = os.path.join(SCRIPT_DIR, "..", "launch", "turtlebot3_mujoco.launch.py")

sys.exit(subprocess.run([sys.executable, LAUNCH] + sys.argv[1:]).returncode)
