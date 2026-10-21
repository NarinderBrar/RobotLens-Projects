#!/usr/bin/env python3
"""Writes a MuJoCo-ready Bebop 2 URDF: the Parrot Bebop 2 model plus a robotlens_sim block that
gives it a floating base and four rotors, so RobotLens's MuJoCo multirotor controller flies it
from cmd_vel (ADR 0061). Rotor constants match the Gazebo Bebop fixture. base_footprint gets a
negligible inertial: without one, MuJoCo merges the fixed base_link into it and computes the
body's mass from the collision box (9.4 kg) instead of using the declared 0.503 kg.

usage: make_drone_urdf.py SOURCE.urdf OUTPUT.urdf [ROBOT_NAME]
"""

import sys
from pathlib import Path

ROTORS = [("rotor_fl_link", "ccw"), ("rotor_fr_link", "cw"), ("rotor_rl_link", "cw"),
          ("rotor_rr_link", "ccw")]


FOOTPRINT = """<link name="base_footprint">
    <inertial>
      <mass value="0.001"/>
      <inertia ixx="1e-7" ixy="0" ixz="0" iyy="1e-7" iyz="0" izz="1e-7"/>
    </inertial>
  </link>"""


def hints() -> str:
    rotors = "\n".join(
        f'      <rotor link="{link}" spin="{spin}" thrust_coefficient="2.5e-6" '
        f'moment_coefficient="0.004" max_rotor_speed="1200"/>' for link, spin in ROTORS)
    return ('  <robotlens_sim base="floating">\n'
            '    <multirotor max_speed="3" max_climb="2" max_yaw_rate="1">\n'
            f'{rotors}\n    </multirotor>\n  </robotlens_sim>\n')


def main() -> None:
    source, output = Path(sys.argv[1]), Path(sys.argv[2])
    name = sys.argv[3] if len(sys.argv) > 3 else "bebop2"
    text = source.read_text()
    head = '<robot name="parrot_bebop_2">'
    if head not in text:
        sys.exit(f"{source} does not start a parrot_bebop_2 robot")
    footprint = '<link name="base_footprint"/>'
    if footprint not in text:
        sys.exit(f"{source} has no bare base_footprint link")
    text = text.replace(footprint, FOOTPRINT, 1)
    output.write_text(text.replace(head, f'<robot name="{name}">\n' + hints(), 1))


if __name__ == "__main__":
    main()
