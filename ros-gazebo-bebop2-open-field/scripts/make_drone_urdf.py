#!/usr/bin/env python3
"""Writes a Gazebo-ready Bebop 2 URDF: the Parrot Bebop 2 model plus the gz-sim
multicopter stack (four motor models and a body-frame velocity controller) in <gazebo> blocks.

usage: make_drone_urdf.py SOURCE.urdf OUTPUT.urdf NAMESPACE [ROBOT_NAME]
"""

import sys
from pathlib import Path

ROTORS = [("rotor_fl_joint", "rotor_fl_link", "ccw", 1, 0),
          ("rotor_fr_joint", "rotor_fr_link", "cw", -1, 1),
          ("rotor_rl_joint", "rotor_rl_link", "cw", -1, 2),
          ("rotor_rr_joint", "rotor_rr_link", "ccw", 1, 3)]
MOTOR_CONSTANT = 2.5e-06
MOMENT_CONSTANT = 4.0e-03


def plugins(namespace: str) -> str:
    blocks = []
    for joint, link, direction, _, actuator in ROTORS:
        blocks.append(f"""    <plugin filename="gz-sim-multicopter-motor-model-system" name="gz::sim::systems::MulticopterMotorModel">
      <robotNamespace>{namespace}</robotNamespace>
      <jointName>{joint}</jointName>
      <linkName>{link}</linkName>
      <turningDirection>{direction}</turningDirection>
      <timeConstantUp>0.0125</timeConstantUp>
      <timeConstantDown>0.025</timeConstantDown>
      <maxRotVelocity>1200.0</maxRotVelocity>
      <motorConstant>{MOTOR_CONSTANT}</motorConstant>
      <momentConstant>{MOMENT_CONSTANT}</momentConstant>
      <commandSubTopic>gazebo/command/motor_speed</commandSubTopic>
      <actuator_number>{actuator}</actuator_number>
      <rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
      <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
      <motorSpeedPubTopic>motor_speed/{actuator}</motorSpeedPubTopic>
      <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim>
      <motorType>velocity</motorType>
    </plugin>""")
    rotors = "\n".join(f"""        <rotor>
          <jointName>{joint}</jointName>
          <forceConstant>{MOTOR_CONSTANT}</forceConstant>
          <momentConstant>{MOMENT_CONSTANT}</momentConstant>
          <direction>{control}</direction>
        </rotor>""" for joint, _, _, control, _ in ROTORS)
    blocks.append(f"""    <plugin filename="gz-sim-multicopter-control-system" name="gz::sim::systems::MulticopterVelocityControl">
      <robotNamespace>{namespace}</robotNamespace>
      <commandSubTopic>gazebo/command/twist</commandSubTopic>
      <enableSubTopic>enable</enableSubTopic>
      <comLinkName>base_footprint</comLinkName>
      <velocityGain>2.7 2.7 2.7</velocityGain>
      <attitudeGain>2 3 0.15</attitudeGain>
      <angularRateGain>0.4 0.52 0.18</angularRateGain>
      <maximumLinearAcceleration>2 2 2</maximumLinearAcceleration>
      <maximumLinearVelocity>3 3 3</maximumLinearVelocity>
      <maximumAngularVelocity>1 1 1</maximumAngularVelocity>
      <rotorConfiguration>
{rotors}
      </rotorConfiguration>
    </plugin>""")
    return "  <gazebo>\n" + "\n".join(blocks) + "\n  </gazebo>\n"


def main() -> None:
    source, output, namespace = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
    text = source.read_text()
    if len(sys.argv) > 4:
        text = text.replace('<robot name="parrot_bebop_2">', f'<robot name="{sys.argv[4]}">', 1)
    output.write_text(text.replace("</robot>", plugins(namespace) + "</robot>", 1))


if __name__ == "__main__":
    main()
