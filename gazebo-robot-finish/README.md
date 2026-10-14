# Gazebo robot finish test

RobotLens project: `.robotlens/gazebo-robot-finish.xml`. It uses ROS 2 communication and the
Gazebo simulator with RobotLens's default ground-plane world.

It checks the built-in procedural robot workflow with Gazebo: add a robot from the Robots panel
(`robot.add`), change its settings (`robot.configure`), and press **Finish robot** (`robot.finish`).
Finishing writes an SDF 1.9 model to `robots/<name>.sdf`, imports it as a normal robot, and the
simulator setup merges it into the world. See ADR 0057 in RobotLens-Docs.
