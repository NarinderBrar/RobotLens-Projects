#!/usr/bin/env python3
"""Scene description fixture for RobotLens's explorer_r2 example.

Publishes the static SDF scene models that belong to this fixture as the
`scene_models` parameter, mirroring how robot_state_publisher publishes
`robot_description`. RobotLens's UrdfFetcher polls
/scene_description_publisher/get_parameters and loads each listed folder as an
SDF scene model alongside the live URDF, so running explorer_r2.launch.py shows
the drone and the open_interior environment together out of the box.

Each entry must be an absolute path to an SDF model folder — either a
Fuel-style model.config + model.sdf, or a Gazebo world folder containing a
.sdf file with a <world> root and inline <model> elements.  Paths are rewritten
to absolute by the launch file so RobotLens resolves mesh URIs against the folder,
exactly like a manual File > Load SDF Model would.
"""

from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

EXAMPLE_DIR = Path(__file__).resolve().parents[1]
WORLDS_DIR = Path(__file__).resolve().parents[2] / 'worlds'


class SceneDescriptionNode(Node):
    def __init__(self):
        super().__init__('scene_description_publisher')
        # open_interior: a Gazebo world folder with inline <model> elements
        # (walls, shelves, blockers) — RobotLens's SdfModelParser::parseFolder
        # detects the <world> root and loads each model automatically.
        self.declare_parameter('scene_models', [str(WORLDS_DIR / 'open_interior')])
        models = self.get_parameter('scene_models').value
        self.get_logger().info(
            'scene_description_publisher started: scene_models=%s' % list(models)
        )


def main(args=None):
    rclpy.init(args=args)
    node = SceneDescriptionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
