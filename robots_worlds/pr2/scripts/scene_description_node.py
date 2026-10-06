#!/usr/bin/env python3
"""Scene description fixture for RobotLens's pr2 example.

Publishes the SDF scene models that belong to this fixture as the
`scene_models` parameter. RobotLens's UrdfFetcher polls
/scene_description_publisher/get_parameters and loads each listed folder as an
SDF scene model. The list contains the native PR2 SDF17 model, the shared
warehouse, and PR2-local table/reference-object visuals, so this applies only
to the PR2 launches.

Each entry must be an absolute path to an SDF model folder -- either a
Fuel-style model.config + model.sdf, or a Gazebo world folder containing a
.sdf file with a <world> root and inline <model> elements. Paths are
absolute already (derived from __file__), so RobotLens resolves mesh URIs
against the folder, exactly like a manual File > Load SDF Model would.
"""

from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

EXAMPLE_DIR = Path(__file__).resolve().parents[1]
WORLDS_DIR = Path(__file__).resolve().parents[2] / 'worlds'
PR2_SCENE_MODELS_DIR = EXAMPLE_DIR / 'scene_models'


class SceneDescriptionNode(Node):
    def __init__(self):
        super().__init__('scene_description_publisher')
        # open_interior: a Gazebo world folder with inline <model> elements
        # (walls, shelves, blockers) -- RobotLens's SdfModelParser::parseFolder
        # detects the <world> root and loads each model automatically.
        self.declare_parameter('scene_models', [
            # The physics launch names its URDF state publisher
            # pr2_tf_publisher, not robot_state_publisher, so this is the
            # authoritative RobotLens PR2 geometry for that launch.
            str(EXAMPLE_DIR / 'PR2_SDF17'),
            str(WORLDS_DIR / 'open_interior'),
            str(PR2_SCENE_MODELS_DIR),
        ])
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
