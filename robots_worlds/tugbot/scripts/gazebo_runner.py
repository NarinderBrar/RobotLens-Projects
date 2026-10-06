#!/usr/bin/env python3
"""
RobotLens tugbot Gazebo fixture runner.

Spawns the tugbot AMR (../tugbot, a Fuel-style SDF model folder --
model.config + model.sdf) into a generated world alongside the shared
warehouse model (robots_worlds/worlds/warehouse, another Fuel-style model folder
-- a static building shell only, no <world> wrapper of its own). No URDF
conversion is needed -- unlike TurtleBot3, tugbot's own model.sdf already
ships complete DiffDrive, PosePublisher, JointStatePublisher, and
JointController plugins (vendored from Fuel; the plugin filenames/class
names were originally Ignition/Gazebo Citadel's ignition-gazebo-*-system /
ignition::gazebo::systems::* and have been rewritten in model.sdf to this
repo's Gazebo Harmonic install's gz-sim-*-system / gz::sim::systems::*
names -- see model.sdf's own plugin-block comment).

Both tugbot's and warehouse's own mesh URIs use their respective
model://tugbot/... / model://warehouse/... schemes (see each model.config's
<name>), which resolve because both folders are already named to match --
GZ_SIM_RESOURCE_PATH just needs to list each folder's *parent* directory
(this example's own directory, and worlds/), no alias symlink needed
(contrast robots_worlds/panda/scripts/gazebo_runner.py, whose model folder is
named "robots" and needs one).
"""
import os
import shutil
import sys
from pathlib import Path

EXAMPLE_DIR = Path(__file__).resolve().parents[1]
WORLDS_DIR = Path(__file__).resolve().parents[2] / 'worlds'
TUGBOT_MODEL_DIR = EXAMPLE_DIR / 'tugbot'
WAREHOUSE_MODEL_DIR = WORLDS_DIR / 'warehouse'
STATE_DIR = Path(os.environ.get('TMPDIR', '/tmp')) / 'robotlens_tugbot_gazebo'
GENERATED_WORLD_PATH = STATE_DIR / 'tugbot_world.sdf'
RESOURCE_DIR = f"{EXAMPLE_DIR}:{WORLDS_DIR}"
# This generated world's own <world name="...">, needed verbatim for the
# /world/<name>/model/tugbot/joint_state bridge topic in launch/tugbot.launch.py.
WORLD_NAME = 'tugbot_view'

# Boilerplate mirrors robots_worlds/worlds/open_interior/open_interior.sdf's own
# physics/plugin/light setup, for consistency with the other gz-sim
# fixtures in this repo. No ground_plane here -- the warehouse model's own
# floor mesh (warehouse_base's collision) serves that purpose.
WORLD_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{world_name}">
    <gravity>0 0 -9.81</gravity>
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre</render_engine>
    </plugin>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <include>
      <uri>{warehouse_dir}</uri>
    </include>
    <include>
      <uri>{tugbot_dir}</uri>
    </include>
  </world>
</sdf>
"""


def require_gz():
    if shutil.which('gz', path='/usr/bin:/bin') is None:
        sys.exit("error: 'gz' (Gazebo Harmonic) not found on /usr/bin:/bin")


def write_warehouse_world():
    """Build the warehouse + tugbot world used by this launch."""
    GENERATED_WORLD_PATH.parent.mkdir(parents=True, exist_ok=True)
    GENERATED_WORLD_PATH.write_text(
        WORLD_TEMPLATE.format(
            world_name=WORLD_NAME,
            warehouse_dir=WAREHOUSE_MODEL_DIR,
            tugbot_dir=TUGBOT_MODEL_DIR,
        ),
        encoding="utf-8",
    )
    return GENERATED_WORLD_PATH
