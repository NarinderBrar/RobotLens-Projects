#!/usr/bin/env python3
"""
RobotLens panda Gazebo fixture runner.

Spawns the Franka Emika Panda arm (robots/, a Fuel-style SDF model
folder -- model.config + model.sdf, no URDF conversion needed, unlike
TurtleBot3) into the shared open_interior world.

The model is spawned <static>true</static>: nothing commands its joints yet
(no ros2_control/MoveIt wiring -- this fixture is spawn+view+/joint_states
only), and an unconstrained revolute-jointed arm under real gravity with no
motor holding it would just droop/collapse. A static model has no dynamics
at all, so it holds its default joint pose cleanly; the JointStatePublisher
plugin still reads and publishes joint state from it (empirically confirmed
against this exact model -- position/limit/damping data all present).

robots/'s own mesh URIs use the model://panda/... scheme (see
model.config's <name>panda</name>), which needs a directory literally named
"panda" (not "robots") somewhere under GZ_SIM_RESOURCE_PATH --
ensure_resource_symlink() creates that alias.
"""
import os
import sys
from pathlib import Path

EXAMPLE_DIR = Path(__file__).resolve().parents[1]
WORLDS_DIR = Path(__file__).resolve().parents[2] / 'worlds'
PANDA_MODEL_DIR = EXAMPLE_DIR / 'robots'
STATE_DIR = Path(os.environ.get('TMPDIR', '/tmp')) / 'robotlens_panda_gazebo'
GENERATED_WORLD_PATH = STATE_DIR / 'panda_world.sdf'
RESOURCE_DIR = STATE_DIR / 'resources'


def ensure_resource_symlink():
    """resources/panda -> robots, so model://panda/... URIs resolve."""
    RESOURCE_DIR.mkdir(parents=True, exist_ok=True)
    link = RESOURCE_DIR / 'panda'
    if link.is_symlink() or link.exists():
        link.unlink()
    link.symlink_to(PANDA_MODEL_DIR, target_is_directory=True)
    return RESOURCE_DIR


def write_open_interior_world():
    """Build the open_interior world and include the static panda arm."""
    world_path = WORLDS_DIR / "open_interior" / "open_interior.sdf"
    world_text = world_path.read_text(encoding="utf-8")
    world_text = world_text.replace(
        """    <!-- Sun -->
    <include>
      <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun</uri>
    </include>""",
        """    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>""",
    )
    if "</world>" not in world_text:
        sys.exit(f"error: open_interior world has no closing world tag: {world_path}")

    include_block = f"""    <include>
      <uri>{PANDA_MODEL_DIR}</uri>
      <static>true</static>
      <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
    </include>
"""
    GENERATED_WORLD_PATH.parent.mkdir(parents=True, exist_ok=True)
    GENERATED_WORLD_PATH.write_text(
        world_text.replace("</world>", include_block + "</world>", 1),
        encoding="utf-8",
    )
    ensure_resource_symlink()
