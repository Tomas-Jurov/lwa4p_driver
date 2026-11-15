"""
The world around the robot.

For now only a ground plane at the lab table height and a light; the worlds converted
from Tomas's SDF files replace it.
"""

from isaacsim.core.experimental.objects import DistantLight
from isaacsim.core.experimental.objects import GroundPlane

GROUND_PATH = '/World/GroundPlane'
LIGHT_PATH = '/World/Light'
LIGHT_INTENSITY = 3000.0


def create_scene(config: dict) -> None:
    GroundPlane(GROUND_PATH, positions=[0.0, 0.0, config['ground_z']])
    DistantLight(LIGHT_PATH).set_intensities(LIGHT_INTENSITY)
