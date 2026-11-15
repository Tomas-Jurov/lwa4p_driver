"""
Isaac Sim backend for the LWA 4P: robot, scene, /clock and ros2_control.

Run through isaac_sim/run.sh, which prepares the URDF and the environment.
"""

import argparse
import json
from pathlib import Path
import sys

from isaacsim import SimulationApp


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-manifest', required=True, type=Path,
                        help='manifest.json written by make_isaac_urdf')
    parser.add_argument('--controllers', required=True, help='ros2_control YAML')
    parser.add_argument('--config', required=True, type=Path, help='isaac_sim/config/sim.yaml')
    parser.add_argument('--cache-dir', required=True, type=Path, help='USD cache directory')
    parser.add_argument('--headless', action='store_true', help='no simulator window')
    parser.add_argument('--max-time', type=float, default=None,
                        help='stop after this much simulated time [s] (for tests)')
    arguments, _ = parser.parse_known_args()
    return arguments


arguments = parse_arguments()
simulation_app = SimulationApp({'headless': arguments.headless})

# Isaac modules exist only after SimulationApp has started.
import isaacsim.core.experimental.utils.app as app_utils  # noqa: E402
import isaacsim.core.experimental.utils.stage as stage_utils  # noqa: E402
from isaacsim.core.rendering_manager import ViewportManager  # noqa: E402
from isaacsim.core.simulation_manager import SimulationManager  # noqa: E402
from simulation import robot_import  # noqa: E402
from simulation import robot_setup  # noqa: E402
from simulation import ros_bridge  # noqa: E402
from simulation import scene  # noqa: E402
import yaml  # noqa: E402

ROBOT_PATH = '/World/lwa4p'


def main() -> int:
    config = yaml.safe_load(arguments.config.read_text())
    robot_manifest = json.loads(arguments.robot_manifest.read_text())

    ros_bridge.enable_ros_extensions(app_utils)
    simulation_app.update()

    usd_path = robot_import.import_robot(robot_manifest, config['import'], arguments.cache_dir)
    print(f'[isaac_sim] robot USD: {usd_path}', flush=True)

    stage_utils.create_new_stage()
    stage_utils.set_stage_units(meters_per_unit=1.0)
    stage = stage_utils.get_current_stage(backend='usd')
    scene.create_scene(config['scene'])
    stage_utils.add_reference_to_stage(
        usd_path=str(usd_path), path=ROBOT_PATH, variants=[('Physics', 'physx')])
    robot_setup.configure_robot(stage, ROBOT_PATH, robot_manifest, config)
    ros_bridge.create_ros_graph(ROBOT_PATH, arguments.controllers)
    simulation_app.update()

    viewport = config['viewport']
    ViewportManager.set_camera_view(
        '/OmniverseKit_Persp', eye=viewport['eye'], target=viewport['target'])
    SimulationManager.setup_simulation(
        dt=config['physics']['dt'], device=config['physics']['device'])
    app_utils.play()
    print('[isaac_sim] running; start the ROS side: '
          'ros2 launch manipulation_isaac_sim isaac_moveit.launch.py', flush=True)

    try:
        while simulation_app.is_running():
            simulation_app.update()
            if (arguments.max_time is not None
                    and SimulationManager.get_simulation_time() >= arguments.max_time):
                break
    except KeyboardInterrupt:
        pass

    app_utils.stop()
    simulation_app.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
