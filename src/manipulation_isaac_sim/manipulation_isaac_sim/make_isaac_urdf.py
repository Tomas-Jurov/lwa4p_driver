"""
Write the robot files for the Isaac Sim side (isaac_sim/run.sh) into one directory.

The URDF, copies of its meshes, the controllers YAML and the import manifest.
The directory is self-contained: Isaac Sim reads it without a ROS installation, for
example when the ROS side runs in a container (docker/run_jazzy_hybrid.sh).
"""

import argparse
import json
import os
import shutil
import sys
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from manipulation_isaac_sim import isaac_urdf
import xacro
import yaml

PACKAGE_NAME = 'manipulation_isaac_sim'
URDF_NAME = 'robot.urdf'
MANIFEST_NAME = 'manifest.json'
MESH_DIR_NAME = 'meshes'
CONTROLLERS_NAME = 'isaac_controllers.yaml'
LINK_INERTIALS_NAME = 'link_inertials.yaml'
COLLISION_OVERRIDES_NAME = 'collision_overrides.yaml'
# datasheet: the arm masses of config/link_inertials.yaml, inertia from the meshes;
# urdf: Tomas's inertials unchanged.
INERTIALS_CHOICES = ('datasheet', 'urdf')


def default_xacro_path() -> str:
    return os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'urdf', 'lwa4p_isaac.urdf.xacro')


def default_srdf_path() -> str:
    return os.path.join(
        get_package_share_directory('lwa4p_moveit_config'), 'config', 'lwa4p.srdf')


def default_controllers_path() -> str:
    return os.path.join(get_package_share_directory(PACKAGE_NAME), 'config', CONTROLLERS_NAME)


def default_link_inertials_path() -> str:
    return os.path.join(get_package_share_directory(PACKAGE_NAME), 'config', LINK_INERTIALS_NAME)


def default_collision_overrides_path() -> str:
    return os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'config', COLLISION_OVERRIDES_NAME)


def load_links(path: str) -> dict:
    with open(path, encoding='utf-8') as file:
        return yaml.safe_load(file)['links']


def parse_arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, help='directory for the robot files')
    parser.add_argument('--xacro', default=None, help='robot description xacro')
    parser.add_argument('--srdf', default=None, help='SRDF with the disabled collision pairs')
    parser.add_argument('--controllers', default=None,
                        help='ros2_control YAML of the controller manager inside Isaac Sim')
    parser.add_argument('--inertials', choices=INERTIALS_CHOICES, default='datasheet',
                        help='datasheet: arm masses from --link-inertials with inertia from '
                             "the meshes (default); urdf: keep Tomas's inertials")
    parser.add_argument('--link-inertials', default=None,
                        help='YAML with the link masses '
                             f'(default: config/{LINK_INERTIALS_NAME})')
    parser.add_argument('--collision-overrides', default=None,
                        help='YAML with the collision shapes replaced for Isaac Sim only '
                             f'(default: config/{COLLISION_OVERRIDES_NAME})')
    return parser.parse_args(argv)


def copy_meshes(mesh_copies: dict[str, str], mesh_dir: str) -> None:
    # Remove the copies of an earlier run, so meshes Tomas deleted do not linger.
    shutil.rmtree(mesh_dir, ignore_errors=True)
    for source, copy in mesh_copies.items():
        os.makedirs(os.path.dirname(copy), exist_ok=True)
        shutil.copyfile(source, copy)


def named_states_with_mimics(root: ElementTree.Element, srdf_xml: str) -> dict:
    """Return the SRDF states, each with the positions of the mimic joints it implies."""
    states = isaac_urdf.named_states(srdf_xml)
    for group_states in states.values():
        for positions in group_states.values():
            positions.update(isaac_urdf.mimic_positions(root, positions))
    return states


def main(argv: list[str] | None = None) -> int:
    arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
    xacro_path = arguments.xacro or default_xacro_path()
    srdf_path = arguments.srdf or default_srdf_path()
    controllers_source = arguments.controllers or default_controllers_path()
    output_dir = os.path.abspath(arguments.output_dir)
    mesh_dir = os.path.join(output_dir, MESH_DIR_NAME)

    root = isaac_urdf.prepare_urdf(
        xacro.process_file(xacro_path).toxml(), get_package_share_directory)
    collision_overrides = isaac_urdf.replace_collisions(
        root, load_links(arguments.collision_overrides or default_collision_overrides_path()))
    if arguments.inertials == 'datasheet':
        isaac_urdf.apply_link_inertials(
            root, load_links(arguments.link_inertials or default_link_inertials_path()))
    mesh_copies = isaac_urdf.bundle_meshes(root, mesh_dir)
    urdf_xml = ElementTree.tostring(root, encoding='unicode')
    with open(srdf_path, encoding='utf-8') as srdf_file:
        srdf_xml = srdf_file.read()

    os.makedirs(output_dir, exist_ok=True)
    copy_meshes(mesh_copies, mesh_dir)
    controllers_path = os.path.join(output_dir, CONTROLLERS_NAME)
    shutil.copyfile(controllers_source, controllers_path)
    urdf_path = os.path.join(output_dir, URDF_NAME)
    with open(urdf_path, 'w', encoding='utf-8') as urdf_file:
        urdf_file.write(urdf_xml)
    manifest = {
        'urdf': urdf_path,
        'controllers': controllers_path,
        'digest': isaac_urdf.source_digest(urdf_xml, mesh_copies.values()),
        'inertials': arguments.inertials,
        'collision_overrides': collision_overrides,
        'effort_limits': isaac_urdf.joint_effort_limits(root),
        'disabled_collision_pairs': isaac_urdf.disabled_collision_pairs(srdf_xml),
        'named_states': named_states_with_mimics(root, srdf_xml),
        'mounted_links': isaac_urdf.mounted_links(root),
    }
    manifest_path = os.path.join(output_dir, MANIFEST_NAME)
    with open(manifest_path, 'w', encoding='utf-8') as manifest_file:
        json.dump(manifest, manifest_file, indent=2)
        manifest_file.write('\n')
    print(manifest_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
