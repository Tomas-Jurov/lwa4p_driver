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

PACKAGE_NAME = 'manipulation_isaac_sim'
URDF_NAME = 'robot.urdf'
MANIFEST_NAME = 'manifest.json'
MESH_DIR_NAME = 'meshes'
CONTROLLERS_NAME = 'isaac_controllers.yaml'


def default_xacro_path() -> str:
    return os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'urdf', 'lwa4p_isaac.urdf.xacro')


def default_srdf_path() -> str:
    return os.path.join(
        get_package_share_directory('lwa4p_moveit_config'), 'config', 'lwa4p.srdf')


def default_controllers_path() -> str:
    return os.path.join(get_package_share_directory(PACKAGE_NAME), 'config', CONTROLLERS_NAME)


def parse_arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, help='directory for the robot files')
    parser.add_argument('--xacro', default=None, help='robot description xacro')
    parser.add_argument('--srdf', default=None, help='SRDF with the disabled collision pairs')
    parser.add_argument('--controllers', default=None,
                        help='ros2_control YAML of the controller manager inside Isaac Sim')
    return parser.parse_args(argv)


def copy_meshes(mesh_copies: dict[str, str], mesh_dir: str) -> None:
    # Remove the copies of an earlier run, so meshes Tomas deleted do not linger.
    shutil.rmtree(mesh_dir, ignore_errors=True)
    for source, copy in mesh_copies.items():
        os.makedirs(os.path.dirname(copy), exist_ok=True)
        shutil.copyfile(source, copy)


def main(argv: list[str] | None = None) -> int:
    arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
    xacro_path = arguments.xacro or default_xacro_path()
    srdf_path = arguments.srdf or default_srdf_path()
    controllers_source = arguments.controllers or default_controllers_path()
    output_dir = os.path.abspath(arguments.output_dir)
    mesh_dir = os.path.join(output_dir, MESH_DIR_NAME)

    root = isaac_urdf.prepare_urdf(
        xacro.process_file(xacro_path).toxml(), get_package_share_directory)
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
        'effort_limits': isaac_urdf.joint_effort_limits(root),
        'disabled_collision_pairs': isaac_urdf.disabled_collision_pairs(srdf_xml),
    }
    manifest_path = os.path.join(output_dir, MANIFEST_NAME)
    with open(manifest_path, 'w', encoding='utf-8') as manifest_file:
        json.dump(manifest, manifest_file, indent=2)
        manifest_file.write('\n')
    print(manifest_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
