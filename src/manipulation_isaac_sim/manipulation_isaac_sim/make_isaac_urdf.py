"""Write the URDF and the import manifest for the Isaac Sim side (isaac_sim/run.sh)."""

import argparse
import json
import os
import sys
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from manipulation_isaac_sim import isaac_urdf
import xacro

PACKAGE_NAME = 'manipulation_isaac_sim'
URDF_NAME = 'robot.urdf'
MANIFEST_NAME = 'manifest.json'


def default_xacro_path() -> str:
    return os.path.join(
        get_package_share_directory(PACKAGE_NAME), 'urdf', 'lwa4p_isaac.urdf.xacro')


def default_srdf_path() -> str:
    return os.path.join(
        get_package_share_directory('lwa4p_moveit_config'), 'config', 'lwa4p.srdf')


def parse_arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, help='directory for the URDF and manifest')
    parser.add_argument('--xacro', default=None, help='robot description xacro')
    parser.add_argument('--srdf', default=None, help='SRDF with the disabled collision pairs')
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    arguments = parse_arguments(sys.argv[1:] if argv is None else argv)
    xacro_path = arguments.xacro or default_xacro_path()
    srdf_path = arguments.srdf or default_srdf_path()

    root = isaac_urdf.prepare_urdf(
        xacro.process_file(xacro_path).toxml(), get_package_share_directory)
    urdf_xml = ElementTree.tostring(root, encoding='unicode')
    with open(srdf_path, encoding='utf-8') as srdf_file:
        srdf_xml = srdf_file.read()

    os.makedirs(arguments.output_dir, exist_ok=True)
    urdf_path = os.path.join(arguments.output_dir, URDF_NAME)
    with open(urdf_path, 'w', encoding='utf-8') as urdf_file:
        urdf_file.write(urdf_xml)
    manifest = {
        'urdf': os.path.abspath(urdf_path),
        'digest': isaac_urdf.source_digest(urdf_xml, isaac_urdf.mesh_paths(root)),
        'effort_limits': isaac_urdf.joint_effort_limits(root),
        'disabled_collision_pairs': isaac_urdf.disabled_collision_pairs(srdf_xml),
    }
    manifest_path = os.path.join(arguments.output_dir, MANIFEST_NAME)
    with open(manifest_path, 'w', encoding='utf-8') as manifest_file:
        json.dump(manifest, manifest_file, indent=2)
        manifest_file.write('\n')
    print(os.path.abspath(manifest_path))
    return 0


if __name__ == '__main__':
    sys.exit(main())
