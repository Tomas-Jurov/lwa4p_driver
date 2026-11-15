"""
Turn the ROS robot description into the input of the Isaac Sim URDF importer.

No ROS imports: the caller passes the expanded URDF, the SRDF and a package resolver.
"""

from collections.abc import Callable
from collections.abc import Iterable
import hashlib
import os
from xml.etree import ElementTree

PACKAGE_PREFIX = 'package://'
FILE_PREFIX = 'file://'
ACTUATED_JOINT_TYPES = ('revolute', 'prismatic', 'continuous')
# Tags the importer does not need: ros2_control belongs to the simulated controller
# manager, which builds its own description from the USD stage.
SIMULATOR_SPECIFIC_TAGS = ('ros2_control', 'gazebo')
# The importer turns a link without <inertial> into a plain transform, not a rigid body.
# A joint hanging off such a link (ftm_joint on arm_tool0) is then lost when
# isaacsim.ros2.control exports the USD back to URDF, and with it the whole tool.
# A connector link therefore gets a negligible mass (1 g against 0.7 kg of the FTM).
CONNECTOR_MASS_KG = 0.001
CONNECTOR_INERTIA_KG_M2 = 1e-7


def prepare_urdf(urdf_xml: str, resolve_package: Callable[[str], str]) -> ElementTree.Element:
    """
    Return the URDF root ready for the importer.

    Without simulator tags, with absolute mesh paths, and with a negligible mass on
    every massless link that connects other links.
    """
    root = ElementTree.fromstring(urdf_xml)
    for tag in SIMULATOR_SPECIFIC_TAGS:
        for element in root.findall(tag):
            root.remove(element)
    for mesh in root.iter('mesh'):
        mesh.set('filename', resolve_mesh_path(mesh.get('filename', ''), resolve_package))
    for link_name in massless_connector_links(root):
        _add_connector_inertial(root.find(f"link[@name='{link_name}']"))
    return root


def massless_connector_links(root: ElementTree.Element) -> list[str]:
    """Return links without <inertial> that are neither the root nor a leaf frame."""
    parent_links = {joint.find('parent').get('link') for joint in root.findall('joint')}
    child_links = {joint.find('child').get('link') for joint in root.findall('joint')}
    return [
        link.get('name') for link in root.findall('link')
        if link.find('inertial') is None
        and link.get('name') in parent_links and link.get('name') in child_links
    ]


def _add_connector_inertial(link: ElementTree.Element) -> None:
    inertial = ElementTree.SubElement(link, 'inertial')
    ElementTree.SubElement(inertial, 'mass', {'value': repr(CONNECTOR_MASS_KG)})
    inertia = {axes: '0' for axes in ('ixy', 'ixz', 'iyz')}
    inertia.update({axis: repr(CONNECTOR_INERTIA_KG_M2) for axis in ('ixx', 'iyy', 'izz')})
    ElementTree.SubElement(inertial, 'inertia', inertia)


def resolve_mesh_path(filename: str, resolve_package: Callable[[str], str]) -> str:
    """Convert a package:// or file:// mesh URI to an absolute file path."""
    if filename.startswith(PACKAGE_PREFIX):
        package, _, relative_path = filename[len(PACKAGE_PREFIX):].partition('/')
        return os.path.join(resolve_package(package), relative_path)
    if filename.startswith(FILE_PREFIX):
        return filename[len(FILE_PREFIX):]
    if not os.path.isabs(filename):
        raise ValueError(f'mesh path is neither a URI nor absolute: {filename!r}')
    return filename


def mesh_paths(root: ElementTree.Element) -> list[str]:
    """Return the sorted unique mesh files referenced by the URDF."""
    return sorted({mesh.get('filename') for mesh in root.iter('mesh')})


def joint_effort_limits(root: ElementTree.Element) -> dict[str, float]:
    """Return the URDF effort limit of every actuated joint."""
    limits = {}
    for joint in root.findall('joint'):
        if joint.get('type') not in ACTUATED_JOINT_TYPES:
            continue
        limit = joint.find('limit')
        if limit is None or limit.get('effort') is None:
            raise ValueError(f'joint {joint.get("name")!r} has no effort limit')
        limits[joint.get('name')] = float(limit.get('effort'))
    return limits


def disabled_collision_pairs(srdf_xml: str) -> list[tuple[str, str]]:
    """Return the link pairs the SRDF excludes from collision checking, sorted."""
    root = ElementTree.fromstring(srdf_xml)
    pairs = {
        tuple(sorted((element.get('link1'), element.get('link2'))))
        for element in root.iter('disable_collisions')
    }
    return sorted(pairs)


def source_digest(urdf_xml: str, files: Iterable[str]) -> str:
    """Hash the URDF text and the bytes of the files it references."""
    digest = hashlib.sha256(urdf_xml.encode())
    for path in sorted(files):
        digest.update(path.encode())
        with open(path, 'rb') as file:
            digest.update(file.read())
    return digest.hexdigest()[:16]
