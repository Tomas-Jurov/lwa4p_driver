"""
Turn the ROS robot description into the input of the Isaac Sim URDF importer.

No ROS imports: the caller passes the expanded URDF, the SRDF and a package resolver.
"""

from collections.abc import Callable
from collections.abc import Iterable
import hashlib
import os
from xml.etree import ElementTree

from manipulation_isaac_sim import mesh_inertia

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
# So does a massless link with a collision shape (the real camera_link): without a rigid
# body its collider would not take part in the simulation.
CONNECTOR_MASS_KG = 0.001
CONNECTOR_INERTIA_KG_M2 = 1e-7
# Tomas's measured lab obstacles (lwa4p_description/urdf/workspace_obstacles.urdf.xacro):
# links fixed to the world link. A simulated world that has them as objects replaces them.
OBSTACLE_PREFIX = 'obstacle_'
WORLD_LINK = 'world'
# An SDF plane becomes a box this thick, its top face on the plane.
PLANE_THICKNESS_M = 0.01


def prepare_urdf(urdf_xml: str, resolve_package: Callable[[str], str]) -> ElementTree.Element:
    """
    Return the URDF root ready for the importer.

    Without simulator tags and workspace obstacles (the world has them), with absolute
    mesh paths, and with a negligible mass on every massless link that connects other
    links or has a collision shape.
    """
    root = ElementTree.fromstring(urdf_xml)
    for tag in SIMULATOR_SPECIFIC_TAGS:
        for element in root.findall(tag):
            root.remove(element)
    remove_workspace_obstacles(root)
    for mesh in root.iter('mesh'):
        mesh.set('filename', resolve_mesh_path(mesh.get('filename', ''), resolve_package))
    for link_name in massless_connector_links(root) + massless_collision_links(root):
        _add_connector_inertial(root.find(f"link[@name='{link_name}']"))
    return root


def apply_link_inertials(root: ElementTree.Element, links: dict[str, dict]) -> dict[str, float]:
    """
    Replace the <inertial> of the listed links; return {link: mass}.

    links maps a link name to {'mass': kg, 'model': 'solid' | 'shell'}. The centre of
    mass and the inertia tensor come from the link's single visual mesh carrying that
    mass, so they follow Tomas's meshes. A link without a visual mesh but with a single
    collision box (the camera) is a solid box. Call after prepare_urdf, which makes mesh
    paths absolute, and after replace_collisions.
    """
    masses = {}
    for name, spec in links.items():
        link = root.find(f"link[@name='{name}']")
        if link is None:
            raise ValueError(f'link inertials: the model has no link {name!r}')
        unknown = set(spec) - {'mass', 'model'}
        if unknown:
            raise ValueError(f'link inertials: unknown keys {sorted(unknown)} for {name!r}')
        mesh = _single_visual_mesh(link)
        if mesh is not None:
            scale = tuple(float(value) for value in mesh.get('scale', '1 1 1').split())
            properties = mesh_inertia.mass_properties(
                mesh_inertia.read_stl(mesh.get('filename')), float(spec['mass']),
                spec['model'], scale)
        else:
            properties = _box_mass_properties(link, float(spec['mass']), spec['model'])
        for inertial in link.findall('inertial'):
            link.remove(inertial)
        _add_inertial(link, properties)
        masses[name] = properties.mass
    return masses


def replace_collisions(root: ElementTree.Element, links: dict[str, list]) -> list[str]:
    """
    Replace the <collision> elements of the listed links; return the links.

    links maps a link name to a list of shapes, each {'box': [x, y, z]},
    {'cylinder': {'radius': r, 'length': l}} or {'sphere': r}, with an optional
    'origin': {'xyz': [...], 'rpy': [...]} in the link frame.
    """
    for name, shapes in links.items():
        link = root.find(f"link[@name='{name}']")
        if link is None:
            raise ValueError(f'collision overrides: the model has no link {name!r}')
        for collision in link.findall('collision'):
            link.remove(collision)
        for shape in shapes:
            _add_collision_shape(link, shape, name)
    return sorted(links)


def _add_collision_shape(link: ElementTree.Element, shape: dict, name: str) -> None:
    geometry_keys = set(shape) & {'box', 'cylinder', 'sphere'}
    unknown = set(shape) - {'box', 'cylinder', 'sphere', 'origin'}
    if len(geometry_keys) != 1 or unknown:
        raise ValueError(f'collision overrides: {name!r} needs one of box, cylinder, sphere '
                         f'and optionally origin, not {sorted(shape)}')
    collision = ElementTree.SubElement(link, 'collision')
    origin = shape.get('origin', {})
    ElementTree.SubElement(collision, 'origin', {
        'xyz': _join(origin.get('xyz', [0.0] * 3)), 'rpy': _join(origin.get('rpy', [0.0] * 3))})
    geometry = ElementTree.SubElement(collision, 'geometry')
    if 'box' in shape:
        ElementTree.SubElement(geometry, 'box', {'size': _join(shape['box'])})
    elif 'cylinder' in shape:
        ElementTree.SubElement(geometry, 'cylinder', {
            'radius': repr(float(shape['cylinder']['radius'])),
            'length': repr(float(shape['cylinder']['length']))})
    else:
        ElementTree.SubElement(geometry, 'sphere', {'radius': repr(float(shape['sphere']))})


def _single_visual_mesh(link: ElementTree.Element) -> ElementTree.Element | None:
    """Return the link's only visual mesh, or None when the link has no visual mesh."""
    visuals = link.findall('visual')
    meshes = [visual.find('geometry/mesh') for visual in visuals]
    if not any(mesh is not None for mesh in meshes):
        return None
    if len(visuals) != 1:
        raise ValueError(f'link inertials: {link.get("name")!r} needs exactly one visual mesh')
    _require_link_frame(visuals[0].find('origin'), link.get('name'), 'visual')
    return meshes[0]


def _box_mass_properties(link: ElementTree.Element, mass: float,
                         model: str) -> mesh_inertia.MassProperties:
    collisions = link.findall('collision')
    boxes = [collision.find('geometry/box') for collision in collisions]
    if len(collisions) != 1 or boxes[0] is None:
        raise ValueError(f'link inertials: {link.get("name")!r} has no visual mesh and not '
                         'exactly one collision box')
    if model != mesh_inertia.SOLID:
        raise ValueError(f'link inertials: a box ({link.get("name")!r}) can only be solid')
    origin = collisions[0].find('origin')
    _require_link_frame(origin, link.get('name'), 'collision', check_xyz=False)
    center = tuple(float(value) for value in
                   (origin.get('xyz', '0 0 0') if origin is not None else '0 0 0').split())
    x, y, z = (float(value) for value in boxes[0].get('size').split())
    diagonal = (mass / 12 * (y * y + z * z), mass / 12 * (x * x + z * z),
                mass / 12 * (x * x + y * y))
    inertia = tuple(tuple(diagonal[row] if row == col else 0.0 for col in range(3))
                    for row in range(3))
    return mesh_inertia.MassProperties(mass, center, inertia)


def _require_link_frame(origin: ElementTree.Element | None, link_name: str, what: str,
                        check_xyz: bool = True) -> None:
    if origin is None:
        return
    values = origin.get('rpy', '0 0 0').split()
    if check_xyz:
        values += origin.get('xyz', '0 0 0').split()
    if any(float(value) != 0.0 for value in values):
        allowed = 'the link frame' if check_xyz else 'unrotated'
        raise ValueError(f'link inertials: the {what} origin of {link_name!r} must be {allowed}')


def _add_inertial(link: ElementTree.Element, properties: mesh_inertia.MassProperties) -> None:
    inertial = ElementTree.SubElement(link, 'inertial')
    ElementTree.SubElement(inertial, 'origin', {
        'xyz': ' '.join(f'{value:.6g}' for value in properties.center_of_mass), 'rpy': '0 0 0'})
    ElementTree.SubElement(inertial, 'mass', {'value': f'{properties.mass:.6g}'})
    tensor = properties.inertia
    ElementTree.SubElement(inertial, 'inertia', {
        f'i{"xyz"[row]}{"xyz"[col]}': f'{tensor[row][col]:.6g}'
        for row, col in ((0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2))})


def remove_workspace_obstacles(root: ElementTree.Element) -> list[str]:
    """Remove the obstacle links fixed to the world link and their joints; return the links."""
    removed = []
    for joint in root.findall('joint'):
        child = joint.find('child').get('link')
        if (joint.get('type') == 'fixed' and joint.find('parent').get('link') == WORLD_LINK
                and child.startswith(OBSTACLE_PREFIX)):
            root.remove(joint)
            root.remove(root.find(f"link[@name='{child}']"))
            removed.append(child)
    return sorted(removed)


def massless_connector_links(root: ElementTree.Element) -> list[str]:
    """Return links without <inertial> that are neither the root nor a leaf frame."""
    parent_links = {joint.find('parent').get('link') for joint in root.findall('joint')}
    child_links = {joint.find('child').get('link') for joint in root.findall('joint')}
    return [
        link.get('name') for link in root.findall('link')
        if link.find('inertial') is None
        and link.get('name') in parent_links and link.get('name') in child_links
    ]


def mounted_links(root: ElementTree.Element) -> list[str]:
    """Return the robot links joined to the root link through fixed joints only, sorted."""
    children = {joint.find('child').get('link') for joint in root.findall('joint')}
    fixed_children = {}
    for joint in root.findall('joint'):
        if joint.get('type') == 'fixed':
            fixed_children.setdefault(joint.find('parent').get('link'), []).append(
                joint.find('child').get('link'))
    queue = [link.get('name') for link in root.findall('link')
             if link.get('name') not in children]
    mounted = []
    while queue:
        link = queue.pop()
        if not link.startswith(OBSTACLE_PREFIX):
            mounted.append(link)
            queue.extend(fixed_children.get(link, []))
    return sorted(mounted)


def add_sdf_obstacles(root: ElementTree.Element, sdf_xml: str) -> list[str]:
    """
    Add the static models of an SDF world as obstacle links fixed to the world link.

    The same mechanism as Tomas's measured lab obstacles, so MoveIt plans around them.
    Dynamic models are left out: they are objects to grasp, not obstacles. No margin is
    added. Returns the new links.
    """
    world = ElementTree.fromstring(sdf_xml).find('world')
    existing = {link.get('name') for link in root.findall('link')}
    added = []
    for model in world.findall('model'):
        if model.findtext('static', 'false').strip() != 'true':
            continue
        name = OBSTACLE_PREFIX + model.get('name')
        if name in existing:
            raise ValueError(f'the description already has a link {name!r}')
        link = ElementTree.SubElement(root, 'link', {'name': name})
        for sdf_link in model.findall('link'):
            if _pose(sdf_link) != ([0.0] * 3, [0.0] * 3):
                raise ValueError(f'{name}: link poses inside a model are not supported')
            for collision in sdf_link.findall('collision'):
                _add_collision(link, collision, name)
        joint = ElementTree.SubElement(root, 'joint', {'name': f'{name}_joint', 'type': 'fixed'})
        ElementTree.SubElement(joint, 'parent', {'link': WORLD_LINK})
        ElementTree.SubElement(joint, 'child', {'link': name})
        xyz, rpy = _pose(model)
        ElementTree.SubElement(joint, 'origin', {'xyz': _join(xyz), 'rpy': _join(rpy)})
        added.append(name)
    return added


def disable_obstacle_collisions(srdf_xml: str, urdf_root: ElementTree.Element) -> str:
    """
    Return the SRDF with collisions disabled among obstacles and the mounted base.

    They never move relative to each other, and they touch (the floor under a stand, the
    base column on its support): MoveIt would find every state in collision.
    """
    srdf = ElementTree.fromstring(srdf_xml)
    obstacles = sorted(link.get('name') for link in urdf_root.findall('link')
                       if link.get('name').startswith(OBSTACLE_PREFIX))
    still = obstacles + mounted_links(urdf_root)
    existing = set(disabled_collision_pairs(srdf_xml))
    for index, first in enumerate(still):
        for second in still[index + 1:]:
            pair = tuple(sorted((first, second)))
            if (first in obstacles or second in obstacles) and pair not in existing:
                ElementTree.SubElement(srdf, 'disable_collisions', {
                    'link1': pair[0], 'link2': pair[1], 'reason': 'Never'})
    return ElementTree.tostring(srdf, encoding='unicode')


def _add_collision(link: ElementTree.Element, collision: ElementTree.Element,
                   name: str) -> None:
    geometry = collision.find('geometry')
    shape = geometry[0] if geometry is not None and len(geometry) else None
    xyz, rpy = _pose(collision)
    element = ElementTree.SubElement(link, 'collision')
    origin = ElementTree.SubElement(element, 'origin')
    urdf_geometry = ElementTree.SubElement(element, 'geometry')
    if shape is not None and shape.tag == 'box':
        ElementTree.SubElement(urdf_geometry, 'box', {'size': shape.findtext('size').strip()})
    elif shape is not None and shape.tag == 'cylinder':
        ElementTree.SubElement(urdf_geometry, 'cylinder', {
            'radius': shape.findtext('radius').strip(),
            'length': shape.findtext('length').strip()})
    elif shape is not None and shape.tag == 'sphere':
        ElementTree.SubElement(urdf_geometry, 'sphere',
                               {'radius': shape.findtext('radius').strip()})
    elif shape is not None and shape.tag == 'plane':
        normal = [float(v) for v in shape.findtext('normal', '0 0 1').split()]
        if normal != [0.0, 0.0, 1.0] or rpy != [0.0] * 3:
            raise ValueError(f'{name}: only horizontal planes are supported')
        size_x, size_y = shape.findtext('size', '100 100').split()
        ElementTree.SubElement(urdf_geometry, 'box', {
            'size': f'{size_x} {size_y} {PLANE_THICKNESS_M}'})
        xyz = [xyz[0], xyz[1], xyz[2] - PLANE_THICKNESS_M / 2]
    else:
        raise ValueError(f'{name}: SDF geometry {getattr(shape, "tag", None)!r} is not supported')
    origin.set('xyz', _join(xyz))
    origin.set('rpy', _join(rpy))


def _pose(element: ElementTree.Element) -> tuple[list[float], list[float]]:
    pose = element.find('pose')
    if pose is None or not (pose.text or '').strip():
        return [0.0] * 3, [0.0] * 3
    if pose.get('relative_to'):
        raise ValueError('SDF poses relative_to another frame are not supported')
    values = [float(value) for value in pose.text.split()]
    return values[:3], values[3:6]


def _join(values: list[float]) -> str:
    return ' '.join(repr(float(value)) for value in values)


def massless_collision_links(root: ElementTree.Element) -> list[str]:
    """Return leaf links without <inertial> that have a collision shape."""
    parent_links = {joint.find('parent').get('link') for joint in root.findall('joint')}
    return [
        link.get('name') for link in root.findall('link')
        if link.find('inertial') is None and link.find('collision') is not None
        and link.get('name') not in parent_links
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


def bundle_meshes(root: ElementTree.Element, mesh_dir: str) -> dict[str, str]:
    """
    Point every mesh of the URDF to a copy under mesh_dir; return {source: copy}.

    The caller copies the files. A copy keeps the path below the common directory of all
    meshes, so equal file names from different packages stay apart.
    """
    sources = mesh_paths(root)
    if not sources:
        return {}
    common = os.path.commonpath([os.path.dirname(source) for source in sources])
    copies = {
        source: os.path.join(mesh_dir, os.path.relpath(source, common)) for source in sources}
    for mesh in root.iter('mesh'):
        mesh.set('filename', copies[mesh.get('filename')])
    return copies


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


def named_states(srdf_xml: str) -> dict[str, dict[str, dict[str, float]]]:
    """Return the SRDF group states as {group: {state: {joint: position}}}."""
    states = {}
    for state in ElementTree.fromstring(srdf_xml).iter('group_state'):
        states.setdefault(state.get('group'), {})[state.get('name')] = {
            joint.get('name'): float(joint.get('value')) for joint in state.iter('joint')}
    return states


def mimic_positions(root: ElementTree.Element,
                    positions: dict[str, float]) -> dict[str, float]:
    """Return the positions of the mimic joints that follow the given joints."""
    result = {}
    for joint in root.findall('joint'):
        mimic = joint.find('mimic')
        if mimic is not None and mimic.get('joint') in positions:
            result[joint.get('name')] = (
                float(mimic.get('multiplier', '1')) * positions[mimic.get('joint')]
                + float(mimic.get('offset', '0')))
    return result


def source_digest(urdf_xml: str, files: Iterable[str]) -> str:
    """Hash the URDF text and the bytes of the files it references."""
    digest = hashlib.sha256(urdf_xml.encode())
    for path in sorted(files):
        digest.update(path.encode())
        with open(path, 'rb') as file:
            digest.update(file.read())
    return digest.hexdigest()[:16]
