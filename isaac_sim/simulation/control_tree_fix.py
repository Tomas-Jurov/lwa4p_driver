"""
Workaround for a bug in isaacsim.ros2.control 0.1.6 (Isaac Sim 6.2).

Before the controller manager starts, the extension exports the USD robot to URDF and
prunes it to the tree of the controlled joints (urdf_synth._prune_to_control_tree).
It takes as roots the parent links of controlled joints that are not children of
controlled joints. For the LWA 4P these are arm_base_link and gripper_body_link, which
hangs below the arm on fixed joints. The breadth-first search starts with both marked
as visited, so it never keeps gripper_body_joint, and the URDF ends up with two roots:
"Failed to parse URDF file".

The replacement walks up from every controlled joint to the real root of its tree
and keeps everything below that root.
"""

from collections import deque
from xml.etree import ElementTree

TESTED_VERSION = '0.1.6'


def install(extension_version: str) -> None:
    """Replace the pruning function of isaacsim.ros2.control with the fixed one."""
    from isaacsim.ros2.control import urdf_synth

    if extension_version != TESTED_VERSION:
        print(f'[isaac_sim] warning: control_tree_fix was written for isaacsim.ros2.control '
              f'{TESTED_VERSION}, found {extension_version}; check whether it is still needed',
              flush=True)
    urdf_synth._prune_to_control_tree = prune_to_control_tree


def prune_to_control_tree(root: ElementTree.Element, control_joint_names: list[str]) -> None:
    """Keep only the tree, or trees, that contain the controlled joints."""
    joints = [joint for joint in root.findall('joint')
              if joint.find('parent') is not None and joint.find('child') is not None]
    parent_of = {joint.find('child').get('link'): joint.find('parent').get('link')
                 for joint in joints}
    joints_from = {}
    for joint in joints:
        joints_from.setdefault(joint.find('parent').get('link'), []).append(joint)

    control_parents = [joint.find('parent').get('link') for joint in joints
                       if joint.get('name') in control_joint_names]
    if not control_parents:
        return
    roots = {_tree_root(link, parent_of) for link in control_parents}

    kept_links = set(roots)
    kept_joints = set()
    queue = deque(roots)
    while queue:
        link = queue.popleft()
        for joint in joints_from.get(link, []):
            child = joint.find('child').get('link')
            kept_joints.add(joint.get('name'))
            if child not in kept_links:
                kept_links.add(child)
                queue.append(child)

    for joint in root.findall('joint'):
        if joint.get('name') not in kept_joints:
            root.remove(joint)
    for link in root.findall('link'):
        if link.get('name') not in kept_links:
            root.remove(link)


def _tree_root(link: str, parent_of: dict[str, str]) -> str:
    seen = {link}
    while link in parent_of and parent_of[link] not in seen:
        link = parent_of[link]
        seen.add(link)
    return link
