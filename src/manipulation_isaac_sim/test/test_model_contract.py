"""
The names our code relies on still exist in Tomas's model and configuration.

If one of these tests fails after a `main -> devel` sync, Tomas renamed something:
update our code and configuration, never his files.
"""

import copy
import os
from xml.etree import ElementTree

from ament_index_python.packages import get_package_share_directory
from manipulation_isaac_sim import isaac_urdf
import pytest
import xacro
import yaml

ARM_JOINTS = [f'arm_{index}_joint' for index in range(1, 7)]
GRIPPER_JOINTS = ['gripper_joint', 'gripper_finger_left_joint', 'gripper_finger_right_joint']
FRAMES = ['world', 'arm_base_link', 'arm_tool0', 'ftm_tool0', 'gripper_tcp', 'camera_link',
          'camera_color_optical_frame', 'camera_depth_optical_frame']
CONTROLLERS = {'arm_controller': ARM_JOINTS, 'gripper_controller': ['gripper_joint']}
ARM_STATES = {'home', 'work1', 'work2'}
GRIPPER_STATES = {'open', 'tube_15mm', 'closed'}


def share(package: str, *path: str) -> str:
    return os.path.join(get_package_share_directory(package), *path)


def load_yaml(path: str) -> dict:
    with open(path, encoding='utf-8') as file:
        return yaml.safe_load(file)


@pytest.fixture(scope='module')
def urdf() -> ElementTree.Element:
    xml = xacro.process_file(
        share('manipulation_isaac_sim', 'urdf', 'lwa4p_isaac.urdf.xacro')).toxml()
    return ElementTree.fromstring(xml)


@pytest.fixture(scope='module')
def srdf() -> ElementTree.Element:
    return ElementTree.parse(share('lwa4p_moveit_config', 'config', 'lwa4p.srdf')).getroot()


def test_joints_exist(urdf):
    joint_names = {joint.get('name') for joint in urdf.findall('joint')}
    missing = set(ARM_JOINTS + GRIPPER_JOINTS) - joint_names
    assert not missing, f'joints missing in the model: {sorted(missing)}'


def test_fingers_mimic_the_gap_joint(urdf):
    for name in GRIPPER_JOINTS[1:]:
        mimic = urdf.find(f"joint[@name='{name}']/mimic")
        assert mimic is not None and mimic.get('joint') == 'gripper_joint', name


def test_frames_exist(urdf):
    link_names = {link.get('name') for link in urdf.findall('link')}
    missing = set(FRAMES) - link_names
    assert not missing, f'frames missing in the model: {sorted(missing)}'


def test_srdf_groups_and_states(srdf):
    assert srdf.find("group[@name='arm']/chain").get('tip_link') == 'gripper_tcp'
    assert srdf.find("group[@name='gripper']/joint[@name='gripper_joint']") is not None
    states = {(state.get('group'), state.get('name')) for state in srdf.findall('group_state')}
    assert {('arm', name) for name in ARM_STATES} <= states
    assert {('gripper', name) for name in GRIPPER_STATES} <= states


def test_moveit_arm_execution_matches_the_real_robot():
    """
    The arm and the execution settings are the real robot's.

    The gripper differs until the simulation gripper adapter is decided (D09): the real
    gripper_node serves GripperCommand, Isaac Sim a trajectory controller on gripper_joint.
    """
    ours = load_yaml(share('manipulation_isaac_sim', 'config', 'moveit_controllers.yaml'))
    real = load_yaml(share('lwa4p_moveit_config', 'config', 'moveit_controllers.yaml'))
    section = 'moveit_simple_controller_manager'
    assert ours[section]['arm_controller'] == real[section]['arm_controller'], \
        'Tomas changed the real arm controller for MoveIt: update ours to match'
    assert ours['trajectory_execution'] == real['trajectory_execution'], \
        'Tomas changed the real execution settings: update ours to match'


def test_workspace_obstacles_are_found(urdf):
    """The lab obstacles are in the description, and removing them keeps the robot."""
    removed = isaac_urdf.remove_workspace_obstacles(copy.deepcopy(urdf))
    assert removed, (
        f'no links named {isaac_urdf.OBSTACLE_PREFIX}* fixed to {isaac_urdf.WORLD_LINK}: '
        'Tomas renamed the workspace obstacles; update isaac_urdf')
    assert not set(removed) & set(FRAMES)


def test_worlds_name_their_moveit_obstacles(urdf):
    """Every world gives MoveIt obstacles: Tomas's measured ones or its SDF's static models."""
    worlds = load_yaml(share('manipulation_isaac_sim', 'config', 'worlds.yaml'))
    assert worlds
    for name, world in worlds.items():
        assert world['workspace_obstacles'] or world.get('sdf_obstacles'), name
        if world.get('sdf_obstacles'):
            with open(share('lwa4p_gazebo', world['sdf_obstacles']), encoding='utf-8') as file:
                added = isaac_urdf.add_sdf_obstacles(copy.deepcopy(urdf), file.read())
            assert added, f'{name}: {world["sdf_obstacles"]} has no static models'


def test_isaac_controllers_serve_moveit():
    controllers = load_yaml(share('manipulation_isaac_sim', 'config', 'isaac_controllers.yaml'))
    for name, joints in CONTROLLERS.items():
        assert controllers[name]['ros__parameters']['joints'] == joints, name
