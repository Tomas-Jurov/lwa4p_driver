"""Unit tests of the URDF preparation for the Isaac importer (no ROS needed)."""

from xml.etree import ElementTree

from manipulation_isaac_sim import isaac_urdf
import pytest

URDF = """<robot name="test">
  <link name="base"><visual><geometry>
    <mesh filename="package://some_description/meshes/base.stl"/>
  </geometry></visual></link>
  <link name="tip"><collision><geometry>
    <mesh filename="file:///opt/meshes/tip.stl"/>
  </geometry></collision></link>
  <joint name="slide" type="prismatic">
    <parent link="base"/><child link="tip"/>
    <limit lower="0" upper="1" effort="80" velocity="1"/>
  </joint>
  <link name="flange"/>
  <link name="frame"/>
  <joint name="flange_joint" type="fixed"><parent link="tip"/><child link="flange"/></joint>
  <joint name="frame_joint" type="fixed"><parent link="flange"/><child link="frame"/></joint>
  <ros2_control name="sim" type="system"/>
  <gazebo reference="tip"/>
</robot>"""

SRDF = """<robot name="test">
  <disable_collisions link1="b" link2="a" reason="Adjacent"/>
  <disable_collisions link1="a" link2="b" reason="Never"/>
  <disable_collisions link1="a" link2="c" reason="Never"/>
</robot>"""


def resolve(package: str) -> str:
    return f'/share/{package}'


def test_simulator_tags_are_removed():
    root = isaac_urdf.prepare_urdf(URDF, resolve)
    assert root.find('ros2_control') is None
    assert root.find('gazebo') is None


def test_mesh_uris_become_absolute_paths():
    root = isaac_urdf.prepare_urdf(URDF, resolve)
    assert isaac_urdf.mesh_paths(root) == [
        '/opt/meshes/tip.stl', '/share/some_description/meshes/base.stl']


def test_relative_mesh_path_is_rejected():
    with pytest.raises(ValueError):
        isaac_urdf.resolve_mesh_path('meshes/base.stl', resolve)


def test_meshes_are_bundled_below_their_common_directory():
    root = isaac_urdf.prepare_urdf(URDF, resolve)
    copies = isaac_urdf.bundle_meshes(root, '/bundle/meshes')
    assert copies == {
        '/opt/meshes/tip.stl': '/bundle/meshes/opt/meshes/tip.stl',
        '/share/some_description/meshes/base.stl':
            '/bundle/meshes/share/some_description/meshes/base.stl',
    }
    assert isaac_urdf.mesh_paths(root) == sorted(copies.values())


def test_bundle_without_meshes_is_empty():
    root = isaac_urdf.prepare_urdf('<robot name="empty"><link name="base"/></robot>', resolve)
    assert isaac_urdf.bundle_meshes(root, '/bundle/meshes') == {}


def test_effort_limits_of_actuated_joints_only():
    root = isaac_urdf.prepare_urdf(URDF, resolve)
    assert isaac_urdf.joint_effort_limits(root) == {'slide': 80.0}


def test_disabled_pairs_are_unique_and_sorted():
    assert isaac_urdf.disabled_collision_pairs(SRDF) == [('a', 'b'), ('a', 'c')]


def test_digest_changes_with_mesh_content(tmp_path):
    mesh = tmp_path / 'mesh.stl'
    mesh.write_bytes(b'one')
    first = isaac_urdf.source_digest(URDF, [str(mesh)])
    mesh.write_bytes(b'two')
    assert isaac_urdf.source_digest(URDF, [str(mesh)]) != first


def test_only_massless_connectors_get_a_mass():
    root = isaac_urdf.prepare_urdf(URDF, resolve)
    assert root.find("link[@name='flange']/inertial/mass").get('value') == '0.001'
    assert root.find("link[@name='frame']/inertial") is None
    assert root.find("link[@name='base']/inertial") is None


WORKSPACE = """<robot name="test">
  <link name="world"/>
  <link name="arm"/>
  <joint name="arm_joint" type="fixed"><parent link="world"/><child link="arm"/></joint>
  <link name="obstacle_table"><collision><geometry><box size="1 1 0.03"/></geometry></collision>
  </link>
  <joint name="obstacle_table_joint" type="fixed">
    <parent link="world"/><child link="obstacle_table"/>
  </joint>
  <link name="camera"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <joint name="camera_joint" type="fixed"><parent link="arm"/><child link="camera"/></joint>
  <link name="obstacle_on_arm"/>
  <joint name="obstacle_on_arm_joint" type="fixed">
    <parent link="arm"/><child link="obstacle_on_arm"/>
  </joint>
  <link name="finger"/>
  <joint name="gap" type="prismatic"><parent link="arm"/><child link="finger"/>
    <limit lower="0" upper="1" effort="80" velocity="1"/></joint>
  <link name="left"/>
  <joint name="left_joint" type="prismatic"><parent link="arm"/><child link="left"/>
    <mimic joint="gap" multiplier="-0.5"/>
    <limit lower="-1" upper="0" effort="80" velocity="1"/></joint>
</robot>"""

STATES = """<robot name="test">
  <group_state name="home" group="arm"><joint name="a" value="0"/><joint name="b" value="1.5"/>
  </group_state>
  <group_state name="open" group="gripper"><joint name="gap" value="0.085"/></group_state>
</robot>"""


def link_names(root) -> set[str]:
    return {link.get('name') for link in root.findall('link')}


def test_only_obstacles_fixed_to_the_world_are_removed():
    root = ElementTree.fromstring(WORKSPACE)
    assert isaac_urdf.remove_workspace_obstacles(root) == ['obstacle_table']
    assert 'obstacle_table' not in link_names(root)
    assert root.find("joint[@name='obstacle_table_joint']") is None
    assert {'world', 'arm', 'camera', 'obstacle_on_arm'} <= link_names(root)


def test_importer_input_has_no_obstacles():
    root = isaac_urdf.prepare_urdf(WORKSPACE, resolve)
    assert 'obstacle_table' not in link_names(root)


def test_massless_leaf_with_collision_gets_a_mass():
    root = isaac_urdf.prepare_urdf(WORKSPACE, resolve)
    assert root.find("link[@name='camera']/inertial/mass").get('value') == '0.001'
    assert root.find("link[@name='obstacle_on_arm']/inertial") is None


def test_named_states_per_group():
    assert isaac_urdf.named_states(STATES) == {
        'arm': {'home': {'a': 0.0, 'b': 1.5}},
        'gripper': {'open': {'gap': 0.085}},
    }


def test_mimic_joints_follow_their_leader():
    root = ElementTree.fromstring(WORKSPACE)
    assert isaac_urdf.mimic_positions(root, {'gap': 0.08}) == {'left_joint': -0.04}
    assert isaac_urdf.mimic_positions(root, {'other': 1.0}) == {}


SDF_WORLD = """<sdf version="1.9"><world name="w">
  <model name="ground"><static>true</static><pose>0 0 -0.06 0 0 0</pose><link name="l">
    <collision name="c"><geometry><plane><normal>0 0 1</normal><size>20 20</size></plane>
    </geometry></collision></link></model>
  <model name="bin"><static>true</static><pose>0.2 -0.3 0 0 0 0.5</pose><link name="l">
    <collision name="wall"><pose>0.1 0 0.05 0 0 0</pose>
      <geometry><box><size>0.01 0.2 0.1</size></box></geometry></collision></link></model>
  <model name="cube"><pose>0.4 0 0.1 0 0 0</pose><link name="l">
    <collision name="c"><geometry><box><size>0.03 0.03 0.03</size></box></geometry>
    </collision></link></model>
</world></sdf>"""

SRDF_EMPTY = '<robot name="test"/>'


def test_static_sdf_models_become_obstacles():
    root = ElementTree.fromstring(WORKSPACE)
    assert isaac_urdf.add_sdf_obstacles(root, SDF_WORLD) == ['obstacle_ground', 'obstacle_bin']
    assert 'obstacle_cube' not in link_names(root)
    joint = root.find("joint[@name='obstacle_bin_joint']")
    assert joint.find('parent').get('link') == 'world'
    assert joint.find('origin').get('rpy') == '0.0 0.0 0.5'
    wall = root.find("link[@name='obstacle_bin']/collision")
    assert wall.find('origin').get('xyz') == '0.1 0.0 0.05'
    floor = root.find("link[@name='obstacle_ground']/collision")
    assert floor.find('geometry/box').get('size') == '20 20 0.01'
    assert floor.find('origin').get('xyz') == '0.0 0.0 -0.005'


def test_mounted_links_stop_at_moving_joints_and_obstacles():
    root = ElementTree.fromstring(WORKSPACE)
    assert isaac_urdf.mounted_links(root) == ['arm', 'camera', 'world']


def test_obstacles_and_mount_do_not_collide_with_each_other():
    root = ElementTree.fromstring(WORKSPACE)
    isaac_urdf.add_sdf_obstacles(root, SDF_WORLD)
    pairs = set(isaac_urdf.disabled_collision_pairs(
        isaac_urdf.disable_obstacle_collisions(SRDF_EMPTY, root)))
    assert ('obstacle_bin', 'obstacle_ground') in pairs
    assert ('arm', 'obstacle_ground') in pairs
    assert ('obstacle_bin', 'obstacle_table') in pairs
    assert not any('finger' in pair or 'left' in pair for pair in pairs)
    assert ('arm', 'camera') not in pairs
