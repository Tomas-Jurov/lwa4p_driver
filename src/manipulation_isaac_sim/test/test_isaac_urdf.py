"""Unit tests of the URDF preparation for the Isaac importer (no ROS needed)."""

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
