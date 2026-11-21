"""Unit tests of the mesh mass properties against closed-form results (no ROS needed)."""

import struct

from manipulation_isaac_sim import mesh_inertia
import pytest

# The 12 outward-facing triangles of the unit cube [0, 1]^3, as corner indices (bits x, y, z).
CUBE_FACES = [
    (0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6),  # z = 0, z = 1
    (0, 1, 5), (0, 5, 4), (2, 6, 7), (2, 7, 3),  # y = 0, y = 1
    (0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5),  # x = 0, x = 1
]


def box(size, offset=(0.0, 0.0, 0.0)):
    """Return the triangles of a box with its minimum corner at offset."""
    corners = [tuple(offset[axis] + size[axis] * ((index >> axis) & 1) for axis in range(3))
               for index in range(8)]
    return [tuple(corners[index] for index in face) for face in CUBE_FACES]


def write_binary_stl(path, triangles):
    with open(path, 'wb') as file:
        file.write(b'\0' * 80 + struct.pack('<I', len(triangles)))
        for triangle in triangles:
            file.write(struct.pack('<12fH', 0.0, 0.0, 0.0,
                                   *[value for vertex in triangle for value in vertex], 0))


def test_solid_box_matches_the_closed_form():
    size, offset, mass = (0.1, 0.2, 0.3), (0.5, -0.1, 0.2), 2.0
    properties = mesh_inertia.mass_properties(box(size, offset), mass, mesh_inertia.SOLID)
    assert properties.center_of_mass == pytest.approx(
        [offset[axis] + size[axis] / 2 for axis in range(3)])
    a, b, c = size
    expected = [mass / 12 * (b * b + c * c), mass / 12 * (a * a + c * c),
                mass / 12 * (a * a + b * b)]
    assert [properties.inertia[axis][axis] for axis in range(3)] == pytest.approx(expected)
    assert properties.inertia[0][1] == pytest.approx(0.0, abs=1e-12)


def test_shell_cube_matches_the_closed_form():
    side, mass = 0.2, 1.5
    properties = mesh_inertia.mass_properties(box((side,) * 3), mass, mesh_inertia.SHELL)
    # Six thin square plates: I = 5/18 m a^2 about every axis through the centre.
    assert [properties.inertia[axis][axis] for axis in range(3)] == pytest.approx(
        [5 / 18 * mass * side * side] * 3)


def test_scale_applies_before_the_integration():
    properties = mesh_inertia.mass_properties(
        box((100.0, 100.0, 100.0)), 1.0, mesh_inertia.SOLID, scale=(0.001, 0.001, 0.001))
    assert properties.center_of_mass == pytest.approx([0.05] * 3)
    assert properties.inertia[2][2] == pytest.approx(1.0 / 6 * 0.1 * 0.1)


def test_products_of_inertia_have_the_tensor_sign():
    # Two point-like boxes at (+1, +1) and (-1, -1): positive x*y, so I_xy is negative.
    triangles = (box((0.01,) * 3, (0.995, 0.995, -0.005))
                 + box((0.01,) * 3, (-1.005, -1.005, -0.005)))
    properties = mesh_inertia.mass_properties(triangles, 2.0, mesh_inertia.SOLID)
    assert properties.inertia[0][1] == pytest.approx(-2.0, rel=1e-3)


def test_binary_and_ascii_stl_read_alike(tmp_path):
    triangles = box((1.0, 2.0, 3.0))
    binary = tmp_path / 'binary.stl'
    write_binary_stl(binary, triangles)
    ascii_stl = tmp_path / 'ascii.stl'
    ascii_stl.write_text('solid box\n' + ''.join(
        'facet normal 0 0 0\nouter loop\n'
        + ''.join(f'vertex {x} {y} {z}\n' for x, y, z in triangle)
        + 'endloop\nendfacet\n' for triangle in triangles) + 'endsolid box\n')
    assert mesh_inertia.read_stl(str(binary)) == mesh_inertia.read_stl(str(ascii_stl))


def test_open_mesh_is_no_solid_but_a_shell():
    triangles = box((1.0, 1.0, 1.0))[:-1]
    with pytest.raises(ValueError, match='open edges'):
        mesh_inertia.mass_properties(triangles, 1.0, mesh_inertia.SOLID)
    assert mesh_inertia.mass_properties(triangles, 1.0, mesh_inertia.SHELL).mass == 1.0


def test_inward_normals_are_rejected():
    triangles = [tuple(reversed(triangle)) for triangle in box((1.0, 1.0, 1.0))]
    with pytest.raises(ValueError, match='inward'):
        mesh_inertia.mass_properties(triangles, 1.0, mesh_inertia.SOLID)


def test_unknown_model_and_bad_mass_are_rejected():
    with pytest.raises(ValueError):
        mesh_inertia.mass_properties(box((1.0, 1.0, 1.0)), 1.0, 'hollow')
    with pytest.raises(ValueError):
        mesh_inertia.mass_properties(box((1.0, 1.0, 1.0)), 0.0, mesh_inertia.SOLID)
