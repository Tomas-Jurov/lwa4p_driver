"""
Mass properties of triangle meshes (STL), for a given total mass.

No ROS imports and no third-party packages. Two mass models:
- solid: the mass fills the closed mesh uniformly (a module full of motors and gears);
- shell: the mass lies uniformly on the surface (a hollow tube with thin walls).
"""

from dataclasses import dataclass
import struct

SOLID = 'solid'
SHELL = 'shell'
MASS_MODELS = (SOLID, SHELL)
STL_HEADER_BYTES = 80
STL_TRIANGLE_BYTES = 50
# Vertices closer than this are one vertex when the mesh is checked for holes [mesh units].
VERTEX_TOLERANCE = 1e-9

Vector = tuple[float, float, float]
Triangle = tuple[Vector, Vector, Vector]
Matrix = tuple[Vector, Vector, Vector]


@dataclass(frozen=True)
class MassProperties:
    """Mass, centre of mass and inertia tensor about the centre of mass, in the mesh frame."""

    mass: float
    center_of_mass: Vector
    inertia: Matrix


def read_stl(path: str) -> list[Triangle]:
    """Return the triangles of a binary or ASCII STL file."""
    with open(path, 'rb') as file:
        data = file.read()
    if len(data) >= STL_HEADER_BYTES + 4:
        count = struct.unpack_from('<I', data, STL_HEADER_BYTES)[0]
        if len(data) == STL_HEADER_BYTES + 4 + count * STL_TRIANGLE_BYTES:
            return _binary_triangles(data, count)
    return _ascii_triangles(data.decode('ascii', errors='replace'), path)


def mass_properties(triangles: list[Triangle], mass: float, model: str,
                    scale: Vector = (1.0, 1.0, 1.0)) -> MassProperties:
    """Return the mass properties of the scaled mesh carrying the given mass."""
    if model not in MASS_MODELS:
        raise ValueError(f'unknown mass model {model!r}; use one of {MASS_MODELS}')
    if mass <= 0.0:
        raise ValueError(f'the mass must be positive, not {mass}')
    scaled = [tuple(_scaled(vertex, scale) for vertex in triangle) for triangle in triangles]
    if model == SOLID:
        open_edges = _open_edge_count(scaled)
        if open_edges:
            raise ValueError(f'the mesh has {open_edges} open edges: no volume for a solid')
        amount, first_moment, second_moment = _solid_integrals(scaled)
        if amount <= 0.0:
            raise ValueError('the mesh encloses no positive volume (inward normals?)')
    else:
        amount, first_moment, second_moment = _shell_integrals(scaled)
        if amount <= 0.0:
            raise ValueError('the mesh has no surface area')
    density = mass / amount
    center = tuple(value / amount for value in first_moment)
    # Second moment about the centre of mass, then inertia = trace(C) * identity - C.
    central = [[density * second_moment[row][col] - mass * center[row] * center[col]
                for col in range(3)] for row in range(3)]
    trace = central[0][0] + central[1][1] + central[2][2]
    inertia = tuple(tuple((trace if row == col else 0.0) - central[row][col]
                          for col in range(3)) for row in range(3))
    return MassProperties(mass, center, inertia)


def _binary_triangles(data: bytes, count: int) -> list[Triangle]:
    triangles = []
    for index in range(count):
        values = struct.unpack_from(
            '<12f', data, STL_HEADER_BYTES + 4 + index * STL_TRIANGLE_BYTES)
        triangles.append((values[3:6], values[6:9], values[9:12]))
    return triangles


def _ascii_triangles(text: str, path: str) -> list[Triangle]:
    vertices = [tuple(float(value) for value in line.split()[1:4])
                for line in text.splitlines() if line.strip().startswith('vertex')]
    if not vertices or len(vertices) % 3:
        raise ValueError(f'{path} is neither a binary nor an ASCII STL file')
    return [tuple(vertices[index:index + 3]) for index in range(0, len(vertices), 3)]


def _scaled(vertex: Vector, scale: Vector) -> Vector:
    return tuple(float(value) * factor for value, factor in zip(vertex, scale))


def _solid_integrals(triangles: list[Triangle]) -> tuple[float, Vector, list[list[float]]]:
    """Volume, first and second moments: signed tetrahedra from the origin to each face."""
    volume = 0.0
    first = [0.0] * 3
    second = [[0.0] * 3 for _ in range(3)]
    for a, b, c in triangles:
        signed = _dot(a, _cross(b, c)) / 6.0
        total = [a[i] + b[i] + c[i] for i in range(3)]
        volume += signed
        for row in range(3):
            first[row] += signed * total[row] / 4.0
            for col in range(3):
                second[row][col] += signed / 20.0 * (
                    a[row] * a[col] + b[row] * b[col] + c[row] * c[col]
                    + total[row] * total[col])
    return volume, tuple(first), second


def _shell_integrals(triangles: list[Triangle]) -> tuple[float, Vector, list[list[float]]]:
    """Area, first and second moments of the surface."""
    area = 0.0
    first = [0.0] * 3
    second = [[0.0] * 3 for _ in range(3)]
    for a, b, c in triangles:
        edge_1 = [b[i] - a[i] for i in range(3)]
        edge_2 = [c[i] - a[i] for i in range(3)]
        normal = _cross(edge_1, edge_2)
        triangle_area = _dot(normal, normal) ** 0.5 / 2.0
        total = [a[i] + b[i] + c[i] for i in range(3)]
        area += triangle_area
        for row in range(3):
            first[row] += triangle_area * total[row] / 3.0
            for col in range(3):
                second[row][col] += triangle_area / 12.0 * (
                    a[row] * a[col] + b[row] * b[col] + c[row] * c[col]
                    + total[row] * total[col])
    return area, tuple(first), second


def _open_edge_count(triangles: list[Triangle]) -> int:
    """Count edges not shared by exactly two faces."""
    def key(vertex: Vector) -> tuple[int, int, int]:
        return tuple(round(value / VERTEX_TOLERANCE) for value in vertex)

    edges = {}
    for triangle in triangles:
        keys = [key(vertex) for vertex in triangle]
        for index in range(3):
            edge = tuple(sorted((keys[index], keys[(index + 1) % 3])))
            edges[edge] = edges.get(edge, 0) + 1
    return sum(1 for count in edges.values() if count != 2)


def _dot(a, b) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b) -> Vector:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
