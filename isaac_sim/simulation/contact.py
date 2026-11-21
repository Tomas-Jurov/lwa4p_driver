"""
Contact materials from config/sim.yaml: the only physics materials on the stage.

The worlds carry none (tools/clean_world.py removes the converted ones) and the URDF importer
writes none, so every friction and restitution value in the simulation is in sim.yaml.
A material is bound with the physics purpose to a model or a link, and applies to every
collider below it.
"""

import re

from pxr import Usd
from pxr import UsdPhysics
from pxr import UsdShade

MATERIALS_PATH = '/World/PhysicsMaterials'
# The material binding purpose PhysX reads; USD defines no token for it.
PHYSICS_PURPOSE = 'physics'


def bind_by_name(stage: Usd.Stage, prims: dict[str, Usd.Prim], assignments: dict,
                 materials: dict, what: str) -> dict[str, str]:
    """
    Bind to each prim the material of the first name pattern that matches it.

    Raises when a pattern matches nothing, so a renamed link or model is not missed silently.
    Returns prim name -> material name.
    """
    bound = {}
    for pattern, material_name in assignments.items():
        names = [name for name in prims if name not in bound and re.fullmatch(pattern, name)]
        if not names:
            raise RuntimeError(f'contact material pattern {pattern!r} matches no {what}')
        material = _material(stage, material_name, materials)
        for name in names:
            UsdShade.MaterialBindingAPI.Apply(prims[name]).Bind(
                material, materialPurpose=PHYSICS_PURPOSE)
            bound[name] = material_name
    return bound


def _material(stage: Usd.Stage, name: str, materials: dict) -> UsdShade.Material:
    path = f'{MATERIALS_PATH}/{name}'
    if stage.GetPrimAtPath(path):
        return UsdShade.Material.Get(stage, path)
    if name not in materials:
        raise ValueError(f'contact material {name!r} is not in contact.materials')
    values = materials[name]
    material = UsdShade.Material.Define(stage, path)
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(values['static_friction'])
    physics.CreateDynamicFrictionAttr(values['dynamic_friction'])
    physics.CreateRestitutionAttr(values['restitution'])
    return material
