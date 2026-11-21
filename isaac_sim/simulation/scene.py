"""
The world around the robot: a static environment USD from isaac_sim/scenes/.

The file is added as a sublayer of the live stage, so its prims keep their paths and every
edit made at run time goes to the in-memory root layer, never into the file. The world's
origin is the robot's world frame, as in Tomas's Gazebo worlds.

A world holds geometry, visual materials, rigid bodies with their masses, collision groups
and its light, as tools/clean_world.py leaves it. The physics scene and the contact
materials come from config/sim.yaml.
"""

from pathlib import Path

from pxr import Sdf
from pxr import Usd
from pxr import UsdGeom
from pxr import UsdPhysics
from simulation import contact

CLEAN_HINT = 'clean it with isaac_sim/tools/clean_world.sh (see isaac_sim/README.md)'


def load_world(stage: Usd.Stage, world_path: Path, contact_config: dict) -> Usd.Prim:
    """Compose the environment into the stage, bind its contact materials, return its root."""
    if not world_path.is_file():
        raise FileNotFoundError(f'no world USD at {world_path}')
    layer = Sdf.Layer.FindOrOpen(str(world_path))
    _check_units(stage, layer, world_path)
    stage.GetRootLayer().subLayerPaths.append(layer.identifier)

    root = stage.GetPrimAtPath(f'/{layer.defaultPrim}') if layer.defaultPrim else None
    if not root:
        raise RuntimeError(f'{world_path.name} has no default prim; {CLEAN_HINT}')
    _check_clean(root, world_path)

    models = {prim.GetName(): prim for prim in root.GetChildren() if _has_collider(prim)}
    bound = contact.bind_by_name(stage, models, contact_config['world'],
                                 contact_config['materials'], f'model in {world_path.name}')
    groups = sum(prim.IsA(UsdPhysics.CollisionGroup) for prim in Usd.PrimRange(root))
    print(f'[isaac_sim] world: {world_path.name}, {len(bound)} models with contact materials, '
          f'{groups} collision groups', flush=True)
    return root


def filter_robot_mount(world_root: Usd.Prim, mount_bodies: list[Usd.Prim]) -> int:
    """
    Keep the robot's mounted base out of contact with the world; return the world colliders.

    The base is bolted to its support, so touching it is expected: the real column reaches
    down to the mounting surface (z -0.110), 4 mm into the lab table, and 5 cm below the
    floor of Tomas's table world, which he built around a shorter Gazebo collision mesh.
    PhysX would push the fixed base out and throw the arm. The arm links still collide with
    the world.
    """
    colliders = [prim.GetPath() for prim in Usd.PrimRange(world_root)
                 if prim.HasAPI(UsdPhysics.CollisionAPI)]
    for body in mount_bodies:
        pairs = UsdPhysics.FilteredPairsAPI.Apply(body).CreateFilteredPairsRel()
        for collider in colliders:
            pairs.AddTarget(collider)
    return len(colliders)


def _check_units(stage: Usd.Stage, layer: Sdf.Layer, world_path: Path) -> None:
    # A sublayer's stage metadata is ignored: the root layer's units apply to its values.
    up_axis = layer.pseudoRoot.GetInfo('upAxis') if layer.pseudoRoot.HasInfo('upAxis') else 'Y'
    meters_per_unit = (layer.pseudoRoot.GetInfo('metersPerUnit')
                       if layer.pseudoRoot.HasInfo('metersPerUnit') else 0.01)
    if up_axis != UsdGeom.GetStageUpAxis(stage) or (
            abs(meters_per_unit - UsdGeom.GetStageMetersPerUnit(stage)) > 1e-9):
        raise RuntimeError(f'{world_path.name} uses upAxis {up_axis}, metersPerUnit '
                           f'{meters_per_unit}; the stage needs Z and 1.0')


def _check_clean(root: Usd.Prim, world_path: Path) -> None:
    """
    Refuse what clean_world.py removes, instead of simulating it wrongly.

    A PhysicsScene in a sublayer is not seen by SimulationManager, which adds its own, and
    PhysX steps the two at different rates. A physics material below a model would override
    the one from sim.yaml. A rigid body inside another makes PhysX add a massless body.
    """
    problems = []
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdPhysics.Scene):
            problems.append(f'physics scene {prim.GetPath()}')
        elif prim.HasAPI(UsdPhysics.MaterialAPI):
            problems.append(f'physics material {prim.GetPath()}')
        elif prim.HasAPI(UsdPhysics.RigidBodyAPI) and any(
                ancestor.HasAPI(UsdPhysics.RigidBodyAPI)
                for ancestor in _ancestors(prim.GetParent(), root)):
            problems.append(f'nested rigid body {prim.GetPath()}')
    if problems:
        shown = '; '.join(problems[:5]) + (f'; and {len(problems) - 5} more'
                                           if len(problems) > 5 else '')
        raise RuntimeError(f'{world_path.name} is not clean ({shown}); {CLEAN_HINT}')


def _ancestors(prim: Usd.Prim, stop: Usd.Prim):
    while prim and prim != stop.GetParent():
        yield prim
        prim = prim.GetParent()


def _has_collider(prim: Usd.Prim) -> bool:
    return any(descendant.HasAPI(UsdPhysics.CollisionAPI) for descendant in Usd.PrimRange(prim))
