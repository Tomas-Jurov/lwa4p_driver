"""
Strip a Gazebo world converted to USD of conversion and Gazebo artifacts.

Usage: ISAACSIM_PATH=<Isaac Sim directory> isaac_sim/tools/clean_world.sh \
           <converted.usd> <Tomas's world.sdf> [<output.usd>]

The input is the output of an SDF -> USD converter run over one of Tomas's
src/lwa4p_gazebo/worlds/*_gz.sdf files; the SDF is read again for what the conversion lost.
The output holds geometry, visual materials, rigid bodies with their masses, collision groups
and the light, under one default prim. The physics scene and the contact materials come from
config/sim.yaml at run time (simulation/scene.py refuses a world that still has its own).
Running the tool on its own output changes nothing.
"""

import argparse
from pathlib import Path
import sys
from xml.etree import ElementTree

from pxr import Gf
from pxr import Sdf
from pxr import Usd
from pxr import UsdGeom
from pxr import UsdLux
from pxr import UsdPhysics
from pxr import UsdShade

LOOKS = 'Looks'
COLLISION_GROUPS = 'CollisionGroups'
# SDF's default collide_bitmask: such a collision collides with everything.
DEFAULT_BITMASK = 0xFFFF
TIME_METADATA = ('startTimeCode', 'endTimeCode', 'timeCodesPerSecond', 'framesPerSecond')
# OmniPBR inputs the converter authors although SDF materials have no <emissive>.
EMISSION_INPUTS = ('inputs:enable_emission', 'inputs:emissive_color', 'inputs:emissive_intensity')
# The converter writes the light's intensity twice; the old unprefixed name is deprecated.
LEGACY_LIGHT_ATTRIBUTES = ('intensity', 'color', 'angle')


def parse_arguments() -> argparse.Namespace:
    """Read the command line."""
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('usd', type=Path, help='the converted world')
    parser.add_argument('sdf', type=Path, help="Tomas's SDF the USD was converted from")
    parser.add_argument('output', type=Path, nargs='?', help='default: overwrite the input')
    parser.add_argument('--light-intensity', type=float, default=3000.0,
                        help='SDF lights have no physical intensity (default: %(default)s)')
    return parser.parse_args()


def main() -> int:
    """Clean one world and print what changed."""
    arguments = parse_arguments()
    world = ElementTree.parse(arguments.sdf).getroot().find('world')
    stage = Usd.Stage.Open(str(arguments.usd))
    root = _root_prim(stage)

    _check_models(root, world)
    report = {
        'physics scenes removed': _remove_physics_scenes(stage),
        'nested rigid bodies fixed': _unnest_rigid_bodies(stage),
        'physics materials removed': _remove_physics_materials(stage),
        'visual materials moved under the root': _move_looks_under_root(stage, root),
        'material binding APIs applied': _apply_binding_api(stage),
        'emission inputs removed': _remove_emission(stage, world),
        'lights set from the SDF': _set_lights(stage, root, world, arguments.light_intensity),
        'collision groups': _author_collision_groups(stage, root, world),
    }
    _set_stage_metadata(stage, root)

    output = arguments.output or arguments.usd
    stage.GetRootLayer().Export(str(output))
    for item, count in report.items():
        print(f'{item}: {count}')
    print(f'wrote {output}')
    return 0


def _root_prim(stage: Usd.Stage) -> Usd.Prim:
    roots = [prim for prim in stage.GetPseudoRoot().GetChildren() if prim.GetName() != LOOKS]
    if len(roots) != 1:
        raise SystemExit(f'expected one root prim besides /{LOOKS}, found '
                         f'{[str(prim.GetPath()) for prim in roots]}')
    return roots[0]


def _check_models(root: Usd.Prim, world: ElementTree.Element) -> None:
    """Refuse a USD whose models differ from the SDF's: it was converted from another version."""
    sdf_models = {model.get('name') for model in world.findall('model')}
    usd_models = {prim.GetName() for prim in root.GetChildren()
                  if not prim.IsA(UsdPhysics.Scene) and not prim.IsA(UsdLux.BoundableLightBase)
                  and not prim.IsA(UsdLux.NonboundableLightBase)
                  and prim.GetName() not in (LOOKS, COLLISION_GROUPS)}
    if sdf_models != usd_models:
        raise SystemExit(f'the USD does not match the SDF; convert it again. '
                         f'Only in the SDF: {sorted(sdf_models - usd_models)}; '
                         f'only in the USD: {sorted(usd_models - sdf_models)}')


def _remove_physics_scenes(stage: Usd.Stage) -> int:
    paths = [prim.GetPath() for prim in stage.Traverse() if prim.IsA(UsdPhysics.Scene)]
    for path in paths:
        stage.RemovePrim(path)
    return len(paths)


def _unnest_rigid_bodies(stage: Usd.Stage) -> int:
    """
    Drop RigidBodyAPI from bodies that contain another rigid body.

    The converter puts it on both a model and its link; PhysX then adds a massless body per
    model. The link keeps it: it carries the mass and is the closest body above the colliders.
    """
    outer = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        descendants = iter(Usd.PrimRange(prim))
        next(descendants)
        if any(child.HasAPI(UsdPhysics.RigidBodyAPI) for child in descendants):
            outer.append(prim)
    for prim in outer:
        _remove_api_schema(stage.GetRootLayer().GetPrimAtPath(prim.GetPath()),
                           'PhysicsRigidBodyAPI')
    return len(outer)


def _remove_physics_materials(stage: Usd.Stage) -> int:
    """
    Remove the converted physics materials and their bindings.

    They hold restitution 1 (SDF worlds without <bounce> mean 0), density 1 kg/m3, and
    ODE's two friction directions mu and mu2 written as dynamic and static friction.
    """
    materials = {prim.GetPath() for prim in stage.Traverse()
                 if prim.HasAPI(UsdPhysics.MaterialAPI)}
    layer = stage.GetRootLayer()
    for prim in stage.Traverse():
        spec = layer.GetPrimAtPath(prim.GetPath())
        for relationship in list(spec.relationships if spec else []):
            if not relationship.name.startswith('material:binding'):
                continue
            targets = set(relationship.targetPathList.GetAddedOrExplicitItems())
            if targets and targets <= materials:
                spec.RemoveProperty(relationship)
    for path in materials:
        stage.RemovePrim(path)
    return len(materials)


def _move_looks_under_root(stage: Usd.Stage, root: Usd.Prim) -> int:
    """Move /Looks under the root prim, so the file is one asset with a default prim."""
    old = Sdf.Path(f'/{LOOKS}')
    if not stage.GetPrimAtPath(old):
        return 0
    new = root.GetPath().AppendChild(LOOKS)
    layer = stage.GetRootLayer()
    if not Sdf.CopySpec(layer, old, layer, new):
        raise RuntimeError(f'could not copy {old} to {new}')
    stage.RemovePrim(old)
    moved = len(stage.GetPrimAtPath(new).GetChildren())

    def retarget(path: Sdf.Path) -> None:
        if not path.IsPropertyPath():
            return
        spec = layer.GetPropertyAtPath(path)
        if isinstance(spec, Sdf.RelationshipSpec):
            paths = spec.targetPathList
        elif isinstance(spec, Sdf.AttributeSpec):
            paths = spec.connectionPathList
        else:
            return
        items = (list(paths.explicitItems) + list(paths.prependedItems)
                 + list(paths.appendedItems) + list(paths.deletedItems))
        for item in items:
            if item.HasPrefix(old):
                paths.ReplaceItemEdits(item, item.ReplacePrefix(old, new))

    layer.Traverse(Sdf.Path.absoluteRootPath, retarget)
    return moved


def _apply_binding_api(stage: Usd.Stage) -> int:
    """Apply MaterialBindingAPI where the converter bound a material without it."""
    count = 0
    for prim in stage.Traverse():
        has_binding = any(relationship.GetName().startswith('material:binding')
                          for relationship in prim.GetRelationships())
        if has_binding and not prim.HasAPI(UsdShade.MaterialBindingAPI):
            UsdShade.MaterialBindingAPI.Apply(prim)
            count += 1
    return count


def _remove_emission(stage: Usd.Stage, world: ElementTree.Element) -> int:
    if world.find('.//emissive') is not None:
        return 0  # the SDF asks for emission: keep what the converter made of it
    count = 0
    for prim in stage.Traverse():
        if not prim.IsA(UsdShade.Shader):
            continue
        for name in EMISSION_INPUTS:
            if prim.GetAttribute(name).IsAuthored():
                prim.RemoveProperty(name)
                count += 1
    return count


def _set_lights(stage: Usd.Stage, root: Usd.Prim, world: ElementTree.Element,
                intensity: float) -> int:
    """Give directional lights the SDF's direction and colour, which the converter dropped."""
    count = 0
    for light in world.findall('light'):
        prim = root.GetChild(light.get('name'))
        if light.get('type') != 'directional' or not prim.IsA(UsdLux.DistantLight):
            continue
        for name in LEGACY_LIGHT_ATTRIBUTES:
            if prim.GetAttribute(name).IsAuthored():
                prim.RemoveProperty(name)
        distant = UsdLux.DistantLight(prim)
        distant.CreateIntensityAttr(intensity)
        diffuse = [float(value) for value in light.findtext('diffuse', '1 1 1 1').split()]
        distant.CreateColorAttr(Gf.Vec3f(*diffuse[:3]))

        # A distant light shines along its local -z.
        direction = Gf.Vec3d(*[float(v) for v in light.findtext('direction', '0 0 -1').split()])
        rotation = Gf.Rotation(Gf.Vec3d(0.0, 0.0, -1.0), direction.GetNormalized())
        xformable = UsdGeom.Xformable(prim)
        translation = xformable.GetLocalTransformation().ExtractTranslation()
        xformable.ClearXformOpOrder()
        for attribute in prim.GetAttributes():
            if attribute.GetName().startswith('xformOp:'):
                prim.RemoveProperty(attribute.GetName())
        xformable.AddTranslateOp().Set(translation)
        xformable.AddOrientOp().Set(Gf.Quatf(rotation.GetQuat()))
        count += 1
    return count


def _author_collision_groups(stage: Usd.Stage, root: Usd.Prim,
                             world: ElementTree.Element) -> str:
    """
    Translate SDF collide bitmasks into UsdPhysics collision groups.

    Two SDF collisions collide when their bitmasks share a bit. Each bitmask other than the
    default becomes a group of the models that use it; groups that share no bit filter each
    other. Prims in no group, such as the robot, keep the default and collide with all.
    """
    groups_path = root.GetPath().AppendChild(COLLISION_GROUPS)
    if stage.GetPrimAtPath(groups_path):
        stage.RemovePrim(groups_path)
    members = {}
    for model in world.findall('model'):
        masks = {int(collision.findtext('surface/contact/collide_bitmask')
                     or str(DEFAULT_BITMASK), 0)
                 for collision in model.iter('collision')}
        if len(masks) > 1:
            raise SystemExit(f'model {model.get("name")} mixes collide bitmasks {masks}; '
                             'per-collision groups are not supported yet')
        if masks and (mask := masks.pop()) != DEFAULT_BITMASK:
            if not 0 < mask < DEFAULT_BITMASK:
                raise SystemExit(f'model {model.get("name")}: bitmask {mask:#x} is not supported')
            members.setdefault(mask, []).append(model.get('name'))
    if not members:
        return 'none'

    UsdGeom.Scope.Define(stage, groups_path)
    groups = {}
    for mask, models in sorted(members.items()):
        group_path = groups_path.AppendChild(f'bitmask_{mask:#06x}')
        group = UsdPhysics.CollisionGroup.Define(stage, group_path)
        includes = group.GetCollidersCollectionAPI().CreateIncludesRel()
        for name in models:
            includes.AddTarget(root.GetPath().AppendChild(name))
        groups[mask] = group
    for mask, group in groups.items():
        filtered = group.CreateFilteredGroupsRel()
        for other_mask, other in groups.items():
            if mask != other_mask and mask & other_mask == 0:
                filtered.AddTarget(other.GetPath())
    return ', '.join(f'{mask:#06x}: {len(models)} models' for mask, models in members.items())


def _set_stage_metadata(stage: Usd.Stage, root: Usd.Prim) -> None:
    layer = stage.GetRootLayer()
    for key in TIME_METADATA:
        if layer.pseudoRoot.HasInfo(key):
            layer.pseudoRoot.ClearInfo(key)
    root_spec = layer.GetPrimAtPath(root.GetPath())
    if not root_spec.typeName:
        root_spec.typeName = 'Xform'
    stage.SetDefaultPrim(root)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)


def _remove_api_schema(spec: Sdf.PrimSpec, schema: str) -> None:
    """Remove an applied schema from the prim's own list op, without leaving a delete."""
    operation = spec.GetInfo('apiSchemas')

    def keep(items):
        return [item for item in items if item != schema]

    if operation.isExplicit:
        result = Sdf.TokenListOp.CreateExplicit(keep(operation.explicitItems))
    else:
        result = Sdf.TokenListOp.Create(prependedItems=keep(operation.prependedItems),
                                        appendedItems=keep(operation.appendedItems),
                                        deletedItems=operation.deletedItems)
    empty = not (result.explicitItems or result.prependedItems or result.appendedItems
                 or result.deletedItems)
    if empty:
        spec.ClearInfo('apiSchemas')
    else:
        spec.SetInfo('apiSchemas', result)


if __name__ == '__main__':
    sys.exit(main())
