"""Physics settings applied to the referenced robot on the live stage."""

import math
import re
from xml.etree import ElementTree

from pxr import PhysxSchema
from pxr import Usd
from pxr import UsdPhysics
from simulation import contact

ANGULAR = 'angular'
LINEAR = 'linear'


def configure_robot(stage: Usd.Stage, robot_path: str, robot_manifest: dict,
                    config: dict) -> list[Usd.Prim]:
    """
    Apply drives, start pose, solver settings, contact materials, self-collision filter.

    Returns the rigid bodies of the mounted base: the links fixed to the world link.
    """
    robot = stage.GetPrimAtPath(robot_path)
    joints = {
        prim.GetName(): prim
        for prim in Usd.PrimRange(robot) if prim.IsA(UsdPhysics.Joint)
    }
    child_link_joints = _child_link_joints(robot_manifest['urdf'])
    _configure_drives(joints, config['drives'], robot_manifest['effort_limits'],
                      _initial_positions(robot_manifest['named_states'], config['initial_state']))
    configured = _configure_joint_dynamics(
        joints, config.get('joint_dynamics', {}), robot_manifest['effort_limits'])
    print(f'[isaac_sim] joint friction and armature: {configured}', flush=True)
    _configure_articulation(robot, config['articulation'])
    colliders = configure_mesh_colliders(robot, config['collision'])
    print(f'[isaac_sim] {len(colliders)} robot mesh colliders: '
          f'{config["collision"]["mesh_approximation"]}', flush=True)
    links = {link: body for link in child_link_joints
             if (body := _link_body(stage, joints, child_link_joints, link)) is not None}
    bound = contact.bind_by_name(stage, links, config['contact']['robot'],
                                 config['contact']['materials'], 'robot link')
    print(f'[isaac_sim] robot contact materials: {bound}', flush=True)
    _filter_collision_pairs(
        stage, joints, child_link_joints, robot_manifest['disabled_collision_pairs'])
    return [links[link] for link in robot_manifest['mounted_links'] if link in links]


def _initial_positions(named_states: dict, initial_state: dict) -> dict[str, float]:
    """Merge the SRDF states chosen per group, for example arm: home, gripper: open."""
    positions = {}
    for group, state in initial_state.items():
        if state not in named_states.get(group, {}):
            raise ValueError(f'the SRDF has no state {state!r} for group {group!r}; it has '
                             f'{sorted(named_states.get(group, {}))}')
        positions.update(named_states[group][state])
    return positions


def _configure_drives(joints: dict, drives: dict, effort_limits: dict,
                      initial_positions: dict) -> None:
    for name, effort_limit in effort_limits.items():
        joint = joints[name]
        axis = ANGULAR if joint.IsA(UsdPhysics.RevoluteJoint) else LINEAR
        # USD stores angles in degrees: gains per degree, targets in degrees.
        to_usd = math.pi / 180.0 if axis == ANGULAR else 1.0
        position_to_usd = 180.0 / math.pi if axis == ANGULAR else 1.0
        stiffness, damping = _gains_for(name, drives)
        initial_position = initial_positions.get(name, 0.0) * position_to_usd

        drive = UsdPhysics.DriveAPI.Apply(joint, axis)
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(stiffness * to_usd)
        drive.CreateDampingAttr(damping * to_usd)
        drive.CreateMaxForceAttr(effort_limit)
        drive.CreateTargetPositionAttr(initial_position)
        PhysxSchema.JointStateAPI.Apply(joint, axis).CreatePositionAttr(initial_position)


def configure_mesh_colliders(robot: Usd.Prim, settings: dict) -> list[Usd.Prim]:
    """
    Set the collision approximation of every robot mesh collider; return the colliders.

    The importer puts the colliders in instances, whose prims are read-only proxies on the
    stage, so the instances that hold a mesh collider are made ordinary prims first (an
    opinion in the in-memory root layer; the cached USD does not change).
    """
    while True:
        instances = [prim for prim in Usd.PrimRange(robot)
                     if prim.IsInstance() and _holds_mesh_collider(prim.GetPrototype())]
        if not instances:
            break
        for prim in instances:
            prim.SetInstanceable(False)
    colliders = mesh_colliders(robot)
    approximation = settings['mesh_approximation']
    decomposition = settings['convex_decomposition']
    for prim in colliders:
        UsdPhysics.MeshCollisionAPI(prim).CreateApproximationAttr().Set(approximation)
        if approximation != UsdPhysics.Tokens.convexDecomposition:
            continue
        api = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
        api.CreateMaxConvexHullsAttr(int(decomposition['max_convex_hulls']))
        api.CreateHullVertexLimitAttr(int(decomposition['hull_vertex_limit']))
        api.CreateVoxelResolutionAttr(int(decomposition['voxel_resolution']))
        api.CreateErrorPercentageAttr(float(decomposition['error_percentage']))
        api.CreateMinThicknessAttr(float(decomposition['min_thickness']))
        api.CreateShrinkWrapAttr(bool(decomposition['shrink_wrap']))
    return colliders


def mesh_colliders(robot: Usd.Prim) -> list[Usd.Prim]:
    """Return the robot's mesh colliders outside instances."""
    return [prim for prim in Usd.PrimRange(robot) if prim.HasAPI(UsdPhysics.MeshCollisionAPI)]


def convex_hull_counts(stage_id: int, colliders: list[Usd.Prim]) -> dict[str, int | str]:
    """
    Cook the colliders now and return {link: number of convex hulls PhysX made}.

    A collider that fails to cook reports the PhysX result instead of a count. The cooked
    shapes go to PhysX's local mesh cache, so only the first start with new meshes or new
    decomposition settings is slow.
    """
    # PhysX's Python modules exist only once Isaac Sim runs.
    from omni.physx import get_physx_cooking_interface
    from omni.physx.bindings._physx import PhysxCollisionRepresentationResult
    from pxr import PhysicsSchemaTools

    cooking = get_physx_cooking_interface()
    counts = {}
    for prim in colliders:
        reply = {}

        def on_result(result, convexes, reply=reply):
            reply['result'], reply['count'] = result, len(convexes)

        cooking.request_convex_collision_representation(
            stage_id=stage_id, collision_prim_id=PhysicsSchemaTools.sdfPathToInt(prim.GetPath()),
            run_asynchronously=False, on_result=on_result)
        valid = reply.get('result') == PhysxCollisionRepresentationResult.RESULT_VALID
        counts[_owning_body_name(prim)] = reply['count'] if valid else str(reply.get('result'))
    return counts


def _holds_mesh_collider(prototype: Usd.Prim) -> bool:
    return any(prim.HasAPI(UsdPhysics.MeshCollisionAPI) for prim in Usd.PrimRange(prototype))


def _owning_body_name(prim: Usd.Prim) -> str:
    """Return the name of the rigid body (the link) a collider belongs to."""
    body = prim
    while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):
        body = body.GetParent()
    return body.GetName() if body else prim.GetName()


def _configure_joint_dynamics(joints: dict, dynamics: dict, actuated: dict) -> list[str]:
    """
    Apply joint friction and armature; return the configured joints.

    Config units are SI per radian. USD wants the viscous coefficient per degree; the
    friction efforts and the armature need no conversion. PhysxJointAxisAPI has no Python
    class, so it is applied by name, as Isaac Sim's own articulation code does.
    """
    unmatched = [pattern for pattern in dynamics
                 if not any(re.fullmatch(pattern, name) for name in actuated)]
    if unmatched:
        raise ValueError(f'joint_dynamics patterns match no actuated joint: {unmatched}')
    configured = []
    for name in actuated:
        pattern = next((pattern for pattern in dynamics if re.fullmatch(pattern, name)), None)
        if pattern is None:
            continue
        values = dynamics[pattern]
        joint = joints[name]
        axis = ANGULAR if joint.IsA(UsdPhysics.RevoluteJoint) else LINEAR
        per_usd_unit = math.pi / 180.0 if axis == ANGULAR else 1.0
        joint.ApplyAPI('PhysxJointAxisAPI', axis)
        for attribute, value in (
                ('staticFrictionEffort', values['static_friction']),
                ('dynamicFrictionEffort', values['dynamic_friction']),
                ('viscousFrictionCoefficient', values['viscous_friction'] * per_usd_unit)):
            usd_attribute = joint.GetAttribute(f'physxJointAxis:{axis}:{attribute}')
            if not usd_attribute:
                raise RuntimeError(f'{name}: PhysX has no joint attribute {attribute!r}')
            usd_attribute.Set(float(value))
        PhysxSchema.PhysxJointAPI.Apply(joint).CreateArmatureAttr(float(values['armature']))
        configured.append(name)
    return configured


def _gains_for(joint_name: str, drives: dict) -> tuple[float, float]:
    for pattern, (stiffness, damping) in drives.items():
        if re.fullmatch(pattern, joint_name):
            return stiffness, damping
    raise ValueError(f'no drive gains in the config match joint {joint_name!r}')


def _configure_articulation(robot: Usd.Prim, settings: dict) -> None:
    roots = [prim for prim in Usd.PrimRange(robot) if prim.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) != 1:
        raise RuntimeError(f'expected one articulation root under the robot, found {len(roots)}')
    articulation = PhysxSchema.PhysxArticulationAPI.Apply(roots[0])
    articulation.CreateSolverPositionIterationCountAttr(settings['solver_position_iterations'])
    articulation.CreateSolverVelocityIterationCountAttr(settings['solver_velocity_iterations'])
    articulation.CreateSleepThresholdAttr(settings['sleep_threshold'])
    articulation.CreateEnabledSelfCollisionsAttr(True)


def _child_link_joints(urdf_path: str) -> dict[str, str]:
    """Map each URDF link to the joint that has it as its child."""
    root = ElementTree.parse(urdf_path).getroot()
    return {joint.find('child').get('link'): joint.get('name') for joint in root.findall('joint')}


def _filter_collision_pairs(stage: Usd.Stage, joints: dict, child_link_joints: dict,
                            pairs: list) -> None:
    """
    Disable PhysX contacts between the link pairs the SRDF disables for MoveIt.

    The importer may rename link prims (arm_base_link becomes arm_base_link_1), but it
    keeps the joint names, so a link is found as body1 of the joint above it.
    """
    for first, second in pairs:
        first_body = _link_body(stage, joints, child_link_joints, first)
        second_body = _link_body(stage, joints, child_link_joints, second)
        if first_body is None or second_body is None:
            continue
        UsdPhysics.FilteredPairsAPI.Apply(first_body).CreateFilteredPairsRel().AddTarget(
            second_body.GetPath())


def _link_body(stage: Usd.Stage, joints: dict, child_link_joints: dict,
               link: str) -> Usd.Prim | None:
    joint = joints.get(child_link_joints.get(link))
    if joint is None:
        return None
    targets = UsdPhysics.Joint(joint).GetBody1Rel().GetTargets()
    if not targets:
        return None
    body = stage.GetPrimAtPath(targets[0])
    return body if body.HasAPI(UsdPhysics.RigidBodyAPI) else None
