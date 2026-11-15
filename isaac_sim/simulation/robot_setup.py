"""Physics settings applied to the referenced robot on the live stage."""

import math
import re
from xml.etree import ElementTree

from pxr import PhysxSchema
from pxr import Usd
from pxr import UsdPhysics

ANGULAR = 'angular'
LINEAR = 'linear'


def configure_robot(stage: Usd.Stage, robot_path: str, robot_manifest: dict, config: dict) -> None:
    """Apply drives, start pose, solver settings and self-collision filtering."""
    robot = stage.GetPrimAtPath(robot_path)
    joints = {
        prim.GetName(): prim
        for prim in Usd.PrimRange(robot) if prim.IsA(UsdPhysics.Joint)
    }
    _configure_drives(joints, config['drives'], robot_manifest['effort_limits'],
                      config['initial_joint_positions'])
    _configure_articulation(robot, config['articulation'])
    _filter_collision_pairs(
        stage, joints, _child_link_joints(robot_manifest['urdf']),
        robot_manifest['disabled_collision_pairs'])


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
