"""Isaac Lab articulation config for the Schunk LWA 4P + WSG 50 (lwa4p_wsg50.urdf).

TEMPLATE - written for Isaac Lab 2.x (``isaaclab`` namespace) but not run here
(no NVIDIA GPU on the machine it was made on). Check the argument names against
your Isaac Lab version; the numbers are the part that matters.

    from lwa4p_cfg import LWA4P_CFG, LWA4P_REAL_SPEED_CFG
    robot = LWA4P_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

Joints (rad / m):
    arm_1_joint .. arm_6_joint           revolute, limits in the URDF
    gripper_finger_left_joint            prismatic 0 .. 0.04475 (opening)
    gripper_finger_right_joint           prismatic 0 .. 0.04475 (opening)
    fingertip gap = left + right         (the real gripper reports this gap)
Frames: gripper_tcp (between the fingertips, 15 mm above the tips),
        camera_color_optical_frame (RealSense, ROS optical convention).
"""
import math
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg

URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lwa4p_wsg50.urdf")

# SCHUNK data (ERB manual 3.1 + slides): ERB 145 (axes 1-4) / ERB 115 (axes 5-6)
MAX_TORQUE_BIG, MAX_TORQUE_WRIST = 64.0, 19.0                  # Nm (rated 35 / 7)
MAX_SPEED = math.radians(72.0)                                  # rad/s, all axes
MAX_ACCEL_BIG, MAX_ACCEL_WRIST = math.radians(250), math.radians(500)   # rad/s^2
# ImplicitActuatorCfg has no acceleration limit: rate-limit your joint targets
# (or use a joint trajectory) so commanded accelerations stay below these.
# what the REAL arm can do with its current (weak) power supply
REAL_SPEED = 0.5            # rad/s  (MoveIt limits; also 0.4 rad/s^2)

HOME = {f"arm_{i}_joint": 0.0 for i in range(1, 7)} | {
    "gripper_finger_left_joint": 0.0425, "gripper_finger_right_joint": 0.0425}   # open 85 mm
# gripper_tcp at (0.40, 0, 0.30) m, pointing straight down (checked with forward kinematics)
READY = HOME | {"arm_2_joint": -0.35, "arm_3_joint": 0.80, "arm_5_joint": 2.00}


def _cfg(arm_speed):
    return ArticulationCfg(
        spawn=sim_utils.UrdfFileCfg(
            asset_path=URDF,
            fix_base=True,
            merge_fixed_joints=False,          # keep gripper_tcp and the camera frames
            make_instanceable=True,
            joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=100.0, damping=10.0)),   # overridden by the actuators below
            rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=False, max_depenetration_velocity=5.0),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False, solver_position_iteration_count=12,
                solver_velocity_iteration_count=1),
        ),
        init_state=ArticulationCfg.InitialStateCfg(joint_pos=READY),
        actuators={
            "shoulder_elbow": ImplicitActuatorCfg(
                joint_names_expr=["arm_[1-4]_joint"],
                effort_limit_sim=MAX_TORQUE_BIG, velocity_limit_sim=arm_speed,
                stiffness=400.0, damping=40.0),
            "wrist": ImplicitActuatorCfg(
                joint_names_expr=["arm_[5-6]_joint"],
                effort_limit_sim=MAX_TORQUE_WRIST, velocity_limit_sim=arm_speed,
                stiffness=100.0, damping=10.0),
            # WSG 50-110: 5..80 N grip force, 420 mm/s / 5 m/s^2 finger to finger
            # -> 0.21 m/s, 2.5 m/s^2 per finger; each finger 0..45.25 mm.
            # Command both fingers with the same target (gap / 2).
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=["gripper_finger_.*_joint"],
                effort_limit_sim=80.0, velocity_limit_sim=0.21,
                stiffness=2000.0, damping=100.0),
        },
    )


# datasheet speed (72 deg/s)
LWA4P_CFG = _cfg(MAX_SPEED)
# speed the real arm actually runs at today - use this for policies you will deploy
LWA4P_REAL_SPEED_CFG = _cfg(REAL_SPEED)
