#!/usr/bin/env python3
import math
import os
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints, JointConstraint,
                             MoveItErrorCodes, OrientationConstraint, PlanningScene, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

ARM_JOINTS = [f"arm_{i}_joint" for i in range(1, 7)]
GRIPPER_LINKS = ["gripper_tcp", "gripper_gap_link", "gripper_body_link",
                 "gripper_finger_left_link", "gripper_finger_right_link"]
ERROR_NAMES = {v: k for k, v in MoveItErrorCodes.__dict__.items()
               if k.isupper() and isinstance(v, int)}


class SimpleMoveIt(Node):
    def __init__(self, name="simple_moveit", velocity_scale=0.5):
        super().__init__(name)
        self.scale = velocity_scale
        self.client = ActionClient(self, MoveGroup, "move_action")
        self.scene = self.create_client(ApplyPlanningScene, "apply_planning_scene")
        self.cartesian = self.create_client(GetCartesianPath, "compute_cartesian_path")
        self.execute = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.get_logger().info("waiting for move_group ...")
        if not self.client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError("move_group not found - is the simulation / arm_start.sh running?")
        srdf = os.path.join(get_package_share_directory("lwa4p_moveit_config"), "config", "lwa4p.srdf")
        self.states = {}
        for gs in ET.parse(srdf).getroot().iter("group_state"):
            self.states[gs.get("name")] = (gs.get("group"), {
                j.get("name"): float(j.get("value")) for j in gs.iter("joint")})

    # ---------- motions ----------
    def named(self, state):
        group, values = self.states[state]
        return self._move(group, self._joint_goal(values), f"'{state}'")

    def joints(self, degrees):
        values = {j: math.radians(d) for j, d in zip(ARM_JOINTS, degrees)}
        return self._move("arm", self._joint_goal(values), f"joints {list(degrees)} deg")

    def gripper(self, gap):
        return self._move("gripper", self._joint_goal({"gripper_joint": gap}),
                          f"gripper {gap * 1000:.0f} mm")

    def pose(self, x, y, z, down=True, yaw=None, tolerance=0.005):
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = "world"
        pc.link_name = "gripper_tcp"
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[tolerance])
        target = Pose()
        target.position.x, target.position.y, target.position.z = x, y, z
        target.orientation.w = 1.0
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(target)
        pc.weight = 1.0
        c.position_constraints.append(pc)
        if down:
            oc = OrientationConstraint()
            oc.header.frame_id = "world"
            oc.link_name = "gripper_tcp"
            # tool z down, fingers closing along 'yaw' (None = any)
            oc.orientation.x = math.cos((yaw or 0.0) / 2.0)
            oc.orientation.y = math.sin((yaw or 0.0) / 2.0)
            oc.orientation.w = 0.0
            oc.absolute_x_axis_tolerance = 0.05
            oc.absolute_y_axis_tolerance = 0.05
            oc.absolute_z_axis_tolerance = math.pi if yaw is None else 0.05
            oc.weight = 1.0
            c.orientation_constraints.append(oc)
        return self._move("arm", c, f"pose ({x:.3f}, {y:.3f}, {z:.3f})")

    def line(self, x, y, z):
        # straight line of gripper_tcp to x, y, z, keeping its orientation - for approach and lift
        f = rclpy.task.Future()
        while not self.tf.can_transform("world", "gripper_tcp", Time()):
            rclpy.spin_until_future_complete(self, f, timeout_sec=0.1)
        target = Pose()
        target.position.x, target.position.y, target.position.z = x, y, z
        target.orientation = self.tf.lookup_transform("world", "gripper_tcp", Time()).transform.rotation
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"
        req.group_name = "arm"
        req.link_name = "gripper_tcp"
        req.start_state.is_diff = True
        req.waypoints = [target]
        req.max_step = 0.005
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = self.scale
        req.max_acceleration_scaling_factor = self.scale
        what = f"line to ({x:.3f}, {y:.3f}, {z:.3f})"
        self.get_logger().info(f"arm: {what} ...")
        if not self.cartesian.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("compute_cartesian_path service not found")
        f = self.cartesian.call_async(req)
        rclpy.spin_until_future_complete(self, f)
        res = f.result()
        if res.fraction < 0.99:
            self.get_logger().error(f"arm: {what} FAILED (only {res.fraction * 100:.0f} % possible)")
            return False
        f = self.execute.send_goal_async(ExecuteTrajectory.Goal(trajectory=res.solution))
        rclpy.spin_until_future_complete(self, f)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        code = rf.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"arm: {what} FAILED ({ERROR_NAMES.get(code, code)})")
            return False
        return True

    # ---------- planning scene ----------
    def add_box(self, name, size, xyz):
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = name
        co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size)))
        p = Pose()
        p.position.x, p.position.y, p.position.z = xyz
        p.orientation.w = 1.0
        co.primitive_poses.append(p)
        co.operation = CollisionObject.ADD
        return self._apply(co)

    def remove(self, name):
        # objects stay in move_group after your script ends - remove what you no longer want
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = name
        co.operation = CollisionObject.REMOVE
        return self._apply(co)

    def attach_box(self, name, size, z=0.0):
        # a grasped object: box (along the finger direction, across, height) centred
        # z below gripper_tcp. MoveIt then shows it and checks it for collisions.
        aco = AttachedCollisionObject(link_name="gripper_tcp", touch_links=GRIPPER_LINKS)
        aco.object.header.frame_id = "gripper_tcp"
        aco.object.id = name
        aco.object.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(size)))
        p = Pose()
        p.position.z = z
        p.orientation.w = 1.0
        aco.object.primitive_poses.append(p)
        aco.object.operation = CollisionObject.ADD
        return self._apply(attached=aco)

    def detach(self, name):
        aco = AttachedCollisionObject(link_name="gripper_tcp")
        aco.object.id = name
        aco.object.operation = CollisionObject.REMOVE
        self._apply(attached=aco)
        return self.remove(name)        # MoveIt puts a detached object into the world

    def _apply(self, co=None, attached=None):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        if co is not None:
            scene.world.collision_objects.append(co)
        if attached is not None:
            scene.robot_state.attached_collision_objects.append(attached)
        if not self.scene.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("apply_planning_scene service not found")
        f = self.scene.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, f)
        return f.result().success

    # ---------- internals ----------
    def _joint_goal(self, values):
        return Constraints(joint_constraints=[
            JointConstraint(joint_name=j, position=v, tolerance_above=1e-3,
                            tolerance_below=1e-3, weight=1.0) for j, v in values.items()])

    def _move(self, group, constraints, what):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name = group
        r.num_planning_attempts = 5
        r.allowed_planning_time = 5.0
        r.max_velocity_scaling_factor = self.scale
        r.max_acceleration_scaling_factor = self.scale
        r.start_state.is_diff = True
        r.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        self.get_logger().info(f"{group}: {what} ...")
        f = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        gh = f.result()
        if not gh.accepted:
            self.get_logger().error("goal rejected by move_group")
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        code = rf.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"{group}: {what} FAILED ({ERROR_NAMES.get(code, code)})")
            return False
        return True
