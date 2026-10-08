#!/usr/bin/env python3
"""Simulation only: make both WSG fingers follow gripper_joint (fingertip gap).

MoveIt / your code command gripper_joint (gripper_controller). This node
sends finger_controller the matching finger positions -gap/2 and +gap/2.
The fingers are PID-driven in Gazebo, so when they hit an object they stop
there and squeeze it - that is what makes grasping work.

It also publishes joint_states_measured: /joint_states with gripper_joint
replaced by the MEASURED finger distance (like the real gripper reports it).
robot_state_publisher and move_group use that, so RViz shows the fingers
resting on the object instead of the commanded 0 mm.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class FingerMimic(Node):
    def __init__(self):
        super().__init__("finger_mimic")
        self.pub = self.create_publisher(Float64MultiArray, "finger_controller/commands", 10)
        self.measured = self.create_publisher(JointState, "joint_states_measured", 10)
        self.create_subscription(JointState, "joint_states", self.on_js, 10)

    def on_js(self, msg):
        names = list(msg.name)
        if "gripper_joint" in names:
            gap = msg.position[names.index("gripper_joint")]
            self.pub.publish(Float64MultiArray(data=[-gap / 2.0, gap / 2.0]))
        if {"gripper_joint", "gripper_finger_left_joint", "gripper_finger_right_joint"} <= set(names):
            pos = list(msg.position)
            pos[names.index("gripper_joint")] = max(0.0, pos[names.index("gripper_finger_right_joint")]
                                                    - pos[names.index("gripper_finger_left_joint")])
            msg.position = pos
        self.measured.publish(msg)


def main():
    rclpy.init()
    node = FingerMimic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
