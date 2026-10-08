import threading
import time

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from lwa4p_driver.wsg50 import E_SUCCESS, STATUS, WSG

JAW_MAX_MM = 109.0


class GripperNode(Node):
    def __init__(self):
        super().__init__("gripper")
        p = self.declare_parameter
        self.channel = p("can_channel", "can1").value
        self.offset = p("finger_offset_mm", 19.5).value
        self.default_force = p("default_force_n", 35.0).value
        self.grasp_speed = p("grasp_speed_mm_s", 80.0).value
        self.move_speed = p("move_speed_mm_s", 50.0).value
        self.accel = p("acceleration_mm_s2", 100.0).value
        self.open_gap = p("open_gap_mm", 30.0).value
        rate = p("state_rate_hz", 10.0).value

        self.lock = threading.Lock()
        self.wsg = WSG(self.channel)
        with self.lock:
            self.wsg.set_accel(self.accel)
            if self.wsg.grasping_state() not in ("HOLDING", "GRASPING"):
                self.wsg.call(0x38)                 # tare force measurement (only when empty)
            flags, names = self.wsg.system_state()
        self.referenced = "REFERENCED" in names
        self.gap_mm, self.force_n, self.gstate = 0.0, 0.0, "?"
        self.poll()

        cb = ReentrantCallbackGroup()
        self.pub = self.create_publisher(JointState, "gripper/state", 10)
        # same value as the URDF/MoveIt joint "gripper_joint" (fingertip gap, m)
        self.js_pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(1.0 / rate, self.publish, callback_group=cb)
        self.create_service(Trigger, "~/home", self.srv_home, callback_group=cb)
        self.create_service(Trigger, "~/open", self.srv_open, callback_group=cb)
        self.create_service(Trigger, "~/stop", self.srv_stop, callback_group=cb)
        ActionServer(self, GripperCommand, "gripper_controller/gripper_cmd",
                     execute_callback=self.execute, goal_callback=lambda _: GoalResponse.ACCEPT,
                     cancel_callback=self.on_cancel, callback_group=cb)
        self.get_logger().info(
            f"WSG 50 on {self.channel}: {'referenced' if self.referenced else 'NOT referenced - call ~/home'}, "
            f"gap {self.gap_mm:.1f} mm, {self.gstate}")

    # ---------- helpers ----------
    def jaw(self, gap_mm):
        return max(0.0, min(JAW_MAX_MM, gap_mm + self.offset))

    def poll(self):
        self.gap_mm = self.wsg.width() - self.offset
        self.force_n = self.wsg.force()
        self.gstate = self.wsg.grasping_state()

    def publish(self):
        if self.lock.acquire(blocking=False):
            try:
                self.poll()
            except Exception as e:
                self.get_logger().warn(f"poll failed: {e}", throttle_duration_sec=5.0)
            finally:
                self.lock.release()
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = ["gripper_gap"]
        m.position = [self.gap_mm / 1000.0]
        m.effort = [self.force_n]
        self.pub.publish(m)
        j = JointState()
        j.header.stamp = m.header.stamp
        j.name = ["gripper_joint"]
        j.position = [max(0.0, self.gap_mm) / 1000.0]
        self.js_pub.publish(j)

    def prepare(self):
        flags, names = self.wsg.system_state()
        if "FAST_STOP" in names:
            self.wsg.ack()
        if self.wsg.grasping_state() in ("HOLDING", "ERROR", "NO PART FOUND", "PART LOST"):
            self.wsg.release(min(JAW_MAX_MM, self.wsg.width() + 5.0), self.move_speed)

    # ---------- action ----------
    def on_cancel(self, _):
        self.wsg.send(0x22)
        return CancelResponse.ACCEPT

    def execute(self, gh):
        gap = gh.request.command.position * 1000.0
        force = gh.request.command.max_effort
        if force == 0.0:
            force = self.default_force
        res = GripperCommand.Result()
        with self.lock:
            try:
                if not self.referenced:
                    raise RuntimeError("gripper not referenced - call ~/home first")
                self.prepare()
                if force > 0.0 and gap < self.gap_mm - 1.0:
                    self.wsg.set_force(max(5.0, min(80.0, force)))
                    st = self.wsg.grasp(self.jaw(gap), self.grasp_speed)
                    what = f"grasp {gap:.1f} mm @ {force:.0f} N"
                else:
                    st = self.wsg.move(self.jaw(gap), self.move_speed)
                    what = f"move to {gap:.1f} mm"
                self.poll()
            except Exception as e:
                self.get_logger().error(f"gripper command failed: {e}")
                gh.abort()
                return res
        res.position = self.gap_mm / 1000.0
        res.effort = self.force_n
        res.stalled = self.gstate == "HOLDING"
        res.reached_goal = st == E_SUCCESS
        self.get_logger().info(f"{what}: {STATUS.get(st, st)}, gap {self.gap_mm:.1f} mm, "
                               f"{self.force_n:.1f} N, {self.gstate}")
        if gh.is_cancel_requested:
            gh.canceled()
        elif st == E_SUCCESS:
            gh.succeed()
        else:
            gh.abort()
        return res

    # ---------- services ----------
    def srv_home(self, req, res):
        with self.lock:
            try:
                self.prepare()
                st = self.wsg.home()
                self.referenced = st == E_SUCCESS
                self.wsg.call(0x38)
                self.poll()
                res.success, res.message = self.referenced, f"home: {STATUS.get(st, st)}, gap {self.gap_mm:.1f} mm"
            except Exception as e:
                res.success, res.message = False, str(e)
        return res

    def srv_open(self, req, res):
        with self.lock:
            try:
                self.prepare()
                st = self.wsg.move(self.jaw(self.open_gap), self.move_speed)
                self.poll()
                res.success, res.message = st == E_SUCCESS, f"open: {STATUS.get(st, st)}, gap {self.gap_mm:.1f} mm"
            except Exception as e:
                res.success, res.message = False, str(e)
        return res

    def srv_stop(self, req, res):
        self.wsg.send(0x22)
        res.success, res.message = True, "stop sent"
        return res


def main():
    rclpy.init()
    node = GripperNode()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.wsg.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
