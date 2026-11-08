#!/usr/bin/env python3
"""Driver for the SCHUNK FTM 75 force-torque sensor on the wrist (CAN).

The sensor sits on the gripper bus (AUX CAN, can1 next to the WSG 50) and
speaks the SCHUNK/CPR "Force Torque Sensor 75CAN" protocol (manual FTL 01.02):
  request  0x50 [0x07, ts]            ts = 0..63, echoed in the answer
  answer   0x51 [st, c1H c1L, c2H c2L, c3H c3L, tempH]
           0x52 [st, c4H c4L, c5H c5L, c6H c6L, tempL]
  st: bits 0-5 timestamp, bit 6 overload, bit 7 system error (st > 63 = error)
The six channels are raw strain-gauge counts (idle ~32000, range ~5000..59000).
Forces/torques are  wrench = C @ (raw - bias)  with the sensor's own 6x6
calibration matrix (quality sheet / calibMatrix.dat). Without the matrix the
node still publishes the bias-free raw counts, which is enough for contact
detection; set `calibration_matrix` (36 numbers, row-major, rows Fx Fy Fz Tx Ty Tz)
to also get geometry_msgs/WrenchStamped.

Topics:  ~/raw        std_msgs/Float64MultiArray  raw counts - bias (6)
         ~/magnitude  std_msgs/Float64            |raw - bias| (counts), for contact checks
         ~/wrench     geometry_msgs/WrenchStamped (only with a calibration matrix)
         ~/wrench_external  WrenchStamped: ~/wrench minus the tool's own weight
                      (tool_mass, tool_com; orientation from TF) = what you or the
                      world push on the gripper with, in any wrist orientation
Service: ~/bias       std_srvs/Trigger            tare: the next bias_samples become zero (returns at once)
"""
import threading
import time

import can
import numpy as np
import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
import tf2_ros
from std_msgs.msg import Float64, Float64MultiArray
from std_srvs.srv import Trigger


class FTM:
    def __init__(self, channel="can1", req_id=0x50):
        self.req_id = req_id
        self.bus = can.Bus(interface="socketcan", channel=channel,
                           can_filters=[{"can_id": req_id + 1, "can_mask": 0x7FF},
                                        {"can_id": req_id + 2, "can_mask": 0x7FF}])
        self.ts = 0
        while self.bus.recv(0.01):
            pass

    def close(self):
        self.bus.shutdown()

    def read(self, timeout=0.05):
        """One measurement: (raw[6] as float, temperature degC, status byte). Raises on timeout."""
        self.ts = (self.ts + 1) % 64
        self.bus.send(can.Message(arbitration_id=self.req_id, data=[0x07, self.ts], is_extended_id=False))
        parts, t0 = {}, time.time()
        while len(parts) < 2 and time.time() - t0 < timeout:
            m = self.bus.recv(timeout)
            if m is None or len(m.data) != 8:
                continue
            if (m.data[0] & 0x3F) != self.ts:
                continue                       # stale answer to an earlier request
            parts[m.arbitration_id - self.req_id] = bytes(m.data)
        if len(parts) < 2:
            raise TimeoutError("no answer from the FTM")
        a, b = parts[1], parts[2]
        raw = [int.from_bytes(a[i:i + 2], "big") for i in (1, 3, 5)] + \
              [int.from_bytes(b[i:i + 2], "big") for i in (1, 3, 5)]
        temp = (256 * a[7] + b[7]) * 0.12 - 201.0
        return np.array(raw, float), temp, a[0] | b[0]


class FTMNode(Node):
    def __init__(self):
        super().__init__("ftm")
        p = self.declare_parameter
        self.channel = p("can_channel", "can1").value
        self.rate = p("rate_hz", 100.0).value
        self.frame = p("frame_id", "ftm_sensor_frame").value
        self.bias_n = p("bias_samples", 50).value
        cm = list(p("calibration_matrix", [0.0]).value)
        self.C = np.array(cm, float).reshape(6, 6) if len(cm) == 36 else None
        self.tool_m = p("tool_mass", 0.0).value
        self.tool_c = np.array(list(p("tool_com", [0.0, 0.0, 0.0]).value), float)
        self.world = p("world_frame", "world").value
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf, self)
        self.w_g0 = None                      # tool gravity wrench at tare time
        self.ftm = FTM(self.channel, p("can_id", 0x50).value)
        self.bias = None
        self.acc = []
        self.lock = threading.Lock()
        self.pub_raw = self.create_publisher(Float64MultiArray, "~/raw", 10)
        self.pub_mag = self.create_publisher(Float64, "~/magnitude", 10)
        self.pub_w = self.create_publisher(WrenchStamped, "~/wrench", 10) if self.C is not None else None
        self.pub_x = (self.create_publisher(WrenchStamped, "~/wrench_external", 10)
                      if self.C is not None and self.tool_m > 0 else None)
        self.create_service(Trigger, "~/bias", self.srv_bias)
        self.misses = 0
        self.last_err = 0.0
        self.last_tf_warn = 0.0
        self.tf_fail = 0
        self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info(f"FTM on {self.channel}, {self.rate:.0f} Hz, "
                               f"{'calibrated wrench' if self.C is not None else 'raw counts only (no calibration matrix)'}")

    def srv_bias(self, _req, res):
        # the samples are collected by the timer of this (single-threaded) node,
        # so only start the tare here; ~/raw resumes once it is done
        with self.lock:
            self.bias, self.acc = None, []
        res.success = True
        res.message = f"taring: averaging the next {self.bias_n} samples (~{self.bias_n / self.rate:.1f} s)"
        return res

    def tick(self):
        try:
            raw, temp, st = self.ftm.read(0.02)
        except TimeoutError:
            # the sensor skips ~5-10 % of the requests; only a run of misses matters
            self.misses += 1
            if self.misses == 10 or self.misses % 500 == 0:
                self.get_logger().warn(f"FTM not answering ({self.misses} requests in a row)")
            return
        self.misses = 0
        if st > 0x3F and time.time() - self.last_err > 1.0:
            self.last_err = time.time()
            what = ("overload " if st & 0x40 else "") + ("system error" if st & 0x80 else "")
            self.get_logger().error(f"FTM status 0x{st:02X}: {what.strip()}")
        with self.lock:
            if self.bias is None:              # collecting the tare
                if self.tool_m > 0 and self.tool_gravity() is None:
                    # the gravity reference of the tare pose needs the TF tree: wait for it
                    if time.time() - self.last_tf_warn > 5.0:
                        self.last_tf_warn = time.time()
                        self.get_logger().warn("tare waits for TF (needed for gravity compensation)")
                    return
                self.acc.append(raw)
                if len(self.acc) >= self.bias_n:
                    self.bias = np.mean(self.acc, axis=0)
                    self.w_g0 = self.tool_gravity()
                    self.get_logger().info(f"bias set ({len(self.acc)} samples), temp {temp:.1f} C"
                                           f"{'' if self.pub_x is None or self.w_g0 is not None else ' - no TF, no gravity compensation'}")
                return
            d = raw - self.bias
        stamp = self.get_clock().now().to_msg()
        self.pub_raw.publish(Float64MultiArray(data=d.tolist()))
        self.pub_mag.publish(Float64(data=float(np.linalg.norm(d))))
        if self.pub_w is not None:
            w = self.C @ d
            m = WrenchStamped()
            m.header.stamp, m.header.frame_id = stamp, self.frame
            (m.wrench.force.x, m.wrench.force.y, m.wrench.force.z,
             m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z) = w.tolist()
            self.pub_w.publish(m)
            if self.pub_x is not None and self.w_g0 is not None:
                wg = self.tool_gravity()
                if wg is not None:
                    x = WrenchStamped(); x.header = m.header
                    (x.wrench.force.x, x.wrench.force.y, x.wrench.force.z,
                     x.wrench.torque.x, x.wrench.torque.y, x.wrench.torque.z) = (w - (wg - self.w_g0)).tolist()
                    self.pub_x.publish(x)

    def tool_gravity(self):
        """Wrench of the tool's own weight in the sensor frame, or None without TF."""
        if self.tool_m <= 0:
            return None
        try:
            q = self.tf.lookup_transform(self.world, self.frame, Time()).transform.rotation
        except Exception as e:
            self.tf_fail += 1
            if self.tf_fail % 50 == 1:
                self.get_logger().warn(f"no TF {self.world}->{self.frame} for gravity compensation "
                                       f"({self.tf_fail} times): ~/wrench_external is skipped meanwhile ({e})")
            return None
        self.tf_fail = 0
        f = Rotation.from_quat([q.x, q.y, q.z, q.w]).inv().apply([0.0, 0.0, -9.81 * self.tool_m])
        return np.concatenate([f, np.cross(self.tool_c, f)])


def main():
    rclpy.init()
    node = FTMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ftm.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
