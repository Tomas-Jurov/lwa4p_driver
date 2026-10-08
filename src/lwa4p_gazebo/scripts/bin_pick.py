#!/usr/bin/env python3
"""Bin picking with the RealSense point cloud (simulation world lwa4p_table.world).

    ros2 run lwa4p_gazebo bin_pick.py              # move everything into the output bin
    ros2 run lwa4p_gazebo bin_pick.py --look       # only detect and show, no motion

Each cycle: arm to 'home' (out of the camera view) -> one point cloud ->
transform to the world frame -> crop to the inside of the bin -> take the
highest object (its top surface) -> grasp from above (straight down and up
again), fingers closing across its narrow side -> lift -> drop it into the output bin. Stops when the bin
is empty (or when a motion fails - it never opens the gripper elsewhere).
Detections are shown in RViz (MarkerArray /bin_pick/markers).
"""
import argparse
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import JointState, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from simple_moveit import SimpleMoveIt

CLOUD_TOPIC = "/camera/depth/color/points"
BIN_CENTER = (0.24, -0.33)      # pick bin, world frame (see the world file)
OUT_BIN_CENTER = (0.24, 0.33)   # output bin
BIN_INNER = (0.28, 0.22)        # inside size x, y
BIN_FLOOR = -0.05               # top of the bin bottom
BIN_WALL_TOP = 0.04
APPROACH_Z = 0.15               # TCP height above the bin before going down
GRIP_OPEN = 0.07                # opening for dropping
FINGER_T, FINGER_W = 0.020, 0.030   # finger thickness / width (see wsg50.urdf.xacro)
SPEED, SPEED_HOLDING = 0.5, 0.25
FINGER_BELOW_TOP = 0.025        # fingertips this far below the top of the object
TCP_ABOVE_FINGERTIP = 0.015     # gripper_tcp is 15 mm above the fingertips
DROP = [(-0.06, -0.04), (0.06, -0.04), (-0.06, 0.05), (0.06, 0.05), (0.0, 0.0)]   # in the output bin
DROP_Z = 0.12


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Perception(Node):
    """Point cloud, TF and finger positions, spun in its own thread."""

    def __init__(self):
        super().__init__("bin_pick_perception")
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.cloud, self.cloud_count = None, 0
        self.fingers = {}
        self.create_subscription(PointCloud2, CLOUD_TOPIC, self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(JointState, "joint_states", self.on_js, 10)
        self.markers = self.create_publisher(MarkerArray, "bin_pick/markers", 10)

    def on_cloud(self, msg):
        self.cloud = msg
        self.cloud_count += 1

    def on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            if n.startswith("gripper_finger"):
                self.fingers[n] = p

    def finger_gap(self):
        """Measured distance between the fingers (the sim fingers stop at the object)."""
        return self.fingers.get("gripper_finger_right_joint", 0.0) - \
            self.fingers.get("gripper_finger_left_joint", 0.0)

    def points_in_world(self, timeout=30.0):
        """A point cloud taken after this call, as an N x 3 array in the world frame.
        Waits for the 2nd new cloud: the 1st may have been rendered while the arm moved.
        A busy laptop renders the simulated camera slowly, hence the long timeout."""
        t0, n0 = time.time(), self.cloud_count
        while self.cloud_count < n0 + 2:
            if time.time() - t0 > timeout:
                raise RuntimeError(f"no point cloud on {CLOUD_TOPIC} for {timeout:.0f} s "
                                   "(simulation too slow? try gui:=false)")
            time.sleep(0.05)
        msg = self.cloud
        p = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        p = p[np.isfinite(p).all(axis=1)]
        tf = self.tf.lookup_transform("world", msg.header.frame_id, Time(),
                                      timeout=Duration(seconds=5.0)).transform
        t = np.array([tf.translation.x, tf.translation.y, tf.translation.z])
        return p @ quat_to_matrix(tf.rotation).T + t

    def show(self, obj):
        m = MarkerArray()
        sphere = Marker(type=Marker.SPHERE, action=Marker.ADD, id=0, ns="bin_pick")
        sphere.header.frame_id = "world"
        sphere.pose.position.x, sphere.pose.position.y, sphere.pose.position.z = obj["x"], obj["y"], obj["top"]
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.02
        sphere.color.r, sphere.color.g, sphere.color.a = 1.0, 0.5, 1.0
        arrow = Marker(type=Marker.ARROW, action=Marker.ADD, id=1, ns="bin_pick")
        arrow.header.frame_id = "world"
        arrow.pose = sphere.pose
        arrow.pose.orientation.z, arrow.pose.orientation.w = math.sin(obj["yaw"] / 2), math.cos(obj["yaw"] / 2)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.06, 0.006, 0.006
        arrow.color.r, arrow.color.a = 1.0, 1.0
        m.markers = [sphere, arrow]
        self.markers.publish(m)


def find_top_object(points):
    """Highest object inside the bin -> dict(x, y, top, yaw, size) or None."""
    cx, cy = BIN_CENTER
    margin = 0.01
    inside = ((np.abs(points[:, 0] - cx) < BIN_INNER[0] / 2 - margin)
              & (np.abs(points[:, 1] - cy) < BIN_INNER[1] / 2 - margin)
              & (points[:, 2] > BIN_FLOOR + 0.008) & (points[:, 2] < BIN_WALL_TOP + 0.15))
    obj = points[inside]
    if len(obj) < 20:
        return None
    # top surfaces: everything within 8 mm of the highest point
    z_top = np.percentile(obj[:, 2], 99.5)
    tops = obj[obj[:, 2] > z_top - 0.008]
    # one object only: points near the highest one (objects are > 5 cm apart)
    seed = tops[np.argmax(tops[:, 2]), :2]
    for _ in range(3):
        near = tops[np.linalg.norm(tops[:, :2] - seed, axis=1) < 0.035]
        seed = near[:, :2].mean(axis=0)
    # principal axes of the top surface: close the fingers across the narrow side
    xy = near[:, :2] - seed
    evals, evecs = np.linalg.eigh(np.cov(xy.T))
    minor = evecs[:, 0]
    yaw = math.atan2(minor[1], minor[0])
    yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2         # parallel gripper: yaw ~ yaw + pi
    extent = math.sqrt(12.0) * np.sqrt(np.maximum(evals, 1e-9))   # side lengths of a rectangle
    return dict(x=float(seed[0]), y=float(seed[1]), top=float(np.median(near[:, 2])),
                yaw=yaw, size=(float(extent[1]), float(extent[0])), points=len(near))


def wall_clearance(x, y, yaw, opening):
    """Smallest distance between the open fingers and the bin walls [m]."""
    d = np.array([math.cos(yaw), math.sin(yaw)])
    n = np.array([-d[1], d[0]])
    corners = [np.array([x, y]) + s * (opening / 2 + FINGER_T) * d + t * FINGER_W / 2 * n
               for s in (-1, 1) for t in (-1, 1)]
    cx, cy = BIN_CENTER
    return min(min(BIN_INNER[0] / 2 - abs(c[0] - cx), BIN_INNER[1] / 2 - abs(c[1] - cy)) for c in corners)


def grasp_options(obj):
    """(yaw, opening) candidates, the one with most room to the walls first.
    Long objects: fingers across the narrow side only. Round/square ones: any direction."""
    long_side, short_side = obj["size"]
    if long_side > 1.2 * short_side:
        options = [(obj["yaw"], short_side + 0.025)]
        if long_side + 0.025 <= 0.085:
            options.append((obj["yaw"] + math.pi / 2, long_side + 0.025))
    else:
        options = [(math.radians(a), long_side + 0.025) for a in range(-90, 90, 15)]
    options = [(yaw, min(0.085, opening)) for yaw, opening in options]
    scored = sorted(((wall_clearance(obj["x"], obj["y"], yaw, o), yaw, o) for yaw, o in options), reverse=True)
    return [(yaw, o, c) for c, yaw, o in scored if c > 0.003]


def gazebo_obstacles(robot):
    """Simulation only: the test tube on the stand as an obstacle at its real place."""
    try:
        from gazebo_msgs.srv import GetEntityState
    except ImportError:
        return
    c = robot.create_client(GetEntityState, "/gazebo/get_entity_state")
    if not c.wait_for_service(timeout_sec=1.0):
        return
    f = c.call_async(GetEntityState.Request(name="test_tube", reference_frame="world"))
    rclpy.spin_until_future_complete(robot, f, timeout_sec=2.0)
    if f.result() is None or not f.result().success:
        return
    p = f.result().state.pose.position
    upright = p.z > 0.13
    robot.add_box("test_tube", (0.03, 0.03, 0.11) if upright else (0.12, 0.12, 0.03), (p.x, p.y, p.z))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--look", action="store_true", help="detect and show only, no motion")
    ap.add_argument("--max", type=int, default=10, help="max number of picks")
    a = ap.parse_args()

    rclpy.init()
    eyes = Perception()
    ex = SingleThreadedExecutor()
    ex.add_node(eyes)
    spinner = threading.Thread(target=ex.spin, daemon=True)
    spinner.start()
    robot = SimpleMoveIt("bin_pick", velocity_scale=SPEED)
    log = robot.get_logger()

    # obstacles for MoveIt: the stand and both bins
    robot.add_box("stand", (0.3, 0.4, 0.16), (0.5, 0.0, 0.02))
    for name, (cx, cy) in (("bin", BIN_CENTER), ("out_bin", OUT_BIN_CENTER)):
        robot.add_box(f"{name}_bottom", (0.30, 0.24, 0.01), (cx, cy, -0.055))
        robot.add_box(f"{name}_wall_xn", (0.01, 0.24, 0.10), (cx - 0.145, cy, -0.01))
        robot.add_box(f"{name}_wall_xp", (0.01, 0.24, 0.10), (cx + 0.145, cy, -0.01))
        robot.add_box(f"{name}_wall_yn", (0.28, 0.01, 0.10), (cx, cy - 0.115, -0.01))
        robot.add_box(f"{name}_wall_yp", (0.28, 0.01, 0.10), (cx, cy + 0.115, -0.01))
    gazebo_obstacles(robot)

    picked = 0
    for attempt in range(a.max):
        if not a.look and not robot.named("home"):
            break
        obj = find_top_object(eyes.points_in_world())
        if obj is None:
            log.info("bin is empty")
            break
        eyes.show(obj)
        log.info(f"object at ({obj['x']:.3f}, {obj['y']:.3f}), top z {obj['top']:.3f}, "
                 f"~{obj['size'][0] * 1000:.0f} x {obj['size'][1] * 1000:.0f} mm, "
                 f"fingers close along {math.degrees(obj['yaw']):.0f} deg ({obj['points']} points)")
        if a.look:
            break

        fingertip = max(BIN_FLOOR + 0.008, obj["top"] - FINGER_BELOW_TOP)
        grasp_z = fingertip + TCP_ABOVE_FINGERTIP
        x, y = obj["x"], obj["y"]
        down = False
        for yaw, opening, clearance in grasp_options(obj)[:4]:
            log.info(f"try: fingers along {math.degrees(yaw):.0f} deg, open {opening * 1000:.0f} mm, "
                     f"{clearance * 1000:.0f} mm from the walls")
            robot.gripper(opening)
            # a parallel gripper can also come in turned by 180 deg
            if not (robot.pose(x, y, APPROACH_Z, yaw=yaw) or robot.pose(x, y, APPROACH_Z, yaw=yaw + math.pi)):
                continue
            if robot.line(x, y, grasp_z):
                down = True
                break
        if not down:
            log.error("no way down to this object - stopping")
            break
        robot.gripper(0.0)                              # close until the fingers stop
        time.sleep(0.5)
        if eyes.finger_gap() < 0.008:
            log.warn("missed (fingers closed completely)")
            robot.line(x, y, APPROACH_Z)
            continue
        # show the grasped object in RViz and let MoveIt avoid collisions with it
        held = eyes.finger_gap()
        across = max(obj["size"]) if held < max(obj["size"]) - 0.005 else min(obj["size"])
        height = obj["top"] - BIN_FLOOR - 0.005
        robot.attach_box("held_object", (held, across, height), z=grasp_z - (obj["top"] - height / 2))
        robot.scale = SPEED_HOLDING
        lifted = robot.line(x, y, APPROACH_Z)
        gap = eyes.finger_gap()
        if not lifted:
            log.error("cannot lift - stopping")
            break
        if gap < 0.008:
            log.warn("dropped it while lifting")
            robot.detach("held_object")
            robot.scale = SPEED
            continue
        log.info(f"holding it, fingers at {gap * 1000:.0f} mm")
        ox, oy = DROP[picked % len(DROP)]
        dx, dy = OUT_BIN_CENTER[0] + ox, OUT_BIN_CENTER[1] + oy
        if not robot.pose(dx, dy, DROP_Z):
            log.error("cannot reach the output bin - stopping (still holding the object)")
            break
        robot.scale = SPEED
        if eyes.finger_gap() < 0.008:
            log.warn("lost it on the way")
            robot.detach("held_object")
            continue
        robot.gripper(GRIP_OPEN)
        robot.detach("held_object")
        picked += 1

    if not a.look:
        robot.named("home")
    robot.remove("test_tube")
    robot.detach("held_object")
    log.info(f"picked {picked} object(s)")
    ex.shutdown()
    spinner.join(timeout=2.0)
    robot.destroy_node()
    eyes.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
