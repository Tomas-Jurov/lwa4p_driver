#!/usr/bin/env python3
"""Build isaac/lwa4p_wsg50.urdf: a plain URDF (no xacro, no ROS paths) of the
Schunk LWA 4P + WSG 50-110 + RealSense D435 for Isaac Sim / Isaac Lab.

    source /opt/ros/humble/setup.bash && source install/setup.bash
    python3 isaac/tools/make_isaac_urdf.py

Starts from the Gazebo description (lwa4p_gazebo.urdf.xacro: arm, gripper,
camera) and changes what a physics engine needs to be right:

- masses from SCHUNK data (manual 1344817: arm 12 kg without base, 15 kg with;
  module data: ERB 145 3.9 kg, ERB 115 1.8 kg). The split of the two connecting
  tubes (2.4 kg together) and of the ERB 115 into ball/flange is estimated.
- the robot base as in the manual's drawing (3.1): foot Ø186 mm, shoulder
  (axes 1/2) 205 mm above the mounting surface -> mounting surface at
  z = -0.105 in the URDF root frame. (The ipa320 pedestal mesh is smaller.)
- inertia tensors and centres of mass computed from the meshes: solid bodies
  for the modules (motors, gears), thin shells for the hollow tubes
- joint limits from the datasheets (see LIMITS below): range = preset software
  limit switches (LWA 4P manual 3.2: +-170 deg, axis 3 +-155 deg), velocity
  72 deg/s, effort = max. torque (ERB 145: 64 Nm, ERB 115: 19 Nm; the ipa320
  values 370/176/41.6/20.1 Nm were far off). Max. acceleration (ERB manual 3.1:
  250 / 500 deg/s^2) has no URDF field -> isaac/lwa4p_cfg.py and README.
- gripper: two real prismatic finger joints opening outwards, 0..45.25 mm each
  (WSG 50 manual: 55 mm stroke per jaw = jaw width 0..110 mm, minus the measured
  19.5 mm finger offset), 80 N, 210 mm/s and 2.5 m/s^2 per finger.
- gripper and camera masses / inertias recomputed from their geometry: housing
  = 1.15 kg WSG minus its two aluminium base jaws, adapter, jaws and fingers
  aluminium (2700 kg/m^3; the fingers are not measured yet), D435 72 g.
- ROS-only tags (ros2_control, gazebo) removed, mesh paths -> meshes/*.stl
"""
import math
import os
import shutil
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
from ament_index_python.packages import get_package_share_directory

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.dirname(HERE)
OUT = os.path.join(OUT_DIR, "lwa4p_wsg50.urdf")
MESH_DIR = os.path.join(OUT_DIR, "meshes")

# link -> (mass kg, model): "solid" = uniform solid, "shell" = thin wall
ARM = {
    "arm_podest_link": (3.0, "solid"),   # robot base: 15 kg - 12 kg
    "arm_1_link": (3.9, "solid"),        # ERB 145 (axes 1-2)
    "arm_2_link": (1.4, "shell"),        # VBE F145-145-P upper arm tube (est.)
    "arm_3_link": (3.9, "solid"),        # ERB 145 (axes 3-4)
    "arm_4_link": (1.0, "shell"),        # VBE F145-115-W90 forearm (est.)
    "arm_5_link": (1.5, "solid"),        # ERB 115 (axes 5-6) without flange (est.)
    "arm_6_link": (0.3, "solid"),        # ERB 115 output / tool flange (est.)
}
# joint: (range +-deg, max. torque Nm, rated torque Nm, max. accel deg/s^2, module)
LIMITS = {1: (170, 64.0, 35.0, 250, "ERB 145"), 2: (170, 64.0, 35.0, 250, "ERB 145"),
          3: (155, 64.0, 35.0, 250, "ERB 145"), 4: (170, 64.0, 35.0, 250, "ERB 145"),
          5: (170, 19.0, 7.0, 500, "ERB 115"), 6: (170, 19.0, 7.0, 500, "ERB 115")}
VELOCITY = math.radians(72.0)                                      # rad/s, all axes
ALU = 2700.0                                                       # kg/m^3
WSG_MASS = 1.15                                                    # kg incl. base jaws (WSG manual)
MOUNT_Z = -0.105                     # mounting surface in the root frame (0.10 - 0.205)
BASE_TOP = 0.10 - 0.072              # bottom of the first ERB 145 ball
FINGER_TRAVEL = (0.110 - 0.0195) / 2  # m per finger: jaw width 0..110 mm - finger offset
FINGER_VEL = 0.21                    # m/s per finger (420 mm/s finger to finger)
FINGER_FORCE = 80.0                  # N (120 N only in overdrive mode)


def load_stl(path):
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3, 3).astype(np.float64)


def mass_props(tris, mass, model):
    """COM and inertia tensor about the COM (link frame) for a given total mass."""
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    S = a + b + c
    if model == "solid":            # signed tetrahedra to the origin
        w = np.einsum("ij,ij->i", a, np.cross(b, c)) / 6.0
        com = (w[:, None] * S / 4.0).sum(0) / w.sum()
        P = np.einsum("ni,nj->nij", a, a) + np.einsum("ni,nj->nij", b, b) + np.einsum("ni,nj->nij", c, c)
        C = (w[:, None, None] / 20.0 * (P + np.einsum("ni,nj->nij", S, S))).sum(0)
    else:                           # thin shell: mass spread over the surface
        w = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
        com = (w[:, None] * S / 3.0).sum(0) / w.sum()
        P = np.einsum("ni,nj->nij", a, a) + np.einsum("ni,nj->nij", b, b) + np.einsum("ni,nj->nij", c, c)
        C = (w[:, None, None] / 12.0 * (P + np.einsum("ni,nj->nij", S, S))).sum(0)
    C = C / w.sum() * mass - mass * np.outer(com, com)
    inertia = np.trace(C) * np.eye(3) - C
    return com, inertia


def primitives(link):
    """[(kind, dims, centre)] of a link's collision boxes/cylinders (no rotations used here)."""
    out = []
    for c in link.findall("collision"):
        o = c.find("origin")
        xyz = np.array([float(v) for v in o.get("xyz", "0 0 0").split()]) if o is not None else np.zeros(3)
        g = c.find("geometry")
        if g.find("box") is not None:
            out.append(("box", [float(v) for v in g.find("box").get("size").split()], xyz))
        elif g.find("cylinder") is not None:
            out.append(("cyl", [float(g.find("cylinder").get("radius")), float(g.find("cylinder").get("length"))], xyz))
    return out


def volume(kind, dims):
    return dims[0] * dims[1] * dims[2] if kind == "box" else math.pi * dims[0] ** 2 * dims[1]


def combine(parts):
    """[(mass, kind, dims, centre)] -> total mass, COM, inertia about the COM."""
    M = sum(p[0] for p in parts)
    com = sum(p[0] * p[3] for p in parts) / M
    I = np.zeros((3, 3))
    for m, kind, dims, c in parts:
        if kind == "box":
            x, y, z = dims
            Ic = np.diag([m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12])
        else:
            r, h = dims
            Ic = np.diag([m * (3 * r * r + h * h) / 12] * 2 + [m * r * r / 2])
        d = c - com
        I += Ic + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))     # parallel axis
    return M, com, I


def set_inertial(link, mass, com, inertia):
    for old in link.findall("inertial"):
        link.remove(old)
    el = ET.SubElement(link, "inertial")
    ET.SubElement(el, "origin", xyz=" ".join(f"{v:.5f}" for v in com), rpy="0 0 0")
    ET.SubElement(el, "mass", value=f"{mass:.4f}")
    I = inertia
    ET.SubElement(el, "inertia", ixx=f"{I[0,0]:.6g}", ixy=f"{I[0,1]:.6g}", ixz=f"{I[0,2]:.6g}",
                  iyy=f"{I[1,1]:.6g}", iyz=f"{I[1,2]:.6g}", izz=f"{I[2,2]:.6g}")


def box_inertia(m, x, y, z):
    return np.diag([m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12])


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def main():
    sim = get_package_share_directory("lwa4p_gazebo")
    desc = get_package_share_directory("lwa4p_description")
    xml = subprocess.run(["xacro", os.path.join(sim, "urdf", "lwa4p_gazebo.urdf.xacro")],
                         check=True, capture_output=True, text=True).stdout
    root = ET.fromstring(xml)
    root.set("name", "lwa4p_wsg50")
    for tag in ("ros2_control", "gazebo"):
        for el in root.findall(tag):
            root.remove(el)
    links = {l.get("name"): l for l in root.findall("link")}
    joints = {j.get("name"): j for j in root.findall("joint")}

    # meshes -> meshes/*.stl next to the URDF
    os.makedirs(MESH_DIR, exist_ok=True)
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename")
        name = os.path.basename(fn)
        shutil.copy(os.path.join(desc, "meshes", "lwa4p", name), os.path.join(MESH_DIR, name))
        mesh.set("filename", f"meshes/{name}")

    # arm: masses, COM and inertia from the meshes
    print(f"{'link':22s} {'mass':>6s}  com [m]                      Ixx/Iyy/Izz [kg m^2]")
    for name, (mass, model) in ARM.items():
        if name == "arm_podest_link":
            continue                # replaced by the real base geometry below
        link = links[name]
        vis = link.find("visual")
        tris = load_stl(os.path.join(MESH_DIR, os.path.basename(vis.find("geometry/mesh").get("filename"))))
        o = vis.find("origin")
        if o is not None:           # mesh placed with an origin (the base is flipped)
            R = rpy_matrix(*[float(v) for v in o.get("rpy", "0 0 0").split()])
            t = np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
            tris = tris @ R.T + t
        com, inertia = mass_props(tris, mass, model)
        set_inertial(link, mass, com, inertia)
        print(f"{name:22s} {mass:6.2f}  {np.round(com, 4)!s:28s} {np.round(np.diag(inertia), 5)}")

    # joint limits from the SCHUNK data
    print(f"\n{'joint':12s} {'range':>10s} {'velocity':>12s} {'max torque':>11s} {'max accel':>13s}")
    for i in range(1, 7):
        rng, effort, _, accel, module = LIMITS[i]
        lim = joints[f"arm_{i}_joint"].find("limit")
        lim.set("lower", f"{-math.radians(rng):.5f}")
        lim.set("upper", f"{math.radians(rng):.5f}")
        lim.set("effort", f"{effort}")
        lim.set("velocity", f"{VELOCITY:.5f}")
        print(f"arm_{i}_joint  +-{rng:3d} deg  {VELOCITY:.4f} rad/s  {effort:6.1f} Nm  {math.radians(accel):6.2f} rad/s2  ({module})")

    # gripper: drop the MoveIt helper joint, two real finger joints
    root.remove(joints["gripper_joint"])
    root.remove(links["gripper_gap_link"])
    for side, sign in (("left", -1), ("right", 1)):
        j = joints[f"gripper_finger_{side}_joint"]
        for m in j.findall("mimic"):
            j.remove(m)
        j.find("axis").set("xyz", f"{sign} 0 0")          # positive = opening
        lim = j.find("limit")
        lim.set("lower", "0.0")
        lim.set("upper", f"{FINGER_TRAVEL:.5f}")
        lim.set("effort", f"{FINGER_FORCE}")
        lim.set("velocity", f"{FINGER_VEL}")
    print(f"gripper fingers 0..{FINGER_TRAVEL * 1000:.2f} mm each (gap 0..{2 * FINGER_TRAVEL * 1000:.1f} mm), "
          f"{FINGER_FORCE:.0f} N, {FINGER_VEL} m/s")

    # gripper + camera: mass and inertia from the geometry
    jaw = primitives(links["gripper_finger_left_link"])[0]
    jaw_mass = volume(jaw[0], jaw[1]) * ALU
    mass_of = {
        "gripper_adapter_link": lambda k, d, i: volume(k, d) * ALU,
        "gripper_body_link": lambda k, d, i: WSG_MASS - 2 * jaw_mass,
        "gripper_finger_left_link": lambda k, d, i: volume(k, d) * ALU,     # [0] jaw, [1] finger
        "gripper_finger_right_link": lambda k, d, i: volume(k, d) * ALU,
        "camera_link": lambda k, d, i: 0.072,
    }
    print(f"\n{'link':28s} {'mass':>7s}  com [m]                  principal inertia [kg m^2]")
    for name, mass_fn in mass_of.items():
        parts = [(mass_fn(k, d, i), k, d, c) for i, (k, d, c) in enumerate(primitives(links[name]))]
        M, com, I = combine(parts)
        set_inertial(links[name], M, com, I)
        print(f"{name:28s} {M:7.4f}  {np.round(com, 4)!s:24s} {np.round(np.linalg.eigvalsh(I), 7)}")

    # robot base: foot plate Ø186 x 12 mm + column Ø130 up to the first module
    base = links["arm_podest_link"]
    for tag in ("visual", "collision"):
        for el in base.findall(tag):
            base.remove(el)
    column_h = BASE_TOP - (MOUNT_Z + 0.012)
    parts = [(0.093, 0.012, MOUNT_Z + 0.006), (0.065, column_h, MOUNT_Z + 0.012 + column_h / 2)]
    for tag in ("visual", "collision"):
        for r, h, zc in parts:
            el = ET.SubElement(base, tag)
            ET.SubElement(el, "origin", xyz=f"0 0 {zc:.4f}", rpy="0 0 0")
            g = ET.SubElement(el, "geometry")
            ET.SubElement(g, "cylinder", radius=f"{r}", length=f"{h:.4f}")
            if tag == "visual":
                ET.SubElement(el, "material", name="schunk_grey")
    m_foot, m_col = 1.0, 2.0
    (r1, h1, z1), (r2, h2, z2) = parts
    zc = (m_foot * z1 + m_col * z2) / 3.0
    I = np.zeros(3)
    for m, r, h, z in ((m_foot, r1, h1, z1), (m_col, r2, h2, z2)):
        I += np.array([m * (3 * r * r + h * h) / 12 + m * (z - zc) ** 2] * 2 + [m * r * r / 2])
    set_inertial(base, 3.0, np.array([0, 0, zc]), np.diag(I))

    # tiny helper links Isaac would otherwise warn about
    for name in ("arm_base_link",):
        set_inertial(links[name], 0.01, np.zeros(3), np.eye(3) * 1e-6)

    ET.indent(root, space="  ")
    header = ("<?xml version=\"1.0\"?>\n<!-- GENERATED by isaac/tools/make_isaac_urdf.py - do not edit by hand.\n"
              "     Schunk LWA 4P + WSG 50-110 + RealSense D435 for Isaac Sim. See isaac/README.md -->\n")
    open(OUT, "w").write(header + ET.tostring(root, encoding="unicode") + "\n")
    total = sum(float(l.find("inertial/mass").get("value")) for l in root.findall("link") if l.find("inertial") is not None)
    arm = sum(m for n, (m, _) in ARM.items() if n != "arm_podest_link")
    print(f"\nwrote {OUT}\narm without base {arm:.2f} kg (SCHUNK: 12), everything {total:.2f} kg")


if __name__ == "__main__":
    sys.exit(main())
