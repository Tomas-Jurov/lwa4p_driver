# 🤖 LWA 4P + WSG 50 for Isaac Sim / Isaac Lab

This folder holds everything you need to simulate this robot in Isaac Sim and train policies (VLA, RL, imitation learning) that can later run on the **real** arm.

| File | What it is |
|---|---|
| [`lwa4p_wsg50.urdf`](lwa4p_wsg50.urdf) | Plain URDF (no xacro, no ROS paths): arm, WSG 50 gripper with two finger joints, RealSense D435 frame |
| [`meshes/`](meshes) | STL meshes, referenced relatively by the URDF |
| [`lwa4p_cfg.py`](lwa4p_cfg.py) | Isaac Lab `ArticulationCfg` template: actuators, limits, home/ready poses |
| [`tools/make_isaac_urdf.py`](tools/make_isaac_urdf.py) | Regenerates the URDF from the ROS description; documents every number |

> [!NOTE]
> The URDF was checked with `check_urdf` and loaded in MuJoCo:
> - all inertias are physically valid;
> - the gravity torques match the SCHUNK payload spec;
> - the arm stays stable when held under gravity.
>
> It was **not** opened in Isaac Sim, because the machine that produced it has no NVIDIA GPU. `lwa4p_cfg.py` is a template for Isaac Lab 2.x. Check its argument names against your version.

---

## 1. Import

**Isaac Sim GUI:** *File → Import → URDF*, and choose `lwa4p_wsg50.urdf`. Recommended settings:
- **Fix Base Link:** on.
- **Merge Fixed Joints:** off. This keeps `gripper_tcp` and the camera frames.
- **Joint drive:** position.
- **Collision:** convex decomposition, because the arm meshes are not convex.

**Isaac Lab:**
```python
from lwa4p_cfg import LWA4P_REAL_SPEED_CFG
robot_cfg = LWA4P_REAL_SPEED_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
```

---

## 2. The robot

### Kinematics and frames

| Frame | Meaning |
|---|---|
| `world` | URDF root = robot base frame, 0.10 m below the shoulder axes. The **mounting surface** (bottom of the base) is at **z = −0.105 m** |
| `arm_tool0` | tool flange |
| `gripper_tcp` | **tool centre point**: between the fingertips, 15 mm above the tips. All grasp poses use this frame |
| `camera_link` / `camera_color_optical_frame` | RealSense, see §3 |

The kinematics come from ipa320/schunk_modular_robotics. I checked them against SCHUNK's dimension drawing (manual §3.1), measured from the mounting surface:

| | SCHUNK drawing | This URDF |
|---|---|---|
| Axes 1/2 (shoulder) | 205 mm | 0.205 ✓ (the root is 0.105 above the mounting surface) |
| Axes 3/4 (elbow) | 555 mm → upper arm 350 mm | 0.350 ✓ |
| Axes 5/6 (wrist) | 860 mm → forearm 305 mm | 0.305 ✓ |
| Flange top | 945 mm → 85 mm above the wrist | 0.084 ✓ |
| Tool flange | Ø 65 mm | adapter Ø 65 ✓ |
| Base | foot Ø 186 mm | foot Ø 186, column Ø 130 (simplified) |

The kinematics have not been calibrated to this specific arm.

### Joints (all values from the datasheets)

| Joint | Module | Range | Max. torque (rated) | Max. speed | Max. accel. |
|---|---|---|---|---|---|
| `arm_1_joint` | ERB 145 | ±170° | 64 Nm (35) | 72 °/s = 1.257 rad/s | 250 °/s² = 4.36 rad/s² |
| `arm_2_joint` | ERB 145 | ±170° | 64 Nm (35) | 72 °/s | 250 °/s² |
| `arm_3_joint` | ERB 145 | **±155°** | 64 Nm (35) | 72 °/s | 250 °/s² |
| `arm_4_joint` | ERB 145 | ±170° | 64 Nm (35) | 72 °/s | 250 °/s² |
| `arm_5_joint` | ERB 115 | ±170° | 19 Nm (7) | 72 °/s | 500 °/s² = 8.73 rad/s² |
| `arm_6_joint` | ERB 115 | ±170° | 19 Nm (7) | 72 °/s | 500 °/s² |
| `gripper_finger_left_joint` | WSG 50 | 0 … 45.25 mm (opening) | 80 N | 210 mm/s | 2.5 m/s² |
| `gripper_finger_right_joint` | WSG 50 | 0 … 45.25 mm (opening) | 80 N | 210 mm/s | 2.5 m/s² |

Sources:
- **Range:** the LWA 4P manual's preset software limit switches (axis 3 is limited to ±155° by the arm geometry).
- **Torques and speeds:** ERB manual §3.1 and the SCHUNK slides. The gear ratios are 160 (ERB 145) and 100 (ERB 115).
- **Gripper:** WSG 50 manual. Each finger moves half of the 420 mm/s and 5000 mm/s² finger-to-finger values; overdrive mode reaches 120 N.
- **URDF fields:** URDF has no acceleration field, so the accelerations are in `lwa4p_cfg.py`. The URDF uses the datasheet ranges exactly.
- **Real robot:** the ROS/MoveIt description keeps 0.02 rad inside them, and the real driver rejects targets within 3° of the drive limits. Train within about ±165° to be safe.

> [!IMPORTANT]
> The real arm currently runs on a **weak power supply**. It is limited to about **0.5 rad/s and 0.4 rad/s²**, because faster or harder moves cause undervoltage faults. For policies that will run on the real robot, use `LWA4P_REAL_SPEED_CFG` and keep accelerations moderate.

### Masses (SCHUNK data)

| Link | Real part | Mass | Source |
|---|---|---|---|
| `arm_podest_link` | robot base | 3.0 kg | manual: 15 kg with base − 12 kg without |
| `arm_1_link` | ERB 145 double-axis module (axes 1–2) | 3.9 kg | SCHUNK module data |
| `arm_2_link` | connecting tube VBE F145-145-P (upper arm, 350 mm) | 1.4 kg | estimated share of the remaining 2.4 kg |
| `arm_3_link` | ERB 145 double-axis module (axes 3–4) | 3.9 kg | SCHUNK module data |
| `arm_4_link` | connecting element VBE F145-115-W90 (forearm) | 1.0 kg | estimated share of the remaining 2.4 kg |
| `arm_5_link` | ERB 115 double-axis module (axes 5–6) | 1.5 kg | 1.8 kg module − flange, estimated |
| `arm_6_link` | ERB 115 output / tool flange | 0.3 kg | estimated |
| **Arm total** | | **12.0 kg** | SCHUNK manual: 12 kg without base |
| `gripper_body_link` | WSG 50 housing (1.15 kg WSG − 2 base jaws) | 1.08 kg | WSG manual: 1.15 kg |
| `gripper_finger_*_link` | base jaw (35 g) + finger (106 g), aluminium by volume | 2 × 0.141 kg | jaw: WSG drawing; finger: **not measured** |
| `gripper_adapter_link` | adapter plate Ø65 × 20 mm, aluminium | 0.179 kg | **not measured** |
| `camera_link` | RealSense D435 | 0.072 kg | Intel |

- **Gripper and camera inertias** are computed from their box and cylinder geometry, using the masses above.
- **Arm centres of mass and inertia tensors** are computed from the meshes. The modules are treated as solid bodies (they're full of motors and gears) and the tubes as thin shells (they're hollow aluminium).
- **Check against the datasheet:** stretched out horizontally with the gripper, joint 2 needs 44 Nm for the arm alone, and 70 Nm with 3 kg in the gripper, which is above the 64 Nm maximum. That fits SCHUNK's "3–6 kg depending on deflection", which is rated at the flange without a gripper.

### Gripper

The real WSG 50 reports and accepts the **fingertip gap** in metres (0 … 0.0895). In this URDF:
```
gap = gripper_finger_left_joint + gripper_finger_right_joint
command both fingers with gap / 2
```
- **Grasping on the real gripper:** close to a target smaller than the object, with a force limit (5–80 N, default 35 N). It stops at the object and holds with that force.
- **In simulation,** use a stiff position drive with an effort limit (80 N), and command a gap slightly smaller than the object.
- **Grasping speed:** grasps on the real WSG use 80 mm/s, because slower ones report false "blocked".
**Gripper geometry,** from the WSG 50 manual (05.00, §3.1 *Outer dimensions*) and measurements:

| | Value | Source |
|---|---|---|
| Housing | 146 × 50 × 72.5 mm (x = jaw direction), Ø 50 ISO mount in the centre of the bottom face | WSG manual |
| Base jaws | stick out to 96.5 mm above the mounting face (24 mm above the housing), ~18 mm wide in x | WSG manual |
| Jaw stroke | jaw inner faces 0 … 110 mm apart (55 mm per jaw), centred | WSG manual |
| Fingers | reach 9.75 mm further inwards than each jaw → **fingertip gap = jaw width − 19.5 mm**, 0 … 90.5 mm (the real gripper reports up to ~109 mm jaw width after homing, so ~89.5 mm gap) | measured on the real gripper |
| Adapter (flange → housing) | 20 mm | **not measured yet** |
| Base jaw depth (y) | 30 mm | **not measured yet** |
| Finger size | 20 thick × 30 wide × 65.5 mm above the jaw; tips 162 mm above the housing bottom | **not measured yet** |

`gripper_tcp` is 0.147 m above the housing bottom, which is 0.251 m from the tool flange with the 20 mm adapter. The values marked *not measured yet* are set in `src/lwa4p_description/urdf/wsg50.urdf.xacro`. Measure them on the real gripper, change them there, and regenerate (§7).

---

## 3. Camera: RealSense D435

| | Value |
|---|---|
| Mount pose (in the URDF) | `world → camera_link`: xyz `0.45 -0.15 0.95`, rpy `0 π/2 0`, looking straight down |
| Image frame | `camera_color_optical_frame` (ROS optical: z forward, x right, y down) |
| Field of view | 69.4° × 42.8° (real D435 colour sensor) |
| Recommended resolution | 424 × 240 (also a real D435 mode). Intrinsics: fx = fy = 306.2, cx = 212, cy = 120 |
| Other resolutions | 640 × 360: f = 462.1 · 848 × 480: f = 612.3 (cx, cy = image centre) |

- **The mount pose is a placeholder** until the real camera is installed and hand–eye calibrated. Then replace the numbers in the URDF, and use the real intrinsics from `/camera/color/camera_info`.
- **Randomise the camera pose** in training (±2 cm, ±2°) so the policy tolerates calibration errors.
- **Isaac camera convention:** an Isaac/USD camera looks along its −Z axis with +Y up, while the ROS optical frame looks along +Z with +Y down. That's a 180° rotation about X. Isaac Lab's `CameraCfg` with `convention="ros"` handles this.
- **A wrist camera** is common for VLAs. If you add one, tell us where it goes so the real arm gets the same mount.

---

## 4. Reference scene

Matches the Gazebo world (`src/lwa4p_gazebo/worlds/lwa4p_table.world`), in the robot frame. In that world the floor is at z = −0.06 m, which is 4.5 cm above the real mounting surface (−0.105), because the Gazebo model still uses the smaller ipa320 pedestal. For a realistic Isaac scene, put the robot and the objects on the same table at z = −0.105, and lower all object heights below by 0.045 m. Expect slightly less reach at table level.

| Object | Position (x, y, z) [m] | Size [m] | Mass |
|---|---|---|---|
| Stand (table) | 0.50, 0, 0.02 (centre) | box 0.30 × 0.40 × 0.16 (top at z = 0.10) | static |
| Test tube | 0.45, 0, 0.15 | cylinder r 0.0075, h 0.10 | 50 g |
| Red cube | 0.45, 0.12, 0.115 | box 0.03 | 50 g |
| Pick bin | 0.24, −0.33, floor | outer 0.30 × 0.24 × 0.10, walls 1 cm | static |
| Output bin | 0.24, +0.33, floor | same | static |
| Red box (in bin) | 0.18, −0.28 | 0.04³, yaw 0.3 rad | 50 g |
| Green box | 0.30, −0.39 | 0.035 × 0.035 × 0.05, yaw −0.4 | 50 g |
| Blue cylinder | 0.29, −0.27 | r 0.018, h 0.05 | 50 g |
| Yellow box | 0.18, −0.39 | 0.05 × 0.03 × 0.03, yaw 0.5 | 50 g |

**Reach:** with the gripper pointing down, the arm reaches about **0.55 m** from its base at table height. Keep task objects within that.

---

## 5. Contract with the real robot (read this before training)

A policy is only deployable if it uses inputs and outputs the real robot has. On the real arm everything runs in ROS 2 Humble:

| | Real robot interface | In simulation |
|---|---|---|
| Joint positions | `/joint_states`: `arm_1_joint` … `arm_6_joint` (rad), `gripper_joint` = fingertip gap (m) | the joint positions; gap = left + right finger |
| Arm command | action `/arm_controller/follow_joint_trajectory` (joint positions over time) | joint position targets |
| Gripper command | action `/gripper_controller/gripper_cmd` (`GripperCommand`: position = gap in m, max_effort = N) | finger targets = gap / 2 |
| RGB | `/camera/color/image_raw` | Isaac camera at the D435 pose |
| Depth / point cloud | `/camera/depth/image_rect_raw`, `/camera/depth/color/points` | depth camera |

**Recommended choices:**
- **Action:** **absolute or delta joint positions** for the 6 arm joints, plus a **gripper command** (binary open/close, or a target gap). Joint-space actions transfer most directly. End-effector deltas also work, but then the real side needs IK.
- **Rate:** **5–10 Hz** policy output. The real driver tracks streamed joint targets at 20 Hz in Profile Position mode, where the drives smooth the motion themselves.
- **Observations:** RGB (424 × 240 or resized), 6 joint positions, gripper gap, and the language instruction.
- **Episode recording:** LeRobot dataset format is the easiest path to SmolVLA, π0 and OpenVLA fine-tuning. Real-robot demonstrations can be recorded in the same format later.

**Sim-to-real checklist:**
- **Domain randomisation:** lighting, textures, camera pose, object poses, masses (±30%) and friction (0.5–1.5).
- **Joint behaviour:** add delay (50–100 ms) and noise to the joint observations, and limit joint speed and acceleration to the real values above.
- **Gripper:** close at 80 mm/s with a 35 N force limit.
- **Fine-tuning:** expect to fine-tune on a few dozen real demonstrations.

---

## 6. Test policies in ROS before the real robot

The repository also has a **Gazebo simulation** with exactly the same ROS interfaces as the real arm (`src/lwa4p_gazebo`, see the main README). A policy-to-ROS bridge can be tested there end to end before it ever moves the real arm.

## 7. Regenerate

After changing the robot description (for example a measured camera pose or real finger geometry):
```bash
cd lwa4p_ws && source /opt/ros/humble/setup.bash && colcon build && source install/setup.bash
python3 isaac/tools/make_isaac_urdf.py
```

## Sources

- SCHUNK *Powerball Lightweight Arm LWA 4P – Assembly and Operating Manual* (1344817), §3.1–3.2: dimension drawing (205/555/860/945 mm, Ø186 base, Ø65 flange), 12/15 kg, axis/module layout, ±0.15 mm, software limits
- SCHUNK *LWA 6DOF PowerBall* slides (C. Parlitz): ERB 115 / ERB 145 module weights 1.8 / 3.9 kg, torques 19 / 64 Nm max, 72 °/s
- WSG 50 *Assembly and operating manual* (05.00, 389474), §3.1–3.2: outer dimensions, base jaws, 1.15 kg, 55 mm per jaw, 5–80 N, 5–420 mm/s
- SCHUNK WSG 050-110-B product data: 146 × 50 × 72.5 mm, 1.2 kg
- ipa320/schunk_modular_robotics: kinematics and meshes (LGPL-3.0)
