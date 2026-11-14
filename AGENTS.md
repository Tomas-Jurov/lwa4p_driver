# AGENTS.md — working notes for the LWA 4P manipulation stack

Notes for agents and developers working on this repository. The READMEs explain how to
*use* the stack. This file covers how it is wired, where the traps are, and what to keep
in sync when you change something. Read it before editing.

- User docs: [README.md](README.md), [docker/README.md](docker/README.md),
  [isaac/README.md](isaac/README.md), [isaac/moveit/README.md](isaac/moveit/README.md),
  [src/lwa4p_mtc_verification/README.md](src/lwa4p_mtc_verification/README.md).

---

## 1. What this is

- The manipulation subsystem of a RoboCup (FEI) medical-assistance robot: a **Schunk LWA 4P**
  6-DoF arm, a **Weiss/Schunk WSG 50-110** parallel gripper, and a **RealSense D435** looking
  down at the work area.
- **ROS 2 Jazzy** on Ubuntu 24.04, with MoveIt 2 and MoveIt Task Constructor (MTC). The migration
  to Jazzy is in progress: read "Migration to Jazzy" below before building or running anything.
- The repo is forked from `Tomas-Jurov/lwa4p_driver`. Git origin is
  `github.com/RoboCup-FEI/manipulation-Simulation`, and the working branch is `main`.
- The repo root **is** the colcon workspace (`src/` lives here). Build from this directory.
  `/heavy_stuff/robocup_ws` around it is not a git repo.
- There is **no mobile base, navigation or task-level code** here yet. The `world` frame is
  the arm base, everything is a fixed-base table setup, and the "medical" content so far is a
  15 mm test tube on a stand plus a bin-picking scene. The MTC package says it is "not the
  manipulation skill server", so a skill server is expected but does not exist yet.

### Team decisions (2026-10-26)

- **The real robot keeps the Python CAN driver (`lwa4p_driver`) as it is.** Simulation keeps
  `ros2_control`. The team accepts the sim-to-real gap and will not write a `ros2_control`
  hardware interface or move to `ros2_canopen`. Do not propose that rewrite.
- The interface shared by sim and real is the **action level**: `arm_controller/follow_joint_trajectory`,
  `gripper_controller/*` and `/joint_states`. Code above it (MoveIt, MTC, a skill server, the
  behaviour tree) must not depend on which backend is running.
- Navigation and VLM perception belong to other teams. This repo consumes their outputs
  (poses, TF) and does not implement them.
- **BehaviorTree.CPP belongs to the AI team.** This team provides the ROS 2 actions it calls.
  Failures from actions must propagate back to the BT, and this team defines the shared
  failure enumeration.
- **The project is migrating to ROS 2 Jazzy.** All new work targets Jazzy.
- **Gazebo is abandoned in favour of Isaac Sim**, which is the only physics simulator. Mock
  hardware covers everything that needs no physics. Do not port or extend Gazebo code (see §7).
- The planned stack is the diagram in [manipulation_architecture.txt](manipulation_architecture.txt).
  The proposed implementation is in
  [manipulation_implementation_proposal.md](manipulation_implementation_proposal.md). See **§13**.

### Migration to Jazzy (in progress)

The repo has not been migrated yet. `docker/Dockerfile`, the launchers in `isaac/moveit/`, the
READMEs, the Docker image tag and some source comments still name the previous ROS distribution.
**Treat those references as out of date:** do not copy them into new work, and do not follow
their instructions as if they were current. Sections below describe the code as it stands where
they name specific packages, and everything must be rechecked on Jazzy.

Checked against the Jazzy package index on 2026-11-01:

| Area | State on Jazzy | What it means |
|---|---|---|
| MoveIt, MTC | Binaries exist (MoveIt 2.12.4, MTC 0.1.8) | APIs may differ from what this repo was written against. Rebuild `lwa4p_mtc_verification` and retest `simple_moveit.py`. |
| Gazebo | Abandoned | Remove `lwa4p_gazebo` (§7). |
| Controllers | `joint_trajectory_controller`, `gripper_controllers`, `forward_command_controller` exist | Recheck their YAML against the Jazzy versions. |
| `warehouse_ros_mongo` | No binary | Listed in `lwa4p_moveit_config/package.xml`, unused. Remove it or skip it in `rosdep`. |
| `pick_ik`, TRAC-IK plugin, `behaviortree_cpp` | Binaries exist | |
| `behaviortree_ros2` | No binary | The AI team must build it from source if they use it. |
| Isaac Sim | NVIDIA recommends Ubuntu 24.04 with Jazzy. `isaac/moveit/isaac.sh` accepts `jazzy`. | Not tested yet. `run.sh` hard-codes the old image tag and distribution, and `simulation.py` needs checking. Controller plugins come from `ros-jazzy-ros2-controllers`. |
| Python | 3.12 | Recheck the `canopen` and `python-can` pins in `lwa4p_driver`. |

The proposal targets Jazzy and Isaac Sim, but its measured results come from the previous stack.
Repeat them before relying on them. Its Gazebo physics results do not carry over at all.

### The host machine (checked 2026-11-01)

- Ubuntu **24.04** with ROS 2 **Jazzy** in `/opt/ros/jazzy` and Python 3.12, matching the
  project's target. The `.vscode` settings point at Jazzy too.
- Installed: `ros2_control`, `joint_trajectory_controller` and `gripper_controllers`.
  **Not installed:** MoveIt, MTC, `pick_ik`, `behaviortree_cpp`. Installing them
  needs `sudo`, so ask first.
- The `lwa4p` Docker image on this host was built from the unmigrated Dockerfile. Do not base new
  work on it.
- The GPU is a GTX 1650 Ti with 4 GB. No Isaac Sim install was found at
  `../isaac-setup/runtime/isaac-sim-6.2.0` or `~/isaacsim`. This GPU has no RTX hardware, so
  **Isaac cannot run here**: use mock hardware on this host, and an RTX machine for physics.

---

## 2. Packages

| Package | Type | Role |
|---|---|---|
| `lwa4p_description` | ament_cmake (data only) | Xacro for the arm, the FTS-LWA force/torque sensor (`fts_lwa.urdf.xacro`), the WSG 50 (`wsg50.urdf.xacro`) and the D435 (`realsense_d435.urdf.xacro`), plus meshes and `display.launch.py`. The sensor and gripper meshes come from Patrik Bakyta's 2018 STU FEI diploma project, used with his permission. The gripper meshes there are the Weiss Robotics WSG 50 ROS package meshes (`nalt/wsg50-ros-pkg`, standard FMF fingers); the sensor mesh source is unknown. The root MIT license does not cover them, and the arm meshes and kinematics come from ipa320/schunk_modular_robotics. The base xacro has **no camera**; the camera is added by `lwa4p_gazebo`. |
| `lwa4p_driver` | ament_python | Real hardware. `driver` = CANopen CiA 402 arm driver (`driver_node.py` + `ds402.py`). `gripper` = WSG 50 node (`gripper_node.py` + `wsg50.py`, a raw binary protocol over CAN). CLIs `test_move` and `gripper_cmd`. Launch file: `bringup.launch.py`. |
| `lwa4p_moveit_config` | ament_cmake | Setup Assistant output: SRDF, kinematics, joint limits, controllers. `demo.launch.py` (mock hardware), `real.launch.py` (driver + move_group + RViz). |
| `lwa4p_gazebo` | ament_cmake + scripts | **To be removed (§7).** Gazebo world, `gazebo.launch.py`, sim controllers, and the Python user API `scripts/simple_moveit.py` plus the demos `go_to.py`, `pick_tube.py`, `bin_pick.py` and the sim helper `finger_mimic.py`. |
| `lwa4p_mtc_verification` | ament_cmake C++17 | `verify_mtc` node: the `/lwa4p/test_ptp` Trigger service runs a 4-stage MTC task and checks the *measured* joint motion. Used for Isaac only. Follow its `.clang-format` (Google-based, 100 columns, Allman braces for functions, classes and namespaces). |
| `lwa4p_calibration_poses` | ament_cmake C++17 | `pose_teacher` and `calibration_sequence` nodes: teach joint poses for hand-eye calibration and replay them. They send trajectories directly to the driver, **without MoveIt or collision checking**. Humble form of `create_service`; on Jazzy use `rclcpp::ServicesQoS()`. |
| `lwa4p_calibration_interfaces` | ament_cmake | The `SendToPose` service definition. |
| `isaac/` (not a ROS package) | scripts | Plain URDF `lwa4p_wsg50.urdf` with datasheet masses, its generator `tools/make_isaac_urdf.py`, the Isaac Lab template `lwa4p_cfg.py`, and the Isaac Sim 6.2 + MoveIt launcher `isaac/moveit/`. |

The dependency chain is `description ← moveit_config`. `moveit_config` also depends
on `lwa4p_driver` at exec time, because `real.launch.py` includes `bringup.launch.py`.

---

## 3. Three backends, one MoveIt interface, and where they differ

All backends expose `arm_controller` and `gripper_controller` and publish `/joint_states`, so
MoveIt client code is meant to be backend-agnostic. **The gripper is modelled differently in
each backend.** Getting this wrong is the most likely bug.

| | Real robot | Isaac Sim 6.2 | Mock (`demo.launch.py`) |
|---|---|---|---|
| Arm action | `/arm_controller/follow_joint_trajectory`, served by the Python driver | JTC in Isaac's in-process `controller_manager` | JTC, mock_components |
| Gripper joint(s) MoveIt commands | `gripper_joint` = **fingertip gap** (m) | **`gripper_finger_left_joint` + `gripper_finger_right_joint`**, each = gap/2. There is **no `gripper_joint`**. | `gripper_joint` |
| Gripper controller type | **GripperCommand** action `gripper_controller/gripper_cmd` (served by `gripper_node`) | FollowJointTrajectory on the two finger joints | `GripperActionController` |
| MoveIt controllers file | `lwa4p_moveit_config/config/moveit_controllers.yaml` | `isaac/moveit/moveit_controllers.yaml` | moveit_config |
| URDF used | `lwa4p_description/urdf/lwa4p.urdf.xacro` (mimic fingers) | `isaac/lwa4p_wsg50.urdf` → `.runtime/robot.urdf` (mesh paths rewritten as `file://`) | moveit_config xacro |
| SRDF | `config/lwa4p.srdf` | **rewritten** by `isaac/moveit/prepare.py`: the gripper group becomes the 2 finger joints, gripper states are split into gap/2, and a `ready` arm state is added | the same |
| `use_sim_time` | no | yes | no |
| Gravity on the arm | real | on, with stiff drives | n/a |
| Camera | not integrated yet (no realsense2 launch) | D435 frame only, no RGB-D published | none |

Consequences:
- `SimpleMoveIt.gripper(gap)` sets a joint constraint on `gripper_joint`. It works on the real
  robot and in mock mode, but **not in Isaac**. Code meant for Isaac must command the
  finger joints, or go through the SRDF named states (`open`, `tube_15mm`, `closed`). Those
  names exist in every backend because `prepare.py` keeps them.
- On the real robot, MoveIt sends `max_effort: 35.0` (from `moveit_controllers.yaml`).
  `gripper_node.execute()` runs a **force-controlled grasp** when `max_effort > 0` and the target
  gap is more than 1 mm below the current gap. Otherwise it runs a position `move`.
  `max_effort == 0` means use `default_force_n`, and a negative value always means move
  (`gripper_cmd move` sends -1).
- Real `/joint_states` has two publishers: the driver (6 arm joints, 20 Hz) and the gripper node
  (`gripper_joint`, 10 Hz). The gripper also publishes `gripper/state` (gap in `position`, force
  in `effort`).

---

## 4. Frames and geometry to keep in mind

- `world` = URDF root = the robot base frame, 0.10 m below the shoulder axes. The real
  mounting surface is at **z = −0.105**, and the Isaac table top is at about −0.105.
- `arm_tool0` = flange, 75.1 mm above `arm_6_link` and **turned by π** about its z-axis (from
  the 2018 diploma model). On it sit the FTS-LWA force/torque sensor (`fts_lwa_base_link`,
  55.35 mm above the flange, turned 0.7 rad), a 14 mm connector plate and the WSG 50. The sensor
  is geometry only: nothing publishes force or torque data.
- `gripper_tcp` = between the finger pads, **9.5 mm above the fingertips**. This is the IK tip
  of the `arm` group (chain `arm_base_link → gripper_tcp`) and the frame all grasp poses use.
  `arm_6_link → gripper_tcp` is 0.28945 m along z, turned +π/2; `arm_tool0 → gripper_tcp` is
  0.21435 m, turned −π/2. From axis 5 to the fingertips the model has 299 mm, and about 300 mm
  was measured on the real robot. Prefer `arm_6_link` as the reference for calibration results.
- The fingers close along the TCP x-axis. `gripper_yaw` (a xacro arg on `lwa4p.urdf.xacro`,
  default π/2) is that direction about the tool axis, measured from the x-axis of
  `arm_6_link`, not of `arm_tool0`. The finger direction has not been checked visually on the
  real robot yet.
- The finger mesh is the standard FMF finger from the Weiss package. Its pads stick out 3 mm
  past the base jaw, while on the real gripper they sit 9.75 mm inside it (`finger_offset_mm`
  19.5). The model puts the pads where the real ones are, so it draws the base jaws 6.75 mm
  closer to the centre than the real ones.
- "Pointing down" in `SimpleMoveIt.pose()` is the quaternion `(x=cos(yaw/2), y=sin(yaw/2),
  z=0, w=0)`, i.e. a 180° flip about a horizontal axis, where `yaw` is the finger closing
  direction. With `yaw=None` the tool's z rotation is free (tolerance π).
- Reach with the gripper pointing down is about **0.55 m** at table height. Beyond that,
  expect `NO_IK_SOLUTION` or `INVALID_MOTION_PLAN`.
- Camera: `world → camera_link` at xyz `0.45 -0.15 0.95`, rpy `0 π/2 0`. This is a
  **placeholder** until hand-eye calibration. Data comes in `camera_color_optical_frame`
  (424×240 at 5 Hz, fx = fy = 306.2).
- Joint *i* is CANopen node *i+2* (nodes 3–8 on `can0`). The WSG uses `can1`, CAN IDs
  TX 0x01 / RX 0x02.
- The WSG reports **jaw width**. The gap used everywhere in ROS is
  `gap = jaw − finger_offset_mm (19.5)`. The usable gap is 0 to about 0.0895 m, and `open` is
  0.085.

## 5. Limits: several files must agree

The real arm's power supply is weak, so it is held to **about 0.5 rad/s and 0.4 rad/s²**.
Faster moves trip DC-link undervoltage `0x3222`. These numbers live in several places:

| File | Value |
|---|---|
| `lwa4p_moveit_config/config/joint_limits.yaml` | 0.5 rad/s, 0.4 rad/s², default scaling 0.8. Gripper 0.05 m/s. |
| `lwa4p_driver/config/driver.yaml` | `max_velocity_deg_s: 30` (≈0.524 rad/s), `acceleration_deg_s2: 60`, `limit_margin_deg: 3` |
| `lwa4p_description/urdf/lwa4p.urdf.xacro` | datasheet velocity of 72°/s, range ±170° (±155° on axis 3) minus 0.02 rad |
| `isaac/lwa4p_cfg.py` | `REAL_SPEED = 0.5`, datasheet accelerations |
| `isaac/moveit/configuration.py` | adds finger limits of 0.05 m/s and 0.1 m/s² (Isaac only) |

The driver **rejects** a trajectory if any segment exceeds 1.05 × `max_velocity_deg_s`, if a
point is within `limit_margin_deg` of the drive's own software limits (read over SDO 0x607D),
if the first point is more than 2° from the current position, or if the drives are not enabled.
If you raise MoveIt limits or scaling, raise the driver limit too, or MoveIt plans will be
rejected on the real arm.

## 6. Real-driver internals (`lwa4p_driver/driver_node.py`)

- It uses **Profile Position mode only**, with absolute targets in milli-degrees. A 20 Hz loop
  thread samples the trajectory (cubic Hermite when velocities are given, otherwise linear)
  at `t + lookahead_s` and sends that as a new PP setpoint, with a velocity sized to arrive in
  time. The drives do the smoothing. This is **not** a streaming CSP or servo interface. Keep
  that in mind before adding MoveIt Servo or a high-rate policy (the README recommends 5–10 Hz).
- `~/enable` brings up NMT, resets faults, sets PP mode, enables the drives in node order, and
  runs a **±0.5° commutation move on every joint**. **The real arm physically moves when you
  call it.**
- Path tolerance violations (8°) hold the arm and abort. Drive faults and CAN errors call
  `trip()`, which disables all drives so the brakes close.
- Every trajectory writes a CSV trace to `~/.ros/lwa4p_trace_*.csv`. `~/diagnose` is read-only:
  fault history, temperature (undocumented object 0x200D.1), and torque/current.
- Threading: one `can_lock` covers all SDO traffic, and `state_lock` covers the trajectory
  state. The node uses a MultiThreadedExecutor with a ReentrantCallbackGroup. Any new SDO
  access **must** take `can_lock`.
- Startup fails fast with `SystemExit` if `can0` is missing or down. You cannot start the
  driver in sim or CI without CAN hardware (a `vcan` would also need responding nodes).
- Parameter defaults in the code differ from the YAML: `lookahead_s` 0.15 vs 0.3,
  `enable_settle_s` 1.0 vs 0.3, and gripper `open_gap_mm` 30 vs 85. The YAML wins when you use
  `bringup.launch.py`. If you run a node bare, you get the code defaults.

## 7. `lwa4p_gazebo` (to be removed)

Gazebo is abandoned, and this package has no future on Jazzy. **Do not port or extend it.** Until
it is removed, it still holds things other code needs, so move these first:

- `isaac/tools/make_isaac_urdf.py` generates the Isaac URDF from `urdf/lwa4p_gazebo.urdf.xacro`.
  That file's camera mount must move to `lwa4p_description`, and the generator must follow.
- `scripts/simple_moveit.py` and `go_to.py` are useful on the real robot and on mock hardware.
- `scripts/bin_pick.py` holds grasp logic (`find_top_object`, `grasp_options`, `wall_clearance`)
  to port into the skill server. Its perception is a simple point-cloud detector: one cloud, TF
  to `world`, crop to the bin, highest top surface, then PCA for yaw and size.
- `isaac/README.md` §4 records the reference scene, so the world file itself can go.
- `docker/Dockerfile` references `gazebo.launch.py` in a build check and in its default command.

## 8. Isaac Sim + MoveIt (`isaac/moveit/`)

Isaac is the only physics simulator, and it has gaps: **object grasping has never been tested**,
and no RGB-D data is published. NVIDIA's minimum for Sim 6.2 is an RTX 4080 with 16 GB; the
integration below was verified on an RTX 3070 with 8 GB, on the previous stack. The proposal's
section 13.3 lists the remaining work.

- `run.sh` validates Isaac's Python, **rebuilds the Docker image** (so compiled MTC stays in sync
  with `src/`), runs `prepare.py`, starts `simulation.py` on the **host** under Isaac's Python
  3.12, waits for `[LWA4P] READY`, then runs `moveit.launch.py` in the container with
  `--network host --ipc host`.
- `simulation.py` imports the URDF to USD (cached by SHA-256 under `.runtime/usd/<hash>/`),
  removes the ArticulationRoot from the separate fixed `camera_link` branch, and hosts a
  `controller_manager` through `isaacsim.ros2.control` with `IsaacSimSystem`. Controller
  config: `isaac/moveit/controllers.yaml`. This depends on the **Sim 6.2 private API**
  (`_isaacsim_ros2_control.setup_cm`), so rerun `--test` after any Sim upgrade.
- `moveit.launch.py` also starts `verify_mtc` and `planning_scene.py` (the `isaac_workbench`
  box). It enables `ExecuteTaskSolutionCapability`.
- `--test` runs `verify_execution.py` (direct JTC, MoveIt arm, both fingers) and then
  `check_ptp.py` (MTC plus a check that a second call returns `BUSY`). Reports go to
  `.runtime/{execution,mtc}.json` and `.runtime/reports/`. The files in
  `isaac/moveit/verification/*.json` are **historical** results that are checked in, not
  outputs.
- Known issue from before the migration: `move_group` crashed with SIGSEGV on SIGINT shutdown,
  after the work was done. Recheck on Jazzy.
- Override with the env vars `ISAACSIM_PATH`, `ROS_DOMAIN_ID` (default 68),
  `LWA4P_ISAAC_CACHE` and `LWA4P_ISAAC_REPORT_DIR`.
- `isaac/lwa4p_cfg.py` is an **unvalidated** Isaac Lab 2.x template. `READY` puts the TCP at
  about (0.40, 0, 0.26), pointing down.

## 9. Build, run and verify

```bash
# Once docker/Dockerfile is migrated (see §1). From the repo root:
docker build --build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)" \
  -f docker/Dockerfile -t lwa4p:jazzy .
```
- The image **COPYs `src/`** at build time and has no bind mount for the real or demo
  flows. **Rebuild after every source change**, or bind-mount `src/` yourself and rebuild with
  colcon inside the container.
- The build also acts as the smoke test. It imports `can`, `canopen`, the driver and the
  gripper modules, and runs `--show-args` on the launch files.
- Inside the image, or natively on a Jazzy machine such as this host:
  `colcon build --symlink-install`. With symlink install, edits to Python, YAML and launch files
  apply immediately. **New** files need a rebuild, and C++ always needs one.
- `rosdep` must skip `warehouse_ros_mongo` (it is in `lwa4p_moveit_config/package.xml`, has no
  Jazzy binary, and is unused).
- `canopen==2.3.0` and `python-can==4.6.1` come from pip and are **not** declared in
  `package.xml` or `setup.py`.
- The MTC packages are only in the Dockerfile's apt list. Because `lwa4p_mtc_verification`
  needs `moveit_task_constructor_core`, a plain `colcon build` on a machine without MTC fails
  on that package. Use `--packages-skip lwa4p_mtc_verification` there.

**DDS and environment.** Use `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` everywhere (`move_group`
hung on Zenoh before the migration, so retest before changing this). Domain IDs and settings differ by flow:
- Dockerfile default: `ROS_DOMAIN_ID=42`, `ROS_LOCALHOST_ONLY=1`.
- Real robot docs: `ROS_DOMAIN_ID=43`, `--network host`.
- Isaac: `ROS_DOMAIN_ID=68`, `ROS_LOCALHOST_ONLY=0`.

Match these when you attach a second terminal.

**Tests.** Only `lwa4p_calibration_poses` has unit tests (gtest for its logic layer), and no
package uses `ament_lint`. Verification today consists of:
1. the Dockerfile's build-time import and `--show-args` checks;
2. `./isaac/moveit/run.sh --test` (needs a GPU and Isaac);
3. manual runs on mock hardware: `demo.launch.py` + `go_to.py home`;
4. `colcon test --packages-select lwa4p_calibration_poses`.

The pure-logic code (`joint_motion_verification.cpp`, `Trajectory.sample` in the driver,
`find_top_object` and `grasp_options` in `bin_pick.py`, `crc16` in `wsg50.py`) is the easiest
place to add tests.

## 10. Safety rules for agents

- **Never** run anything that talks to `can0` or `can1` (driver, gripper node, `test_move`,
  `gripper_cmd`, `real.launch.py`, `~/enable`, `gripper_cmd home`) unless the user explicitly
  says the real robot is ready and someone is at the E-stop. Each of these moves hardware.
- Run only one process per CAN bus.
- Do not raise velocity or acceleration limits for the real arm without being asked (see §5).
- Treat `isaac/lwa4p_wsg50.urdf` as **generated**. Change the xacro in `lwa4p_description` (or
  the numbers in `make_isaac_urdf.py`), then regenerate with
  `python3 isaac/tools/make_isaac_urdf.py` in a sourced Jazzy environment. The generator
  currently reads the xacro in `lwa4p_gazebo` (arm + gripper + camera, see §7).

## 11. Change checklist: what must stay in sync

| If you change… | …also update |
|---|---|
| Joint names, groups or named states (SRDF) | `simple_moveit.py` (reads the SRDF at runtime for `named()`), `go_to.py` docs, `prepare.py` (assumes the gripper group/state layout), the MTC task (uses `arm`, `wsg50`, `gripper_tcp`, `tube_15mm`, `open`), `driver.yaml` `joint_names` |
| Gripper or sensor geometry or offset (`wsg50.urdf.xacro`, `fts_lwa.urdf.xacro`, the tool mounting in `lwa4p.urdf.xacro`) | `gripper.yaml` `finger_offset_mm`, `bin_pick.py` `FINGER_T`/`FINGER_W`/`TCP_ABOVE_FINGERTIP`, regenerate the Isaac URDF, the `0.0425` checks in `joint_motion_verification.cpp` and `prepare.py` `READY` |
| Camera pose | the `realsense_d435` call in `lwa4p_gazebo.urdf.xacro`, `isaac/README.md` §3, regenerate the Isaac URDF |
| World objects | `isaac/moveit/simulation.py`, `isaac/moveit/planning_scene.py`, `isaac/README.md` §4 |
| Controller names | the `moveit_controllers.yaml` files, `lwa4p_moveit_config/config/ros2_controllers.yaml`, `isaac/moveit/controllers.yaml`, the driver `controller_name`, the gripper action name in `gripper_node.py` |
| Adding Python deps to the driver | `package.xml` / `setup.py` **and** the Dockerfile pip line |
| Anything in `src/` used by Isaac | nothing manual: `run.sh` rebuilds the image |

## 12. Known rough edges (not bugs to fix unasked, but don't be surprised)

- Error strings in `gripper_cmd.py` and `simple_moveit.py` mention `arm_start.sh`, which does not
  exist. The real entry point is `real.launch.py`.
- `isaac/moveit/README.md` mentions `./isaac-setup/lwa4p-moveit.sh` in a "shared RoboCup
  workspace", which is not present here.
- The base `lwa4p.urdf.xacro` uses placeholder inertias (1 kg). Only the Isaac URDF has real
  masses.
- `kinematics.yaml` uses KDL with a 0.05 s timeout. There is no `ompl_planning.yaml` in
  `lwa4p_moveit_config/config`, so OMPL settings come from `moveit_configs_utils` defaults.
  Pilz Cartesian limits are present.
- The real camera has no driver or launch integration, and hand-eye calibration has not been
  done.
- `.gitignore` excludes `.vscode/`, `*.csv` and `*.log`. Driver traces and Isaac logs are never
  committed.

---

## 13. Planned architecture (RoboCup 2027)

The planned stack is the diagram in [manipulation_architecture.txt](manipulation_architecture.txt)
(UTF-8, LF line endings). [manipulation_implementation_proposal.md](manipulation_implementation_proposal.md)
is the proposed implementation: action definitions, the failure contract with the BT, MTC stage
lists, delivery plan, and prototype results. Read it before building any of this. It is a
proposal under review, so where it and this section differ, ask. Nothing in the diagram is
implemented yet, except what the "Today" column below says.

### What changed from the first diagram, and why

The first version of the diagram is in commit `27fed6e`. The current diagram already reflects
this table.

| Layer in the first diagram | Owner | Today | Notes for the plan |
|---|---|---|---|
| Compute split (GPU / CPU cores 4–5 / cores 6–7, RT priority) | — | nothing | The GPU part concerns the perception team. Core isolation and RT priority **do not apply** to the real arm: its driver is a 20 Hz Python loop sending Profile Position targets over SDO. |
| Perception and world model (VLM, then a TF2 3D pose) | other team | `bin_pick.py` has a simple point-cloud detector (sim only) | Contract: `PoseStamped` in a TF frame. VLM latency is in seconds, so use it for task-level detection only, never closed-loop. Grasping thin objects (tubes) needs **orientation and size**, not just a depth-lookup point. Ask for them, or fit them from the cloud locally as `bin_pick.py` does. |
| Task supervisor (BehaviorTree.CPP) | AI team | nothing | This team provides the actions and the failure enumeration. See the split with MTC below. |
| MTC | this team | only `lwa4p_mtc_verification`, a sim test utility and not a skill | MTC tasks need `move_group/ExecuteTaskSolutionCapability`. Only `isaac/moveit/moveit.launch.py` enables it. **`real.launch.py` and `demo.launch.py` do not by default.** Add it before running MTC skills there. |
| OMPL (RRTConnect) + Pilz LIN/PTP | this team | `move_group` loads OMPL, CHOMP and Pilz from `moveit_configs_utils` defaults. Isaac forces `ompl` only. | Prototype result: Pilz LIN failed `Place` on `arm_3_joint` limits even with realistic Cartesian limits, while MTC's `CartesianPath` passed. Use `CartesianPath` for straight moves. `pilz_cartesian_limits.yaml` allows 1.0 m/s, far beyond the real arm. |
| Kinematics (pick_ik / TRAC-IK) | this team | KDL (`kinematics.yaml`) | That one file applies to every backend. Prototype result over 132 top-down picks: KDL 56, pick_ik 57, at 2.5× the time, so KDL stays. pick_ik and the TRAC-IK plugin both have Jazzy binaries. |
| Ruckig smoothing | this team | not configured | Deferred. It would only affect OMPL segments, and the real driver resamples every trajectory at 20 Hz. |
| MoveIt Servo / "velocity bypass" | — | — | **Not possible on the real arm.** The driver serves only `FollowJointTrajectory` goals in Profile Position mode and has no velocity or streaming input. At most, treat it as sim-only research. |
| `ros2_control` (controller_manager, JTC, forward_command, joint_state_broadcaster) | — | Isaac, mock | **Simulation only.** On the real robot, `lwa4p_driver` serves `arm_controller/follow_joint_trajectory` and `gripper_node` serves `gripper_controller/gripper_cmd`. There is no controller_manager, no 100–250 Hz loop, and no velocity controller. |
| `ros2_canopen` / Lely, 1 Mbit/s, sync PDO | — | — | **Superseded by the driver decision.** The real bus is 500 kbit/s, SDO only, PP mode, 20 Hz, through `lwa4p_driver` (see §6). |
| Hardware: LWA 4P, E-stop | — | as §6 | The diagram **omits the WSG 50 gripper**, which sits on `can1`, speaks its own protocol (not CiA 402), and is served by `gripper_node`. Every skill involving grasping depends on it. |

### Design rules for new manipulation code

1. **Keep sim and real apart at the action interface.** Skills and the behaviour tree talk to
   MoveIt and MTC, plus `/joint_states`, TF and `gripper/state`. Never call `controller_manager`
   services (e.g. `switch_controller`), and never require Isaac services. Sim-only helpers must
   stay optional.
2. **Abstract the gripper.** Skills use SRDF named states (`open`, `tube_15mm`, `closed`) or one
   "grasp(width, force)" call. Never use raw joint names: the real robot uses
   `gripper_joint`, Isaac uses two finger joints (§3). Every backend serves the same
   `GripperCommand` behaviour, so grasp checks use one code path: `gripper_node` on the real
   robot, and the planned `sim_gripper_server` in simulation.
3. **Plan within the real driver's limits, even in simulation.** The simulation controllers
   accept trajectories the driver rejects:
   - segments faster than about 30°/s;
   - a start more than 2° from the current position;
   - targets within 3° of the drive limits;
   - tracking error above 8°.

   Keep velocity and acceleration scaling within `joint_limits.yaml`. Consider a shared
   pre-execution check, so failures show up in simulation first. Test each new skill once on the
   real arm at low speed.
4. **Split responsibilities between the behaviour tree and MTC.** MTC owns the motion
   sequence (approach → grasp → lift → place) and backtracks over IK and grasp candidates. The
   skill server verifies the grasp. The behaviour tree works at skill level: requesting
   perception, starting a skill, retrying or re-detecting, recovering, and coordinating with
   navigation. Do not duplicate the motion stages as behaviour-tree nodes.
5. **Expose skills as ROS actions** (a "skill server"). The AI team's behaviour tree calls them.
   Interfaces belong in a dedicated `*_interfaces` package. **Every goal must end with a result
   carrying a failure code**, so never reject a goal in the goal callback: a rejected goal
   returns no result. Convert all low-level errors to the enumeration in one place.
6. **Planning scene.** Perception results must become MoveIt collision objects, because MoveIt
   sees nothing on its own. Objects persist in `move_group` until you remove them, so own their
   lifecycle (add, attach, detach, remove) in the skill server.
7. **Frames.** Manipulation plans in `world` (= the arm base, §4). When the arm is mounted on
   the mobile base, `world` must hang off the base TF tree, the base must stay still during
   execution, and the camera pose needs hand-eye calibration (still a placeholder).
8. **Platform.** Target ROS 2 Jazzy and Isaac Sim. Check that a dependency has a Jazzy binary
   before adding it.

**Naming.** The `lwa4p_` prefix is only for hardware-specific packages (description, driver,
MoveIt config, sim models). New layers above the action interface are hardware-agnostic and
**must not** use `lwa4p_` or other robot-specific names: no `lwa4p`, `wsg`, `schunk` or
`erb` in package names, node names, action names or BT node IDs. Robot specifics (group names,
frames, named states, gripper widths and forces) come from parameters or YAML, so the skills
can move to another arm or gripper without renaming.

Suggested homes for new code, which **do not exist yet**:
- `manipulation_interfaces`: skill actions and the failure enumeration;
- `manipulation_skills`: skill server + MTC tasks, in C++ (you can reuse the
  `.clang-format` from `lwa4p_mtc_verification`);
- `manipulation_sim`: the simulation gripper adapter;
- `lwa4p_bringup`: launch files and robot-specific configuration.

BT nodes and XML trees belong to the AI team, not this repo. Agree on the names with the team
before creating them.
