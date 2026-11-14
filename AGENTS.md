# AGENTS.md

Rules for agents and developers working in this repository.
Read this file before you change anything.
How to use the robot and the simulators is in the READMEs of Tomas's packages.

## 1. What this repository is

- The manipulation subsystem of the RoboCup-FEI robot for RoboCup 2027 (healthcare): a Schunk LWA 4P arm, a Weiss WSG 50 gripper, a Schunk FTM 75 force/torque sensor, and a RealSense D435 on the wrist.
- A fork of [`Tomas-Jurov/lwa4p_driver`](https://github.com/Tomas-Jurov/lwa4p_driver).
  Tomas owns the driver, the robot model, the MoveIt configuration and the Gazebo simulation.
  The team owns everything it adds on top.
- **ROS 2 Jazzy only.**
  Humble is not supported, and no new code may target it.
- The repository root is the colcon workspace.

| Path | Owner | Rule |
| --- | --- | --- |
| Every file in `main`: `src/lwa4p_*`, `docker/`, `README.md`, `LICENSE`, `.gitignore`, `80-can.rules` | Tomas | Never change it in `devel` (§2) |
| `AGENTS.md`, `docs/`, `isaac_sim/`, `.github/`, our packages in `src/` | Team | Normal pull requests into `devel` |

## 2. Branches and merging

### Invariant

**Every file that exists in `main` is byte-identical in `devel`.**
`devel` is `main` plus files that only the team added.
CI enforces this and rejects a pull request that breaks it.

### Branches

- `main` mirrors Tomas's `main`.
  Nobody commits to it.
  It changes only through "Sync fork" on GitHub.
- `devel` is the default branch and holds the team's work.
- Work branches start from `devel`.

### Workflows

1. **Tomas pushed to his `main`.**
   Click "Sync fork" on `main`, open a pull request `main → devel` and merge it right away.
2. **Team work.**
   Create a branch from `devel`, open a pull request into `devel`, and merge after CI passes.
3. **The driver, the model or another Tomas file must change.**
   Create a branch from `main` and open a pull request into `Tomas-Jurov/lwa4p_driver:main`.
   After Tomas merges it, run workflow 1.
   Until then, `devel` waits; do not patch his files in `devel`.
4. **Your branch is behind `devel`** (for example, after workflow 1).
   Rebase it onto `devel` or merge `devel` into it, then let CI run again.
   Force-push is allowed only on your own work branches.

A branch created from an older `devel` does **not** undo Tomas's changes merged later, as long as it does not touch his files.
Damage happens when a branch touches them by accident: a formatter run over `src/`, a copied package, changed line endings, or a conflict resolved with "take ours".
CI exists to catch exactly that.

### Merge rules

- Merge commits only.
  Never squash or rebase-merge into `devel` or `main`: a squashed `main → devel` breaks every later sync.
- Do not delete `main`.
- Humans open and merge pull requests.
  Agents prepare branches and push only when asked.
- Before you open a pull request, check that the base repository is `RoboCup-FEI/manipulation-lwa4p`.
  In a fork, GitHub proposes `Tomas-Jurov/lwa4p_driver` by default.
- The repository is public.
  Commit only with a GitHub `noreply` e-mail address.

### What CI checks on pull requests into `devel`

1. **Tomas's files are untouched.**
   Against `git merge-base origin/main HEAD` (the last `main` merged into `devel`), the merge result may only add files.
   Comparing with the current `main` instead would fail every pull request between "Sync fork" and the `main → devel` merge.
2. **The workspace builds** on Jazzy.
3. **The model contract holds** (added together with `isaac_sim/`): the joint names, frames and controller names our code relies on still exist in Tomas's model.
   If Tomas renames one, the `main → devel` pull request fails with a clear message.

The `devel` ruleset requires these checks, a pull request, and an up-to-date branch, with an empty bypass list.

## 3. Where our code goes

- **ROS packages:** `src/<name>/`.
  The name says what the package does.
  The `lwa4p_` prefix is only for code tightly coupled to this hardware.
  Layers above the action interface (§6) use no robot names (`lwa4p`, `wsg`, `schunk`) in package, node, action or BT node names.
  Robot specifics (groups, frames, named states, widths, forces) come from parameters or YAML.
- **Isaac Sim:** `isaac_sim/`.
- **Documents:** `docs/`.
- **Tomas's `.gitignore` applies to our files too.**
  It ignores a directory named `isaac` at any depth, `*.csv`, `*.log`, `*.ply`, `charuco*` and `src/lwa4p_gazebo/scripts/*.py`.
  Never name a directory `isaac`.
  Put our own ignore rules into a nested `.gitignore` inside our directories.
- **Do not commit anything generated from Tomas's model** (URDF copies, USD files, meshes, SRDF variants).
  Generate them at build or run time from his xacro, so his changes reach us automatically.
- **If our code needs a Tomas file to behave differently,** wrap it, do not edit it: an own xacro that includes his macro, launch arguments, parameter overrides.
  If that is impossible, use workflow 3.
- **License:** MIT for all our code (`<license>MIT</license>` in `package.xml`), the same as the root `LICENSE`.
  Tomas's packages keep their own licenses (`lwa4p_description` LGPL-3.0, `lwa4p_moveit_config` BSD).
  Never change them.
- **Credit external sources** in the README of the package that uses them.

## 4. Code style contract

The team-wide rules that keep the repository consistent.
A package's own `.clang-format` or linter configuration takes precedence over this list.
Never reformat Tomas's files.

- **C++:** C++17, formatted by a `.clang-format` in each of our packages (Google-based, 100 columns, braces on a new line for classes, functions and namespaces).
  Do not put a `.clang-format` in the repository root or in `src/`: editors would apply it to Tomas's code.
  Headers are `.hpp` with `#pragma once`, under `include/<package>/`.
  Split each node into `<name>_node.cpp` (only `main` and spin), `<name>_ros.cpp` (ROS interface, no logic) and logic files that build and test without ROS.
  Types in `PascalCase`, functions and variables in `snake_case`, members end with `_`, constants in `UPPER_CASE`.
  Package code lives in the package namespace.
- **Python:** PEP 8, 100 columns, single quotes, the `ament` linters.
  Move non-trivial logic into a module without ROS.
- **Packages:** parameters in YAML files, not in launch files; Python launch files; resources found through the ament index, not relative paths; a `README.md` in every package.
- **Markdown:** one sentence per line, an empty line around headings and code blocks, a language on every code block.

## 5. Build and run

```bash
docker build -f docker/Dockerfile.jazzy -t lwa4p:jazzy .   # Tomas's image: packages only, no COPY
docker/run_jazzy.sh                                        # mounts src/ read-only, builds, opens a shell
```

- On a native Jazzy machine: `colcon build --symlink-install` in the repository root.
- Use `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` everywhere.
- If our packages need more system packages, add a Dockerfile in our own directory that starts `FROM lwa4p:jazzy`.
  Do not change Tomas's Dockerfiles.
- Mock hardware, no physics: `ros2 launch lwa4p_moveit_config demo.launch.py`.
- Gazebo Harmonic (Tomas's): `ros2 launch lwa4p_gazebo gazebo.launch.py`.
- Isaac Sim: planned in `isaac_sim/` (§6).

## 6. Backends and the shared interface

Everything above the action level must work unchanged on every backend:
`/arm_controller/follow_joint_trajectory`, the MoveIt `gripper` group and its named states, `/joint_states`, TF.

| | Real robot | Mock | Gazebo (Tomas) | Isaac Sim (ours, planned) |
| --- | --- | --- | --- | --- |
| Arm | Python driver, Profile Position (§7) | JTC, mock components | JTC | JTC through `topic_based_ros2_control` |
| `gripper_controller` | `GripperCommand`, `gripper_node` | `GripperActionController` | JTC on `gripper_joint` | as in Gazebo |
| Force/torque | `ftm_node` (§7) | none | none | `WrenchStamped` on the `ftm_node` topics |
| Camera | not started by Tomas's launch files | none | `/camera/...` from `gz_bridge.yaml` | the same topics as Gazebo |
| Physics | real | none | Tomas's workarounds (no gravity on the arm, PID force drives) | real: gravity, friction, contacts |

- Skills command the gripper through the MoveIt `gripper` group, its named states (`open`, `tube_15mm`, `closed`), or one shared `grasp(width, force)` abstraction.
  Never through raw finger joints.
- **Isaac Sim rules:**
  the geometry, scene and ROS names copy Tomas's Gazebo setup;
  the physics does not copy his workarounds, because RoboCup tasks need working friction and force feedback;
  no private `ros2_control` API;
  worlds are converted from Tomas's SDF files (`lab` by default, `table` as an option).
  Physics parameters, the URDF import and the camera come from [`DaverSVK/ARM_simulation`](https://github.com/DaverSVK/ARM_simulation), which was made for this team: reuse its code and credit it.

## 7. Real-robot facts that constrain our code

- **Speed:** the power supply is weak.
  MoveIt allows 0.5 rad/s and 0.4 rad/s² with scaling 0.8, and the driver 30°/s.
  Faster moves trip DC-link undervoltage (`0x3222`).
  Do not raise these limits without a team decision.
- **The driver rejects a trajectory** if a segment exceeds 1.05 × its speed limit, a point is within 3° of a drive limit, the first point is more than 2° from the current position, or the drives are not enabled.
  It aborts on a tracking error above 8°.
  Plan within these limits in simulation too: the simulators accept what the robot rejects.
- **No streaming input in Tomas's current public driver:** it samples each trajectory at 20 Hz and sends Profile Position targets.
  MoveIt Servo, velocity control and high-rate policies therefore do not work on the real arm with this driver.
  This is a limit of the current driver, not of the hardware; a streaming or admittance input is an open question with Tomas.
- **CAN:** the arm is on `can0` (nodes 3–8).
  The WSG 50 (`gripper_node`) and the FTM 75 (`ftm_node`) share `can1` in separate processes.
  Processes may share a bus only with unique CAN IDs and with each process filtering its own IDs.
- **Gripper:** the ROS gap is `jaw width − finger_offset_mm` (19.5 mm).
- **FTM 75:** `ftm_node` publishes `/ftm/raw` and `/ftm/magnitude`, and `/ftm/wrench` and `/ftm/wrench_external` only when `calibration_matrix` is set in `ftm.yaml`.
  The matrix is not in the repository yet.

## 8. The robot model

- Kinematics, tool geometry, camera calibration (`lwa4p_description/config/camera_calibration.yaml`) and lab obstacles come from Tomas's `lwa4p_description`.
  Read them from TF or the URDF; do not copy the numbers into our code.
- Main frames: `world`, `arm_base_link`, `arm_tool0` (flange), `ftm_tool0`, `gripper_tcp`, `camera_link`, `camera_color_optical_frame`.
- `gripper_tcp` is the IK tip of the `arm` group and the frame for grasp poses, 15 mm above the fingertips.
- SRDF states: `home`, `work1`, `work2` for `arm`; `open`, `tube_15mm`, `closed` for `gripper`.
- Reach with the gripper pointing down is about 0.5 m at table height.
- **Open questions** (changes go to Tomas through workflow 3):
  the tool length differs by about 10 mm from Patrik Bakyta's 2018 diploma model;
  whether `gripper_tcp` should move to the centre of the finger pads, 9.5 mm above the tips.

## 9. Safety rules for agents

- **Never** run anything that talks to `can0` or `can1` (`bringup.launch.py`, `real.launch.py`, the driver, gripper and FTM nodes, `~/enable`, `test_move`, `gripper_cmd`) unless the user says the robot is ready and someone is at the E-stop.
  `~/enable` alone moves every joint by ±0.5° (commutation check).
- Do not raise velocity or acceleration limits unless asked (§7).
- Do not install system packages or use `sudo` without asking.

## 10. Architecture and design rules

- Interface decisions and open questions: [docs/manipulation_interface_consolidation.md](docs/manipulation_interface_consolidation.md).
  Follow its `Agreed` items; do not implement `Open` items without a recorded decision.
- Application architecture: Dmitrii's architecture sketch, with its busy worker policy (not published in this repository).
  Ask Dmitrii before designing the command/runner layer, admission or worker behaviour.
- Diagram: [docs/manipulation_architecture.txt](docs/manipulation_architecture.txt).
- Proposed implementation (Viktor Budylin): [docs/manipulation_implementation_proposal.md](docs/manipulation_implementation_proposal.md).
  It is a snapshot from the archived `manipulation-Simulation@478a2ec`; its file paths refer to that repository.
  Where it and the consolidation differ, the consolidation wins.
- How the first diagram compares with the real hardware: [docs/architecture_review.md](docs/architecture_review.md).

**Decisions:**

- v1 exposes exactly three actions: `GoToPosition`, `Grip`, `Release`, with force behaviour from v1.
  There are no `Pick` or `Place` actions; the AI team composes them from the three.
- `GoToPosition` receives TCP-pose candidates and selects the best feasible plan found within one shared search-time budget.
- One worker serves all three actions, without a queue.
  A request that arrives while it is busy is accepted and immediately aborted with outcome `BUSY`; it never runs.
- The real robot runs Tomas's driver.
  We do not write a parallel driver (a `ros2_control` hardware interface, `ros2_canopen`) in `devel`; new driver capabilities are requested from Tomas through workflow 3.
  `ros2_control` exists only in simulation.
- Navigation and perception belong to other teams; this repository consumes their poses and TF.
- BehaviorTree.CPP and the BT XML belong to the AI team.
  This team provides the ROS 2 actions and the failure enumeration.
- Tomas removed his example solutions on purpose.
  Write our own; do not port the logic of `bin_pick.py` or `pick_tube.py`.
- From Patrik Bakyta's diploma, models and measurements may be used with credit; his algorithms may not.

**Design rules:**

1. Skills talk to MoveIt, MTC, `/joint_states`, TF and the gripper abstraction.
   They never call `controller_manager` services or require simulator services.
2. MTC plans only inside `GoToPosition`.
   The server adds no implicit pick or place sequences; `Grip` reports grasp evidence and the server monitors holding.
   Keep the application layer free of ROS, with ROS, MoveIt and MTC behind adapters.
3. Skills are ROS 2 actions with interfaces in a dedicated `*_interfaces` package.
   Every accepted goal ends with a result carrying an outcome code.
   Convert low-level errors to the enumeration in one place.
4. Ownership of the planning scene is open (decision D06 in the consolidation).
   Do not make the skill server the owner of world objects; a separate environment component may manage them later.
   Attaching and detaching the held object belongs to the server.
5. Manipulation plans in `world`, the arm base.
   On the mobile robot, `world` hangs off the base TF tree and the base stays still during execution.
6. MTC needs `ExecuteTaskSolutionCapability` in `move_group`.
   Tomas's launch files do not enable it, so enable it in our own launch file.
7. For straight moves, use MTC `CartesianPath`, not Pilz LIN; keep KDL as the IK solver (see `docs/architecture_review.md`).
8. Check that a dependency has a Jazzy binary before adding it.

Planned packages (names not final): `manipulation_interfaces` (actions, failure enumeration), `manipulation_skill_server` (skill server and MTC, C++), `manipulation_sim` (simulation gripper adapter), `lwa4p_bringup` (launch files and robot configuration).
