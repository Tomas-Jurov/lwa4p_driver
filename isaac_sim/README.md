# Isaac Sim backend

The LWA 4P with the FTM 75 and the WSG 50 in Isaac Sim 6.2, driven by MoveIt on ROS 2 Jazzy through `ros2_control`.
Dmitrii Ognev.

The robot comes from Tomas's `lwa4p_description`, generated on every start; nothing generated from his model is committed.
The ROS names copy his Gazebo simulation, so code above the action level runs unchanged on both.
The physics does not copy his Gazebo workarounds: gravity acts on the arm, and the drives are position drives with force limits.

## How it works

```text
isaac_sim/run.sh
 ├─ ros2 run manipulation_isaac_sim make_isaac_urdf   (host Jazzy, or docker/run_jazzy_hybrid.sh)
 │    Tomas's xacro → .cache/robot/: robot.urdf, meshes/, isaac_controllers.yaml, manifest.json
 └─ $ISAACSIM_PATH/python.sh sim.py                    (Isaac Sim's Python, its bundled Jazzy libraries)
      ├─ URDF → USD with the official URDF importer, cached in .cache/usd/<key>/
      ├─ drives, solver, self-collision filter (simulation/robot_setup.py)
      ├─ ground plane at the lab table height (simulation/scene.py)
      └─ OmniGraph: /clock + isaacsim.ros2.control (simulation/ros_bridge.py)
           controller_manager inside Isaac Sim, stepped with the physics:
           joint_state_broadcaster, arm_controller, gripper_controller

ros2 launch manipulation_isaac_sim isaac_moveit.launch.py   (host Jazzy, or docker/run_jazzy_hybrid.sh)
      robot_state_publisher, controller spawner, move_group, RViz
```

- **Tested (2026-11-14, RTX 3070 Laptop, with the window, real-time factor 0.75):** MoveIt plans and executes `work2` and `home`, settled joint error ≤ 0.007 rad (axis 2 sags under gravity); the gripper closes to 20 mm and opens to 85 mm, error ≤ 1 mm.
- **`ros2_control`:** NVIDIA's `isaacsim.ros2.control` extension, a public API of Isaac Sim 6.2.
  It runs the standard `controller_manager` inside the simulator and exposes the articulation as a `hardware_interface::SystemInterface`.
  No topic bridge and no extra package on the ROS side; `topic_based_ros2_control` has no Jazzy binary.
- **Gripper:** `gripper_controller` commands `gripper_joint`, the fingertip gap, as in Tomas's model.
  The fingers are its URDF mimic joints; the extension sets their position targets to ∓gap/2, and each finger pushes with its own drive up to 80 N.
  In free air `/joint_states` reported `gripper_joint` equal to the finger gap while the fingers were still moving (2026-11-14).
  Whether it stays equal when the fingers stop on an object is not tested yet; the finger positions are in `/joint_states` too.
- **USD cache:** the key covers the URDF, the mesh bytes, the import settings and the importer version.
  When Tomas changes the model, the next start imports it again (about 30 s).
  Delete `.cache/` to force a new import.
- **`simulation/control_tree_fix.py`:** works around a bug in `isaacsim.ros2.control` 0.1.6 that drops `gripper_body_joint` from the URDF the extension builds for the controller manager.
  Remove it once NVIDIA fixes the extension; it warns when the extension version changes.

## Physics

| Setting | Value | Source |
| --- | --- | --- |
| Step | 120 Hz, CPU, TGS | controller manager runs once per step |
| Solver iterations | 64 position, 16 velocity | `ARM_simulation` |
| Arm drives, axes 1–4 / 5–6 | 4000 / 1000 N·m/rad, damping 120 / 30 N·m·s/rad | `ARM_simulation` |
| Gripper drives | 2000 N/m, 100 N·s/m | `ARM_simulation`'s 200/200 timed out `gripper_controller`; grip force not tuned |
| Drive force limits | the URDF effort limits: 64, 19, 80 N | Tomas's model |
| Masses | Tomas's model (1 kg per arm link) | Tomas's model |
| Collisions | convex decomposition; self-collision on, except the SRDF pairs | |
| Ground | z = −0.1062 m, the table top Tomas measured | placeholder until the worlds are converted |

All values are in [`config/sim.yaml`](config/sim.yaml).

## Requirements

- Isaac Sim 6.2 (standalone).
  The ROS 2 bridge and `isaacsim.ros2.control` ship with it, including their Jazzy libraries.
- The ROS side, one of:
  - ROS 2 Jazzy with MoveIt on the host, and this workspace built: `colcon build` in the repository root;
  - Docker with Tomas's image, `docker build -f docker/Dockerfile.jazzy -t lwa4p:jazzy .`, and no ROS on the host.
- An NVIDIA GPU; development runs on an RTX 3070 Laptop (8 GB).

## Run

With ROS 2 Jazzy on the host:

```bash
# terminal 1: the simulator; the first start compiles shaders and imports the robot
export ROS_DOMAIN_ID=<n>
ISAACSIM_PATH=<Isaac Sim 6.2 directory> isaac_sim/run.sh             # with the window
ISAACSIM_PATH=<Isaac Sim 6.2 directory> isaac_sim/run.sh --headless  # without

# terminal 2, after "[isaac_sim] running"
export ROS_DOMAIN_ID=<n>
source install/setup.bash
ros2 launch manipulation_isaac_sim isaac_moveit.launch.py
```

With the ROS side in a container:

```bash
# terminal 1: the ROS side; builds the workspace and writes the robot files, then opens a shell
docker/run_jazzy_hybrid.sh

# terminal 2: the simulator, with the robot files the container wrote
ISAACSIM_PATH=<Isaac Sim 6.2 directory> isaac_sim/run.sh

# terminal 1, after "[isaac_sim] launched fully"
ros2 launch manipulation_isaac_sim isaac_moveit.launch.py
```

- `docker/run_jazzy_hybrid.sh` starts the same container as Tomas's `docker/run_jazzy.sh`, and adds two things.
  It passes `ROS_DOMAIN_ID` into the container.
  It mounts `isaac_sim/.cache/robot` read-write at the same absolute path, and writes the robot files there with `make_isaac_urdf`.
  The paths inside the files are therefore valid on the host, and Isaac Sim needs no ROS installation.
- The container writes the robot files only when it starts.
  After a change to the model or to `isaac_controllers.yaml`, restart the container before `run.sh`.
- While the `lwa4p_jazzy_hybrid` container runs, `run.sh` uses its robot files, even when the host has ROS 2 Jazzy too.
  Without the container, `run.sh` generates them with the host's ROS 2 if it finds `manipulation_isaac_sim` there, and otherwise uses the files the last container left.
- A second shell in the container: `docker exec -it lwa4p_jazzy_hybrid bash`.

Check without MoveIt:

```bash
ros2 control list_controllers
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  '{joint_names: [arm_1_joint, arm_2_joint, arm_3_joint, arm_4_joint, arm_5_joint, arm_6_joint],
    points: [{positions: [0.0, -0.3094, 1.414, 0.8955, 0.0, 0.0], time_from_start: {sec: 5}}]}'
```

If RViz or Isaac Sim crash at start from the VS Code terminal (a snap), start them from a normal terminal.

## Not yet

Tomas's worlds (SDF → USD), the D435 camera, the FTM 75 wrench, grasp tests with tubes, Patrik Bakyta's meshes.
The plan is in the team's TODO and [`AGENTS.md`](../AGENTS.md) §6.

## Credits

- Physics parameters and the URDF import approach: [`DaverSVK/ARM_simulation@66a8a66`](https://github.com/DaverSVK/ARM_simulation), made for this team.
- `meshes/`: FTS-LWA sensor and WSG 50 gripper with fingers from Patrik Bakyta's diploma thesis (STU FEI, 2018), used with his permission.
  Taken from the archived `RoboCup-FEI/manipulation-Simulation@478a2ec`.
  Not used yet; planned as an optional visual model in Tomas's frames.
