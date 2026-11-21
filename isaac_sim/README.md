# Isaac Sim backend

The LWA 4P with the FTM 75 and the WSG 50 in Isaac Sim 6.2, driven by MoveIt on ROS 2 Jazzy through `ros2_control`.

The robot comes from Tomas's `lwa4p_description`, generated on every start; nothing generated from his model is committed.
The environment is a static USD world in [`scenes/`](scenes/),
composed with a dynamic robot description into a single USD upon launch.
The robot description is the real robot's, and MoveIt plans with the same model as on the real arm.

## How it works

```text
isaac_sim/run.sh
 ├─ ros2 run manipulation_isaac_sim make_isaac_urdf   (host Jazzy, or docker/run_jazzy_hybrid.sh)
 │    Tomas's xacro → .cache/robot/: robot.urdf, meshes/, isaac_controllers.yaml, manifest.json
 └─ $ISAACSIM_PATH/python.sh sim.py                    (Isaac Sim's Python, its bundled Jazzy libraries)
      ├─ URDF → USD with the official URDF importer, cached in .cache/usd/<key>/
      ├─ one stage: scenes/<world>.usd as a sublayer, the robot USD referenced at /World/lwa4p
      │    (simulation/scene.py)
      ├─ drives, joint friction, solver, collision approximation, contact materials,
      │    self-collision filter (simulation/robot_setup.py)
      └─ OmniGraph: /clock + isaacsim.ros2.control (simulation/ros_bridge.py)
           controller_manager inside Isaac Sim, stepped with the physics:
           joint_state_broadcaster, arm_controller, gripper_controller

ros2 launch manipulation_isaac_sim isaac_moveit.launch.py   (host Jazzy, or docker/run_jazzy_hybrid.sh)
      robot_state_publisher, controller spawner, move_group, RViz
```

- **Tested (2026-11-14, RTX 3070 Laptop, with the window, real-time factor 0.75, Tomas's masses, no joint friction):** MoveIt plans and executes `work2` and `home`, settled joint error ≤ 0.007 rad (axis 2 sags under gravity); the gripper closes to 20 mm and opens to 85 mm, error ≤ 1 mm.
  With the datasheet masses and the joint friction ([Physics](#physics)), only a headless start is checked so far (2026-11-21): the robot imports, and friction and armature apply to all six arm joints.
- **`ros2_control`:** NVIDIA's `isaacsim.ros2.control` extension, a public API of Isaac Sim 6.2.
  It runs the standard `controller_manager` inside the simulator and exposes the articulation as a `hardware_interface::SystemInterface`.
  No topic bridge and no extra package on the ROS side; `topic_based_ros2_control` has no Jazzy binary.
- **Gripper:** `gripper_controller` commands `gripper_joint`, the fingertip gap, as in Tomas's model.
  The fingers are its URDF mimic joints; the extension sets their position targets to ∓gap/2, and each finger pushes with its own drive up to 80 N.
  In free air `/joint_states` reported `gripper_joint` equal to the finger gap while the fingers were still moving (2026-11-14).
  Whether it stays equal when the fingers stop on an object is not tested yet; the finger positions are in `/joint_states` too.
- **Mounted base:** the links fixed to `world` (`arm_podest_link`, `arm_base_link`) do not collide with the world (`FilteredPairsAPI`), because the base is bolted to its support.
  The real column reaches down to the mounting surface (z −0.110), 4 mm into the lab table and 5 cm below the floor of the table world, which Tomas built around a shorter Gazebo collision mesh; without the filter PhysX throws the arm.
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
| Arm masses | 3.9 / 1.4 / 3.9 / 1.0 / 1.5 / 0.3 kg = 12 kg; centre of mass and inertia from Tomas's meshes ([below](#masses)) | LWA 4P manual 3.2, ERB manual 3.1; the split of the connecting elements and of the ERB 115 estimated |
| Tool masses | FTM 75 0.7 kg, WSG 50 1.3 kg with fingers and adapter | Tomas's model, matching their datasheets |
| Camera mass | D435 72 g, a solid box of its body | Intel's datasheet; Tomas's description gives the camera no mass (1 g with `ISAAC_INERTIALS=urdf`) |
| Joint friction, axes 1–4 / 5–6 | Coulomb 4.8–5.7 / 0.3 N·m, viscous 15.5–21.8 / 0.4–0.5 N·m·s/rad | measured on another LWA 4P (Memar, Esfahani 2015, Table 2), fitted to PhysX's Coulomb + viscous model |
| Armature, axes 1–4 / 5–6 | 0.083 / 0.03 kg·m² | estimate (`sim.yaml` explains it), not measured |
| Arm mesh colliders | convex decomposition, error 10 %, up to 32 hulls per link, shrink-wrapped | PhysX's defaults except shrink-wrap; [below](#collisions) |
| Primitive colliders | base column, FTM, gripper as in Tomas's model; camera: the D435 body | Tomas's model; Intel's `realsense2_description` |
| Self-collision | on, except the SRDF pairs | |
| Gravity | the PhysX default, −z | the worlds carry no physics scene |
| World friction | 0.8 static and dynamic, restitution 0 | `mu` in Tomas's lab SDF; his worlds have no `<bounce>`; not measured |
| Finger friction | 0.5 static and dynamic, restitution 0 | PhysX's default, which `ARM_simulation` grasped with; not measured |
| Environment | static colliders; movable objects as rigid bodies with the converted masses | `scenes/`, from Tomas's SDF worlds |
| Robot collisions with the world | all links except the mounted base | |

All values are in [`config/sim.yaml`](config/sim.yaml), except the masses.

- **Friction:** `joint_dynamics` in `sim.yaml`, applied as `PhysxJointAxisAPI` friction efforts and `PhysxJointAPI` armature.
  The harmonic drives have high friction: at the MoveIt speed limit of 0.5 rad/s, axes 1–4 lose 12–16 N·m, about 40 % of their 35 N·m rated torque.
  Moving stretched out at full speed, axis 2 needs about 50 N·m against gravity plus 16 N·m of friction, more than its 64 N·m limit, as on the real arm ("3–6 kg depending on deflection").
  Cold drives have about 10 % more viscous friction than these warm-drive values.
- **Sag:** PhysX position drives are springs without an integral term, so a joint sags by load ÷ stiffness, where the real drives hold position.
  With the datasheet masses, axis 2 stretched out horizontally sags by about 0.7° (50 N·m ÷ 4000 N·m/rad), about 1 cm at the gripper, more than the driver's 0.5° goal tolerance.
  A gravity feedforward on the drives is not done yet.

### Masses

`make_isaac_urdf` sets the arm link masses of [`config/link_inertials.yaml`](../src/manipulation_isaac_sim/config/link_inertials.yaml) in the URDF it writes for the importer, and computes each link's centre of mass and inertia tensor from its mesh in Tomas's model: solid for the rotary modules, a thin shell for the hollow connecting elements.
Tomas's own values (1 kg and 0.01 kg·m² per arm link, centre of mass on the joint axis) are placeholders; they make the arm half as heavy as the real one, and axis 2 holds 36.5 instead of about 50 N·m stretched out.

To simulate with Tomas's masses instead, set `ISAAC_INERTIALS=urdf` for the script that writes the robot files (`make_isaac_urdf --inertials urdf`):

```bash
ISAAC_INERTIALS=urdf ISAACSIM_PATH=<Isaac Sim 6.2 directory> isaac_sim/run.sh   # host ROS 2
ISAAC_INERTIALS=urdf docker/run_jazzy_hybrid.sh                                 # hybrid container
```

- The default is `datasheet`; `sim.py` prints which one the robot files have (`[isaac_sim] robot USD: … (inertials: …)`).
- `run.sh` warns when the robot files of the container or of an earlier run have other inertials than `ISAAC_INERTIALS` asks for.
- MoveIt and `robot_state_publisher` always use Tomas's description; masses do not matter to them.

## Requirements

- Isaac Sim 6.2 (standalone).
  The ROS 2 bridge and `isaacsim.ros2.control` ship with it, including their Jazzy libraries.
- The ROS side, one of:
  - ROS 2 Jazzy with MoveIt on the host, and this workspace built: `colcon build` in the repository root;
  - Docker with Tomas's image, `docker build -f docker/Dockerfile.jazzy -t lwa4p:jazzy .`
- An NVIDIA GPU; development runs on an RTX 3070 Laptop (8 GB VRAM).

## Run

With ROS 2 Jazzy on the host:

```bash
# terminal 1: the simulator; the first start compiles shaders and imports the robot
export ROS_DOMAIN_ID=<n>
ISAACSIM_PATH=<Isaac Sim 6.2 directory> isaac_sim/run.sh             # with the window
ISAACSIM_PATH=<Isaac Sim 6.2 directory> isaac_sim/run.sh --headless  # without
ISAACSIM_PATH=<Isaac Sim 6.2 directory> isaac_sim/run.sh --world table  # the table world

# terminal 2, after "[isaac_sim] launched fully"
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

Grasp tests with tubes, Patrik Bakyta's meshes, a gravity feedforward for the drives, the measured finger shape.