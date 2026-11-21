# Isaac Sim backend

The LWA 4P with the FTM 75 and the WSG 50 in Isaac Sim 6.2, driven by MoveIt on ROS 2 Jazzy through `ros2_control`.
Dmitrii Ognev.

The robot comes from Tomas's `lwa4p_description`, generated on every start; nothing generated from his model is committed.
The environment is a static USD world in [`scenes/`](scenes/), converted from Tomas's Gazebo worlds; both are composed into one stage on every start.
The robot description is the real robot's, and MoveIt plans with the same model as on the real arm, including Tomas's measured lab obstacles in the `lab` world.
Nothing copies his Gazebo simulation or its workarounds: gravity acts on the arm, the drives are position drives with force limits, and the real base column collides.
The simulation differs from the description only where a generous planning shape would touch objects the real robot misses: the camera collides with the D435 body instead of Tomas's box ([Collisions](#collisions)).

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
- **USD cache:** the key covers the URDF, the mesh bytes, the import settings and the importer version.
  When Tomas changes the model, the next start imports it again (about 30 s).
  Delete `.cache/` to force a new import.
- **Worlds:** `lab` ([`scenes/lwa4p_lab.usd`](scenes/lwa4p_lab.usd), the default) and `table` ([`scenes/lwa4p_table.usd`](scenes/lwa4p_table.usd)), converted from `src/lwa4p_gazebo/worlds/lwa4p_<world>_gz.sdf` and cleaned with `tools/clean_world.sh` ([below](#converting-a-world)).
  Their origin is the robot's `world` frame, so the robot sits at the identity pose.
- **MoveIt in each world:** `isaac_moveit.launch.py world:=<name>` must name the world `run.sh --world` runs ([`worlds.yaml`](../src/manipulation_isaac_sim/config/worlds.yaml)).
  In `lab`, MoveIt keeps Tomas's measured obstacles (table, T-profile frame, cap dispenser, tube racks), which the lab world has at the same poses.
  In `table`, it gets the static models of Tomas's table SDF instead (floor, stand, both bins), as obstacle links with the same mechanism and no margin.
  Movable objects are obstacles in neither world.
  Isaac Sim never imports the obstacles: the world has them as objects.
- **Mounted base:** the links fixed to `world` (`arm_podest_link`, `arm_base_link`) do not collide with the world (`FilteredPairsAPI`), because the base is bolted to its support.
  The real column reaches down to the mounting surface (z −0.110), 4 mm into the lab table and 5 cm below the floor of the table world, which Tomas built around a shorter Gazebo collision mesh; without the filter PhysX throws the arm.
- **Start pose:** the SRDF states `home` and `open` (`initial_state` in `sim.yaml`); `make_isaac_urdf` adds the finger positions from the mimic joints.
  The file is a sublayer, not a reference, so every run-time edit goes to the in-memory root layer and the file on disk never changes.
  A world holds geometry, visual materials, rigid bodies with their masses, collision groups and its light; the physics scene and the contact materials come from `sim.yaml`.
  `simulation/scene.py` refuses a world that is not clean.
- **Contact materials:** `contact` in `sim.yaml` lists the materials and binds them by name: one to every world model, one to the finger links.
  They are the only physics materials on the stage; the other robot links keep PhysX's default (0.5, 0.5, 0).
  Checked (2026-11-21) with the values PhysX reports through the tensor API, and with a 5 cm drop of `red_cube`: it settles at its resting height without bouncing.
- **Collision groups:** the Isaac equivalent of Unity's layers is `UsdPhysics.CollisionGroup`: a group's `colliders` collection lists its members, and `physics:filteredGroups` lists the groups it does not collide with.
  `UsdPhysics.FilteredPairsAPI` is the per-pair version, used here for the SRDF self-collision pairs.
  `clean_world.py` translates Tomas's SDF `collide_bitmask` values: one group per bitmask, and groups whose bitmasks share no bit filter each other.
  In `lab`, the racks and holders (`0x01`) do not collide with the tubes, the rack supports and the holder guide (`0x02`); everything else, the robot too, collides with both.
  At rest the tubes do not touch the rack mesh either way; the filter matters when a tube goes in or out, which is not tested yet.
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

### Collisions

Tomas's model has the arm links' STL meshes as colliders, and boxes and cylinders for the base column, the FTM 75, the gripper and the camera.
PhysX simulates a mesh on a moving body only as convex shapes, and the importer writes one convex hull per mesh.
A single hull fills the bend of the upper arm (`arm_2_link`, 1.6 × its volume) and of the forearm (`arm_4_link`, 3.6 ×), so a folded wrist could hit the forearm's inside and objects could touch where the real arm has no material.
MoveIt plans with the exact meshes, so its plans could collide in the simulation only.

- **Convex decomposition:** `collision` in `sim.yaml`; `simulation/robot_setup.py` sets it on the live stage, so the cached USD and its import stay unchanged.
  The importer puts the colliders in instances, which are read-only on the stage; `robot_setup.py` makes the instances that hold a mesh collider ordinary prims first.
  `sim.py` asks PhysX for the cooked hulls of each link and prints them (`[isaac_sim] convex hulls per link`).
- **Checked (2026-11-21, headless, RTX 5080 Laptop):** the ERB modules `arm_1`, `arm_3`, `arm_5` stay one hull each, `arm_2` and `arm_4` get 32 hulls, the flange `arm_6` 22.
  The first start cooks for 3.8 s; later starts read PhysX's mesh cache.
  Simulation speed was the same as with single hulls: 20 s simulated in 31 s.
  To see the hulls, show the colliders in the viewport (the eye icon → Show By Type → Physics → Colliders).
- **`import.collision_type` is gone:** Isaac Sim 6.2's importer uses it only together with `collision_from_visuals`, which would replace all of Tomas's collision shapes with his visuals; the setting never had an effect here.
- **Camera:** Tomas's camera collision box (60 × 130 × 50 mm) holds the D435, its bracket, screws and cable plug with a margin, so MoveIt keeps clear of them.
  For Isaac Sim only, `make_isaac_urdf` replaces it with the D435 body (25 × 90 × 25 mm) at Intel's position relative to `camera_link`, from [`collision_overrides.yaml`](../src/manipulation_isaac_sim/config/collision_overrides.yaml).
  The bracket behind the camera is not modelled, so the simulation lets the wrist come closer to objects than the real bracket allows; MoveIt still plans with the large box.
- **Gripper fingers:** boxes, as in Tomas's model.
  Flat parallel pads give PhysX stable grasp contacts; the real finger shape (a groove for tubes?) is not measured yet, and Patrik Bakyta's finger mesh (73 mm) does not match the measured 79 mm.

### Masses

`make_isaac_urdf` sets the arm link masses of [`config/link_inertials.yaml`](../src/manipulation_isaac_sim/config/link_inertials.yaml) in the URDF it writes for the importer, and computes each link's centre of mass and inertia tensor from its mesh in Tomas's model: solid for the rotary modules, a thin shell for the hollow connecting elements.
Nothing is committed, so a change in Tomas's meshes reaches the inertia on the next start.
Tomas's own values (1 kg and 0.01 kg·m² per arm link, centre of mass on the joint axis) are placeholders; they make the arm half as heavy as the real one, and axis 2 holds 36.5 instead of about 50 N·m stretched out.

To simulate with Tomas's masses instead, set `ISAAC_INERTIALS=urdf` for the script that writes the robot files (`make_isaac_urdf --inertials urdf`):

```bash
ISAAC_INERTIALS=urdf ISAACSIM_PATH=<Isaac Sim 6.2 directory> isaac_sim/run.sh   # host ROS 2
ISAAC_INERTIALS=urdf docker/run_jazzy_hybrid.sh                                 # hybrid container
```

- The default is `datasheet`; `sim.py` prints which one the robot files have (`[isaac_sim] robot USD: … (inertials: …)`).
- `run.sh` warns when the robot files of the container or of an earlier run have other inertials than `ISAAC_INERTIALS` asks for.
- The two variants are different URDFs, so each has its own USD in the cache and switching does not import again after the first time.
- MoveIt and `robot_state_publisher` always use Tomas's description; masses do not matter to them.

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
ISAACSIM_PATH=<Isaac Sim 6.2 directory> isaac_sim/run.sh --world table  # the table world

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

## Converting a world

Convert Tomas's `src/lwa4p_gazebo/worlds/lwa4p_<world>_gz.sdf` to USD, then clean the result in place:

```bash
ISAACSIM_PATH=<Isaac Sim directory> isaac_sim/tools/clean_world.sh \
  isaac_sim/scenes/lwa4p_<world>.usd src/lwa4p_gazebo/worlds/lwa4p_<world>_gz.sdf
```

It uses the USD libraries shipped with Isaac Sim and does not start the simulator; any Python with `pxr` (for example `usd-core`) runs `clean_world.py` as well.
It refuses a USD whose models differ from the SDF, and running it on its own output changes nothing.
It removes or fixes what the conversion and Gazebo left behind:

| Artifact | Why it is wrong | Fix |
| --- | --- | --- |
| A `PhysicsScene` in the world | `SimulationManager` adds its own, and PhysX steps the two at different rates | removed; `sim.yaml` sets the step |
| `RigidBodyAPI` on a model and on its link | PhysX adds a massless body per model | kept on the link only |
| Physics materials | restitution 1 everywhere; density 1 kg/m³; ODE's two friction directions `mu`, `mu2` written as dynamic and static friction; Gazebo grip workarounds (friction 2–3, ground 100) | removed; `sim.yaml` binds the contact materials |
| Lost `collide_bitmask` | tubes collide with the rack mesh that Tomas filtered out | collision groups |
| Materials in a second root prim `/Looks`, no default prim | the file cannot be referenced | moved under the root prim, which becomes the default prim |
| Bindings without `MaterialBindingAPI` | deprecated in newer USD | API applied |
| Black emission enabled on every material | the SDF has no `<emissive>` | removed |
| Light: intensity 100, direction dropped, deprecated attributes | dim, and straight down instead of the SDF's direction | 3000 (`--light-intensity`), the SDF's direction and colour |
| Animation time codes | the world has no animation | removed |

Tomas's SDF changes do not reach the committed USD: convert and clean the world again when his `worlds/` change.

## Not yet

Converting the worlds at start (they are converted once and committed), the D435 camera, the FTM 75 wrench, grasp tests with tubes, Patrik Bakyta's meshes, a gravity feedforward for the drives, a MoveIt test with the datasheet masses, the joint friction and the decomposed colliders, the measured finger shape.
The plan is in the team's TODO and [`AGENTS.md`](../AGENTS.md) §6.

## Credits

- Physics parameters, the URDF import approach and the split of the arm masses: [`DaverSVK/ARM_simulation@66a8a66`](https://github.com/DaverSVK/ARM_simulation), made for this team.
- Joint friction: A. H. Memar, E. T. Esfahani, "Modeling and Dynamic Parameter Identification of the SCHUNK Powerball Robotic Arm", ASME IDETC/CIE 2015, paper DETC2015-47703, Table 2.
- Camera collision shape: Intel's [`realsense2_description`](https://github.com/IntelRealSense/realsense-ros) (`_d435.urdf.xacro`, branch `ros2-master`).
- Masses, gear ratios, torques: SCHUNK assembly and operating manuals of the LWA 4P (1344817, 04.00) and of the ERB 115/145 rotary modules (389537, 02.00).
- `meshes/`: FTS-LWA sensor and WSG 50 gripper with fingers from Patrik Bakyta's diploma thesis (STU FEI, 2018), used with his permission.
  Taken from the archived `RoboCup-FEI/manipulation-Simulation@478a2ec`.
  Not used yet; planned as an optional visual model in Tomas's frames.
