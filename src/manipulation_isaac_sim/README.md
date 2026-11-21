# manipulation_isaac_sim

The ROS 2 side of the Isaac Sim backend for the LWA 4P arm.
Dmitrii Ognev.

The Isaac Sim side (the simulator process, physics, scene) lives in [`isaac_sim/`](../../isaac_sim/README.md).

## Overview

- `urdf/lwa4p_isaac.urdf.xacro`: the robot description of this backend.
  It is Tomas's model, set up as in his Gazebo simulation: no lab obstacles, the RealSense D435 frames at the hand-eye calibrated pose.
  `robot_state_publisher`, MoveIt and the Isaac importer all use it.
- `make_isaac_urdf`: writes the robot files for the Isaac Sim side into one directory: the URDF for the Isaac URDF importer, copies of its meshes, `isaac_controllers.yaml` and a `manifest.json`.
  The directory is self-contained, so Isaac Sim reads it without ROS, also when the ROS side runs in a container (`docker/run_jazzy_hybrid.sh`).
  `isaac_sim/run.sh` or `docker/run_jazzy_hybrid.sh` calls it on every start, so changes in Tomas's model reach the simulation without any committed copies.
  The URDF differs from the description only in what the importer needs:
  - `<ros2_control>` and `<gazebo>` tags removed;
  - mesh URIs replaced by absolute paths of the copies in the output directory;
  - a 1 g mass on massless links between other links (`arm_tool0`, `ftm_tool0`).
    The importer makes such links plain transforms, and the URDF export inside `isaacsim.ros2.control` then loses every joint below them, including the gripper.

  The manifest holds the source digest (URDF and mesh bytes, the USD cache key), the URDF effort limits (the drive force limits) and the SRDF collision pairs that MoveIt ignores (also ignored by PhysX).
- `config/isaac_controllers.yaml`: the controllers of the controller manager inside Isaac Sim.
  Names, joints and tolerances are those of Tomas's Gazebo simulation.
- `config/moveit_controllers.yaml`: a copy of Tomas's `lwa4p_gazebo/config/moveit_controllers.yaml`.
  A test fails when his file changes.
- `launch/isaac_moveit.launch.py`: `robot_state_publisher`, the controller spawner, `move_group` and RViz, all with `use_sim_time`.

### ROS interface

The same as Tomas's Gazebo simulation:

| Name | Type | From |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/Clock` | Isaac Sim |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` in Isaac Sim |
| `/arm_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory` | Isaac Sim |
| `/gripper_controller/follow_joint_trajectory` | `control_msgs/FollowJointTrajectory`, joint `gripper_joint` (fingertip gap) | Isaac Sim |
| `/robot_description`, TF | | `robot_state_publisher` |

Not yet: the camera topics, the FTM wrench, object poses.

## Build

```bash
colcon build --symlink-install   # in the repository root
```

## Run

```bash
# terminal 1: the simulator (see isaac_sim/README.md)
ISAACSIM_PATH=<Isaac Sim 6.2 directory> isaac_sim/run.sh
# terminal 2, after the simulator prints "[isaac_sim] running"
source install/setup.bash
ros2 launch manipulation_isaac_sim isaac_moveit.launch.py
```

Use the same `ROS_DOMAIN_ID` in both terminals.

## Tests

```bash
colcon test --packages-select manipulation_isaac_sim && colcon test-result --verbose
```

- `test_isaac_urdf.py`: the URDF preparation, without ROS.
- `test_model_contract.py`: the joints, frames, SRDF groups, named states and controllers our code relies on still exist in Tomas's model.
  After a `main → devel` sync, a failure here means Tomas renamed something; update our side, never his files.
