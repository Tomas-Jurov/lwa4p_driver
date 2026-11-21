# manipulation_isaac_sim

The ROS 2 side of the Isaac Sim backend for the LWA 4P arm.
Dmitrii Ognev.

The Isaac Sim side (the simulator process, physics, scene) lives in [`isaac_sim/`](../../isaac_sim/README.md).

## Overview

- `urdf/lwa4p_isaac.urdf.xacro`: the robot description of this backend: Tomas's real-robot model with his defaults (lab obstacles, base column, camera collision box).
  It adds only `camera_color_optical_frame` and `camera_depth_optical_frame`, which `realsense2_camera` publishes on the real robot.
  `robot_state_publisher`, MoveIt and the Isaac importer all use it.
- `config/worlds.yaml`: what MoveIt sees in each Isaac Sim world.
  In `lab` it keeps the obstacles, as on the real robot.
  In `table` the launch file removes them from the parsed URDF and adds the static models of `lwa4p_gazebo/worlds/lwa4p_table_gz.sdf` as `obstacle_*` links instead, with their collisions against each other and the mounted base disabled in the SRDF, as Tomas's SRDF does for his.
  They are never switched off through the xacro argument `workspace_obstacles`, which would also swap the base column for a Gazebo workaround.
- `make_isaac_urdf`: writes the robot files for the Isaac Sim side into one directory: the URDF for the Isaac URDF importer, copies of its meshes, `isaac_controllers.yaml` and a `manifest.json`.
  The directory is self-contained, so Isaac Sim reads it without ROS, also when the ROS side runs in a container (`docker/run_jazzy_hybrid.sh`).
  `isaac_sim/run.sh` or `docker/run_jazzy_hybrid.sh` calls it on every start, so changes in Tomas's model reach the simulation without any committed copies.
  The URDF differs from the description only in what the importer needs:
  - `<ros2_control>` and `<gazebo>` tags removed;
  - the workspace obstacles removed (`obstacle_*` links fixed to `world`): the world has them as objects;
  - a 1 g mass on massless links with a collision shape (the real `camera_link`), so their colliders become rigid bodies;
  - mesh URIs replaced by absolute paths of the copies in the output directory;
  - a 1 g mass on massless links between other links (`arm_tool0`, `ftm_tool0`).
    The importer makes such links plain transforms, and the URDF export inside `isaacsim.ros2.control` then loses every joint below them, including the gripper.

  The manifest holds the source digest (URDF and mesh bytes, the USD cache key), the URDF effort limits (the drive force limits), the SRDF collision pairs that MoveIt ignores (also ignored by PhysX), and the SRDF named states with their mimic joints (the start pose).
- `config/isaac_controllers.yaml`: the controllers of the controller manager inside Isaac Sim, under the names MoveIt uses on every backend.
- `config/moveit_controllers.yaml`: the arm and the execution settings are the real robot's (`lwa4p_moveit_config`; a test fails when Tomas changes them).
  The gripper is a trajectory controller on `gripper_joint`, where the real robot has `GripperCommand`: Isaac Sim ships no gripper action controller, and the simulation gripper adapter is open (D09).
- `launch/isaac_moveit.launch.py`: `robot_state_publisher`, the controller spawner, `move_group` and RViz, all with `use_sim_time`; `world:=lab|table` (default `lab`) as `isaac_sim/run.sh --world`.

### ROS interface

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
ros2 launch manipulation_isaac_sim isaac_moveit.launch.py   # world:=table with run.sh --world table
```

Use the same `ROS_DOMAIN_ID` in both terminals.

## Tests

```bash
colcon test --packages-select manipulation_isaac_sim && colcon test-result --verbose
```

- `test_isaac_urdf.py`: the URDF preparation, without ROS.
- `test_model_contract.py`: the joints, frames, SRDF groups, named states, workspace obstacles and controllers our code relies on still exist in Tomas's model.
  After a `main → devel` sync, a failure here means Tomas renamed something; update our side, never his files.
