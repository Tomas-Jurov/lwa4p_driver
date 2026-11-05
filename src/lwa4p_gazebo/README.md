# LWA 4P simulation (Gazebo + MoveIt)

Schunk LWA 4P arm with an FTM 75 force-torque sensor, a WSG 50 gripper and a
RealSense D435 on the wrist, controlled through MoveIt. On ROS 2 Humble it runs
in Gazebo Classic 11; on Jazzy it runs in Gazebo Harmonic. The launch command is
the same on both, because `gazebo.launch.py` picks the simulator from
`ROS_DISTRO`. The controller names are the same as on the real robot
(`arm_controller`, `gripper_controller`), so code that works here also runs on
the real arm.

The world contains:

- a 10 cm stand in front of the robot, with a 15 mm test tube (at x = 0.45, y = 0)
  and a red cube on it;
- a bin with four objects (x = 0.24, y = −0.33) and an empty second bin
  (y = +0.33);
- the RealSense on the wrist, at the hand–eye calibrated pose of the real camera
  (`lwa4p_description/config/camera_calibration.yaml`).

## Setup (once)

```bash
cd ~/lwa4p_ws                          # wherever the workspace is
source /opt/ros/humble/setup.bash      # or /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

In every new terminal:

```bash
source /opt/ros/humble/setup.bash      # or /opt/ros/jazzy/setup.bash
source ~/lwa4p_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Start the simulation

```bash
ros2 launch lwa4p_gazebo gazebo.launch.py              # Gazebo window + RViz
ros2 launch lwa4p_gazebo gazebo.launch.py gui:=false   # RViz only (faster on a laptop)
```

Wait until the log shows `You can start planning now!` and the four
controllers are `activated`. RViz shows the camera point cloud and image.
To reset the world, stop the launch (Ctrl+C) and start it again.

## Move with the mouse (RViz)

In the MotionPlanning panel:

1. Set **Planning Group** to `arm`.
2. Choose a **Goal State** (`home`, `work1`, `work2`), or drag the orange
   interactive marker.
3. Click **Plan & Execute**.

To use the gripper, switch **Planning Group** to `gripper`. The goal states
are `open` (85 mm), `tube_15mm` and `closed`.

## Camera topics

The simulated RealSense publishes the same topics as the real camera driver:

| Topic | Content |
|---|---|
| `/camera/color/image_raw` | RGB image, 424 × 240, 5 Hz (light on a laptop; see `realsense_d435.urdf.xacro`) |
| `/camera/depth/image_rect_raw` | depth in metres, aligned to the colour image |
| `/camera/depth/color/points` | `PointCloud2` with XYZ + RGB; transform it with TF using its `header.frame_id` |
| `/camera/color/camera_info` | intrinsics |

## Your own scripts

Put them in `scripts/`. Every `*.py` there is installed by `colcon build` and
started with `ros2 run lwa4p_gazebo <name>.py`. Git ignores them, so they stay on
your machine.

## Good to know

- **MoveIt doesn't know about Gazebo objects.** Add the obstacles you care about
  to the planning scene, or MoveIt will plan straight through them.
- **Reach:** with the gripper pointing down, the arm reaches only about 0.5 m
  from its base at table height (the FTM 75 makes the tool 5 cm longer).
  Farther points fail with `INVALID_MOTION_PLAN` or `NO_IK_SOLUTION`.
- **`gripper_tcp`** is the point between the fingertips, 15 mm up from the
  tips. Plan with it as the tip link of the `arm` group.
- **Gripper:** `gripper_joint` is the fingertip gap in metres. The simulated
  fingers stop at a part and squeeze it, much like the real WSG grasp.
- **Sim-only parts:**
  - The robot links have no gravity, because the real arm holds itself with
    its brakes.
  - The floor is 6 cm below the robot base.
  - `finger_mimic.py` makes the two fingers follow `gripper_joint`, and
    publishes the measured gap on `joint_states_measured` for RViz and MoveIt.
  - Gazebo Classic drives the joints with PID force control (gains in
    `config/ros2_controllers.yaml`). Without it, Gazebo can't hold objects
    between the fingers.
  - Gazebo Harmonic only: `worlds/lwa4p_table_gz.sdf`,
    `config/ros2_controllers_gz.yaml`, `config/gz_bridge.yaml` (ROS ↔ gz
    topics) and `gz_cloud_frame.py`. The last one fixes the point cloud's frame,
    because Harmonic publishes the cloud in the camera body axes but labels it
    with the optical frame.
- **Real robot differences:** speeds are limited (about 0.5 rad/s). On the
  real arm, `gripper_controller` is a `GripperCommand` action rather than a
  trajectory; planning through MoveIt's `gripper` group works on both.
- **Without Gazebo** (kinematics only, no physics):
  `ros2 launch lwa4p_moveit_config demo.launch.py`
