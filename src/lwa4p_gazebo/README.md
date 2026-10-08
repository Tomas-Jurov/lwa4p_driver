# LWA 4P simulation (Gazebo + MoveIt)

Schunk LWA 4P arm with a WSG 50 gripper in Gazebo Classic, controlled through
MoveIt. The controller names are the same as on the real robot
(`arm_controller`, `gripper_controller`), so code that works here also runs on
the real arm.

The world contains:

- a 10 cm stand in front of the robot, with a 15 mm test tube (at x = 0.45, y = 0)
  and a red cube on it;
- a bin with four objects (x = 0.24, y = −0.33) and an empty output bin
  (y = +0.33);
- a RealSense D435 above the work area, looking straight down.

## Setup (once)

```bash
cd ~/Documents/schunk/lwa4p_ws        # or wherever the workspace is
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

In every new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/schunk/lwa4p_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Start the simulation

```bash
ros2 launch lwa4p_gazebo gazebo.launch.py              # Gazebo window + RViz
ros2 launch lwa4p_gazebo gazebo.launch.py gui:=false   # RViz only (faster on a laptop)
```

Wait until the log shows `You can start planning now!` and the four
controllers are `activated`. RViz shows the camera point cloud and image.

## 1. Move with the mouse (RViz)

In the MotionPlanning panel:

1. Set **Planning Group** to `arm`.
2. Choose a **Goal State** (`home`, `work1`, `work2`), or drag the orange
   interactive marker.
3. Click **Plan & Execute**.

To use the gripper, switch **Planning Group** to `gripper`. The goal states
are `open` (85 mm), `tube_15mm` and `closed`.

## 2. Move from the terminal

```bash
ros2 run lwa4p_gazebo go_to.py home
ros2 run lwa4p_gazebo go_to.py --joints 0 30 60 0 90 0     # degrees, arm_1..arm_6
ros2 run lwa4p_gazebo go_to.py --pose 0.45 0 0.30          # gripper tip x y z [m], pointing down
ros2 run lwa4p_gazebo go_to.py --gap 0.03                  # gripper opening [m]
```

## 3. Pick and place in Python

```bash
ros2 run lwa4p_gazebo pick_tube.py
```

This picks up the test tube, lifts it, and puts it down 10 cm to the side.
Read [scripts/pick_tube.py](scripts/pick_tube.py) first. Your own programs
only need the helper in [scripts/simple_moveit.py](scripts/simple_moveit.py):

```python
import rclpy
from simple_moveit import SimpleMoveIt

rclpy.init()
robot = SimpleMoveIt()
robot.add_box("stand", (0.3, 0.4, 0.16), (0.5, 0.0, 0.02))   # obstacle for MoveIt
robot.named("home")
robot.gripper(0.085)
robot.pose(0.45, 0.0, 0.29)    # gripper_tcp (between the fingertips), pointing down
robot.line(0.45, 0.0, 0.17)    # straight down, keeping the orientation
robot.gripper(0.013)           # a bit less than the part = squeeze it
robot.line(0.45, 0.0, 0.29)    # straight up
```

`pose(x, y, z, yaw=...)` also sets the direction in which the fingers close.
Objects you add with `add_box` stay in MoveIt after your script ends; delete
them with `robot.remove("name")`.

Put your script next to `simple_moveit.py`, or add that folder to `PYTHONPATH`.

## 4. Camera and bin picking

The simulated RealSense publishes the same kind of data as the real camera
driver:

| Topic | Content |
|---|---|
| `/camera/color/image_raw` | RGB image, 424 × 240, 5 Hz (light on a laptop; see `realsense_d435.urdf.xacro`) |
| `/camera/depth/image_rect_raw` | depth in metres, aligned to the colour image |
| `/camera/depth/color/points` | `PointCloud2` with XYZ + RGB, in `camera_color_optical_frame` |
| `/camera/color/camera_info` | intrinsics |

```bash
ros2 run lwa4p_gazebo bin_pick.py --look   # only detect the next object and mark it in RViz
ros2 run lwa4p_gazebo bin_pick.py          # move everything into the output bin
```

What [scripts/bin_pick.py](scripts/bin_pick.py) does in each cycle:

1. Moves the arm to `home`, out of the camera view, and takes one point cloud.
2. Transforms the cloud into the `world` frame (TF) and keeps only the points
   inside the bin, above its floor.
3. Takes the highest object and its top surface. The centre of that surface
   gives the grasp point. Its principal axes give the object size and the
   narrow side.
4. Chooses the finger direction and opening that leave the most room to the
   bin walls.
5. Goes above the object (`pose`), straight down (`line`), closes, and goes
   straight up. Then it checks the fingers to see whether it still holds the
   object.
6. Drops the object into the output bin.

The detection is intentionally simple, so it is a good place to start
improving: segment all objects (clustering), estimate full 6D poses, or try a
learned grasp detector.

To move the camera, edit the `realsense_d435` line in
[urdf/lwa4p_gazebo.urdf.xacro](urdf/lwa4p_gazebo.urdf.xacro). The camera
description itself is in `lwa4p_description/urdf/realsense_d435.urdf.xacro`.
Use the same file with `gazebo:=false` for the real camera mount.

## Good to know

- **MoveIt doesn't know about Gazebo objects.** Add obstacles with `add_box`,
  or MoveIt will plan straight through them. `bin_pick.py` adds the stand, both
  bins, and the test tube at its current place.
- **Reach:** with the gripper pointing down, the arm reaches about 0.55 m from
  its base at table height. Farther points fail with `INVALID_MOTION_PLAN` or
  `NO_IK_SOLUTION`.
- **`gripper_tcp`** is the point between the fingertips, 15 mm up from the
  tips. `pose()` moves that point.
- **Grasping:** command a gap a little smaller than the part. The fingers stop
  at the part and squeeze it, much like the real WSG grasp.
- **Sim-only parts:**
  - The robot links have no gravity, because the real arm holds itself with
    its brakes.
  - The floor is 6 cm below the robot base.
  - `finger_mimic.py` makes the two fingers follow `gripper_joint`.
  - The joints are driven by PID force control (gains in
    `config/ros2_controllers.yaml`). Without it, Gazebo can't hold objects
    between the fingers.
- **Real robot differences:** speeds are limited (about 0.5 rad/s). On the
  real arm, `gripper_controller` is a GripperCommand action rather than a
  trajectory. `simple_moveit.py` and MoveIt handle both, so your code doesn't
  change.
- **Without Gazebo** (kinematics only, no physics):
  `ros2 launch lwa4p_moveit_config demo.launch.py`
