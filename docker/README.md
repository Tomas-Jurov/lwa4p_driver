# Run LWA 4P on Ubuntu 24.04 + ROS 2 Jazzy

Run the project in Docker. The container includes ROS 2 Humble, MoveIt 2 and
Gazebo Classic; your host's Jazzy installation can stay as it is.

## 1. Prepare and build

Install [Docker Engine](https://docs.docker.com/engine/install/ubuntu/) and make
sure `docker version` shows both Client and Server without `sudo`.

Open a desktop terminal on the host and change to the project root (the directory
containing `docker/` and `src/`). Then run:

```bash
sudo apt install x11-xserver-utils
docker build --build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)" -f docker/Dockerfile -t lwa4p:humble .

xhost +si:localuser:"$(id -un)"
```

Repeat the build after changing project source or configuration files.
Run all ROS commands below inside the corresponding container.

## 2. Simulation

### With Gazebo: physics, gripper and RGB-D camera

In the host terminal:

```bash
docker run --rm -it --init --name lwa4p-sim --shm-size=512m -e DISPLAY="$DISPLAY" -e LIBGL_ALWAYS_SOFTWARE=1 --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly lwa4p:humble ros2 launch lwa4p_gazebo gazebo.launch.py
```

Wait for `You can start planning now!` and all four controllers to activate.
In RViz, select planning group `arm`, choose `home`, `work1` or `work2`, and
click **Plan & Execute**. Select `gripper` for `open` and `closed`.

For command-line control, open a second host terminal:

```bash
docker exec -it lwa4p-sim bash
```

Then run inside the container:

```bash
ros2 control list_controllers
ros2 run lwa4p_gazebo go_to.py home
ros2 run lwa4p_gazebo go_to.py --gap 0.03
ros2 run lwa4p_gazebo bin_pick.py --look
```

For a lighter Gazebo session, append `gui:=false` to the launch command.
Physics and the camera stay active; only the Gazebo window is hidden.

### Without Gazebo: lightweight MoveIt + RViz demo

This mode uses mock hardware to display planned movements. It has no physics,
camera or physical grasping.

In a host terminal:

```bash
docker run --rm -it --init --name lwa4p-light --shm-size=512m -e DISPLAY="$DISPLAY" -e LIBGL_ALWAYS_SOFTWARE=1 --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly lwa4p:humble ros2 launch lwa4p_moveit_config demo.launch.py
```

Use **Plan & Execute** in RViz, as above. For command-line control:

```bash
docker exec -it lwa4p-light bash
```

Inside the container:

```bash
ros2 run lwa4p_gazebo go_to.py home
ros2 run lwa4p_gazebo go_to.py --joints 0 30 60 0 90 0
ros2 run lwa4p_gazebo go_to.py --gap 0.03
```

## 3. Real robot

Stop the simulation first. Keep the workspace clear and someone at the E-stop.
The `enable` service moves each joint about +0.5° and back; gripper homing also
causes movement. Run only one driver on each CAN bus.

### Configure CAN on the host

Connect the PEAK PCAN-USB Pro adapter and check the channel wiring:

| Interface | Connection | Bitrate |
| --- | --- | --- |
| `can0` | Arm, CANopen nodes 3–8 | 500000 bit/s |
| `can1` | AUX CAN, WSG 50 gripper | 500000 bit/s |

With robot control stopped, run on the host:

```bash
sudo apt install can-utils iproute2

for iface in can0 can1; do sudo ip link set "$iface" down; sudo ip link set "$iface" type can bitrate 500000; sudo ip link set "$iface" txqueuelen 20; sudo ip link set "$iface" up; done

ip -details link show can0
ip -details link show can1
```

Repeat after reconnecting the adapter or rebooting.

### Start the driver, MoveIt and RViz

In the host terminal:

```bash
docker run --rm -it --init --name lwa4p-real --network host --shm-size=512m -e ROS_DOMAIN_ID=43 -e DISPLAY="$DISPLAY" -e LIBGL_ALWAYS_SOFTWARE=1 --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly lwa4p:humble ros2 launch lwa4p_moveit_config real.launch.py can_channel:=can0
```

In a second host terminal:

```bash
docker exec -it lwa4p-real bash
```

Inside the container, check joint states, then enable the arm and home the gripper:

```bash
ros2 topic echo /joint_states --once
ros2 service call /lwa4p_driver/enable std_srvs/srv/Trigger '{}'
ros2 run lwa4p_driver gripper_cmd home
```

Wait for each command to succeed. Use **Plan & Execute** in RViz to move the
robot. The gripper must be connected on `can1` for this launch.

### Stop the robot

Inside the container:

```bash
ros2 service call /gripper/stop std_srvs/srv/Trigger '{}'
ros2 service call /lwa4p_driver/disable std_srvs/srv/Trigger '{}'
```

Then press **Ctrl+C** in the launch terminal.

## Stop a simulation

Press **Ctrl+C** in its launch terminal, or run on the host:

```bash
docker stop --timeout 30 lwa4p-sim
```

Use `lwa4p-light` instead for the demo without Gazebo. To reset the Gazebo world,
stop the container and start it again.

After closing all RViz/Gazebo containers, revoke display access:

```bash
xhost -si:localuser:"$(id -un)"
```
