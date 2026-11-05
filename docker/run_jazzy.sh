#!/usr/bin/env bash
# Start the Jazzy container with this workspace (sources read-only), X11 for
# Gazebo/RViz windows, and host networking (CAN interfaces, ROS discovery).
#   docker/run_jazzy.sh                      # interactive shell
#   docker/run_jazzy.sh ros2 launch lwa4p_gazebo gazebo.launch.py
WS="$(cd "$(dirname "$0")/.." && pwd)"
xhost +local:root >/dev/null 2>&1 || true
exec docker run --rm -it --name lwa4p_jazzy \
  --network host --ipc host \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  -v "$WS/src":/ws/src:ro \
  lwa4p:jazzy bash -c 'source /opt/ros/jazzy/setup.bash && cd /ws &&
    colcon build --event-handlers console_cohesion- >/tmp/build.log 2>&1 \
      || { tail -30 /tmp/build.log; exit 1; }
    source install/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp &&
    exec "${@:-bash}"' _ "$@"
