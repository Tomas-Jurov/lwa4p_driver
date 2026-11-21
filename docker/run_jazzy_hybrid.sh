#!/usr/bin/env bash
# Hybrid setup: the ROS side in Tomas's lwa4p:jazzy image, Isaac Sim (isaac_sim/run.sh)
# natively on a host without its own ROS installation. Team file, not part of Tomas's main.
#
# The same container as docker/run_jazzy.sh (sources read-only, X11, host network and IPC),
# with three additions:
#   - ROS_DOMAIN_ID of this shell is passed into the container (default 0);
#   - Fast DDS uses UDP only. Its default shared-memory transport fails between this root
#     container and Isaac Sim running as the host user: discovery works, but the segments
#     the container creates in /dev/shm are not writable for the host user, so no data or
#     service response arrives (the spawner times out on list_controllers);
#   - on start, make_isaac_urdf writes the robot files for isaac_sim/run.sh into
#     isaac_sim/.cache/robot. That directory is mounted read-write at the same absolute path,
#     so the paths inside the files are valid on the host too. ISAAC_INERTIALS=urdf keeps
#     the arm masses of Tomas's model (make_isaac_urdf --inertials, default datasheet).
#
# Usage (build the image first: docker build -f docker/Dockerfile.jazzy -t lwa4p:jazzy .):
#   docker/run_jazzy_hybrid.sh                 # interactive shell
#   docker/run_jazzy_hybrid.sh ros2 launch manipulation_isaac_sim isaac_moveit.launch.py
#   docker exec -it lwa4p_jazzy_hybrid bash    # another shell in the running container
set -euo pipefail

REPOSITORY="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_DIR="${REPOSITORY}/isaac_sim/.cache/robot"

# Create it as the host user; Docker would create a missing mount point as root.
mkdir -p "${ROBOT_DIR}"
xhost +local:root >/dev/null 2>&1 || true
exec docker run --rm -it --name lwa4p_jazzy_hybrid \
  --network host --ipc host \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e ROBOT_DIR="${ROBOT_DIR}" -e HOST_IDS="$(id -u):$(id -g)" \
  -e ISAAC_INERTIALS="${ISAAC_INERTIALS:-datasheet}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  -v "${REPOSITORY}/src":/ws/src:ro \
  -v "${ROBOT_DIR}":"${ROBOT_DIR}":rw \
  lwa4p:jazzy bash -c 'source /opt/ros/jazzy/setup.bash && cd /ws &&
    colcon build --event-handlers console_cohesion- >/tmp/build.log 2>&1 \
      || { tail -30 /tmp/build.log; exit 1; }
    source install/setup.bash && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp &&
    ros2 run manipulation_isaac_sim make_isaac_urdf --output-dir "${ROBOT_DIR}" \
      --inertials "${ISAAC_INERTIALS}" >/dev/null &&
    chown -R "${HOST_IDS}" "${ROBOT_DIR}" &&
    echo "[run_jazzy_hybrid] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, robot files in ${ROBOT_DIR}," \
      "inertials ${ISAAC_INERTIALS}" &&
    exec "${@:-bash}"' _ "$@"
