#!/usr/bin/env bash
# Start the Isaac Sim backend: get the robot files generated from Tomas's model, then run
# sim.py with Isaac Sim's own Python and its bundled ROS 2 Jazzy libraries.
#
# Usage: ISAACSIM_PATH=<Isaac Sim 6.2 directory> isaac_sim/run.sh [--headless] [--max-time S]
# The robot files (.cache/robot) come from, in this order:
#   1. the running docker/run_jazzy_hybrid.sh container, which wrote them when it started;
#   2. ROS 2 Jazzy on the host with this workspace built: generated here on every start;
#   3. the files an earlier hybrid container left behind.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY="$(dirname "${HERE}")"
CACHE="${HERE}/.cache"

if [[ -z "${ISAACSIM_PATH:-}" || ! -x "${ISAACSIM_PATH}/python.sh" ]]; then
  echo "Set ISAACSIM_PATH to the Isaac Sim 6.2 directory (the one with python.sh)." >&2
  exit 1
fi
ros_side() {
  # setup.bash reads unset variables, so relax -u while sourcing.
  set +u
  source /opt/ros/jazzy/setup.bash
  source "${REPOSITORY}/install/setup.bash"
  set -u
  "$@"
}

hybrid_container_running() {
  [[ "$(docker container inspect -f '{{.State.Running}}' lwa4p_jazzy_hybrid 2>/dev/null)" \
    == true ]]
}

host_ros_ready() {
  # A leftover install/ (for example a symlink install without its build/) has setup.bash
  # but no usable packages, so ask ROS for the package instead of testing for files.
  [[ -f /opt/ros/jazzy/setup.bash && -f "${REPOSITORY}/install/setup.bash" ]] \
    && (ros_side ros2 pkg prefix manipulation_isaac_sim) >/dev/null 2>&1
}

robot_dir="${CACHE}/robot"
robot_manifest="${robot_dir}/manifest.json"
if hybrid_container_running && [[ -f "${robot_manifest}" ]]; then
  echo "[run.sh] robot files: written by the running lwa4p_jazzy_hybrid container in ${robot_dir}"
elif host_ros_ready; then
  echo "[run.sh] robot files: generated with the host's ROS 2 in ${robot_dir}"
  ros_side ros2 run manipulation_isaac_sim make_isaac_urdf --output-dir "${robot_dir}" >/dev/null
elif [[ -f "${robot_manifest}" ]]; then
  echo "[run.sh] robot files: left by an earlier docker/run_jazzy_hybrid.sh in ${robot_dir};" \
    "restart it if the model changed since"
else
  echo "No robot files in ${robot_dir}." >&2
  echo "Start docker/run_jazzy_hybrid.sh first, or build the workspace with ROS 2 Jazzy on" \
    "the host: cd ${REPOSITORY} && colcon build" >&2
  exit 1
fi
controllers="${robot_dir}/isaac_controllers.yaml"

# Isaac Sim's Python must not see the system ROS installation. Its ROS 2 bridge loads
# the Jazzy libraries shipped with Isaac Sim instead.
isaac_library_path="${ISAACSIM_PATH}/exts/isaacsim.ros2.core/jazzy/lib"
# Prefer the CUDA libraries shipped with Isaac Sim: an older system CUDA can lack
# symbols its PyTorch needs.
for cuda_directory in "${ISAACSIM_PATH}"/exts/isaacsim.pip.nv/pip_prebundle/nvidia/*/lib; do
  if [[ -d "${cuda_directory}" ]]; then
    isaac_library_path="${isaac_library_path}:${cuda_directory}"
  fi
done

exec env -u PYTHONPATH -u PYTHONHOME -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u COLCON_PREFIX_PATH -u ROS_VERSION -u ROS_PYTHON_VERSION \
  ROS_DISTRO=jazzy \
  RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  LD_LIBRARY_PATH="${isaac_library_path}" \
  OMNI_KIT_ACCEPT_EULA=YES \
  PYTHONUNBUFFERED=1 \
  "${ISAACSIM_PATH}/python.sh" "${HERE}/sim.py" \
  --robot-manifest "${robot_manifest}" \
  --controllers "${controllers}" \
  --config "${HERE}/config/sim.yaml" \
  --cache-dir "${CACHE}/usd" \
  "$@"
