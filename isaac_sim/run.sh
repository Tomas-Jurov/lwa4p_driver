#!/usr/bin/env bash
# Start the Isaac Sim backend: generate the URDF from Tomas's model, then run sim.py
# with Isaac Sim's own Python and its bundled ROS 2 Jazzy libraries.
#
# Usage: ISAACSIM_PATH=<Isaac Sim 6.2 directory> isaac_sim/run.sh [--headless] [--max-time S]
# Needs the workspace built: colcon build (in the repository root).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPOSITORY="$(dirname "${HERE}")"
CACHE="${HERE}/.cache"

if [[ -z "${ISAACSIM_PATH:-}" || ! -x "${ISAACSIM_PATH}/python.sh" ]]; then
  echo "Set ISAACSIM_PATH to the Isaac Sim 6.2 directory (the one with python.sh)." >&2
  exit 1
fi
if [[ ! -f "${REPOSITORY}/install/setup.bash" ]]; then
  echo "Build the workspace first: cd ${REPOSITORY} && colcon build" >&2
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

robot_manifest="$(ros_side ros2 run manipulation_isaac_sim make_isaac_urdf \
  --output-dir "${CACHE}/urdf")"
controllers="$(ros_side ros2 pkg prefix --share manipulation_isaac_sim)/config/isaac_controllers.yaml"

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
