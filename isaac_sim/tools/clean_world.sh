#!/usr/bin/env bash
# Run clean_world.py with the USD libraries shipped with Isaac Sim; the simulator does not start.
# Any other Python with the pxr module (for example pip's usd-core) runs clean_world.py as well.
#
# Usage: ISAACSIM_PATH=<Isaac Sim directory> isaac_sim/tools/clean_world.sh \
#            <converted.usd> <Tomas's world.sdf> [<output.usd>]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -z "${ISAACSIM_PATH:-}" || ! -x "${ISAACSIM_PATH}/python.sh" ]]; then
  echo "Set ISAACSIM_PATH to the Isaac Sim directory (the one with python.sh)." >&2
  exit 1
fi
usd_libs=""
for candidate in "${ISAACSIM_PATH}"/{extscache,extsDeps,kit/exts}/omni.usd.libs-*; do
  if [[ -d "${candidate}/pxr" ]]; then
    usd_libs="${candidate}"
  fi
done
if [[ -z "${usd_libs}" ]]; then
  echo "No omni.usd.libs extension with pxr found under ${ISAACSIM_PATH}." >&2
  exit 1
fi
exec env PYTHONPATH="${usd_libs}" LD_LIBRARY_PATH="${usd_libs}/bin" \
  "${ISAACSIM_PATH}/python.sh" "${HERE}/clean_world.py" "$@"
