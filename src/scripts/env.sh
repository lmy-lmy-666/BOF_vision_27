#!/usr/bin/env bash
# Source this file; keep the caller's current directory unchanged.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "Use: source src/scripts/env.sh" >&2
  exit 1
fi
# ROS setup scripts inspect optional unset variables. Preserve the caller's
# nounset setting while allowing those scripts to run (run_live uses set -u).
bof_load_ros_environment() {
  local bof_env_status=0 bof_env_nounset=0 bof_env_root
  bof_env_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd -P)" || return 1
  if [[ $- == *u* ]]; then bof_env_nounset=1; fi
  set +u
  if ! source /opt/ros/jazzy/setup.bash; then
    bof_env_status=1
  elif [[ ! -f "$bof_env_root/install/setup.bash" ]]; then
    echo "Workspace is not built. Run: bash src/scripts/build.sh" >&2
    bof_env_status=1
  elif ! source "$bof_env_root/install/setup.bash"; then
    bof_env_status=1
  fi
  if [[ "$bof_env_nounset" == 1 ]]; then set -u; fi
  return "$bof_env_status"
}
if bof_load_ros_environment; then
  unset -f bof_load_ros_environment
  return 0
else
  unset -f bof_load_ros_environment
  return 1
fi
