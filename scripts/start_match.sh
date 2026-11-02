#!/usr/bin/env bash
# Match startup: wait for the gimbal link and camera, then launch camera + perception + aim with respawn.
# Usage: bash scripts/start_match.sh [--config PATH] [--yolo NAME]
# Env: BOF_CPUS="2-5" pins the vision processes to those cores (leaves the rest to lidar/navigation).
set -euo pipefail
cd -P "$(dirname "${BASH_SOURCE[0]}")/.."
config="$PWD/src/bof_bringup/config/match.yaml"; yolo=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --config) config="$(realpath "$2")"; shift 2 ;;
    --yolo) yolo="$2"; shift 2 ;;
    *) echo "unknown argument $1" >&2; exit 2 ;;
  esac
done
log_dir="$PWD/artifacts/logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$log_dir"
export ROS_LOG_DIR="$log_dir"
source scripts/env.sh
# Fail once with a clear message instead of a respawn loop when the build lacks the camera node.
if [[ ! -x "$PWD/install/bof_vision/lib/bof_vision/hik_camera_node" ]]; then
  echo "hik_camera_node is not built. Rebuild with: bash scripts/build.sh -DBOF_WITH_MVS=ON -DMVS_ROOT=... -DBOF_WITH_OPENVINO=ON -DOpenVINO_DIR=..." | tee -a "$log_dir/start.log" >&2
  exit 78   # EX_CONFIG; the systemd unit does not restart on it
fi
# Serial port from the merged configuration (follows extends), default the udev name.
port="$(python3 - "$config" <<'PY' 2>/dev/null || true
import pathlib, sys, yaml
def merged(path):
    data = yaml.safe_load(open(path)) or {}
    base = merged(pathlib.Path(path).parent / data['extends']) if data.get('extends') else {}
    port = (data.get('aim') or {}).get('gimbal_link', {}).get('serial_port')
    return port or base
print(merged(sys.argv[1]) or '')
PY
)"
port="${port:-/dev/bof_gimbal}"
for _ in $(seq 60); do [[ -e "$port" ]] && break; echo "waiting for $port" | tee -a "$log_dir/start.log"; sleep 1; done
[[ -e "$port" ]] || echo "gimbal link $port not present; starting anyway (aim_node keeps retrying)" | tee -a "$log_dir/start.log"
args=(bof_bringup vision.launch.py "config:=$config" source:=hik aim:=true respawn:=true)
[[ -n "$yolo" ]] && args+=("yolo:=$yolo")
echo "$(date) ros2 launch ${args[*]} (logs in $log_dir)" | tee -a "$log_dir/start.log"
if [[ -n "${BOF_CPUS:-}" ]]; then
  taskset -c "$BOF_CPUS" ros2 launch "${args[@]}" 2>&1 | tee -a "$log_dir/launch.log"
else
  ros2 launch "${args[@]}" 2>&1 | tee -a "$log_dir/launch.log"
fi
