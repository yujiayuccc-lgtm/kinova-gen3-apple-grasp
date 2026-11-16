#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/song/workspace/ros2_kortex_ws/install/setup.bash
source /home/song/workspace/apple_grasp_safety_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set -euo pipefail

execute=false
if [[ "${1:-}" == "--execute" ]]; then
  execute=true
elif [[ $# -ne 0 ]]; then
  echo "Usage: restore_observation_pose.sh [--execute]" >&2
  exit 2
fi

config="$(ros2 pkg prefix apple_grasp_scene)/share/apple_grasp_scene/config/saved_observation_pose.yaml"
joint_target="$(/usr/bin/python3 -c '
import sys, yaml
with open(sys.argv[1], encoding="utf-8") as stream:
    values = yaml.safe_load(stream)["saved_observation_pose"]["joint_positions_rad"]
if len(values) != 7:
    raise SystemExit("saved pose must contain exactly seven joint positions")
print("[" + ",".join(str(float(value)) for value in values) + "]")
' "$config")"

if [[ "$execute" == true ]]; then
  if [[ "${APPLE_RESTORE_OBSERVATION_EXECUTE:-}" != "YES" ]]; then
    echo "Refusing real motion: set APPLE_RESTORE_OBSERVATION_EXECUTE=YES and pass --execute." >&2
    exit 2
  fi
  fault="$(timeout 8 ros2 topic echo /fault_controller/internal_fault --once)"
  if ! grep -q 'data: false' <<<"$fault"; then
    echo "Refusing real motion: robot fault state is unavailable or active." >&2
    exit 3
  fi
fi

ros2 run apple_grasp_scene scene_plan_only --ros-args \
  -p "joint_target:=${joint_target}" \
  -p "execute_authorized:=${execute}"

