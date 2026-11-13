#!/usr/bin/env bash
set -eo pipefail

# Repeatable apple pick/place sequence for the verified workstation geometry.
# Safe default: without --execute, only the first approach is planned and no
# arm or gripper command is sent.

EXECUTE=false
if [[ "${1:-}" == "--execute" ]]; then
  EXECUTE=true
fi

source /opt/ros/humble/setup.bash
source /home/song/workspace/ros2_kortex_ws/install/setup.bash
source /home/song/桌面/apple_grasp_safety_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set -u

move_z() {
  local target_z="$1"
  ros2 run apple_grasp_scene scene_plan_only --ros-args \
    -p target_z:="${target_z}" -p execute_authorized:="${EXECUTE}"
}

gripper() {
  local position="$1"
  local effort="$2"
  ros2 action send_goal \
    /robotiq_gripper_controller/gripper_cmd \
    control_msgs/action/GripperCommand \
    "{command: {position: ${position}, max_effort: ${effort}}}"
}

if [[ "${EXECUTE}" != "true" ]]; then
  echo "DRY RUN: planning approach to z=0.150 m; no commands will be executed."
  move_z 0.150
  echo "DRY RUN COMPLETE. Use --execute only after the apple and workspace are verified."
  exit 0
fi

if [[ "${APPLE_GRASP_EXECUTE:-}" != "YES" ]]; then
  echo "Refusing real motion: set APPLE_GRASP_EXECUTE=YES and pass --execute." >&2
  exit 2
fi

echo "REAL SEQUENCE: approach"
move_z 0.150
echo "REAL SEQUENCE: force-limited close"
gripper 0.8 10.0
echo "REAL SEQUENCE: verification lift"
move_z 0.200
echo "REAL SEQUENCE: placement descent"
move_z 0.150
echo "REAL SEQUENCE: release"
gripper 0.0 5.0
echo "REAL SEQUENCE: retreat"
move_z 0.250
echo "APPLE PICK/PLACE SEQUENCE COMPLETE"
