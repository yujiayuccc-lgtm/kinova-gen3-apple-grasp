#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/song/workspace/ros2_kortex_ws/install/setup.bash
source /home/song/桌面/apple_grasp_safety_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [[ "${1:-}" != "--execute" || "${APPLE_TWO_PATH_EXECUTE:-}" != "YES" ]]; then
  echo "Refusing motion: set APPLE_TWO_PATH_EXECUTE=YES and pass --execute." >&2
  exit 2
fi

move_z() {
  ros2 run apple_grasp_scene scene_plan_only --ros-args \
    -p target_z:="$1" -p execute_authorized:=true
}

move_xyz() {
  ros2 run apple_grasp_scene scene_plan_only --ros-args \
    -p target_x:="$1" -p target_y:="$2" -p target_z:="$3" \
    -p execute_authorized:=true
}

release() {
  ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
    control_msgs/action/GripperCommand \
    '{command: {position: 0.0, max_effort: 5.0}}' --feedback
}

place_and_return_to_observation() {
  echo "CYCLE: lower held apple to table"
  move_z 0.150
  echo "CYCLE: release apple"
  release
  echo "CYCLE: vertical retreat"
  move_z 0.250
  echo "CYCLE: return horizontally to observation axis"
  move_xyz 0.680 -0.260 0.250
  echo "CYCLE: rise to observation height"
  move_xyz 0.680 -0.260 0.400
}

export APPLE_VISION_PICK_EXECUTE=YES

echo "INITIAL: place currently held apple"
place_and_return_to_observation

echo "TEST 1/2: VERTICAL PATH"
ros2 run apple_grasp_scene vision_pick_apple.sh --path vertical --execute
echo "TEST 1/2: place result"
place_and_return_to_observation

echo "TEST 2/2: DOGLEG PATH"
ros2 run apple_grasp_scene vision_pick_apple.sh --path dogleg --execute
echo "TEST 2/2: place result"
place_and_return_to_observation

echo "TWO_PATH_AUTOMATIC_TEST_SUCCESS"
