#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/song/workspace/ros2_kortex_ws/install/setup.bash
source /home/song/桌面/apple_grasp_safety_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

if [[ "${1:-}" != "--execute" || "${APPLE_SEARCH_PICK_EXECUTE:-}" != "YES" ]]; then
  echo "Refusing motion: set APPLE_SEARCH_PICK_EXECUTE=YES and pass --execute." >&2
  exit 2
fi

move_xyz() {
  ros2 run apple_grasp_scene scene_plan_only --ros-args \
    -p target_x:="$1" -p target_y:="$2" -p target_z:="$3" \
    -p execute_authorized:=true
}

center() {
  move_xyz 0.680 -0.260 0.400
}

release() {
  ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
    control_msgs/action/GripperCommand \
    '{command: {position: 0.0, max_effort: 5.0}}' --feedback
}

found=false
found_name=""
points=(
  "center 0.680 -0.260 0.400"
  "right 0.620 -0.380 0.400"
  "left 0.640 -0.160 0.380"
  "near 0.600 -0.260 0.380"
  "near_extended 0.520 -0.260 0.400"
  "far 0.700 -0.260 0.320"
)

for spec in "${points[@]}"; do
  read -r name x y z <<<"$spec"
  if [[ "$name" != center ]]; then
    center
  fi
  echo "SEARCH_VISIT point=$name target=($x,$y,$z)"
  move_xyz "$x" "$y" "$z"
  if detection="$(ros2 run apple_grasp_scene stable_apple_locator.sh 2>&1)"; then
    echo "$detection"
    echo "SEARCH_FOUND point=$name"
    found=true
    found_name="$name"
    break
  fi
  echo "SEARCH_NOT_FOUND point=$name reason=$detection"
done

if [[ "$found" != true ]]; then
  center
  echo "SEARCH_FAILED: apple not found in verified viewpoints" >&2
  exit 5
fi

export APPLE_VISION_PICK_EXECUTE=YES
echo "SEARCH_PICK_START point=$found_name"
ros2 run apple_grasp_scene vision_pick_apple.sh --path auto --execute

echo "SEARCH_PLACE: lower apple"
ros2 run apple_grasp_scene scene_plan_only --ros-args \
  -p target_z:=0.150 -p execute_authorized:=true
echo "SEARCH_PLACE: release apple"
release
echo "SEARCH_PLACE: retreat"
ros2 run apple_grasp_scene scene_plan_only --ros-args \
  -p target_z:=0.250 -p execute_authorized:=true
echo "SEARCH_PLACE: return to center axis at retreat height"
move_xyz 0.680 -0.260 0.250
center
echo "SEARCH_PICK_PLACE_SUCCESS"
