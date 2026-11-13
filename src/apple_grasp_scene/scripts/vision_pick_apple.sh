#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/song/workspace/ros2_kortex_ws/install/setup.bash
source /home/song/桌面/apple_grasp_safety_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

execute=false
path_mode=auto
while [[ $# -gt 0 ]]; do
  case "$1" in
    --execute) execute=true; shift ;;
    --path) path_mode="${2:-}"; shift 2 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
if [[ "$path_mode" != auto && "$path_mode" != vertical && "$path_mode" != dogleg ]]; then
  echo "--path must be auto, vertical, or dogleg" >&2
  exit 2
fi

locator_output="$(ros2 run apple_grasp_scene stable_apple_locator.sh)"
echo "${locator_output}"
read -r apple_x apple_y apple_z radius wrist_x wrist_y points residual < <(
  /usr/bin/python3 -c '
import json,sys
d=json.load(sys.stdin); a=d["apple_base_m"]; w=d["grasp_wrist_m"]
print(a["x"],a["y"],a["z"],d["apple_radius_m"],w["x"],w["y"],d["fit_points"],d["fit_median_residual_m"])
' <<<"${locator_output}"
)

/usr/bin/python3 -c '
import sys
x,y,z,r,wx,wy,points,res=map(float,sys.argv[1:])
checks=[
 (0.55<=x<=0.80,"apple x outside verified workspace"),
 (-0.45<=y<=-0.10,"apple y outside verified workspace"),
 (0.025<=z<=0.060,"apple center height is implausible"),
 (0.025<=r<=0.055,"detected apple radius is implausible"),
 (0.54<=wx<=0.78 and -0.43<=wy<=-0.08,"wrist target outside verified workspace"),
 (points>=1000,"insufficient fitted depth points"),
 (res<=0.003,"sphere-fit residual is too large")]
failed=[msg for ok,msg in checks if not ok]
if failed:
 print("VISION_SAFETY_REJECTED: "+"; ".join(failed),file=sys.stderr); raise SystemExit(3)
print("VISION_SAFETY_ACCEPTED")
' "$apple_x" "$apple_y" "$apple_z" "$radius" "$wrist_x" "$wrist_y" "$points" "$residual"

move_to() {
  local x="$1" y="$2" z="$3" authorized="$4"
  ros2 run apple_grasp_scene scene_plan_only --ros-args \
    -p target_x:="$x" -p target_y:="$y" -p target_z:="$z" \
    -p execute_authorized:="$authorized"
}

# Dogleg approaches from the side facing the original observation axis, never
# from the table edge.  Auto mode uses it for targets displaced by >75 mm.
read -r selected_path side_y lateral_distance < <(
  /usr/bin/python3 -c '
import math,sys
mode,x,y=sys.argv[1],float(sys.argv[2]),float(sys.argv[3])
d=math.hypot(x-0.680,y+0.260)
selected=("dogleg" if d>0.075 else "vertical") if mode=="auto" else mode
side=y+(0.080 if y < -0.260 else -0.080)
side=max(-0.41,min(-0.10,side))
print(selected,side,d)
' "$path_mode" "$wrist_x" "$wrist_y"
)
echo "PATH_SELECTED=${selected_path} LATERAL_DISTANCE_M=${lateral_distance}"

preview_route() {
  if [[ "$selected_path" == vertical ]]; then
    echo "PREVIEW: vertical high alignment"
    move_to "$wrist_x" "$wrist_y" 0.250 false
  else
    echo "PREVIEW: dogleg safe-side waypoint"
    move_to "$wrist_x" "$side_y" 0.300 false
  fi
}

execute_route() {
  if [[ "$selected_path" == vertical ]]; then
    echo "VISION PICK [vertical]: high alignment"
    move_to "$wrist_x" "$wrist_y" 0.250 true
  else
    echo "VISION PICK [dogleg]: safe-side waypoint"
    move_to "$wrist_x" "$side_y" 0.300 true
    echo "VISION PICK [dogleg]: diagonal pre-grasp"
    move_to "$wrist_x" "$wrist_y" 0.200 true
  fi
  echo "VISION PICK: final vertical approach"
  move_to "$wrist_x" "$wrist_y" 0.150 true
}

if [[ "$execute" != true ]]; then
  echo "DRY RUN: previewing selected route entry; no motion or gripper command."
  preview_route
  echo "VISION_PICK_DRY_RUN_COMPLETE"
  exit 0
fi

if [[ "${APPLE_VISION_PICK_EXECUTE:-}" != "YES" ]]; then
  echo "Refusing real motion: set APPLE_VISION_PICK_EXECUTE=YES and pass --execute." >&2
  exit 2
fi

execute_route
echo "VISION PICK: force-limited close"
gripper_output="$(ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  '{command: {position: 0.8, max_effort: 10.0}}' --feedback)"
echo "$gripper_output"

GRIPPER_RESULT="$gripper_output" /usr/bin/python3 -c '
import os,re,sys
s=os.environ["GRIPPER_RESULT"]
p_all=re.findall(r"position:\s*([0-9.]+)",s)
p=p_all[-1] if p_all else None
stalled=re.search(r"stalled:\s*(true|false)",s)
if not p or not stalled or stalled.group(1)!="true" or float(p)>=0.70:
 print("GRASP_CONTACT_NOT_DETECTED; lift aborted",file=sys.stderr); raise SystemExit(4)
print("GRASP_CONTACT_CONFIRMED position="+p)
'

echo "VISION PICK: verification lift"
move_to "$wrist_x" "$wrist_y" 0.200 true
echo "VISION_PICK_SUCCESS"
