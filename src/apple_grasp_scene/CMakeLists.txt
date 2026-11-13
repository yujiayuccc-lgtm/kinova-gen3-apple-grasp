cmake_minimum_required(VERSION 3.8)
project(apple_grasp_scene)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(scene_plan_only src/scene_plan_only.cpp)
target_compile_features(scene_plan_only PUBLIC cxx_std_17)
add_executable(gripper_check src/gripper_check.cpp)
target_compile_features(gripper_check PUBLIC cxx_std_17)
ament_target_dependencies(
  scene_plan_only
  control_msgs
  moveit_core
  moveit_ros_planning_interface
  rclcpp
  rclcpp_action
  shape_msgs
)
ament_target_dependencies(
  gripper_check
  control_msgs
  rclcpp
  rclcpp_action
)

install(TARGETS scene_plan_only gripper_check DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/apple_pick_place.sh
  scripts/apple_locator.py
  scripts/stable_apple_locator.py
  scripts/stable_apple_locator.sh
  scripts/locate_apple.sh
  scripts/vision_pick_apple.sh
  scripts/test_both_pick_paths.sh
  scripts/search_pick_place_apple.sh
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
