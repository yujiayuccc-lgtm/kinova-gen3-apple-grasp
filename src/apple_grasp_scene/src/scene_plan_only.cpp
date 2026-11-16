#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace
{
moveit_msgs::msg::CollisionObject box(
  const std::string& id, double x, double y, double z,
  double size_x, double size_y, double size_z)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "base_link";
  object.id = id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {size_x, size_y, size_z};

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

moveit_msgs::msg::CollisionObject sphere(
  const std::string& id, double x, double y, double z, double radius)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "base_link";
  object.id = id;

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  primitive.dimensions = {radius};

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}
}  // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "apple_grasp_scene_plan_only",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  int exit_code = 1;
  try {
    const bool execute_authorized = node->get_parameter_or<bool>("execute_authorized", false);
    const double target_z = node->get_parameter_or<double>("target_z", 0.250);
    auto display_publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", rclcpp::QoS(1).reliable().transient_local());
    moveit::planning_interface::PlanningSceneInterface scene;

    // Coordinate convention verified on site: +X front, +Y left, +Z up. The table top is
    // kept 5 mm below base_link z=0 to avoid numerical contact with the base.
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(box("work_table", -0.20, -0.20, -0.025, 0.80, 1.60, 0.04));

    // Rear cabinet: its front face is 0.50 m behind the robot base.
    objects.push_back(box("rear_cabinet", -0.60, 0.00, 0.75, 0.20, 1.00, 1.50));

    // Right cabinet/wall: its nearest face is 0.80 m right of the base.
    objects.push_back(box("right_wall", 0.20, -0.85, 0.75, 0.80, 0.10, 1.50));

    // The blocks were physically removed. Remove any stale keepout object left
    // in the planning scene from an earlier run.
    scene.removeCollisionObjects({"blocks_keepout"});

    // The apple is the intended contact target during this final approach, so it
    // must not be treated as an obstacle.  All environmental keep-outs remain.
    scene.removeCollisionObjects({"apple"});

    if (!scene.applyCollisionObjects(objects)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to apply collision objects.");
      throw std::runtime_error("planning scene update failed");
    }

    const auto known = scene.getKnownObjectNames();
    RCLCPP_INFO(node->get_logger(), "Applied %zu collision objects:", known.size());
    for (const auto& name : known) {
      RCLCPP_INFO(node->get_logger(), "  - %s", name.c_str());
    }

    moveit::planning_interface::MoveGroupInterface move_group(node, "manipulator");
    move_group.setPlanningTime(10.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);
    move_group.setStartStateToCurrentState();

    // Start from the live wrist pose so the preview preserves its exact current
    // orientation instead of asking IK for a visually equivalent wrist rotation.
    move_group.setPoseReferenceFrame("base_link");
    const auto current_pose = move_group.getCurrentPose("end_effector_link").pose;
    const double target_x = node->get_parameter_or<double>("target_x", current_pose.position.x);
    const double target_y = node->get_parameter_or<double>("target_y", current_pose.position.y);

    geometry_msgs::msg::Pose hover = current_pose;
    // Post-release retreat: raise the open gripper 100 mm vertically while
    // preserving the live wrist orientation and horizontal position.
    hover.position.x = target_x;
    hover.position.y = target_y;
    hover.position.z = target_z;
    hover.orientation.x = node->get_parameter_or<double>(
      "target_qx", current_pose.orientation.x);
    hover.orientation.y = node->get_parameter_or<double>(
      "target_qy", current_pose.orientation.y);
    hover.orientation.z = node->get_parameter_or<double>(
      "target_qz", current_pose.orientation.z);
    hover.orientation.w = node->get_parameter_or<double>(
      "target_qw", current_pose.orientation.w);
    const double quaternion_norm = std::sqrt(
      hover.orientation.x * hover.orientation.x +
      hover.orientation.y * hover.orientation.y +
      hover.orientation.z * hover.orientation.z +
      hover.orientation.w * hover.orientation.w);
    if (quaternion_norm < 0.99 || quaternion_norm > 1.01) {
      throw std::runtime_error("target orientation quaternion is not normalized");
    }
    {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const bool use_pose_planning = node->get_parameter_or<bool>(
        "use_pose_planning", false);
      const std::string named_target = node->get_parameter_or<std::string>(
        "named_target", "");
      const std::vector<double> joint_target =
        node->get_parameter_or<std::vector<double>>(
          "joint_target", std::vector<double>{});
      double cartesian_fraction = 0.0;
      bool planning_succeeded = false;
      if (!joint_target.empty()) {
        if (joint_target.size() != 7) {
          throw std::runtime_error("joint_target must contain exactly 7 values");
        }
        if (!move_group.setJointValueTarget(joint_target)) {
          throw std::runtime_error("joint_target violates robot joint bounds");
        }
        planning_succeeded =
          move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
      } else if (!named_target.empty()) {
        if (!move_group.setNamedTarget(named_target)) {
          throw std::runtime_error("named target was not found: " + named_target);
        }
        planning_succeeded =
          move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
      } else if (use_pose_planning) {
        move_group.setPoseTarget(hover, "end_effector_link");
        planning_succeeded =
          move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS;
        move_group.clearPoseTargets();
      } else {
        cartesian_fraction = move_group.computeCartesianPath(
          std::vector<geometry_msgs::msg::Pose>{hover}, 0.0025, 0.0,
          plan.trajectory_, true);
        planning_succeeded = cartesian_fraction >= 0.999;
      }
      const auto live_state = move_group.getCurrentState();
      moveit::core::robotStateToRobotStateMsg(*live_state, plan.start_state_);
      if (planning_succeeded) {
        const auto& joint_trajectory = plan.trajectory_.joint_trajectory;
        bool excessive_joint_motion = joint_trajectory.points.empty();
        double total_joint_motion = 0.0;
        double largest_joint_motion = 0.0;
        if (!joint_trajectory.points.empty()) {
          const auto& start = joint_trajectory.points.front().positions;
          const auto& finish = joint_trajectory.points.back().positions;
          for (std::size_t i = 0; i < start.size() && i < finish.size(); ++i) {
            const double delta = std::remainder(finish[i] - start[i], 2.0 * M_PI);
            const double magnitude = std::abs(delta);
            total_joint_motion += magnitude;
            largest_joint_motion = std::max(largest_joint_motion, magnitude);
            RCLCPP_INFO(
              node->get_logger(), "%s endpoint change: %.1f deg",
              joint_trajectory.joint_names[i].c_str(), delta * 180.0 / M_PI);
          }
          // Reject a solution if any single joint rotates more than 60 degrees.
          excessive_joint_motion = largest_joint_motion > (M_PI / 3.0);
        }
        RCLCPP_INFO(
          node->get_logger(), "Joint-motion check: largest %.1f deg, total %.1f deg.",
          largest_joint_motion * 180.0 / M_PI, total_joint_motion * 180.0 / M_PI);
        if (excessive_joint_motion) {
          RCLCPP_ERROR(node->get_logger(), "Rejected plan: excessive joint rotation; nothing was executed.");
          throw std::runtime_error("joint-motion safety check failed");
        }

        // Cartesian interpolation contains positions only. Add conservative
        // timestamps before either preview or controller execution.
        robot_trajectory::RobotTrajectory timed_trajectory(
          move_group.getRobotModel(), "manipulator");
        timed_trajectory.setRobotTrajectoryMsg(*live_state, plan.trajectory_);
        trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
        if (!time_parameterization.computeTimeStamps(timed_trajectory, 0.05, 0.05)) {
          throw std::runtime_error("5-percent trajectory time parameterization failed");
        }
        timed_trajectory.getRobotTrajectoryMsg(plan.trajectory_);

        moveit_msgs::msg::DisplayTrajectory display;
        display.model_id = move_group.getRobotModel()->getName();
        display.trajectory_start = plan.start_state_;
        display.trajectory.push_back(plan.trajectory_);
        display_publisher->publish(display);
        if (!joint_target.empty()) {
          RCLCPP_INFO(
            node->get_logger(), "JOINT_TARGET_PLAN_SUCCESS: %zu points.",
            plan.trajectory_.joint_trajectory.points.size());
        } else if (!named_target.empty()) {
          RCLCPP_INFO(
            node->get_logger(),
            "NAMED_TARGET_PLAN_SUCCESS: target=%s, %zu points.",
            named_target.c_str(), plan.trajectory_.joint_trajectory.points.size());
        } else if (use_pose_planning) {
          RCLCPP_INFO(
            node->get_logger(),
            "POSE_PLAN_SUCCESS: target=(%.3f, %.3f, %.3f) m, %zu points.",
            target_x, target_y, target_z,
            plan.trajectory_.joint_trajectory.points.size());
        } else {
          RCLCPP_INFO(
            node->get_logger(),
            "CARTESIAN_PLAN_SUCCESS: target=(%.3f, %.3f, %.3f) m, %zu points, %.1f%% path.",
            target_x, target_y, target_z,
            plan.trajectory_.joint_trajectory.points.size(), cartesian_fraction * 100.0);
        }
        RCLCPP_INFO(node->get_logger(), "Published trajectory animation to RViz.");
        std::this_thread::sleep_for(std::chrono::seconds(2));

        if (!execute_authorized) {
          RCLCPP_INFO(node->get_logger(), "Preview only; trajectory was NOT executed.");
          exit_code = 0;
        } else {
          using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
          auto controller_client = rclcpp_action::create_client<FollowJointTrajectory>(
            node, "/joint_trajectory_controller/follow_joint_trajectory");
          if (!controller_client->wait_for_action_server(std::chrono::seconds(5))) {
            throw std::runtime_error("joint trajectory controller action is unavailable");
          }

          FollowJointTrajectory::Goal goal;
          goal.trajectory = plan.trajectory_.joint_trajectory;
          RCLCPP_WARN(
            node->get_logger(),
            "AUTHORIZED REAL MOTION: sending safety-checked path at 5%% limits.");
          auto goal_future = controller_client->async_send_goal(goal);
          if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            throw std::runtime_error("controller did not acknowledge trajectory goal");
          }
          const auto goal_handle = goal_future.get();
          if (!goal_handle) {
            throw std::runtime_error("controller rejected trajectory goal");
          }
          auto result_future = controller_client->async_get_result(goal_handle);
          if (result_future.wait_for(std::chrono::seconds(120)) != std::future_status::ready) {
            controller_client->async_cancel_goal(goal_handle);
            throw std::runtime_error("controller trajectory timed out and was cancelled");
          }
          const auto wrapped_result = result_future.get();
          if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
              wrapped_result.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL) {
              RCLCPP_INFO(node->get_logger(), "VERTICAL_DESCENT_EXECUTION_SUCCESS");
              exit_code = 0;
          } else {
            RCLCPP_ERROR(
              node->get_logger(), "Controller execution failed, action_code=%d, error_code=%d",
              static_cast<int>(wrapped_result.code), wrapped_result.result->error_code);
          }
        }
      } else {
        if (!joint_target.empty()) {
          RCLCPP_ERROR(
            node->get_logger(), "Joint-target planning failed; nothing was executed.");
        } else if (!named_target.empty()) {
          RCLCPP_ERROR(
            node->get_logger(), "Named-target planning failed; nothing was executed.");
        } else if (use_pose_planning) {
          RCLCPP_ERROR(node->get_logger(), "Pose planning failed; nothing was executed.");
        } else {
          RCLCPP_ERROR(
            node->get_logger(), "Vertical Cartesian path incomplete (%.1f%%); nothing was executed.",
            cartesian_fraction * 100.0);
        }
      }
    }
  } catch (const std::exception& error) {
    RCLCPP_ERROR(node->get_logger(), "Scene plan-only test failed: %s", error.what());
  }

  rclcpp::shutdown();
  if (spinner.joinable()) {
    spinner.join();
  }
  return exit_code;
}
