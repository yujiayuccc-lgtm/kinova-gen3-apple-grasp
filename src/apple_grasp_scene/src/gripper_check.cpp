#include <chrono>
#include <memory>

#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("apple_gripper_readiness_check");
  using GripperCommand = control_msgs::action::GripperCommand;
  auto client = rclcpp_action::create_client<GripperCommand>(
    node, "/robotiq_gripper_controller/gripper_cmd");
  const bool ready = client->wait_for_action_server(std::chrono::seconds(8));
  if (ready) {
    RCLCPP_INFO(node->get_logger(), "GRIPPER_ACTION_READY; no command was sent.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "GRIPPER_ACTION_UNAVAILABLE; no command was sent.");
  }
  rclcpp::shutdown();
  return ready ? 0 : 1;
}
