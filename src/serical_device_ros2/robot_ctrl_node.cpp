#include <cstdio>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "auto_aim_interfaces/msg/robot_ctrl.hpp"
#include "serial_main.h"

namespace rm_auto_aim
{

class RobotCtrlSub : public rclcpp::Node
{
public:
  explicit RobotCtrlSub(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("robot_ctrl", options)
  {
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    subscription_ =
      this->create_subscription<
      auto_aim_interfaces::msg::RobotCtrl>(
      "/Robot_ctrl_data",
      10,
      std::bind(
        &RobotCtrlSub::robotCtrlSend,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "--- RobotCtrlSub Node Started ---");
  }

private:
  void robotCtrlSend(
    const auto_aim_interfaces::msg::RobotCtrl::ConstSharedPtr & message)
  {
    io::RobotCtrlData command{};

    command.yaw = message->yaw;
    command.yaw_vel = message->yaw_vel;
    command.yaw_acc = message->yaw_acc;

    command.pitch = message->pitch;
    command.pitch_vel = message->pitch_vel;
    command.pitch_acc = message->pitch_acc;

    command.target_lock = message->target_lock;
    command.fire_command = message->fire_command;

    serial_.SenderMain(command);
  }

  SerialMain serial_;

  rclcpp::Subscription<
    auto_aim_interfaces::msg::RobotCtrl>::SharedPtr subscription_;
};

}  // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RobotCtrlSub)
