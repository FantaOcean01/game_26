#include <cstdio>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "auto_aim_interfaces/msg/robot_ctrl.hpp"
#include "protocol_new.hpp"
#include "serial_main.h"

namespace rm_auto_aim
{

class RobotCtrlSub : public rclcpp::Node
{
public:
  explicit RobotCtrlSub(
    const rclcpp::NodeOptions & options =
    rclcpp::NodeOptions())
  : Node("robot_ctrl", options)
  {
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    subscription_ =
      this->create_subscription<
      auto_aim_interfaces::msg::RobotCtrl>(
      "/Robot_ctrl_data",
      10,
      std::bind(
        &RobotCtrlSub::RobotCtrlSend,
        this,
        std::placeholders::_1));

    if (!serial_.Ready()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Serial device is not ready.");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "RobotCtrlSub node started.");
  }

private:
  void RobotCtrlSend(
    const auto_aim_interfaces::msg::RobotCtrl::
    ConstSharedPtr & message)
  {
    io::RobotCtrlData control{};

    control.yaw = message->yaw;
    control.yaw_vel = message->yaw_vel;
    control.yaw_acc = message->yaw_acc;

    control.pitch = message->pitch;
    control.pitch_vel = message->pitch_vel;
    control.pitch_acc = message->pitch_acc;

    control.target_lock = message->target_lock;
    control.fire_command = message->fire_command;

    if (!serial_.SendControl(control)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Failed to send control frame.");
    }
  }

  SerialMain serial_;

  rclcpp::Subscription<
    auto_aim_interfaces::msg::RobotCtrl>::SharedPtr
    subscription_;
};

}  // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(
  rm_auto_aim::RobotCtrlSub)
