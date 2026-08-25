#include <chrono>
#include <cstddef>
#include <cstdio>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "auto_aim_interfaces/msg/vision.hpp"
#include "serial_main.h"

namespace rm_auto_aim
{

class VisionPub : public rclcpp::Node
{
public:
  explicit VisionPub(
    const rclcpp::NodeOptions & options =
    rclcpp::NodeOptions())
  : Node("vision_pub", options)
  {
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    publisher_ =
      this->create_publisher<
      auto_aim_interfaces::msg::Vision>(
      "/Vision_data", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&VisionPub::TimerCallback, this));

    if (!serial_.Ready()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Serial device is not ready.");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "VisionPub node started.");
  }

private:
  void TimerCallback()
  {
    if (!serial_.ReceiveVision()) {
      return;
    }

    const auto & data = serial_.VisionData();

    auto_aim_interfaces::msg::Vision message;

    message.header.frame_id = "vision";
    message.header.stamp = this->now();

    message.id = data.id;
    message.mode = data.mode;

    message.yaw = data.yaw;
    message.yaw_vel = data.yaw_vel;

    message.pitch = data.pitch;
    message.pitch_vel = data.pitch_vel;

    message.roll = data.roll;

    for (
      std::size_t i = 0;
      i < message.quaternion.size();
      ++i)
    {
      message.quaternion[i] =
        data.quaternion[i];
    }

    message.shoot_speed = data.shoot_speed;
    message.bullet_count = data.bullet_count;
    message.game_progress = data.game_progress;

    publisher_->publish(message);
  }

  SerialMain serial_;

  rclcpp::Publisher<
    auto_aim_interfaces::msg::Vision>::SharedPtr
    publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(
  rm_auto_aim::VisionPub)

