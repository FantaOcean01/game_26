#include <chrono>
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
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("vision_pub", options)
  {
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    publisher_ =
      this->create_publisher<auto_aim_interfaces::msg::Vision>(
      "/Vision_data", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&VisionPub::timer_callback, this));

    RCLCPP_INFO(
      this->get_logger(),
      "--- VisionPub Node Started ---");
  }

private:
  void timer_callback()
  {
    if (!serial_.ReceiverMain()) {
      return;
    }

    auto_aim_interfaces::msg::Vision message;

    message.header.frame_id = "vision";
    message.header.stamp = this->now();

    message.id = serial_.vision_msg_.id;
    message.mode = serial_.vision_msg_.mode;

    message.yaw = serial_.vision_msg_.yaw;
    message.yaw_vel = serial_.vision_msg_.yaw_vel;

    message.pitch = serial_.vision_msg_.pitch;
    message.pitch_vel = serial_.vision_msg_.pitch_vel;

    message.roll = serial_.vision_msg_.roll;

    for (std::size_t i = 0; i < message.quaternion.size(); ++i) {
      message.quaternion[i] =
        serial_.vision_msg_.quaternion[i];
    }

    message.shoot_speed =
      serial_.vision_msg_.shoot_speed;

    message.bullet_count =
      serial_.vision_msg_.bullet_count;

    message.game_progress =
      serial_.vision_msg_.game_progress;

    publisher_->publish(message);
  }

  SerialMain serial_;

  rclcpp::Publisher<
    auto_aim_interfaces::msg::Vision>::SharedPtr publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::VisionPub)
