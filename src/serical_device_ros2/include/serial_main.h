#ifndef SERICAL_DEVICE_ROS2__SERIAL_MAIN_H_
#define SERICAL_DEVICE_ROS2__SERIAL_MAIN_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "crc.h"
#include "protocol_new.hpp"
#include "serial_device.h"

class SerialMain
{
public:
  explicit SerialMain(
    const std::string & device_path = "/dev/robomaster");

  ~SerialMain() = default;

  // 电控串口帧 -> VisionData
  bool ReceiveVision();

  // RobotCtrlData -> 串口控制帧
  bool SendControl(const io::RobotCtrlData & control);

  bool Ready() const noexcept
  {
    return ready_;
  }

  const io::VisionData & VisionData() const noexcept
  {
    return vision_data_;
  }

private:
  static constexpr std::size_t BUFFER_SIZE = 512;

  bool Initialize();

  // 确保读取到指定数量的字节
  bool ReadExact(uint8_t * destination, std::size_t length);

  // 从连续串口流中读取一整帧
  bool ReadFrame(std::size_t & frame_length);

  // 校验并解析VisionData
  bool DecodeVisionFrame(std::size_t frame_length);

  // 将RobotCtrlData打包到send_buffer_
  std::size_t PackControlFrame(
    const io::RobotCtrlData & control);

  std::string device_path_;
  std::unique_ptr<SerialDevice> device_;

  std::array<uint8_t, BUFFER_SIZE> receive_buffer_{};
  std::array<uint8_t, BUFFER_SIZE> send_buffer_{};

  io::FrameHeader receive_header_{};
  io::FrameHeader send_header_{};

  io::VisionData vision_data_{};

  bool ready_ = false;
};

#endif  // SERICAL_DEVICE_ROS2__SERIAL_MAIN_H_
