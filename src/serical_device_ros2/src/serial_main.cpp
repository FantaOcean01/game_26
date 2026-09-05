#include "serial_main.h"

#include <cmath>
#include <cstring>
#include <iostream>

SerialMain::SerialMain(const std::string & device_path)
: device_path_(device_path)
{
  ready_ = Initialize();

  if (!ready_) {
    std::cerr
      << "Failed to initialize serial device: "
      << device_path_
      << std::endl;
  }
}

bool SerialMain::Initialize()
{
  // 与比赛提供的通信配置保持一致
  device_ = std::make_unique<SerialDevice>(
    device_path_, 921600);

  return device_->Init();
}

bool SerialMain::ReadExact(
  uint8_t * destination,
  std::size_t length)
{
  if (!ready_ || destination == nullptr) {
    return false;
  }

  std::size_t total_read = 0;

  while (total_read < length) {
    const int result = device_->Read(
      destination + total_read,
      static_cast<int>(length - total_read));

    if (result <= 0) {
      return false;
    }

    total_read += static_cast<std::size_t>(result);
  }

  return true;
}

bool SerialMain::ReadFrame(std::size_t & frame_length)
{
  receive_buffer_.fill(0);
  frame_length = 0;

  // 先在连续字节流中寻找帧头0xA5
  uint8_t byte = 0;

  while (true) {
    if (!ReadExact(&byte, 1)) {
      return false;
    }

    if (byte == io::HEADER_SOF) {
      receive_buffer_[0] = byte;
      break;
    }
  }

  // 已经读取1字节SOF，再读取帧头剩余部分
  if (!ReadExact(
      receive_buffer_.data() + 1,
      sizeof(io::FrameHeader) - 1))
  {
    return false;
  }

  std::memcpy(
    &receive_header_,
    receive_buffer_.data(),
    sizeof(io::FrameHeader));

  // CRC8只校验5字节帧头
  if (!Verify_CRC8_Check_Sum(
      receive_buffer_.data(),
      sizeof(io::FrameHeader)))
  {
    std::cerr << "Frame header CRC8 error." << std::endl;
    return false;
  }

  constexpr std::size_t FIXED_FRAME_BYTES =
    sizeof(io::FrameHeader) +
    sizeof(uint16_t) +          // cmd_id
    sizeof(uint16_t) +          // CRC16
    sizeof(io::MsgEndInfo);     // 0D 0A

  // 防止异常data_length造成缓冲区越界
  if (
    receive_header_.data_length >
    BUFFER_SIZE - FIXED_FRAME_BYTES)
  {
    std::cerr
      << "Invalid payload length: "
      << receive_header_.data_length
      << std::endl;

    return false;
  }

  frame_length =
    FIXED_FRAME_BYTES +
    receive_header_.data_length;

  // 帧头已经读完，按data_length读取剩余字节
  return ReadExact(
    receive_buffer_.data() + sizeof(io::FrameHeader),
    frame_length - sizeof(io::FrameHeader));
}

bool SerialMain::DecodeVisionFrame(
  std::size_t frame_length)
{
  const std::size_t expected_length =
    sizeof(io::FrameHeader) +
    sizeof(uint16_t) +
    receive_header_.data_length +
    sizeof(uint16_t) +
    sizeof(io::MsgEndInfo);

  if (frame_length != expected_length) {
    std::cerr << "Unexpected frame length." << std::endl;
    return false;
  }

  // CRC16覆盖：帧头＋cmd_id＋payload＋CRC16
  const std::size_t crc16_length =
    sizeof(io::FrameHeader) +
    sizeof(uint16_t) +
    receive_header_.data_length +
    sizeof(uint16_t);

  if (!Verify_CRC16_Check_Sum(
      receive_buffer_.data(),
      static_cast<uint32_t>(crc16_length)))
  {
    std::cerr << "Frame CRC16 error." << std::endl;
    return false;
  }

  io::MsgEndInfo end_info{};

  std::memcpy(
    &end_info,
    receive_buffer_.data() + crc16_length,
    sizeof(io::MsgEndInfo));

  if (
    end_info.end1 != io::END1_SOF ||
    end_info.end2 != io::END2_SOF)
  {
    std::cerr << "Invalid frame ending." << std::endl;
    return false;
  }

  uint16_t command_id = 0;

  std::memcpy(
    &command_id,
    receive_buffer_.data() + sizeof(io::FrameHeader),
    sizeof(uint16_t));

  if (command_id != io::VISION_ID) {
    std::cerr
      << "Unexpected command id: "
      << command_id
      << std::endl;

    return false;
  }

  if (
    receive_header_.data_length !=
    sizeof(io::VisionData))
  {
    std::cerr
      << "Unexpected VisionData size: "
      << receive_header_.data_length
      << std::endl;

    return false;
  }

  const std::size_t payload_offset =
    sizeof(io::FrameHeader) +
    sizeof(uint16_t);

  std::memcpy(
    &vision_data_,
    receive_buffer_.data() + payload_offset,
    sizeof(io::VisionData));

  return true;
}

bool SerialMain::ReceiveVision()
{
  if (!ready_) {
    return false;
  }

  std::size_t frame_length = 0;

  if (!ReadFrame(frame_length)) {
    return false;
  }

  return DecodeVisionFrame(frame_length);
}

std::size_t SerialMain::PackControlFrame(
  const io::RobotCtrlData & control)
{
  send_buffer_.fill(0);

  send_header_.sof = io::HEADER_SOF;
  send_header_.data_length =
    sizeof(io::RobotCtrlData);
  send_header_.seq++;
  send_header_.crc8 = 0;

  Append_CRC8_Check_Sum(
    reinterpret_cast<uint8_t *>(&send_header_),
    sizeof(io::FrameHeader));

  std::size_t index = 0;

  std::memcpy(
    send_buffer_.data() + index,
    &send_header_,
    sizeof(io::FrameHeader));

  index += sizeof(io::FrameHeader);

  const uint16_t command_id =
    io::CHASSIS_CTRL_CMD_ID;

  std::memcpy(
    send_buffer_.data() + index,
    &command_id,
    sizeof(uint16_t));

  index += sizeof(uint16_t);

  std::memcpy(
    send_buffer_.data() + index,
    &control,
    sizeof(io::RobotCtrlData));

  index += sizeof(io::RobotCtrlData);

  // Append函数要求长度中包含最后两个CRC16字节
  const std::size_t crc16_length =
    index + sizeof(uint16_t);

  Append_CRC16_Check_Sum(
    send_buffer_.data(),
    static_cast<uint32_t>(crc16_length));

  index += sizeof(uint16_t);

  return index;
}

bool SerialMain::SendControl(
  const io::RobotCtrlData & control)
{
  if (!ready_) {
    return false;
  }

  const bool valid_lock =
    control.target_lock == io::TARGET_LOCKED ||
    control.target_lock == io::TARGET_UNLOCKED;

  const bool valid_fire =
    control.fire_command == io::FIRE_STOP ||
    control.fire_command == io::FIRE_CONTINUOUS ||
    control.fire_command == io::FIRE_SINGLE;

  const bool finite_angles =
    std::isfinite(control.yaw) &&
    std::isfinite(control.yaw_vel) &&
    std::isfinite(control.yaw_acc) &&
    std::isfinite(control.pitch) &&
    std::isfinite(control.pitch_vel) &&
    std::isfinite(control.pitch_acc);

  if (!valid_lock || !valid_fire || !finite_angles) {
    std::cerr << "Rejected unsafe control data." << std::endl;
    return false;
  }

  const std::size_t frame_length =
    PackControlFrame(control);

  std::size_t total_written = 0;

  while (total_written < frame_length) {
    const int result = device_->Write(
      send_buffer_.data() + total_written,
      static_cast<int>(frame_length - total_written));

    if (result <= 0) {
      return false;
    }

    total_written += static_cast<std::size_t>(result);
  }

  return true;
}
