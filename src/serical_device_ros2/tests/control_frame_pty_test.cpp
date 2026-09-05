// Copyright (c) 2026 FantaOcean
// Licensed under the Apache License, Version 2.0

#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>
#include <array>
#include <cstring>

#include <pty.h>
#include <unistd.h>

#include "crc.h"
#include "protocol_new.hpp"
#include "serial_main.h"

namespace
{

bool read_exact(int fd, uint8_t * data, std::size_t length)
{
  std::size_t total_read = 0;

  while (total_read < length) {
    const ssize_t result =
      ::read(fd, data + total_read, length - total_read);

    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }

      std::perror("read");
      return false;
    }

    if (result == 0) {
      return false;
    }

    total_read += static_cast<std::size_t>(result);
  }

  return true;
}

}  // namespace

int main()
{
  constexpr std::size_t expected_length =
    sizeof(io::FrameHeader) +
    sizeof(uint16_t) +
    sizeof(io::RobotCtrlData) +
    sizeof(uint16_t);

  static_assert(expected_length == 35);

  int master_fd = -1;
  int slave_fd = -1;
  char slave_name[128]{};

  if (::openpty(
      &master_fd,
      &slave_fd,
      slave_name,
      nullptr,
      nullptr) != 0)
  {
    std::perror("openpty");
    return 1;
  }

  ::close(slave_fd);

  SerialMain serial(slave_name);

  if (!serial.Ready()) {
    std::cerr << "SerialMain initialization failed." << std::endl;
    ::close(master_fd);
    return 1;
  }

  io::RobotCtrlData control{};
  control.yaw = 12.5f;
  control.yaw_vel = 1.25f;
  control.yaw_acc = 0.5f;
  control.pitch = -3.0f;
  control.pitch_vel = -0.25f;
  control.pitch_acc = 0.125f;
  control.target_lock = io::TARGET_UNLOCKED;
  control.fire_command = io::FIRE_STOP;

  if (!serial.SendControl(control)) {
    std::cerr << "SendControl failed." << std::endl;
    ::close(master_fd);
    return 1;
  }

  std::array<uint8_t, expected_length> frame{};

  if (!read_exact(master_fd, frame.data(), frame.size())) {
    std::cerr << "Failed to read complete frame." << std::endl;
    ::close(master_fd);
    return 1;
  }

  io::FrameHeader header{};
  uint16_t command_id = 0;
  io::RobotCtrlData decoded_control{};

  std::size_t index = 0;

  std::memcpy(
    &header,
    frame.data() + index,
    sizeof(header));
  index += sizeof(header);

  std::memcpy(
    &command_id,
    frame.data() + index,
    sizeof(command_id));
  index += sizeof(command_id);

  std::memcpy(
    &decoded_control,
    frame.data() + index,
    sizeof(decoded_control));
  index += sizeof(decoded_control);

  const std::size_t crc16_length =
    index + sizeof(uint16_t);

  index += sizeof(uint16_t);

  const bool header_ok =
    header.sof == io::HEADER_SOF &&
    header.data_length == sizeof(io::RobotCtrlData);

  const bool command_id_ok =
    command_id == io::CHASSIS_CTRL_CMD_ID;

  const bool crc8_ok =
    Verify_CRC8_Check_Sum(
      frame.data(),
      sizeof(io::FrameHeader));

  const bool crc16_ok =
    Verify_CRC16_Check_Sum(
      frame.data(),
      static_cast<uint32_t>(crc16_length));

  const bool payload_ok =
    std::memcmp(
      &decoded_control,
      &control,
      sizeof(control)) == 0;

  const bool field_ok =
    decoded_control.yaw == 12.5f &&
    decoded_control.yaw_vel == 1.25f &&
    decoded_control.yaw_acc == 0.5f &&
    decoded_control.pitch == -3.0f &&
    decoded_control.pitch_vel == -0.25f &&
    decoded_control.pitch_acc == 0.125f &&
    decoded_control.target_lock == io::TARGET_UNLOCKED &&
    decoded_control.fire_command == io::FIRE_STOP;

  // 验证最后两字节为 CRC16 (frame[33], frame[34])
  constexpr uint16_t CRC16_INIT_VAL = 0xFFFF;
  const uint16_t expected_crc16 =
    Get_CRC16_Check_Sum(frame.data(), 33, CRC16_INIT_VAL);
  const bool last_bytes_are_crc16 =
    frame[33] == static_cast<uint8_t>(expected_crc16 & 0xFF) &&
    frame[34] == static_cast<uint8_t>((expected_crc16 >> 8) & 0xFF);

  // 验证各字段 offset
  constexpr std::size_t payload_offset =
    sizeof(io::FrameHeader) + sizeof(uint16_t);
  constexpr std::size_t crc16_offset =
    payload_offset + sizeof(io::RobotCtrlData);

  constexpr std::size_t target_lock_offset_in_payload =
    offsetof(io::RobotCtrlData, target_lock);
  constexpr std::size_t target_lock_total_offset =
    payload_offset + target_lock_offset_in_payload;

  constexpr std::size_t fire_command_offset_in_payload =
    offsetof(io::RobotCtrlData, fire_command);
  constexpr std::size_t fire_command_total_offset =
    payload_offset + fire_command_offset_in_payload;

  static_assert(payload_offset == 7);
  static_assert(crc16_offset == 33);
  static_assert(target_lock_offset_in_payload == 24);
  static_assert(target_lock_total_offset == 31);
  static_assert(fire_command_offset_in_payload == 25);
  static_assert(fire_command_total_offset == 32);

  const bool offsets_ok =
    (payload_offset == 7) &&
    (crc16_offset == 33) &&
    (target_lock_offset_in_payload == 24) &&
    (target_lock_total_offset == 31) &&
    (fire_command_offset_in_payload == 25) &&
    (fire_command_total_offset == 32) &&
    (frame.size() == 35);

  io::RobotCtrlData invalid_fire = control;
  invalid_fire.fire_command = 3;

  io::RobotCtrlData invalid_lock = control;
  invalid_lock.target_lock = 0;

  io::RobotCtrlData invalid_number = control;
  invalid_number.yaw =
    std::numeric_limits<float>::quiet_NaN();

  const bool rejected_invalid_fire =
    !serial.SendControl(invalid_fire);

  const bool rejected_invalid_lock =
    !serial.SendControl(invalid_lock);

  const bool rejected_invalid_number =
    !serial.SendControl(invalid_number);

  std::cout << std::boolalpha;
  std::cout << "frame_length: " << frame.size() << '\n';
  std::cout << "header_ok: " << header_ok << '\n';
  std::cout << "command_id_ok: " << command_id_ok << '\n';
  std::cout << "crc8_ok: " << crc8_ok << '\n';
  std::cout << "crc16_ok: " << crc16_ok << '\n';
  std::cout << "payload_ok: " << payload_ok << '\n';
  std::cout << "field_ok: " << field_ok << '\n';
  std::cout << "last_bytes_are_crc16: " << last_bytes_are_crc16 << '\n';
  std::cout << "payload_offset: " << payload_offset << '\n';
  std::cout << "crc16_offset: " << crc16_offset << '\n';
  std::cout << "target_lock_payload_offset: " << target_lock_offset_in_payload << '\n';
  std::cout << "target_lock_total_offset: " << target_lock_total_offset << '\n';
  std::cout << "fire_command_payload_offset: " << fire_command_offset_in_payload << '\n';
  std::cout << "fire_command_total_offset: " << fire_command_total_offset << '\n';
  std::cout << "offsets_ok: " << offsets_ok << '\n';
  std::cout << "rejected_invalid_fire: " << rejected_invalid_fire << '\n';
  std::cout << "rejected_invalid_lock: " << rejected_invalid_lock << '\n';
  std::cout << "rejected_invalid_number: " << rejected_invalid_number << '\n';

  // ---- 逐字节分段标注 ----
  std::cout << "\nraw_hex:";
  for (const uint8_t byte : frame) {
    std::cout
      << ' '
      << std::hex
      << std::setw(2)
      << std::setfill('0')
      << static_cast<int>(byte);
  }
  std::cout << std::dec << '\n';

  auto print_segment =
    [&](const char * label,
        std::size_t begin,
        std::size_t end_exclusive)
    {
      std::cout << label << ':';
      for (std::size_t i = begin; i < end_exclusive; ++i) {
        std::cout
          << ' '
          << std::hex
          << std::setw(2)
          << std::setfill('0')
          << static_cast<int>(frame[i]);
      }
      std::cout << std::dec << '\n';
    };

  print_segment(
    "header_5B[SOF+len+seq+crc8]",
    0,
    sizeof(io::FrameHeader));
  print_segment(
    "cmd_id_2B",
    sizeof(io::FrameHeader),
    payload_offset);
  print_segment(
    "payload_26B[0..25]",
    payload_offset,
    crc16_offset);
  print_segment(
    "crc16_2B",
    crc16_offset,
    frame.size());

  const auto * payload =
    reinterpret_cast<const uint8_t *>(&decoded_control);
  auto dump_field =
    [&](const char * label,
        std::size_t offset,
        std::size_t width)
    {
      std::cout << label << "[off " << offset << "]:";
      for (std::size_t i = 0; i < width; ++i) {
        std::cout
          << ' '
          << std::hex
          << std::setw(2)
          << std::setfill('0')
          << static_cast<int>(payload[offset + i]);
      }
      std::cout << std::dec << '\n';
    };

  std::cout << "\npayload_field_bytes:\n";
  dump_field("yaw_f32        ", 0, 4);
  dump_field("yaw_vel_f32    ", 4, 4);
  dump_field("yaw_acc_f32    ", 8, 4);
  dump_field("pitch_f32      ", 12, 4);
  dump_field("pitch_vel_f32  ", 16, 4);
  dump_field("pitch_acc_f32  ", 20, 4);
  dump_field("target_lock_i8 ", 24, 1);
  dump_field("fire_command_i8", 25, 1);

  std::cout
    << "decoded: yaw=" << decoded_control.yaw
    << " yaw_vel=" << decoded_control.yaw_vel
    << " yaw_acc=" << decoded_control.yaw_acc
    << " pitch=" << decoded_control.pitch
    << " pitch_vel=" << decoded_control.pitch_vel
    << " pitch_acc=" << decoded_control.pitch_acc
    << " target_lock=" << static_cast<int>(decoded_control.target_lock)
    << " fire_command=" << static_cast<int>(decoded_control.fire_command)
    << '\n';

  ::close(master_fd);

  const bool all_ok =
    header_ok &&
    command_id_ok &&
    crc8_ok &&
    crc16_ok &&
    payload_ok &&
    field_ok &&
    last_bytes_are_crc16 &&
    offsets_ok &&
    rejected_invalid_fire &&
    rejected_invalid_lock &&
    rejected_invalid_number;

  std::cout << (all_ok ? "PASS" : "FAIL") << std::endl;

  return all_ok ? 0 : 1;
}
