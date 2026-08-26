#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>

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
    sizeof(uint16_t) +
    sizeof(io::MsgEndInfo);

  static_assert(expected_length == 37);

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
  io::MsgEndInfo end_info{};

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

  std::memcpy(
    &end_info,
    frame.data() + index,
    sizeof(end_info));

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

  const bool end_ok =
    end_info.end1 == io::END1_SOF &&
    end_info.end2 == io::END2_SOF;

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
  std::cout << "end_ok: " << end_ok << '\n';
  std::cout
    << "rejected_invalid_fire: "
    << rejected_invalid_fire << '\n';
  std::cout
    << "rejected_invalid_lock: "
    << rejected_invalid_lock << '\n';
  std::cout
    << "rejected_invalid_number: "
    << rejected_invalid_number << '\n';

  std::cout << "frame_hex:";

  for (const uint8_t byte : frame) {
    std::cout
      << ' '
      << std::hex
      << std::setw(2)
      << std::setfill('0')
      << static_cast<int>(byte);
  }

  std::cout << std::dec << '\n';

  ::close(master_fd);

  const bool all_ok =
    header_ok &&
    command_id_ok &&
    crc8_ok &&
    crc16_ok &&
    payload_ok &&
    end_ok &&
    rejected_invalid_fire &&
    rejected_invalid_lock &&
    rejected_invalid_number;

  std::cout << (all_ok ? "PASS" : "FAIL") << std::endl;

  return all_ok ? 0 : 1;
}
