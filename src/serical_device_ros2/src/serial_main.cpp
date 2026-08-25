#include "serial_main.h"

#include <chrono>
#include <cstring>
SerialMain::SerialMain(std::string device_path) : device_path_(device_path)
{
	if (!(CommInit()))
	{
		std::cout<<"serial init error!!!!!!!!!!"<<std::endl;
	};
}

void SerialMain::SenderMain(const io::RobotCtrlData & command)
{
        robot_ctrl_ = command;

        const uint16_t send_length = SenderPackSolve(
          reinterpret_cast<uint8_t *>(&robot_ctrl_),
        sizeof(io::RobotCtrlData),
        io::CHASSIS_CTRL_CMD_ID,
      send_buff_.get());

    device_ptr_->Write(send_buff_.get(), send_length);
}

bool SerialMain::CommInit()
{
	
	device_ptr_ = std::make_shared<SerialDevice>(device_path_, 115200); // 比特率115200
	
	if (!device_ptr_->Init())
	{
		return false;
	}
	
	recv_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH]);
	send_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH]);
	
	std::memset(&frame_receive_header_, 0, sizeof(io::FrameHeader));
	std::memset(&frame_send_header_, 0, sizeof(io::FrameHeader));
	
	return true;
}

bool SerialMain::ReceiverMain()
{
        const uint16_t frame_length = ReadFrame();
        if (frame_length == 0) {
                return false;
        }

        return ParseFrame(frame_length);
}

uint16_t SerialMain::ReadFrame()
{
        std::memset(recv_buff_.get(), 0, BUFF_LENGTH);
	
        uint16_t rx_length = 0;
        uint16_t idle_reads = 0;
        bool header_found = false;
        uint8_t byte = 0;

        while (rx_length < BUFF_LENGTH)
	{
                const int ret = device_ptr_->Read(&byte, 1);
                if (ret <= 0) {
                        if (!header_found) {
                                return 0;
                        }
		
                        if (++idle_reads >= 100) {
                                return 0;
                        }

                        std::this_thread::sleep_for(
                          std::chrono::milliseconds(1));
                        continue;
                }
                idle_reads = 0;
		
                if (!header_found) {
                        if (byte == io::HEADER_SOF) {
                                header_found = true;
                                recv_buff_[0] = byte;
                                rx_length = 1;
			}
                        continue;
		}

                recv_buff_[rx_length] = byte;
                rx_length++;

                if (rx_length == sizeof(io::FrameHeader)) {
                        std::memcpy(
                          &frame_receive_header_,
                          recv_buff_.get(),
                          sizeof(io::FrameHeader));

                        const uint16_t frame_overhead =
                          sizeof(io::FrameHeader) + sizeof(uint16_t) +
                          sizeof(uint16_t) + sizeof(io::MsgEndInfo);

                        if (
                          frame_receive_header_.data_length >
                          BUFF_LENGTH - frame_overhead) {
                                std::cerr << "Invalid data length: "
                                          << frame_receive_header_.data_length
                                          << std::endl;
                                return 0;
                        }

                        if (!Verify_CRC8_Check_Sum(
                              recv_buff_.get(), sizeof(io::FrameHeader))) {
                                std::cerr << "CRC8 error!!" << std::endl;
                                return 0;
                        }
		}

                if (rx_length >= sizeof(io::FrameHeader)) {
                        const uint16_t expected_length =
                          sizeof(io::FrameHeader) + sizeof(uint16_t) +
                          frame_receive_header_.data_length + sizeof(uint16_t) +
                          sizeof(io::MsgEndInfo);

                        if (rx_length == expected_length) {
                                return rx_length;
                        }
		}
	}

        return 0;
}

bool SerialMain::ParseFrame(uint16_t frame_length)
{
        const uint16_t expected_length =
          sizeof(io::FrameHeader) + sizeof(uint16_t) +
          frame_receive_header_.data_length + sizeof(uint16_t) +
          sizeof(io::MsgEndInfo);

        if (frame_length != expected_length) {
                return false;
        }

        uint16_t cmd_id = 0;
        std::memcpy(
          &cmd_id,
          recv_buff_.get() + sizeof(io::FrameHeader),
          sizeof(uint16_t));

        if (cmd_id != io::VISION_ID) {
                std::cerr << "Unknown cmd_id: 0x"
                          << std::hex << cmd_id << std::dec << std::endl;
                return false;
        }

        if (frame_receive_header_.data_length != sizeof(io::VisionData)) {
                std::cerr << "Unexpected VisionData length: "
                          << frame_receive_header_.data_length
                          << ", expected: " << sizeof(io::VisionData)
                          << std::endl;
                return false;
        }

        const uint16_t crc_checked_length =
          sizeof(io::FrameHeader) + sizeof(uint16_t) +
          frame_receive_header_.data_length + sizeof(uint16_t);

        if (!Verify_CRC16_Check_Sum(
              recv_buff_.get(), crc_checked_length)) {
                std::cerr << "CRC16 error!!" << std::endl;
                return false;
        }

        io::MsgEndInfo end_info;
        std::memcpy(
          &end_info,
          recv_buff_.get() + crc_checked_length,
          sizeof(io::MsgEndInfo));

        if (
          end_info.end1 != io::END1_SOF ||
          end_info.end2 != io::END2_SOF) {
                std::cerr << "Invalid frame end!!" << std::endl;
                return false;
        }

        std::memcpy(
          &vision_msg_,
          recv_buff_.get() + sizeof(io::FrameHeader) + sizeof(uint16_t),
          sizeof(io::VisionData));

        return true;
}

uint16_t SerialMain::SenderPackSolve(uint8_t *data, uint16_t data_length,
									 uint16_t cmd_id, uint8_t *send_buf)
{
        const uint16_t total_length =
          sizeof(io::FrameHeader) + sizeof(uint16_t) + data_length +
          sizeof(uint16_t) + sizeof(io::MsgEndInfo);

        if (total_length > BUFF_LENGTH) {
                return 0;
        }
	
        uint16_t index = 0;
	frame_send_header_.sof = io::HEADER_SOF;
	frame_send_header_.data_length = data_length;
	frame_send_header_.seq++;
	
	Append_CRC8_Check_Sum((uint8_t *)&frame_send_header_, sizeof(io::FrameHeader));
	
	std::memcpy(send_buf, &frame_send_header_, sizeof(io::FrameHeader));//assign frame header
	
	index += sizeof(io::FrameHeader);
	
	std::memcpy(send_buf + index, &cmd_id, sizeof(uint16_t));//assign cmd
	
	index += sizeof(uint16_t);
	
	std::memcpy(send_buf + index, data, data_length);//assign data
        index += data_length;

        Append_CRC16_Check_Sum(send_buf, index + sizeof(uint16_t));
        index += sizeof(uint16_t);
	
        const io::MsgEndInfo end_info;
        std::memcpy(send_buf + index, &end_info, sizeof(io::MsgEndInfo));
        index += sizeof(io::MsgEndInfo);
	
        return index;
}
