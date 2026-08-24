#include "serial_main.h"

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
	
	int a = 0;
	int flag = 0;
	bool get = false;
	uint8_t last_len = 0;
	int count = 2;
	while (count--)
	{
		// uint16_t read_length = device_ptr_->Read(recv_buff_.get(),BUFF_LENGTH);
		
		last_len = device_ptr_->ReadUntil2(recv_buff_.get(), io::END1_SOF, io::END2_SOF, 128);
		
		while (flag == 0 && last_len == 1)
		{
			if ((recv_buff_[a] == io::END1_SOF) && (recv_buff_[a + 1] == io::END2_SOF))
			{
				flag = 1;
				SearchFrameSOF(recv_buff_.get(), a);
				get = true;
			}
			// printf("%x  ",recv_buff_[a]);
			a++;
		}
		flag = 0;
		a = 0;
	}
	return get;
}

void SerialMain::SearchFrameSOF(uint8_t *frame, uint16_t total_len)
{
	uint16_t i;
	// uint16_t index = 0;
	// int a = 0;
	
//	std::cout<<total_len<<std::endl;
	for (i = 0; i < total_len;)
	{
		if (*frame == io::HEADER_SOF)
		{
			ReceiveDataSolve(frame);
			i = total_len;
		}
		else
		{
			frame++;
			i++;
		}
	}
}

uint16_t SerialMain::ReceiveDataSolve(uint8_t * frame)
{
  uint16_t index = 0;
  uint16_t cmd_id = 0;

  // 1. 检查帧头起始字节
  if (*frame != io::HEADER_SOF) {
    return 0;
  }

  // 2. 读取5字节帧头
  std::memcpy(
    &frame_receive_header_,
    frame,
    sizeof(io::FrameHeader));

  index += sizeof(io::FrameHeader);

  // 3. 检查帧头CRC8和整帧CRC16
  if (
    !Verify_CRC8_Check_Sum(frame, sizeof(io::FrameHeader)) ||
    !Verify_CRC16_Check_Sum(
      frame,
      frame_receive_header_.data_length + 9))
  {
    std::cout << "CRC error!!" << std::endl;
    return 0;
  }

  // 4. 读取2字节cmd_id
  std::memcpy(
    &cmd_id,
    frame + index,
    sizeof(uint16_t));

  index += sizeof(uint16_t);

  // 5. 根据cmd_id判断数据类型
  switch (cmd_id) {
    case io::VISION_ID:
    {
      // 新版VisionData必须是47字节
      if (frame_receive_header_.data_length != sizeof(io::VisionData)) {
        std::cerr
          << "Unexpected VisionData length: "
          << frame_receive_header_.data_length
          << ", expected: "
          << sizeof(io::VisionData)
          << std::endl;

        return 0;
      }

      // 把47字节载荷复制到VisionData结构体
      std::memcpy(
        &vision_msg_,
        frame + index,
        sizeof(io::VisionData));

      break;
    }

    default:
      std::cerr
        << "Unknown cmd_id: 0x"
        << std::hex
        << cmd_id
        << std::dec
        << std::endl;
      break;
  }

  // 帧头5 + cmd_id 2 + payload + CRC16 2
  index += frame_receive_header_.data_length + 2;

  return index;
}
uint16_t SerialMain::SenderPackSolve(uint8_t *data, uint16_t data_length,
									 uint16_t cmd_id, uint8_t *send_buf)
{
	
	uint8_t index = 0;
	frame_send_header_.sof = io::HEADER_SOF;
	frame_send_header_.data_length = data_length;
	frame_send_header_.seq++;
	
	Append_CRC8_Check_Sum((uint8_t *)&frame_send_header_, sizeof(io::FrameHeader));
	
	std::memcpy(send_buf, &frame_send_header_, sizeof(io::FrameHeader));//assign frame header
	
	index += sizeof(io::FrameHeader);
	
	std::memcpy(send_buf + index, &cmd_id, sizeof(uint16_t));//assign cmd
	
	index += sizeof(uint16_t);
	
	std::memcpy(send_buf + index, data, data_length);//assign data
	
	Append_CRC16_Check_Sum(send_buf, data_length + 9);
	
	return data_length + 9;
}
