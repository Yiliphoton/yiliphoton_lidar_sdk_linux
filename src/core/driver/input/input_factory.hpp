#pragma once

#include "core/driver/input/input.hpp"
#include "core/driver/input/raw/input_raw.hpp"

#include "core/driver/input/socket/input_sock.hpp"

#ifndef DISABLE_PCAP_PARSE
#include "core/driver/input/pcap/input_pcap.hpp"

#endif

namespace lidar_sdk
{
namespace lidar
{

class InputFactory
{
public:
  static std::shared_ptr<Input> createInput(InputType type, const InputParam& param,
      double sec_to_delay, std::function<void(const uint8_t*, size_t)>& cb_feed_pkt);
};

inline std::shared_ptr<Input> InputFactory::createInput(InputType type, const InputParam& param,
    double sec_to_delay, std::function<void(const uint8_t*, size_t)>& cb_feed_pkt)
{
  std::shared_ptr<Input> input;

  switch(type)
  {
    case InputType::ONLINE_LIDAR:
      {
        input = std::make_shared<InputSock>(param);
      }
      break;

#ifndef DISABLE_PCAP_PARSE
    case InputType::PCAP_FILE:
      {
        input = std::make_shared<InputPcap>(param, sec_to_delay);
      }
      break;
#endif

    case InputType::RAW_PACKET:
      {
        std::shared_ptr<InputRaw> inputRaw;

        inputRaw = std::make_shared<InputRaw>(param);

        cb_feed_pkt = std::bind(&InputRaw::feedPacket, inputRaw, 
            std::placeholders::_1, std::placeholders::_2);

        input = inputRaw;
      }
      break;

    default:

      LOG_ERROR << "Wrong Input Type " << type << "." << LOG_REND;

      if (type == InputType::PCAP_FILE) 
      {
        LOG_ERROR << "To use InputType::PCAP_FILE, please do not specify the make option DISABLE_PCAP_PARSE." << LOG_REND;
      }

      input = nullptr;
  }

  return input;
}

}  // namespace lidar
}  // namespace lidar_sdk
