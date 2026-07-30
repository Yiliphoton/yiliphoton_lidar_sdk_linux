#pragma once

#include "ylp_driver/driver/input/input.hpp"

#include <sstream>

#ifdef _WIN32
#define WIN32
#else //__linux__
#endif

#include <pcap.h>

namespace asensing
{
namespace lidar
{
class InputPcap : public Input
{
public:
  InputPcap(const YLPInputParam& input_param, double sec_to_delay)
    : Input(input_param), pcap_(NULL), pcap_offset_(ETH_HDR_LEN), pcap_tail_(0), difop_filter_valid_(false), 
    usec_to_delay_((uint64_t)(sec_to_delay / input_param.pcap_rate * 1000000))
  {
    if (input_param.use_vlan)
    {
      pcap_offset_ += VLAN_HDR_LEN;
    }

    pcap_offset_ += input_param.user_layer_bytes;
    pcap_tail_   += input_param.tail_layer_bytes;

    std::stringstream msop_stream, difop_stream;
    if (input_param_.use_vlan)
    {
      msop_stream << "vlan && ";
      difop_stream << "vlan && ";
    }

    msop_stream << "udp dst port " << input_param_.msop_port;
    difop_stream << "udp dst port " << input_param_.difop_port;

    msop_filter_str_ = msop_stream.str();
    difop_filter_str_ = difop_stream.str();
  }

  virtual bool init();
  virtual bool start();
  virtual ~InputPcap();

private:
  void recvPacket();

private:
  pcap_t* pcap_;
  size_t pcap_offset_;
  size_t pcap_tail_;
  std::string msop_filter_str_;
  std::string difop_filter_str_;
  bpf_program msop_filter_;
  bpf_program difop_filter_;
  bool difop_filter_valid_;
  uint64_t usec_to_delay_;
};

inline bool InputPcap::init()
{
  if (init_flag_)
    return true;

  char errbuf[PCAP_ERRBUF_SIZE];
  pcap_ = pcap_open_offline(input_param_.pcap_path.c_str(), errbuf);
  if (pcap_ == NULL)
  {
    cb_excep_(Error(ERRCODE_PCAPWRONGPATH));
    return false;
  }

  pcap_compile(pcap_, &msop_filter_, msop_filter_str_.c_str(), 1, 0xFFFFFFFF);

  if ((input_param_.difop_port != 0) && (input_param_.difop_port != input_param_.msop_port))
  {
    pcap_compile(pcap_, &difop_filter_, difop_filter_str_.c_str(), 1, 0xFFFFFFFF);
    difop_filter_valid_ = true;
  }

  init_flag_ = true;
  return true;
}

inline bool InputPcap::start()
{
  if (start_flag_)
    return true;

  if (!init_flag_)
  {
    cb_excep_(Error(ERRCODE_STARTBEFOREINIT));
    return false;
  }

  to_exit_recv_ = false;
  recv_thread_ = std::thread(std::bind(&InputPcap::recvPacket, this));

  start_flag_ = true;
  return true;
}

inline InputPcap::~InputPcap()
{
  stop();

  pcap_freecode(&msop_filter_);
  if (difop_filter_valid_)
    pcap_freecode(&difop_filter_);

  if (pcap_ != NULL)
  {
    pcap_close(pcap_);
    pcap_ = NULL;
  }
}

inline void InputPcap::recvPacket()
{
#ifndef WIN32
  pthread_setname_np(pthread_self(), "recv_packet");
#endif

  while (!to_exit_recv_)
  {
    struct pcap_pkthdr* header;
    const u_char* pkt_data;
    int ret = pcap_next_ex(pcap_, &header, &pkt_data);
    if (ret < 0)  // reach file end.
    {
      pcap_close(pcap_);
      pcap_ = NULL;

      if (input_param_.pcap_repeat)
      {
        cb_excep_(Error(ERRCODE_PCAPREPEAT));

        char errbuf[PCAP_ERRBUF_SIZE];
        pcap_ = pcap_open_offline(input_param_.pcap_path.c_str(), errbuf);
        continue;
      }
      else
      {
        cb_excep_(Error(ERRCODE_PCAPEXIT));
        break;
      }
    }

    if (pcap_offline_filter(&msop_filter_, header, pkt_data) != 0)
    {
      std::shared_ptr<Buffer> pkt = cb_get_pkt_(ETH_LEN);
      memcpy(pkt->data(), pkt_data + pcap_offset_, header->len - pcap_offset_ - pcap_tail_);
      pkt->setData(0, header->len - pcap_offset_ - pcap_tail_);
      pushPacket(pkt);
    }
    else if (difop_filter_valid_ && (pcap_offline_filter(&difop_filter_, header, pkt_data) != 0))
    {
      std::shared_ptr<Buffer> pkt = cb_get_pkt_(ETH_LEN);
      memcpy(pkt->data(), pkt_data + pcap_offset_, header->len - pcap_offset_ - pcap_tail_);
      pkt->setData(0, header->len - pcap_offset_ - pcap_tail_);
      pushPacket(pkt);
    }
    else
    {
      continue;
    }

    while(this->viewer_stop_flag_){
      LIMIT_CALL(cb_excep_(Error(ERRCODE_PCAPSTOP)), 1);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(usec_to_delay_));
  }
}

}  // namespace lidar
}  // namespace asensing
