#pragma once

#include "ylp_driver/common/ag_log.hpp"
#include <string>
#include <vector>
#include <map>

namespace asensing
{
namespace lidar
{
enum LidarType
{
  V192 = 0x01,
  P60  = 0x02,
};

inline std::string lidarTypeToStr(const LidarType& type)
{
  switch (type)
  {
    case LidarType::V192: return "V192";
    case LidarType::P60:  return "P60";
    default:
      YLP_ERROR << "Unknown lidar type" << YLP_REND;
      return "ERROR";
  }
}

inline LidarType strToLidarType(const std::string& type)
{
  if (type == "V192")
    return lidar::LidarType::V192;
  if (type == "P60")
    return lidar::LidarType::P60;

  YLP_ERROR << "Wrong lidar type: " << type << ". Supported: V192, P60." << YLP_REND;
  exit(-1);
}

enum InputType
{
  ONLINE_LIDAR = 1,
  PCAP_FILE,
  RAW_PACKET
};

inline std::string inputTypeToStr(const InputType& type)
{
  std::string str = "";
  switch (type)
  {
    case InputType::ONLINE_LIDAR:
      str = "ONLINE_LIDAR";
      break;
    case InputType::PCAP_FILE:
      str = "PCAP_FILE";
      break;
    case InputType::RAW_PACKET:
      str = "RAW_PACKET";
      break;
    default:
      str = "ERROR";
      YLP_ERROR << "YLP_ERROR" << YLP_REND;
  }
  return str;
}

struct YLPTransformParam  // The Point transform parameter
{
  float x = 0.0f;      // unit, m
  float y = 0.0f;      // unit, m
  float z = 0.0f;      // unit, m
  float roll = 0.0f;   // unit, radian
  float elevation = 0.0f;  // unit, radian
  float azimuth = 0.0f;    // unit, radian

  void print() const
  {
    YLP_INFO << "------------------------------------------------------" << YLP_REND;
    YLP_INFO << "             Yiliphoton Transform Parameters " << YLP_REND;
    YLP_INFOL << "x: " << x << YLP_REND;
    YLP_INFOL << "y: " << y << YLP_REND;
    YLP_INFOL << "z: " << z << YLP_REND;
    YLP_INFOL << "roll: " << roll << YLP_REND;
    YLP_INFOL << "elevation: " << elevation << YLP_REND;
    YLP_INFOL << "azimuth: " << azimuth << YLP_REND;
    YLP_INFO << "------------------------------------------------------" << YLP_REND;
  }
};

struct YLPDecoderParam  // LiDAR decoder parameter
{
  bool config_from_file = false; // Internal use only for debugging
  std::string angle_path = "";   // Internal use only for debugging
  std::string calib_file = "";   // The calibration file name
  bool wait_for_difop = true;    // true: start sending point cloud until receive difop packet
  float min_distance = 0.0f;     // min/max distances of point cloud range. valid if min distance or max distance > 0
  float max_distance = 0.0f; 
  bool use_lidar_clock = false;  // true: use LiDAR clock as timestamp; false: use system clock as timestamp
  bool write_pkt_ts = false;     // rewrite pkt timestamp or not; (prerequisite: use_lidar_clock must be false)
  bool dense_points = false;     // true: discard NAN points; false: reserve NAN points
  bool ts_first_point = false;    // true: time-stamp point cloud with the first point; false: with the last point;
  bool process_by_thread = true; // true: process point cloud data by thread; false: process in decode thread
  bool enable_dual_mode = true;  // true: support dual echo mode
  bool enable_s192line_mode = false; // true: enable software simulation 192 lines (only for A2)
  bool enable_fov_aligned = false;   // true: adjust point angle, maybe resize the frame
  bool imu_motion_compensation = false;
  YLPTransformParam transform_param; // Used to transform points
  std::vector<double> extrinsic;

  void print() const
  {
    YLP_INFO << "------------------------------------------------------" << YLP_REND;
    YLP_INFO << "             Yiliphoton Decoder Parameters " << YLP_REND;
    YLP_INFOL << "wait_for_difop: " << wait_for_difop << YLP_REND;
    YLP_INFOL << "min_distance: " << min_distance << YLP_REND;
    YLP_INFOL << "max_distance: " << max_distance << YLP_REND;
    YLP_INFOL << "use_lidar_clock: " << use_lidar_clock << YLP_REND;
    YLP_INFOL << "write_pkt_ts: " << write_pkt_ts << YLP_REND;
    YLP_INFOL << "dense_points: " << dense_points << YLP_REND;
    YLP_INFOL << "config_from_file: " << config_from_file << YLP_REND;
    YLP_INFOL << "angle_path: " << angle_path << YLP_REND;
    YLP_INFOL << "process_by_thread: " << process_by_thread << YLP_REND;
    YLP_INFOL << "enable_dual_mode: " << enable_dual_mode << YLP_REND;
    YLP_INFOL << "enable_s192line_mode: " << enable_s192line_mode << YLP_REND;
    YLP_INFOL << "enable_fov_aligned: " << enable_fov_aligned << YLP_REND;
    YLP_INFOL << "imu_motion_compensation: " << imu_motion_compensation << YLP_REND;
    YLP_INFO << "------------------------------------------------------" << YLP_REND;
    transform_param.print();
  }
};

struct YLPInputParam  // The LiDAR input parameter
{
  uint16_t msop_port = 51180;                  // Msop packet port number
  uint16_t difop_port = 9988;                  // Difop packet port number
  std::string host_address = "0.0.0.0";        // 192.168.101.101   // Address of host
  std::string group_address = "0.0.0.0";       // Address of multicast group
  std::string pcap_path = "";                  // Absolute path of pcap file
  bool pcap_repeat = true;                     // true: The pcap bag will repeat play
  float pcap_rate = 1.0f;                      // Rate to read the pcap file
  bool use_vlan = false;                       // Vlan on-off
  uint16_t user_layer_bytes = 0;    // Bytes of user layer. thers is no user layer if it is 0
  uint16_t tail_layer_bytes = 0;    // Bytes of tail layer. thers is no tail layer if it is 0

  void print() const
  {
    YLP_INFO << "------------------------------------------------------" << YLP_REND;
    YLP_INFO << "             Yiliphoton Input Parameters " << YLP_REND;
    YLP_INFOL << "msop_port: " << msop_port << YLP_REND;
    YLP_INFOL << "difop_port: " << difop_port << YLP_REND;
    YLP_INFOL << "host_address: " << host_address << YLP_REND;
    YLP_INFOL << "group_address: " << group_address << YLP_REND;
    YLP_INFOL << "pcap_path: " << pcap_path << YLP_REND;
    YLP_INFOL << "pcap_rate: " << pcap_rate << YLP_REND;
    YLP_INFOL << "pcap_repeat: " << pcap_repeat << YLP_REND;
    YLP_INFOL << "use_vlan: " << use_vlan << YLP_REND;
    YLP_INFOL << "user_layer_bytes: " << user_layer_bytes << YLP_REND;
    YLP_INFOL << "tail_layer_bytes: " << tail_layer_bytes << YLP_REND;
    YLP_INFO << "------------------------------------------------------" << YLP_REND;
  }
};

struct YLPDriverParam  // The LiDAR driver parameter
{
  LidarType lidar_type = LidarType::V192;          // Lidar type
  InputType input_type = InputType::ONLINE_LIDAR; // Input type
  std::string fixed_frame = "ylplidar";            // The frame id of LiDAR mesage
  YLPInputParam input_param;                       // Input parameter
  YLPDecoderParam decoder_param;                   // Decoder parameter

  void print() const
  {
    YLP_INFO << "------------------------------------------------------" << YLP_REND;
    YLP_INFO << "             Yiliphoton Driver Parameters " << YLP_REND;
    YLP_INFOL << "input type: " << inputTypeToStr(input_type) << YLP_REND;
    YLP_INFOL << "lidar_type: " << lidarTypeToStr(lidar_type) << YLP_REND;
    YLP_INFOL << "frame_id: "   << fixed_frame << YLP_REND;
    YLP_INFOL << "------------------------------------------------------" << YLP_REND;

    input_param.print();
    decoder_param.print();
  }
};

struct DeviceInfo
{
  uint8_t sn[6];
  uint8_t mac[6];
  uint8_t version[6];
};

struct DeviceStatus
{
  float voltage = 0.0f;
};

}  // namespace lidar
}  // namespace asensing
