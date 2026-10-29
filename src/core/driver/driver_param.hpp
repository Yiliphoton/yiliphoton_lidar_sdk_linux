#pragma once

#include "core/common/log.hpp"
#include <string>
#include <vector>
#include <map>

namespace lidar_sdk
{
namespace lidar
{
enum LidarType
{
  V192 = 0x01,
  P60  = 0x02,
  P60_IMU = 0x03,
};

inline std::string lidarTypeToStr(const LidarType& type)
{
  switch (type)
  {
    case LidarType::V192: return "V192";
    case LidarType::P60:  return "P60";
    case LidarType::P60_IMU: return "P60_IMU";
    default:
      LOG_ERROR << "Unknown lidar type" << LOG_REND;
      return "ERROR";
  }
}

inline LidarType strToLidarType(const std::string& type)
{
  if (type == "V192")
    return lidar::LidarType::V192;
  if (type == "P60")
    return lidar::LidarType::P60;
  if (type == "P60_IMU")
    return lidar::LidarType::P60_IMU;

  LOG_ERROR << "Wrong lidar type: " << type << ". Supported: V192, P60, P60_IMU." << LOG_REND;
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
      LOG_ERROR << "LOG_ERROR" << LOG_REND;
  }
  return str;
}

struct TransformParam  // The Point transform parameter
{
  float x = 0.0f;      // unit, m
  float y = 0.0f;      // unit, m
  float z = 0.0f;      // unit, m
  float roll = 0.0f;   // unit, radian
  float elevation = 0.0f;  // unit, radian
  float azimuth = 0.0f;    // unit, radian

  void print() const
  {
    LOG_INFO << "------------------------------------------------------" << LOG_REND;
    LOG_INFO << "             LiDAR Transform Parameters " << LOG_REND;
    LOG_INFOL << "x: " << x << LOG_REND;
    LOG_INFOL << "y: " << y << LOG_REND;
    LOG_INFOL << "z: " << z << LOG_REND;
    LOG_INFOL << "roll: " << roll << LOG_REND;
    LOG_INFOL << "elevation: " << elevation << LOG_REND;
    LOG_INFOL << "azimuth: " << azimuth << LOG_REND;
    LOG_INFO << "------------------------------------------------------" << LOG_REND;
  }
};

struct DecoderParam  // LiDAR decoder parameter
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
  unsigned int echo_option = 0; // 0: merge; 1: first echo; 2: second echo; 3: dual echo
  bool enable_s192line_mode = false; // true: enable software simulation 192 lines (only for A2)
  bool enable_fov_aligned = false;   // true: adjust point angle, maybe resize the frame
  TransformParam transform_param; // Used to transform points
  std::vector<double> extrinsic;

  void print() const
  {
    LOG_INFO << "------------------------------------------------------" << LOG_REND;
    LOG_INFO << "             LiDAR Decoder Parameters " << LOG_REND;
    LOG_INFOL << "wait_for_difop: " << wait_for_difop << LOG_REND;
    LOG_INFOL << "min_distance: " << min_distance << LOG_REND;
    LOG_INFOL << "max_distance: " << max_distance << LOG_REND;
    LOG_INFOL << "use_lidar_clock: " << use_lidar_clock << LOG_REND;
    LOG_INFOL << "write_pkt_ts: " << write_pkt_ts << LOG_REND;
    LOG_INFOL << "dense_points: " << dense_points << LOG_REND;
    LOG_INFOL << "config_from_file: " << config_from_file << LOG_REND;
    LOG_INFOL << "angle_path: " << angle_path << LOG_REND;
    LOG_INFOL << "process_by_thread: " << process_by_thread << LOG_REND;
    LOG_INFOL << "enable_dual_mode: " << enable_dual_mode << LOG_REND;
    LOG_INFOL << "echo_option: " << echo_option << LOG_REND;
    LOG_INFOL << "enable_s192line_mode: " << enable_s192line_mode << LOG_REND;
    LOG_INFOL << "enable_fov_aligned: " << enable_fov_aligned << LOG_REND;
    LOG_INFO << "------------------------------------------------------" << LOG_REND;
    transform_param.print();
  }
};

struct InputParam  // The LiDAR input parameter
{
  uint16_t msop_port = 8899;                   // Msop packet port number
  uint16_t difop_port = 0;                     // Difop packet port number; disabled for P60/V192
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
    LOG_INFO << "------------------------------------------------------" << LOG_REND;
    LOG_INFO << "             LiDAR Input Parameters " << LOG_REND;
    LOG_INFOL << "msop_port: " << msop_port << LOG_REND;
    LOG_INFOL << "difop_port: " << difop_port << LOG_REND;
    LOG_INFOL << "host_address: " << host_address << LOG_REND;
    LOG_INFOL << "group_address: " << group_address << LOG_REND;
    LOG_INFOL << "pcap_path: " << pcap_path << LOG_REND;
    LOG_INFOL << "pcap_rate: " << pcap_rate << LOG_REND;
    LOG_INFOL << "pcap_repeat: " << pcap_repeat << LOG_REND;
    LOG_INFOL << "use_vlan: " << use_vlan << LOG_REND;
    LOG_INFOL << "user_layer_bytes: " << user_layer_bytes << LOG_REND;
    LOG_INFOL << "tail_layer_bytes: " << tail_layer_bytes << LOG_REND;
    LOG_INFO << "------------------------------------------------------" << LOG_REND;
  }
};

struct DriverParam  // The LiDAR driver parameter
{
  LidarType lidar_type = LidarType::V192;          // Lidar type
  InputType input_type = InputType::ONLINE_LIDAR; // Input type
  std::string fixed_frame = "lidar";            // The frame id of LiDAR mesage
  InputParam input_param;                       // Input parameter
  DecoderParam decoder_param;                   // Decoder parameter

  void print() const
  {
    LOG_INFO << "------------------------------------------------------" << LOG_REND;
    LOG_INFO << "             LiDAR Driver Parameters " << LOG_REND;
    LOG_INFOL << "input type: " << inputTypeToStr(input_type) << LOG_REND;
    LOG_INFOL << "lidar_type: " << lidarTypeToStr(lidar_type) << LOG_REND;
    LOG_INFOL << "frame_id: "   << fixed_frame << LOG_REND;
    LOG_INFOL << "------------------------------------------------------" << LOG_REND;

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
}  // namespace lidar_sdk
