#pragma once

#include "core/common/error_code.hpp"
#include "core/driver/driver_param.hpp"
#include "core/msg/lidar_imu_data.hpp"
#include "core/driver/decoder/member_checker.hpp"
#include "core/driver/decoder/trigon.hpp"
#include "core/driver/decoder/section.hpp"
#include "core/driver/decoder/basic_attr.hpp"
#include "core/driver/decoder/split_strategy.hpp"
#include "core/driver/algorithm/filter.hpp"
#include "core/driver/decoder/reflectivity_compensation.hpp"
#include "core/common/sync_queue.hpp"
#include "core/common/block_buffer.hpp"
#include "core/common/json.hpp"


#ifdef ENABLE_TRANSFORM
#include <Eigen/Dense>
#endif

#include <cmath>
#include <array>
#include <atomic>
#include <functional>
#include <memory>
#include <thread>

namespace lidar_sdk
{
namespace lidar
{

enum EchoMode
{
  ECHO_SINGLE = 0,
  ECHO_DUAL
};

struct DecoderConstParam
{
  uint16_t MSOP_LEN;
  uint16_t MSOP2_LEN;
  uint16_t DIFOP_LEN;

  uint8_t MSOP_ID_LEN;
  uint8_t DIFOP_ID_LEN;
  uint8_t MSOP_ID[8];
  uint8_t DIFOP_ID[8];

  uint16_t LASER_NUM;
  uint16_t BLOCKS_PER_PKT;
  uint16_t CHANNELS_PER_BLOCK;

  float DISTANCE_MIN;
  float DISTANCE_MAX;
  float DISTANCE_RES;
  float TEMPERATURE_RES;
};

#define INIT_ONLY_ONCE() \
  static bool init_flag = false; \
  if (init_flag) return param; \
  init_flag = true;

template <typename T_PointCloud>
class Decoder
{
public:

#ifdef ENABLE_TRANSFORM
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

  virtual void decodeDifopPkt(const uint8_t* pkt, size_t size) = 0;
  virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size) = 0;


  virtual bool processPointCloud(std::shared_ptr<T_PointCloud> cloud) = 0;
  virtual ~Decoder() = default;

  void processDifopPkt(const uint8_t* pkt, size_t size);
  bool processMsopPkt(const uint8_t* pkt, size_t size);


  explicit Decoder(const DecoderConstParam& const_param, const DecoderParam& param);

  float getTemperature();
  bool getDeviceInfo(DeviceInfo& info);
  bool getDeviceStatus(DeviceStatus& status);
  double getPacketDuration();
  double prevPktTs();
  void transformPoint(float& x, float& y, float& z);
  void setIntensity0To255(BlockBuffer& frame);

#ifdef POINT_TYPE_XYZIRTC
  float intensityToPackedRGB(uint8_t inten_0_255);
#endif

  void regCallback(
      const std::function<void(const Error&)>& cb_excep,
      const std::function<void(uint16_t, double)>& cb_split_frame,
      const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_finish);

  void regImuCallback(const std::function<void(const LidarImuData&)>& cb_imu)
  {
    cb_imu_ = cb_imu;
  }

  void startPointCloudProcessing();
  void stopPointCloudProcessing();
  void enqueuePointCloud(std::shared_ptr<T_PointCloud> cloud);

  std::shared_ptr<T_PointCloud> point_cloud_;

  void setReflectivityCompensator(std::shared_ptr<ReflectivityCompensator> compensator) {
    reflectivity_compensator_ = std::move(compensator);
  }

#ifndef UNIT_TEST
protected:
#endif

  double cloudTs();

  std::shared_ptr<BlockBuffer> blockGet(size_t w, size_t h, bool dual);
  void blockPut(std::shared_ptr<BlockBuffer> block, bool stuffed);

  DecoderConstParam const_param_;
  DecoderParam param_;
  std::function<void(uint16_t, double)> cb_split_frame_;
  std::function<void(std::shared_ptr<T_PointCloud>)> cb_finish_;
  std::function<void(const Error&)> cb_excep_;
  std::function<void(const LidarImuData&)> cb_imu_;

#ifdef ENABLE_TRANSFORM
  Eigen::Matrix4d trans_;
#endif

  Trigon trigon_;
#define SIN(angle) this->trigon_.sin(angle)
#define COS(angle) this->trigon_.cos(angle)

  double packet_duration_;
  DistanceSection distance_section_;

  EchoMode echo_mode_;
  float temperature_;
  DeviceInfo device_info_;
  DeviceStatus device_status_;

  bool angles_ready_;
  double prev_pkt_ts_;
  double prev_point_ts_;
  double first_point_ts_;

  bool IsCalibrated;
  bool CalibEnabled;

  std::shared_ptr<SoftwareFilter> filter_;

  SyncQueue<std::shared_ptr<BlockBuffer>> free_block_queue_;
  SyncQueue<std::shared_ptr<BlockBuffer>> block_queue_;
  SyncQueue<std::shared_ptr<T_PointCloud>> point_cloud_queue_;
  std::thread point_cloud_thread_;
  std::atomic<bool> to_exit_point_cloud_processing_ {false};
  bool point_cloud_processing_started_ = false;

  std::shared_ptr<ReflectivityCompensator> reflectivity_compensator_;
};

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::regCallback(
    const std::function<void(const Error&)>& cb_excep,
    const std::function<void(uint16_t, double)>& cb_split_frame,
    const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_finish)
{
  cb_excep_ = cb_excep;
  cb_split_frame_ = cb_split_frame;
  cb_finish_ = cb_finish;
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::startPointCloudProcessing()
{
  if (point_cloud_processing_started_)
  {
    return;
  }

  to_exit_point_cloud_processing_ = false;
  point_cloud_thread_ = std::thread([this] {
    while (!to_exit_point_cloud_processing_)
    {
      std::shared_ptr<T_PointCloud> cloud = point_cloud_queue_.popWait(100000);
      if (cloud)
      {
        processPointCloud(cloud);
      }
    }
  });
  point_cloud_processing_started_ = true;
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::stopPointCloudProcessing()
{
  if (!point_cloud_processing_started_)
  {
    return;
  }

  to_exit_point_cloud_processing_ = true;
  if (point_cloud_thread_.joinable())
  {
    point_cloud_thread_.join();
  }
  point_cloud_processing_started_ = false;
  point_cloud_queue_.clear();
  block_queue_.clear();
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::enqueuePointCloud(std::shared_ptr<T_PointCloud> cloud)
{
  point_cloud_queue_.push(std::move(cloud));
}

template <typename T_PointCloud>
inline Decoder<T_PointCloud>::Decoder(const DecoderConstParam& const_param, const DecoderParam& param)
  : const_param_(const_param)
  , param_(param)
  , packet_duration_(0)
  , distance_section_(const_param.DISTANCE_MIN, const_param.DISTANCE_MAX, param.min_distance, param.max_distance)
  , echo_mode_(ECHO_SINGLE)
  , temperature_(0.0)
  , angles_ready_(false)
  , prev_pkt_ts_(0.0)
  , prev_point_ts_(0.0)
  , first_point_ts_(0.0)
  , IsCalibrated(false)
  , CalibEnabled(true)
{
#ifdef ENABLE_TRANSFORM
  Eigen::AngleAxisd current_rotation_x(param_.transform_param.roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd current_rotation_y(param_.transform_param.elevation, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd current_rotation_z(param_.transform_param.azimuth, Eigen::Vector3d::UnitZ());
  Eigen::Translation3d current_translation(param_.transform_param.x, param_.transform_param.y,
                                           param_.transform_param.z);
  trans_ = (current_translation * current_rotation_z * current_rotation_y * current_rotation_x).matrix();
#endif
}

template <typename T_PointCloud>
inline float Decoder<T_PointCloud>::getTemperature()
{
  return temperature_;
}

template <typename T_PointCloud>
inline bool Decoder<T_PointCloud>::getDeviceInfo(DeviceInfo& info)
{
  memcpy(&info, &device_info_, sizeof(DeviceInfo));
  return true;
}

template <typename T_PointCloud>
inline bool Decoder<T_PointCloud>::getDeviceStatus(DeviceStatus& status)
{
  memcpy(&status, &device_status_, sizeof(DeviceStatus));
  return true;
}

template <typename T_PointCloud>
inline double Decoder<T_PointCloud>::getPacketDuration()
{
  return packet_duration_;
}

template <typename T_PointCloud>
inline double Decoder<T_PointCloud>::prevPktTs()
{
  return prev_pkt_ts_;
}

template <typename T_PointCloud>
inline double Decoder<T_PointCloud>::cloudTs()
{
  return first_point_ts_;
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::transformPoint(float& x, float& y, float& z)
{
#ifdef ENABLE_TRANSFORM
  Eigen::Vector4d target_ori(x, y, z, 1);
  Eigen::Vector4d target_rotate = trans_ * target_ori;
  x = target_rotate(0);
  y = target_rotate(1);
  z = target_rotate(2);
#endif
}

#ifdef POINT_TYPE_XYZIRTC
template <typename T_PointCloud>
inline float Decoder<T_PointCloud>::intensityToPackedRGB(uint8_t inten_0_255)
{
  // Reflectivity is an 8-bit value. Preserve the existing color mapping while
  // avoiding the branch/rounding work for every point.
  static const std::array<float, 256> rgb_lut = [] {
    std::array<float, 256> table{};
    for (size_t i = 0; i < table.size(); ++i)
    {
      const float t = static_cast<float>(i);
      float r, g, b;
      if (t < 64.f) {
        const float k = t / 64.f;
        r = 0.f; g = 48.f + k * 207.f; b = 186.f + k * 69.f;
      } else if (t < 128.f) {
        const float k = (t - 64.f) / 64.f;
        r = 0.f; g = 255.f; b = 255.f - k * 255.f;
      } else if (t < 192.f) {
        const float k = (t - 128.f) / 64.f;
        r = k * 255.f; g = 255.f; b = 0.f;
      } else {
        const float k = (t - 192.f) / 63.f;
        r = 255.f; g = 255.f - k * 255.f; b = 0.f;
      }

      const uint32_t v =
          (uint32_t(static_cast<uint8_t>(std::round(r))) << 16) |
          (uint32_t(static_cast<uint8_t>(std::round(g))) << 8) |
          uint32_t(static_cast<uint8_t>(std::round(b)));
      float out;
      std::memcpy(&out, &v, sizeof(float));
      table[i] = out;
    }
    return table;
  }();

  return rgb_lut[inten_0_255];
}
#endif

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::setIntensity0To255(BlockBuffer& frame)
{
  auto applyToPoints = [](std::vector<RawPointT>& points) {
    for (auto& pt : points)
      pt.reflectivity = static_cast<uint8_t>(pt.reflectivity / 126.0f * 255.0f);
  };

  applyToPoints(frame.points_);
  if (frame.is_dual)
    applyToPoints(frame.points2_);
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::processDifopPkt(const uint8_t* pkt, size_t size)
{
  if (size != this->const_param_.DIFOP_LEN)
  {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGDIFOPLEN)), 1);
    return;
  }

  if (memcmp(pkt, this->const_param_.DIFOP_ID, const_param_.DIFOP_ID_LEN) != 0)
  {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGDIFOPID)), 1);
    return;
  }

  decodeDifopPkt(pkt, size);
}

template <typename T_PointCloud>
inline bool Decoder<T_PointCloud>::processMsopPkt(const uint8_t* pkt, size_t size)
{
  constexpr static int CLOUD_POINT_MAX = 1000000;

  if (this->point_cloud_ && (this->point_cloud_->points.size() > CLOUD_POINT_MAX))
  {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_CLOUDOVERFLOW)), 1);
    this->point_cloud_->points.clear();
  }

  if (param_.wait_for_difop && !angles_ready_)
  {
    DELAY_LIMIT_CALL(cb_excep_(Error(ERRCODE_NODIFOPRECV)), 1);
    return false;
  }

  if (size != this->const_param_.MSOP_LEN && size != this->const_param_.MSOP2_LEN)
  {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGMSOPLEN)), 1);
    return false;
  }

  if (memcmp(pkt, this->const_param_.MSOP_ID, this->const_param_.MSOP_ID_LEN) != 0)
  {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGMSOPID)), 1);
    return false;
  }

#ifdef ENABLE_CRC32_CHECK
  if (!isCrc32Correct(pkt, size))
  {
    LIMIT_CALL(this->cb_excep_(Error(ERRCODE_WRONGCRC32)), 1);
    return false;
  }
#endif

  return decodeMsopPkt(pkt, size);
}


template <typename T_PointCloud>
inline std::shared_ptr<BlockBuffer> Decoder<T_PointCloud>::blockGet(size_t w, size_t h, bool dual)
{
  const size_t size = w * h;

  std::shared_ptr<BlockBuffer> block = free_block_queue_.pop();
  if (!block || block->size() != size || (dual && block->points2_.empty()))
    return std::make_shared<BlockBuffer>(w, h, dual);

  block->resetForReuse(w, h, dual);
  return block;
}

template <typename T_PointCloud>
inline void Decoder<T_PointCloud>::blockPut(std::shared_ptr<BlockBuffer> block, bool stuffed)
{
  constexpr static int RAW_POINT_CLOUD_POOL_MAX = 100;

  if (!stuffed)
  {
    free_block_queue_.push(block);
    return;
  }

  size_t sz = block_queue_.push(block);
  if (sz > RAW_POINT_CLOUD_POOL_MAX)
    block_queue_.clear();
}

}  // namespace lidar
}  // namespace lidar_sdk
