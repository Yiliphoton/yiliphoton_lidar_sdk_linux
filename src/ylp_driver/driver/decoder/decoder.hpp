#pragma once

#include "ylp_driver/common/error_code.hpp"
#include "ylp_driver/driver/driver_param.hpp"
#include "ylp_driver/driver/decoder/member_checker.hpp"
#include "ylp_driver/driver/decoder/trigon.hpp"
#include "ylp_driver/driver/decoder/section.hpp"
#include "ylp_driver/driver/decoder/basic_attr.hpp"
#include "ylp_driver/driver/decoder/split_strategy.hpp"
#include "ylp_driver/driver/algorithm/filter.hpp"
#include "ylp_driver/driver/decoder/reflectivity_compensation.hpp"
#include "ylp_driver/common/sync_queue.hpp"
#include "ylp_driver/common/block_buffer.hpp"
#include "ylp_driver/common/json.hpp"

#ifdef ENABLE_MOTION_COMPENSATION
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "ylp_driver/msg/imu_msg.hpp"
#endif

#ifdef ENABLE_TRANSFORM
#include <Eigen/Dense>
#endif

#include <cmath>
#include <functional>
#include <memory>

namespace asensing
{
namespace lidar
{

enum YLPEchoMode
{
  ECHO_SINGLE = 0,
  ECHO_DUAL
};

struct YLPDecoderConstParam
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

#ifdef ENABLE_MOTION_COMPENSATION
  virtual bool decodeMsopPktWithImu(const uint8_t* pkt, size_t size, const ImuDataPtr& imu_data) = 0;
#endif

  virtual bool processPointCloud(std::shared_ptr<T_PointCloud> cloud) = 0;
  virtual ~Decoder() = default;

  void processDifopPkt(const uint8_t* pkt, size_t size);
  bool processMsopPkt(const uint8_t* pkt, size_t size);

#ifdef ENABLE_MOTION_COMPENSATION
  bool processMsopPktWithImu(const uint8_t* pkt, size_t size, const ImuDataPtr& imu_data);
  EulerAngles ToEulerAngle(const ImuDataPtr imu_data);
  float* GetMovingParameters(const ImuDataPtr q, bool flag_new_frame);
#endif

  explicit Decoder(const YLPDecoderConstParam& const_param, const YLPDecoderParam& param);

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

  YLPDecoderConstParam const_param_;
  YLPDecoderParam param_;
  std::function<void(uint16_t, double)> cb_split_frame_;
  std::function<void(std::shared_ptr<T_PointCloud>)> cb_finish_;
  std::function<void(const Error&)> cb_excep_;

#ifdef ENABLE_TRANSFORM
  Eigen::Matrix4d trans_;
#endif

  Trigon trigon_;
#define SIN(angle) this->trigon_.sin(angle)
#define COS(angle) this->trigon_.cos(angle)

  double packet_duration_;
  DistanceSection distance_section_;

  YLPEchoMode echo_mode_;
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
inline Decoder<T_PointCloud>::Decoder(const YLPDecoderConstParam& const_param, const YLPDecoderParam& param)
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
  return (param_.ts_first_point ? first_point_ts_ : prev_point_ts_);
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
  float t = std::max(0.f, std::min(255.f, float(inten_0_255)));

  float r, g, b;
  if (t < 64.f) {
    float k = t / 64.f;
    r = 0.f; g = 48.f + k * 207.f; b = 186.f + k * 69.f;
  } else if (t < 128.f) {
    float k = (t - 64.f) / 64.f;
    r = 0.f; g = 255.f; b = 255.f - k * 255.f;
  } else if (t < 192.f) {
    float k = (t - 128.f) / 64.f;
    r = k * 255.f; g = 255.f; b = 0.f;
  } else {
    float k = (t - 192.f) / 63.f;
    r = 255.f; g = 255.f - k * 255.f; b = 0.f;
  }

  uint32_t v = (uint32_t(static_cast<uint8_t>(std::round(r))) << 16) |
               (uint32_t(static_cast<uint8_t>(std::round(g))) << 8) |
               uint32_t(static_cast<uint8_t>(std::round(b)));
  float out;
  std::memcpy(&out, &v, sizeof(float));
  return out;
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

#ifdef ENABLE_MOTION_COMPENSATION
template <typename T_PointCloud>
inline bool Decoder<T_PointCloud>::processMsopPktWithImu(const uint8_t* pkt, size_t size, const ImuDataPtr& imu_data)
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

  return decodeMsopPktWithImu(pkt, size, imu_data);
}

template <typename T_PointCloud>
EulerAngles Decoder<T_PointCloud>::ToEulerAngle(const ImuDataPtr q)
{
  EulerAngles angles = {0};
  tf2::Quaternion quat(q->orientation.x, q->orientation.y, q->orientation.z, q->orientation.w);
  tf2::Matrix3x3 mat(quat);
  mat.getEulerYPR(angles.yaw, angles.pitch, angles.roll);
  angles.roll  = RADIAN_TO_DEGREE(angles.roll);
  angles.pitch = RADIAN_TO_DEGREE(angles.pitch);
  angles.yaw   = RADIAN_TO_DEGREE(angles.yaw);
  return angles;
}

template <typename T_PointCloud>
float* Decoder<T_PointCloud>::GetMovingParameters(const ImuDataPtr imu_data, bool flag_new_frame)
{
  static auto s_timestamp = imu_data->timestamp;
  static float angle_f[3] = {0.0f};
  static float ratation_matrix[9] = {0};
  static float deta_angle[3] = {0.0f};
  static int data_pack_num = 1;
  static EulerAngles rpy_start, rpy_new;
  static bool new_imu_message = true;

  const float time_gap = 0.1f / 64 / 21;

  if (imu_data->timestamp != s_timestamp)
  {
    s_timestamp = imu_data->timestamp;
    new_imu_message = true;
  }

  if (flag_new_frame)
  {
    rpy_start = ToEulerAngle(imu_data);
    angle_f[0] = angle_f[1] = angle_f[2] = 0.0f;
    deta_angle[0] = deta_angle[1] = deta_angle[2] = 0.0f;
  }
  else
  {
    angle_f[0] += time_gap * imu_data->angular_velocity.x + deta_angle[0];
    angle_f[1] += time_gap * imu_data->angular_velocity.y + deta_angle[1];
    angle_f[2] += time_gap * imu_data->angular_velocity.z + deta_angle[2];

    if (new_imu_message)
    {
      rpy_new = ToEulerAngle(imu_data);
      deta_angle[0] += (rpy_new.roll  - rpy_start.roll  - angle_f[0]) / data_pack_num;
      deta_angle[1] += (rpy_new.pitch - rpy_start.pitch - angle_f[1]) / data_pack_num;
      deta_angle[2] += (rpy_new.yaw   - rpy_start.yaw   - angle_f[2]) / data_pack_num;
      angle_f[0] = rpy_new.roll  - rpy_start.roll;
      angle_f[1] = rpy_new.pitch - rpy_start.pitch;
      angle_f[2] = rpy_new.yaw   - rpy_start.yaw;
      new_imu_message = false;
      data_pack_num = 0;
    }
    data_pack_num++;
  }

  float roll  =  angle_f[0] * 100;
  float pitch = -angle_f[1] * 100;
  float yaw   = -angle_f[2] * 100;

  ratation_matrix[0] =  COS(pitch) * COS(yaw);
  ratation_matrix[1] = -COS(pitch) * SIN(yaw);
  ratation_matrix[2] =  SIN(pitch);
  ratation_matrix[3] =  SIN(roll) * SIN(pitch) * COS(yaw) + COS(roll) * SIN(yaw);
  ratation_matrix[4] =  SIN(roll) * SIN(pitch) * SIN(yaw) + COS(roll) * COS(yaw);
  ratation_matrix[5] = -SIN(roll) * COS(pitch);
  ratation_matrix[6] =  COS(roll) * SIN(pitch) * COS(yaw) + SIN(roll) * SIN(yaw);
  ratation_matrix[7] =  COS(roll) * SIN(pitch) * SIN(yaw) + SIN(roll) * COS(yaw);
  ratation_matrix[8] =  COS(roll) * COS(pitch);

  return &ratation_matrix[0];
}
#endif // ENABLE_MOTION_COMPENSATION

template <typename T_PointCloud>
inline std::shared_ptr<BlockBuffer> Decoder<T_PointCloud>::blockGet(size_t w, size_t h, bool dual)
{
  const size_t size = w * h;

  std::shared_ptr<BlockBuffer> block = free_block_queue_.pop();
  if (!block || block->size() != size || (dual && block->points2_.empty()))
    return std::make_shared<BlockBuffer>(w, h, dual);

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
}  // namespace asensing
