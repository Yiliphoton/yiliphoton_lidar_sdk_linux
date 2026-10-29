#pragma once

#include "core/driver/decoder/decoder.hpp"
#include "core/driver/decoder/V192/decoder_V192.hpp"
#include "core/driver/decoder/P60/decoder_P60.hpp"
#include "core/driver/decoder/P60_IMU/decoder_P60_IMU.hpp"

namespace lidar_sdk
{
namespace lidar
{

template <typename T_PointCloud>
class DecoderFactory
{
public:
  static std::shared_ptr<Decoder<T_PointCloud>> createDecoder(
      LidarType type, const DecoderParam& param);
};

template <typename T_PointCloud>
inline std::shared_ptr<Decoder<T_PointCloud>> DecoderFactory<T_PointCloud>::createDecoder(
    LidarType type, const DecoderParam& param)
{
  if (type == LidarType::V192)
    return std::make_shared<DecoderV192<T_PointCloud>>(param);
  if (type == LidarType::P60)
    return std::make_shared<DecoderP60<T_PointCloud>>(param);
  if (type == LidarType::P60_IMU)
    return std::make_shared<DecoderP60IMU<T_PointCloud>>(param);

  LOG_ERROR << "Unsupported LiDAR type. Supported: V192, P60, P60_IMU." << LOG_REND;
  return nullptr;
}

}  // namespace lidar
}  // namespace lidar_sdk
