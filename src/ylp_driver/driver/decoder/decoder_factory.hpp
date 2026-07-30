#pragma once

#include "ylp_driver/driver/decoder/decoder.hpp"
#include "ylp_driver/driver/decoder/V192/decoder_V192.hpp"
#include "ylp_driver/driver/decoder/P60/decoder_P60.hpp"

namespace asensing
{
namespace lidar
{

template <typename T_PointCloud>
class DecoderFactory
{
public:
  static std::shared_ptr<Decoder<T_PointCloud>> createDecoder(
      LidarType type, const YLPDecoderParam& param);
};

template <typename T_PointCloud>
inline std::shared_ptr<Decoder<T_PointCloud>> DecoderFactory<T_PointCloud>::createDecoder(
    LidarType type, const YLPDecoderParam& param)
{
  if (type == LidarType::V192)
    return std::make_shared<DecoderV192<T_PointCloud>>(param);
  if (type == LidarType::P60)
    return std::make_shared<DecoderP60<T_PointCloud>>(param);

  YLP_ERROR << "Unsupported LiDAR type. Supported: V192, P60." << YLP_REND;
  return nullptr;
}

}  // namespace lidar
}  // namespace asensing
