#pragma once

#include <vector>
#include <string>
#include <Eigen/Eigen>
// #include "ylp_driver/common/lidar_scan.hpp"

struct PointXYZI
{
  float x;
  float y;
  float z;
  uint8_t intensity;
};

struct PointXYZIRT
{
  float x;
  float y;
  float z;
  uint8_t intensity;
  uint8_t ring;
  uint16_t range;
  float confidence;
#ifdef ENABLE_POINT_EXTEND_FIELD
  uint16_t azimuth;
  uint16_t elevation;
#endif
  double timestamp;
#ifdef COMPILE_TOOLS
  uint32_t FrameSn;
#endif
};

struct PointXYZIRTC
{
  float x;
  float y;
  float z;
  uint8_t intensity;
  uint8_t ring;
  uint16_t range;
  float confidence;
#ifdef ENABLE_POINT_EXTEND_FIELD
  uint16_t azimuth;
  uint16_t elevation;
#endif
  double timestamp;
  float rgb;
#ifdef COMPILE_TOOLS
  uint32_t FrameSn;
#endif
};

template <typename T_Point>
class PointCloudT
{
public:
  typedef T_Point PointT;
  typedef std::vector<PointT> VectorT;

  static constexpr int N_FIELDS = 6;
  template < typename T >
  using img_t = Eigen::Array < T, -1, -1, Eigen::RowMajor >;
  using raw_t = double;
  using data_t = Eigen::Array < raw_t, Eigen::Dynamic, N_FIELDS >;

  using DynStride = Eigen::Stride < Eigen::Dynamic, Eigen::Dynamic >;

  /** XYZ coordinates with dimensions arranged contiguously in columns */
  using Points = Eigen::Array < double, Eigen::Dynamic, 3 >;

  /** Data fields reported per channel */
  enum Field { RANGE, INTENSITY, CONFIDENCE, PW, AZIMUTH, ELEVATION };

  PointCloudT()
  {
    // data.resize(999 * 96, N_FIELDS);
  }

  PointCloudT(size_t w, size_t h)
  : width {static_cast <uint32_t> (w)},
    height {static_cast <uint32_t> (h)},
    data {width * height, N_FIELDS}
  {
    
  }

  ~PointCloudT() = default;

  bool setArraySize(size_t w, size_t h)
  {
    width = w;
    height = h;
    data.resize(width * height, N_FIELDS);
    
    return true;
  }

  Eigen::Map < img_t < raw_t >> field(Field f) {
    return Eigen::Map < img_t < raw_t >> (data.col(f).data(), height, width);
  }

  Eigen::Map < data_t, Eigen::Unaligned, DynStride > block(size_t m_id) {
    return Eigen::Map < data_t, Eigen::Unaligned, DynStride > (
      data.row(m_id).data(), height, N_FIELDS, {width * height, width});
  }

  uint32_t height = 0;    // Height of point cloud
  uint32_t width = 0;     // Width of point cloud
  bool is_dense = false;  // If is_dense is true, the point cloud does not contain NAN points,
  bool is_dual = false;
  bool is_complete = true;
  double timestamp = 0.0;
  uint32_t seq = 0;           // Sequence number of message
  uint32_t frame_id = 0;
  std::string fixed_frame = "";  // Point cloud frame id

  // std::vector<PointT> points;
  VectorT points;
  data_t data {};
};
