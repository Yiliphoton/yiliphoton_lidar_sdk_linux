
#pragma once

#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <vector>
#include <array>
#include <Eigen/Eigen>

namespace lidar_sdk
{
namespace lidar
{

/* For filter algorithm */
typedef struct
{
  double timestamp;
  float  distance;
  float  azimuth;
  float  elevation;
  uint8_t  reflectivity;
  uint8_t  confidence;
  uint8_t  ring;
} RawPointT;

typedef struct
{
  RawPointT *points;
  size_t     size;
} lidarPointCloud;

class BlockBuffer
{
public:
  static constexpr int N_FIELDS = 6;
  template < typename T >
  using img_t = Eigen::Array < T, -1, -1, Eigen::RowMajor >;
  using raw_t = double; // uint32_t
  using data_t = Eigen::Array < raw_t, Eigen::Dynamic, N_FIELDS >;
  // using data_t = Eigen::ArrayXd < Eigen::Dynamic, N_FIELDS >;

  using DynStride = Eigen::Stride < Eigen::Dynamic, Eigen::Dynamic >;

  /** XYZ coordinates with dimensions arranged contiguously in columns */
  using Points = Eigen::Array < double, Eigen::Dynamic, 3 >;

  /** Data fields reported per channel */
  enum Field { RANGE, INTENSITY, CONFIDENCE, PW, AZIMUTH, ELEVATION };

  BlockBuffer(size_t w, size_t h, bool dual)
  : width {static_cast < std::ptrdiff_t > (w)},
    height {static_cast < std::ptrdiff_t > (h)},
    points_size_ {w * h},
    data {points_size_, N_FIELDS},
    is_dual(dual)
  {
    points_.resize(points_size_); // reserve
    std::fill(points_.begin(), points_.end(), RawPointT{});
    data.setZero();
    payload_flags.fill(0);

    if (dual) {
      points2_.resize(points_size_);
      std::fill(points2_.begin(), points2_.end(), RawPointT{});
    }
  }

  ~BlockBuffer() = default;

  // Keep the class layout unchanged: vendor algorithm libraries may be built
  // against this ABI. Only clear existing storage before a pooled buffer is reused.
  void resetForReuse(size_t w, size_t h, bool dual)
  {
    width = static_cast<std::ptrdiff_t>(w);
    height = static_cast<std::ptrdiff_t>(h);
    points_size_ = w * h;
    is_dual = dual;
    is_complete = true;
    is_aligned = false;
    points_offset = 0;
    frame_id = 0;

    if (points_.size() != points_size_)
      points_.resize(points_size_);
    std::fill(points_.begin(), points_.end(), RawPointT{});

    if (dual)
    {
      if (points2_.size() != points_size_)
        points2_.resize(points_size_);
      std::fill(points2_.begin(), points2_.end(), RawPointT{});
    }
    else
    {
      points2_.clear();
    }

    data.resize(points_size_, N_FIELDS);
    data.setZero();
    payload_flags.fill(0);
  }


  std::vector<RawPointT> getData()
  {
    return points_;
  }

  size_t size() const
  {
    return points_size_;
  }

  Eigen::Map < img_t < raw_t >> field(Field f) {
    return Eigen::Map < img_t < raw_t >> (data.col(f).data(), height, width);
  }

  Eigen::Map < data_t, Eigen::Unaligned, DynStride > block(size_t m_id) {
    return Eigen::Map < data_t, Eigen::Unaligned, DynStride > (
      data.row(m_id).data(), height, N_FIELDS, {width * height, width});
  }

  bool is_dual = false;
  bool is_complete = true;
  bool is_aligned = false;
  uint8_t points_offset = 0;
  
  uint32_t frame_id = 0;
  size_t points_size_;
  std::ptrdiff_t width {0};
  std::ptrdiff_t height {0};
  std::array<uint8_t, 1300> payload_flags;

  std::vector<RawPointT> points_;   // Echo one
  std::vector<RawPointT> points2_;  // Echo two
  data_t data {};                   // Matrix
};
}  // namespace lidar
}  // namespace lidar_sdk
