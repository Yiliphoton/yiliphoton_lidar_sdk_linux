#define PCL_NO_PRECOMPILE
#pragma once
#ifdef PCL_USE_PCDIO
#include <pcl/io/pcd_io.h>
#else
#include <pcl/io/io.h>
#endif

#include <pcl/point_types.h>
//#include <pcl_conversion/pcl_conversion.h>

typedef pcl::PointXYZI PointXYZI;

struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint8_t ring;
#ifdef ENABLE_POINT_EXTEND_FIELD
  std::uint16_t azimuth;
  std::uint16_t elevation;
#endif
  float time;
  #ifdef COMPILE_TOOLS
  std::uint32_t FrameSn;
  #endif
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

#ifdef ENABLE_POINT_EXTEND_FIELD
#ifdef COMPILE_TOOLS
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, azimuth, azimuth)
    (std::uint16_t, elevation, elevation)
    (float, time, time)
    (std::uint32_t, FrameSn, FrameSn))
#else
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, azimuth, azimuth)
    (std::uint16_t, elevation, elevation)
    (float, time, time))
    
#endif // COMPILE_TOOLS
#else
#ifdef COMPILE_TOOLS
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint8_t, ring, ring)
    (float, time, time)
    (std::uint32_t, FrameSn, FrameSn))
#else
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, 
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint8_t, ring, ring)
    (float, time, time))
#endif // COMPILE_TOOLS
#endif // ENABLE_POINT_EXTEND_FIELD


struct PointXYZIRTC
{
  float x;
  float y;
  float z;
  uint8_t intensity;
  uint8_t ring;
  uint16_t range;
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

#ifdef ENABLE_POINT_EXTEND_FIELD
#ifdef COMPILE_TOOLS
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRTC,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint8_t, intensity, intensity)
    (uint8_t, ring, ring)
    (uint16_t, range, range)
    (uint16_t, azimuth, azimuth)
    (uint16_t, elevation, elevation)
    (double, timestamp, timestamp)
    (float, rgb, rgb)
    (uint32_t, FrameSn, FrameSn))
#else
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRTC,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint8_t, intensity, intensity)
    (uint8_t, ring, ring)
    (uint16_t, range, range)
    (uint16_t, azimuth, azimuth)
    (uint16_t, elevation, elevation)
    (double, timestamp, timestamp)
    (float, rgb, rgb))
#endif // COMPILE_TOOLS
#else
#ifdef COMPILE_TOOLS
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRTC,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint8_t, intensity, intensity)
    (uint8_t, ring, ring)
    (uint16_t, range, range)
    (double, timestamp, timestamp)
    (float, rgb, rgb)
    (uint32_t, FrameSn, FrameSn))
#else
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRTC,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint8_t, intensity, intensity)
    (uint8_t, ring, ring)
    (uint16_t, range, range)
    (double, timestamp, timestamp)
    (float, rgb, rgb))
#endif // COMPILE_TOOLS
#endif // ENABLE_POINT_EXTEND_FIELD

template <typename T_Point>
class PointCloudT : public pcl::PointCloud<T_Point>
{
public:
  typedef T_Point PointT;
  typedef typename pcl::PointCloud<T_Point>::VectorType VectorT;

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

  double timestamp = 0.0;
  uint32_t seq = 0;           // Sequence number of message
  uint32_t frame_id = 0;
  std::string fixed_frame = "";  // Point cloud frame id
  bool is_dual = false;
  bool is_complete = true;

  uint32_t height = 0;    // Height of point cloud
  uint32_t width = 0;     // Width of point cloud

  data_t data {};

  Eigen::Map < img_t < raw_t >> field(Field f) {
    return Eigen::Map < img_t < raw_t >> (data.col(f).data(), height, width);
  }

  Eigen::Map < data_t, Eigen::Unaligned, DynStride > block(size_t m_id) {
    return Eigen::Map < data_t, Eigen::Unaligned, DynStride > (
      data.row(m_id).data(), height, N_FIELDS, {width * height, width});
  }
};

