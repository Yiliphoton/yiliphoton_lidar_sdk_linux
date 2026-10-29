#pragma once

#include <cstdint>

namespace lidar_sdk
{
namespace lidar
{

struct LidarImuData
{
  double timestamp;
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float temperature;
};

}  // namespace lidar
}  // namespace lidar_sdk
