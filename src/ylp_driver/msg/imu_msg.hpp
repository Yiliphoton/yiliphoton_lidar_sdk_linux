
#pragma once
#include <memory>

struct ImuVector3 {
  float x;
  float y;
  float z;
};

struct ImuQuaternion {
  float x;
  float y;
  float z;
  float w;
};

struct EulerAngles
{
  double roll;
  double pitch;
  double yaw;
};

typedef struct ImuData {
  // 时间戳（int32 sec + uint32 nanosec）
  uint64_t timestamp;

  // 3轴线性加速度数据
  struct ImuVector3 linear_acceleration;

  // 3轴角速度数据
  struct ImuVector3 angular_velocity;

  // 3轴姿态（方向）估计数据（四元数表示）
  struct ImuQuaternion orientation;

  // 9轴姿态（方向）协方差矩阵数据（用于估计姿态的不确定性）
  float orientation_covariance[9];

  // 6轴线性加速度协方差矩阵数据（用于估计线性加速度的不确定性）
  float linear_acceleration_covariance[9];

  // 6轴角速度协方差矩阵数据（用于估计角速度的不确定性）
  float angular_velocity_covariance[9];
} ImuDataT;



#ifdef ROS2_FOUND
#include <sensor_msgs/msg/imu.hpp>
typedef sensor_msgs::msg::Imu::SharedPtr AGImuMsgPtr;

// using ImuDataPtr = std::shared_ptr<ImuDataT>;
typedef std::shared_ptr<ImuDataT> ImuDataPtr;
#endif

#ifdef ROS1_FOUND
#include <sensor_msgs/Imu.h>
typedef sensor_msgs::ImuConstPtr AGImuMsgPtr;

using ImuDataPtr = sensor_msgs::ImuConstPtr;
// typedef std::shared_ptr<ImuDataT> ImuDataPtr;
#endif

