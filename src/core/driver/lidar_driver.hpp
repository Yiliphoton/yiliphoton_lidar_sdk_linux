
#pragma once

#include "core/driver/lidar_driver_impl.hpp"
#include "core/msg/packet.hpp"

namespace lidar_sdk
{
namespace lidar
{

std::string getDriverVersion();

/**
 * @brief This is the LiDAR driver interface class
 */
template <typename T_PointCloud>
class LidarDriver
{
public:

  /**
   * @brief Constructor, instanciate the driver pointer
   */
  LidarDriver() 
    : driver_ptr_(std::make_shared<LidarDriverImpl<T_PointCloud>>())
  {
  }

  /**
   * @brief Register the lidar point cloud callback function to driver. When point cloud is ready, this function will be
   * called
   * @param callback The callback function
   */
  inline void regPointCloudCallback(const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get_cloud,
      const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put_cloud)
  {
    driver_ptr_->regPointCloudCallback(cb_get_cloud, cb_put_cloud);
  }

  /**
   * @brief Register the lidar difop packet message callback function to driver. When lidar difop packet message is
   * ready, this function will be called
   * @param callback The callback function
   */
  inline void regPacketCallback(const std::function<void(const Packet&)>& cb_put_pkt)
  {
    driver_ptr_->regPacketCallback(cb_put_pkt);
  }

  /**
   * @brief Register the exception message callback function to driver. When error occurs, this function will be called
   * @param callback The callback function
   */
  inline void regExceptionCallback(const std::function<void(const Error&)>& cb_excep)
  {
    driver_ptr_->regExceptionCallback(cb_excep);
  }

  inline void regImuCallback(const std::function<void(const LidarImuData&)>& cb_imu)
  {
    driver_ptr_->regImuCallback(cb_imu);
  }

  inline void ChangeViewerStopFlag(bool value)
  {
    driver_ptr_->implChangeViewerStopFlag(value);
  }
  
  inline void ChangeInitFlag(bool value)
  {
    driver_ptr_->implChangeInitFlag(value);
  }

  inline double GetFrameRate()
  {
    return driver_ptr_->implGetFrameRate();
  }

  /**
   * @brief The initialization function, used to set up parameters and instance objects,
   *        used when get packets from online lidar or pcap
   * @param param The custom struct DriverParam
   * @return If successful, return true; else return false
   */
  inline bool init(const DriverParam& param)
  {
    return driver_ptr_->init(param);
  }

  /**
   * @brief Start the thread to receive and decode packets
   * @return If successful, return true; else return false
   */
  inline bool start()
  {
    return driver_ptr_->start();
  }

  /**
   * @brief Decode lidar msop/difop messages
   * @param pkt_msg The lidar msop/difop packet
   */
  inline void decodePacket(const Packet& pkt)
  {
    driver_ptr_->decodePacket(pkt);
  }

  /**
   * @brief Get the current lidar temperature
   * @param temp The variable to store lidar temperature
   * @return if get temperature successfully, return true; else return false
   */
  inline bool getTemperature(float& temp)
  {
    return driver_ptr_->getTemperature(temp);
  }

  /**
   * @brief Get device info
   * @param info The variable to store device info
   * @return if get device info successfully, return true; else return false
   */
  inline bool getDeviceInfo(DeviceInfo& info)
  {
    return driver_ptr_->getDeviceInfo(info);
  }

  /**
   * @brief Get device status
   * @param info The variable to store device status
   * @return if get device info successfully, return true; else return false
   */
  inline bool getDeviceStatus(DeviceStatus& status)
  {
    return driver_ptr_->getDeviceStatus(status);
  }

  /**
   * @brief Stop all threads
   */
  inline void stop()
  {
    driver_ptr_->stop();
  }


private:
  std::shared_ptr<LidarDriverImpl<T_PointCloud>> driver_ptr_;  // The driver pointer
};

}  // namespace lidar
}  // namespace lidar_sdk
