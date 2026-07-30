#include "ylp_driver/api/lidar_driver.hpp"

#ifdef ENABLE_PCL_POINTCLOUD
#include "ylp_driver/msg/pcl_point_cloud_msg.hpp"
#else
#include "ylp_driver/msg/point_cloud_msg.hpp"
#endif

#include <signal.h>

//#define ORDERLY_EXIT
#define DEMO_DEBUG        0

typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

using namespace asensing::lidar;

class DriverClient
{
public:

  DriverClient(const std::string name)
    : name_(name)
  {
  }

  bool init(const YLPDriverParam& param)
  {
    YLP_INFO << "------------------------------------------------------" << YLP_REND;
    YLP_INFO << "                      " << name_ << YLP_REND;
    YLP_INFO << "------------------------------------------------------" << YLP_REND;
    param.print();

    driver_.regPointCloudCallback (std::bind(&DriverClient::driverGetPointCloudFromCallerCallback, this),
                                  std::bind(&DriverClient::driverReturnPointCloudToCallerCallback, this, std::placeholders::_1));
    driver_.regExceptionCallback (std::bind(&DriverClient::exceptionCallback, this, std::placeholders::_1));

    if (!driver_.init(param))
    {
      YLP_ERROR << name_ << ": Failed to initialize driver." << YLP_REND;
      return false;
    }

    return true;
  }

  bool start()
  {
    to_exit_process_ = false;
    cloud_handle_thread_ = std::thread(std::bind(&DriverClient::processCloud, this));
#ifndef WIN32
    pthread_setname_np(cloud_handle_thread_.native_handle(), "process_cloud");
#endif

    driver_.start();
    YLP_DEBUG << name_ << ": Started driver." << YLP_REND;

    return true;
  }

  void stop()
  {
    driver_.stop();

    to_exit_process_ = true;
    cloud_handle_thread_.join();
  }

protected:

  std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
  {
    std::shared_ptr<PointCloudMsg> msg = free_cloud_queue_.pop();
    if (msg.get() != NULL)
    {
      return msg;
    }

    return std::make_shared<PointCloudMsg>();
  }

  void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
  {
    stuffed_cloud_queue_.push(msg);
  }

  void processCloud(void)
  {
    while (!to_exit_process_)
    {
      std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue_.popWait();
      if (msg.get() == NULL)
      {
        continue;
      }

      YLP_MSG << name_ << ": msg: " << msg->seq << " point cloud size: " << msg->points.size() << YLP_REND;

      free_cloud_queue_.push(msg);
    }
  }

  void exceptionCallback(const Error& code)
  {
    YLP_WARNING << name_ << ": " << code.toString() << YLP_REND;
  }

  std::string name_;
  LidarDriver<PointCloudMsg> driver_;
  bool to_exit_process_;
  std::thread cloud_handle_thread_;
  SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue_;
  SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue_;
};

static void sigHandler(int sig)
{
  YLP_MSG << "stop." << YLP_REND;
  exit(1);
}

int main(int argc, char* argv[])
{
  signal(SIGINT, sigHandler);  // bind ctrl+c signal with the sigHandler function
  
  YLP_TITLE << "------------------------------------------------------" << YLP_REND;
  YLP_TITLE << "            YLP_Driver Core Version: v" << getDriverVersion() << YLP_REND;
  YLP_TITLE << "------------------------------------------------------" << YLP_REND;

  YLPDriverParam param_left;                  // Create a parameter object
  param_left.input_type = InputType::ONLINE_LIDAR;
  param_left.input_param.msop_port = 51180;  // Set the lidar msop port number, the default is 51180
  param_left.input_param.difop_port = 9988;  // Set the lidar difop port number, the default is 9988
  param_left.lidar_type = LidarType::V192;     // Set the lidar type. Make sure this type is correct

  DriverClient client_left("LEFT ");
  if (!client_left.init(param_left))                         // Call the init function
  {
    return -1;
  }

  YLPDriverParam param_right;                  // Create a parameter object
  param_right.input_type = InputType::ONLINE_LIDAR;
  param_right.input_param.msop_port = 51181;  // Set the lidar msop port number, the default is 51180
  param_right.input_param.difop_port = 9989;  // Set the lidar difop port number, the default is 9988
  param_right.lidar_type = LidarType::V192;     // Set the lidar type. Make sure this type is correct

  DriverClient client_right("RIGHT");
  if (!client_right.init(param_right))                         // Call the init function
  {
    return -1;
  }

  client_left.start();
  client_right.start();

#ifdef ORDERLY_EXIT
  std::this_thread::sleep_for(std::chrono::seconds(10));

  client_left.stop();
  client_right.stop();
#else
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
#endif

  return 0;
}
