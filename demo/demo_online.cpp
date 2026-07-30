#include "ylp_driver/api/lidar_driver.hpp"

#ifdef ENABLE_PCL_POINTCLOUD
#include "ylp_driver/msg/pcl_point_cloud_msg.hpp"
#else
#include "ylp_driver/msg/point_cloud_msg.hpp"
#endif

#include <signal.h>

//#define ORDERLY_EXIT
#define DEMO_DEBUG        0

// typedef PointXYZI PointT;
typedef PointXYZIRT PointT;
typedef PointCloudT<PointT> PointCloudMsg;

using namespace asensing::lidar;

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this fucntion, the driver gets an free/unused point cloud message from the caller.
// @param msg  The free/unused point cloud message.
//
std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
{
  // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here.
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}

//
// @brief point cloud callback function. The caller should register it to the lidar driver.
//        Via this function, the driver gets/returns a stuffed point cloud message to the caller. 
// @param msg  The stuffed point cloud message.
//
void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
{
  // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see processCloud() below)
  stuffed_cloud_queue.push(msg);
}

//
// @brief exception callback function. The caller should register it to the lidar driver.
//        Via this function, the driver inform the caller that something happens.
// @param code The error code to represent the error/warning/information
//
void exceptionCallback(const Error& code)
{
  // Note: This callback function runs in the packet-receving and packet-parsing/point-cloud_constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here.
  YLP_WARNING << code.toString() << YLP_REND;
}

bool to_exit_process = false;
void processCloud(void)
{
  while (!to_exit_process)
  {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }

    // Well, it is time to process the point cloud msg, even it is time-consuming.
    YLP_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << std::fixed << " ts: " << msg->timestamp
           << " timestamp offset: " << (getTimeHostWithAG() - msg->timestamp) << YLP_REND;

#if DEMO_DEBUG
    for (auto it = msg->points.begin(); it != msg->points.end(); it++)
    {
      std::cout << std::fixed << std::setprecision(3) 
                << "(" << it->x << ", " << it->y << ", " << it->z << ", " << (int)it->intensity << ")" 
                << std::endl;
    }
#endif

    free_cloud_queue.push(msg);
  }
}

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

  YLPDriverParam param;                   // Create a parameter object
  param.input_type = InputType::ONLINE_LIDAR;
  param.input_param.msop_port = 51180;   // Set the lidar msop port number, the default is 51180
  param.input_param.difop_port = 9988;   // Set the lidar difop port number, the default is 9988
  param.lidar_type = LidarType::V192;      // Set the lidar type. Make sure this type is correct
  param.decoder_param.wait_for_difop = false;
  param.decoder_param.use_lidar_clock = true;
  param.print();

  LidarDriver<PointCloudMsg> driver;               // Declare the driver object
  driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback, driverReturnPointCloudToCallerCallback); // Register the point cloud callback functions
  driver.regExceptionCallback(exceptionCallback);  // Register the exception callback function
  if (!driver.init(param))                         // Call the init function
  {
    YLP_ERROR << "Driver Initialize Error..." << YLP_REND;
    return -1;
  }

  std::thread cloud_handle_thread = std::thread(processCloud);
#ifndef WIN32
  pthread_setname_np(cloud_handle_thread.native_handle(), "process_cloud");
#endif

  driver.start();  // The driver thread will start
  YLP_DEBUG << "Yiliphoton Lidar-Driver Linux online demo start......" << YLP_REND;

#ifdef ORDERLY_EXIT
  std::this_thread::sleep_for(std::chrono::seconds(10));

  driver.stop();

  to_exit_process = true;
  cloud_handle_thread.join();
#else
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
#endif

  return 0;
}
