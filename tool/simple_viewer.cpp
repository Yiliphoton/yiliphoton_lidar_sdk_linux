#include "ylp_driver/api/lidar_driver.hpp"
#include "ylp_driver/msg/pcl_point_cloud_msg.hpp"

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>

// #define ENABLE_RANGE_IMAGE

using namespace asensing::lidar;
using namespace pcl::visualization;

typedef PointCloudT<PointXYZIRT> PointCloudMsg;
typedef PointXYZIRT PCLMsg;

std::shared_ptr<PCLVisualizer> pcl_viewer;
std::mutex mtx_viewer;

#ifdef ENABLE_RANGE_IMAGE
pcl::RangeImage::Ptr range_image_ptr;
// pcl::RangeImage& range_image = *range_image_ptr;
std::shared_ptr<RangeImageVisualizer> range_image_widget;
// pcl::visualization::RangeImageVisualizer range_image_widget;
#endif
SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;

// --------------------
// -----Parameters-----
// --------------------
#ifdef ENABLE_RANGE_IMAGE
float noise_level = 0.0f;
float min_range = 0.0f;
int border_size = 1;

float angular_resolution_x = 0.5f,
      angular_resolution_y = angular_resolution_x;
// pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
#endif
bool ViewerStopFlag =false;

bool checkKeywordExist(int argc, const char* const* argv, const char* str)
{
  for (int i = 1; i < argc; i++)
  {
    if (strcmp(argv[i], str) == 0)
    {
      return true;
    }
  }
  return false;
}

bool parseArgument(int argc, const char* const* argv, const char* str, std::string& val)
{
  int index = -1;

  for (int i = 1; i < argc; i++)
  {
    if (strcmp(argv[i], str) == 0)
    {
      index = i + 1;
    }
  }

  if (index > 0 && index < argc)
  {
    val = argv[index];
    return true;
  }

  return false;
}

void parseParam(int argc, char* argv[], YLPDriverParam& param)
{
  std::string result_str;

  //
  // input param
  //
  parseArgument(argc, argv, "-pcap", param.input_param.pcap_path);
  if (param.input_param.pcap_path.empty())
  {
    param.input_type = InputType::ONLINE_LIDAR;
  }
  else
  {
    param.input_type = InputType::PCAP_FILE;
    param.input_param.pcap_repeat = false;
  }

  if (parseArgument(argc, argv, "-msop", result_str))
  {
    param.input_param.msop_port = std::stoi(result_str);
  }

  if (parseArgument(argc, argv, "-difop", result_str))
  {
    param.input_param.difop_port = std::stoi(result_str);
  }

  parseArgument(argc, argv, "-group", param.input_param.group_address);
  parseArgument(argc, argv, "-host", param.input_param.host_address);
 
  //
  // decoder param
  //
  if (parseArgument(argc, argv, "-type", result_str))
  {
    param.lidar_type = strToLidarType(result_str);
  }

  parseArgument(argc, argv, "-calib", param.decoder_param.calib_file);
  
  param.decoder_param.wait_for_difop = false;

  if (parseArgument(argc, argv, "-x", result_str))
  {
    param.decoder_param.transform_param.x = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-y", result_str))
  {
    param.decoder_param.transform_param.y = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-z", result_str))
  {
    param.decoder_param.transform_param.z = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-roll", result_str))
  {
    param.decoder_param.transform_param.roll = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-elevation", result_str))
  {
    param.decoder_param.transform_param.elevation = std::stof(result_str);
  }

  if (parseArgument(argc, argv, "-azimuth", result_str))
  {
    param.decoder_param.transform_param.azimuth = std::stof(result_str);
  }
}

void printHelpMenu()
{
  YLP_MSG << "Arguments: " << YLP_REND;
  YLP_MSG << "  -type   = LiDAR type (A0, A2)" << YLP_REND;
  YLP_MSG << "  -pcap   = The path of the pcap file, off-line mode if it is true, else online mode." << YLP_REND;
  YLP_MSG << "  -calib  = The calibration file name, if not specified, use default calibration file." << YLP_REND;
  YLP_MSG << "  -msop   = LiDAR msop port number,the default value is 51180" << YLP_REND;
  YLP_MSG << "  -difop  = LiDAR difop port number,the default value is 9988" << YLP_REND;
  YLP_MSG << "  -group  = LiDAR destination group address if multi-cast mode." << YLP_REND;
  YLP_MSG << "  -host   = Host address." << YLP_REND;
  YLP_MSG << "  -x      = Transformation parameter, unit: m " << YLP_REND;
  YLP_MSG << "  -y      = Transformation parameter, unit: m " << YLP_REND;
  YLP_MSG << "  -z      = Transformation parameter, unit: m " << YLP_REND;
  YLP_MSG << "  -roll   = Transformation parameter, unit: radian " << YLP_REND;
  YLP_MSG << "  -elevation  = Transformation parameter, unit: radian " << YLP_REND;
  YLP_MSG << "  -azimuth    = Transformation parameter, unit: radian " << YLP_REND;
  YLP_MSG << "" << YLP_REND;
  YLP_MSG << "Example: simple_viewer -type A0 -pcap /opt/data/A0-test.pcap" << YLP_REND;
}

void exceptionCallback(const Error& code)
{
  YLP_WARNING << code.toString() << YLP_REND;
}

std::shared_ptr<PointCloudMsg> pointCloudGetCallback(void)
{
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}

void pointCloudPutCallback(std::shared_ptr<PointCloudMsg> msg)
{
  stuffed_cloud_queue.push(msg);
}

bool to_exit_process = false;
void processCloud(void)
{
  std::deque<double> fps_queue {};
  std::size_t fps_size = 10;
  double duration_min = 0.02;

  while (!to_exit_process)
  {
    auto start_time = std::chrono::high_resolution_clock::now();

    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }

    auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate time duration in seconds
    double duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1e6;
    double fps = 1.0 / duration;

    if (duration > duration_min) {
      fps_queue.push_back(fps);
    }

    if (fps_queue.size() > fps_size) {
      fps_queue.pop_front();
    }

    double sum = 0.0;
    for (const auto& value : fps_queue) {
      sum += value;
    }

    fps = sum / fps_queue.size();

    // Get the current rendering framerate
    
    YLP_DEBUG << "[" << msg->frame_id << "] Width = " << msg->width << ", Height = " << msg->height 
             << ", Total = " << msg->size() << ", is_dual = " << msg->is_dual << YLP_REND;
    // YLP_INFOL << msg->data << YLP_REND;

    if (!msg->is_complete) {
      YLP_WARNING << "This frame is incomplete" << YLP_REND;
      free_cloud_queue.push(msg);
      continue;
    }
    //
    // show the point cloud
    //
    pcl::PointCloud<PCLMsg>::Ptr pcl_pointcloud(new pcl::PointCloud<PCLMsg>);
    pcl_pointcloud->points.swap(msg->points);
    pcl_pointcloud->height = msg->height;
    pcl_pointcloud->width = msg->width;
    pcl_pointcloud->is_dense = msg->is_dense;

    PointCloudColorHandlerGenericField<PCLMsg> point_color_handle(pcl_pointcloud, "range");

    {
      const std::lock_guard<std::mutex> lock(mtx_viewer);
      pcl_viewer->updatePointCloud<PCLMsg>(pcl_pointcloud, point_color_handle, "ylplidar");
    }

    // show frame per second
    pcl_viewer->removeAllShapes();
    pcl_viewer->addText(std::to_string(fps) + " FPS", 10, 10, "fps_text", 0); // pcl_viewer->getFPS() is incorrect

#ifdef ENABLE_RANGE_IMAGE
    {
      pcl::PointCloud<PCLMsg>& point_cloud = *pcl_pointcloud;

      pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
      Eigen::Affine3f scene_sensor_pose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);

    // Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
    // scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f (pcl_pointcloud->sensor_origin_[0],
    //                                                          pcl_pointcloud->sensor_origin_[1],
    //                                                          pcl_pointcloud->sensor_origin_[2])) *
    //                     Eigen::Affine3f(pcl_pointcloud->sensor_orientation_);


      pcl::RangeImage& range_image = *range_image_ptr;
      float angularResolutionX = (float)(0.1f * (M_PI / 180.0f));
      float angularResolutionY = (float)(0.26f * (M_PI / 180.0f));
      float maxAngleWidth = (float)(120.0f * (M_PI / 180.0f));
      float maxAngleHeight = (float)(25.0f * (M_PI / 180.0f));

      if (!point_cloud.empty()) {

      range_image.createFromPointCloud (point_cloud, angularResolutionX, angularResolutionY,
                                    maxAngleWidth, maxAngleHeight,
                                    scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

//      range_image.createFromPointCloud (point_cloud, angular_resolution_x, angular_resolution_y,
//                                    pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
//                                    scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

      range_image_widget->showRangeImage(range_image);
      }
    }
#endif

    free_cloud_queue.push(msg);
  }
}

int main(int argc, char* argv[])
{
  YLP_TITLE << "------------------------------------------------------" << YLP_REND;
  YLP_TITLE << "            YLP_Driver Viewer Version: v" << getDriverVersion() << YLP_REND;
  YLP_TITLE << "------------------------------------------------------" << YLP_REND;

  if (argc < 2)
  {
    printHelpMenu();
    return 0;
  }

  if (checkKeywordExist(argc, argv, "-h") || checkKeywordExist(argc, argv, "--help"))
  {
    printHelpMenu();
    return 0;
  }

  std::thread cloud_handle_thread = std::thread(processCloud);
#ifndef WIN32
  pthread_setname_np(cloud_handle_thread.native_handle(), "process_cloud");
#endif

  YLPDriverParam param;
  param.input_param.pcap_rate = 0.5;
  param.decoder_param.enable_fov_aligned = true;
  param.decoder_param.process_by_thread = true;
  param.decoder_param.transform_param.x = 0.0;
  param.decoder_param.transform_param.y = 0.0;
  param.decoder_param.transform_param.z = 0.0;
  parseParam(argc, argv, param);
  param.print();

  pcl_viewer = std::make_shared<PCLVisualizer>("Simple PointCloud Viewer");
  pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0);
  // pcl_viewer->addText ("Raw point clouds", 10, 10, "v1_text", 1);
  pcl_viewer->setCameraPosition(0, 0, 100, 0, 0, -1, 1, 0, 0);
  pcl_viewer->addCoordinateSystem(1.0);
  pcl_viewer->setShowFPS(false);

#ifdef ENABLE_RANGE_IMAGE
  range_image_widget = std::make_shared<RangeImageVisualizer>("Range Image");

  // pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
  range_image_ptr.reset(new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;
  // range_image = *range_image_ptr;

//  range_image_widget->showRangeImage(range_image);
#endif

  pcl::PointCloud<PCLMsg>::Ptr pcl_pointcloud(new pcl::PointCloud<PCLMsg>);
  pcl::visualization::PointCloudColorHandlerGenericField<PCLMsg> fildColor(pcl_pointcloud, "intensity");
  pcl_viewer->addPointCloud<PCLMsg>(pcl_pointcloud, "ylplidar");
  pcl_viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, 2, "ylplidar");
  
  LidarDriver<PointCloudMsg> driver;
  driver.regExceptionCallback(exceptionCallback);
  driver.regPointCloudCallback(pointCloudGetCallback, pointCloudPutCallback);
  if (!driver.init(param))
  {
    YLP_ERROR << "Driver Initialize Error..." << YLP_REND;
    return -1;
  }

  YLP_DEBUG << "Point Type : XYZIRT (" << sizeof(PointXYZIRT) << " bytes)" << YLP_REND;
  YLP_INFO << "Yiliphoton Lidar-Driver Viewer start......" << YLP_REND;

  driver.start();

  while (!pcl_viewer->wasStopped())
  {
//    range_image_widget->showRangeImage(range_image);

    {
      const std::lock_guard<std::mutex> lock(mtx_viewer);
      pcl_viewer->spinOnce();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  driver.stop();

  to_exit_process = true;
  cloud_handle_thread.join();

  return 0;
}
