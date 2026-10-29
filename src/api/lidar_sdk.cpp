#include "lidar_sdk/lidar_sdk.h"

#include "core/driver/lidar_driver.hpp"
#include "core/common/version.hpp"
#include "core/msg/point_cloud_msg.hpp"
#include "core/msg/lidar_imu_data.hpp"

#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#ifndef _WIN32
#include <dlfcn.h>
#endif
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

namespace {

using InternalPointCloud = PointCloudT<PointXYZIRT>;

void libraryAnchor() {}

std::string configString(const char* value) {
  return std::string(value, strnlen(value, LIDAR_SDK_STRING_CAPACITY));
}

std::string defaultCalibrationPath(lidar_sdk_model_t model) {
  const char* filename = (model == LIDAR_SDK_MODEL_P60 || model == LIDAR_SDK_MODEL_P60_IMU)
      ? "P60-Correction.json" : "V192-Correction.json";
#ifdef _WIN32
  return filename;
#else
  Dl_info info {};
  if (dladdr(reinterpret_cast<void*>(&libraryAnchor), &info) == 0 || !info.dli_fname) {
    return filename;
  }

  std::string library_path(info.dli_fname);
  const std::string::size_type separator = library_path.rfind('/');
  if (separator == std::string::npos) {
    return filename;
  }

  return library_path.substr(0, separator) + "/../share/lidar_sdk/config/" + filename;
#endif
}

const char* validateDistanceRangeConfig(const lidar_sdk_config_t& config) {
  const float min_distance = config.min_distance_m;
  const float max_distance = config.max_distance_m;

  if (!std::isfinite(min_distance) || !std::isfinite(max_distance)) {
    return "invalid config: min_distance_m and max_distance_m must be finite numbers";
  }
  if (min_distance < 0.0F || max_distance < 0.0F) {
    return "invalid config: min_distance_m and max_distance_m must be non-negative";
  }
  if (min_distance == 0.0F && max_distance == 0.0F) {
    return nullptr;
  }
  if (min_distance > 0.0F && max_distance == 0.0F) {
    return "invalid config: max_distance_m must be set when min_distance_m is set";
  }
  if (min_distance >= max_distance) {
    return "invalid config: max_distance_m must be greater than min_distance_m";
  }
  return nullptr;
}

class DriverContext {
public:
  DriverContext() = default;

  ~DriverContext() {
    stop();
  }

  lidar_sdk_status_t setFrameCallback(lidar_sdk_frame_callback_t callback, void* user_data) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    frame_callback_ = callback;
    frame_user_data_ = user_data;
    return LIDAR_SDK_SUCCESS;
  }

  lidar_sdk_status_t setErrorCallback(lidar_sdk_error_callback_t callback, void* user_data) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    error_callback_ = callback;
    error_user_data_ = user_data;
    return LIDAR_SDK_SUCCESS;
  }

  lidar_sdk_status_t setImuCallback(lidar_sdk_imu_callback_t callback, void* user_data) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    imu_callback_ = callback;
    imu_user_data_ = user_data;
    return LIDAR_SDK_SUCCESS;
  }

  lidar_sdk_status_t init(const lidar_sdk_config_t& config) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (initialized_ || running_) {
      return LIDAR_SDK_INVALID_STATE;
    }

    lidar_sdk::lidar::DriverParam param;
    switch (config.model) {
      case LIDAR_SDK_MODEL_P60:
        param.lidar_type = lidar_sdk::lidar::LidarType::P60;
        break;
      case LIDAR_SDK_MODEL_P60_IMU:
        param.lidar_type = lidar_sdk::lidar::LidarType::P60_IMU;
        break;
      default:
        param.lidar_type = lidar_sdk::lidar::LidarType::V192;
        break;
    }
    param.input_type = config.input_type == LIDAR_SDK_INPUT_PCAP
        ? lidar_sdk::lidar::InputType::PCAP_FILE
        : lidar_sdk::lidar::InputType::ONLINE_LIDAR;
    param.input_param.msop_port = config.msop_port;
    param.input_param.difop_port = config.difop_port;
    param.input_param.host_address = configString(config.host_address);
    param.input_param.group_address = configString(config.group_address);
    param.input_param.pcap_path = configString(config.pcap_path);
    param.input_param.pcap_repeat = config.pcap_repeat != 0;
    param.input_param.pcap_rate = config.pcap_rate;
    param.decoder_param.use_lidar_clock = config.use_lidar_clock != 0;
    param.decoder_param.dense_points = config.dense_points != 0;
    param.decoder_param.echo_option = config.echo_option;
    param.decoder_param.min_distance = config.min_distance_m;
    param.decoder_param.max_distance = config.max_distance_m;
    param.decoder_param.calib_file = config.calibration_path[0] != '\0'
        ? configString(config.calibration_path)
        : defaultCalibrationPath(config.model);
    param.decoder_param.wait_for_difop = false;
    param.decoder_param.process_by_thread = true;

    driver_ = std::make_unique<lidar_sdk::lidar::LidarDriver<InternalPointCloud>>();
    driver_->regPointCloudCallback(
        [this] { return acquireCloud(); },
        [this](std::shared_ptr<InternalPointCloud> cloud) { enqueueCloud(std::move(cloud)); });
    driver_->regExceptionCallback(
        [this](const lidar_sdk::lidar::Error& error) { reportError(error); });
    driver_->regImuCallback(
        [this](const lidar_sdk::lidar::LidarImuData& data) { dispatchImu(data); });

    if (!driver_->init(param)) {
      driver_.reset();
      return LIDAR_SDK_INITIALIZATION_FAILED;
    }

    initialized_ = true;
    return LIDAR_SDK_SUCCESS;
  }

  lidar_sdk_status_t start() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!initialized_ || !driver_) {
      return LIDAR_SDK_INVALID_STATE;
    }
    if (running_) {
      return LIDAR_SDK_SUCCESS;
    }

    {
      std::lock_guard<std::mutex> output_lock(output_mutex_);
      output_stopping_ = false;
    }
    output_thread_ = std::thread(&DriverContext::dispatchFrames, this);
    if (!driver_->start()) {
      {
        std::lock_guard<std::mutex> output_lock(output_mutex_);
        output_stopping_ = true;
      }
      output_cv_.notify_all();
      output_thread_.join();
      return LIDAR_SDK_START_FAILED;
    }

    running_ = true;
    return LIDAR_SDK_SUCCESS;
  }

  lidar_sdk_status_t stop() {
    std::unique_lock<std::mutex> lock(state_mutex_);
    if (!running_) {
      return LIDAR_SDK_SUCCESS;
    }

    driver_->stop();
    running_ = false;
    {
      std::lock_guard<std::mutex> output_lock(output_mutex_);
      output_stopping_ = true;
    }
    output_cv_.notify_all();
    lock.unlock();

    if (output_thread_.joinable()) {
      output_thread_.join();
    }

    std::lock_guard<std::mutex> output_lock(output_mutex_);
    while (!output_queue_.empty()) {
      recycleCloud(std::move(output_queue_.front()));
      output_queue_.pop_front();
    }
    return LIDAR_SDK_SUCCESS;
  }

  void reportConfigError(const char* message) {
    reportErrorMessage(LIDAR_SDK_INVALID_ARGUMENT, message);
  }

private:
  std::shared_ptr<InternalPointCloud> acquireCloud() {
    std::lock_guard<std::mutex> lock(pool_mutex_);
    if (free_clouds_.empty()) {
      return std::make_shared<InternalPointCloud>();
    }

    std::shared_ptr<InternalPointCloud> cloud = std::move(free_clouds_.front());
    free_clouds_.pop_front();
    return cloud;
  }

  void recycleCloud(std::shared_ptr<InternalPointCloud> cloud) {
    if (!cloud) {
      return;
    }
    cloud->points.clear();
    std::lock_guard<std::mutex> lock(pool_mutex_);
    if (free_clouds_.size() < kMaxFreeClouds) {
      free_clouds_.push_back(std::move(cloud));
    }
  }

  void enqueueCloud(std::shared_ptr<InternalPointCloud> cloud) {
    std::shared_ptr<InternalPointCloud> dropped;
    {
      std::lock_guard<std::mutex> lock(output_mutex_);
      if (output_queue_.size() >= kMaxOutputFrames) {
        dropped = std::move(output_queue_.front());
        output_queue_.pop_front();
      }
      output_queue_.push_back(std::move(cloud));
    }
    recycleCloud(std::move(dropped));
    output_cv_.notify_one();
  }

  void dispatchFrames() {
    while (true) {
      std::shared_ptr<InternalPointCloud> cloud;
      {
        std::unique_lock<std::mutex> lock(output_mutex_);
        output_cv_.wait(lock, [this] { return output_stopping_ || !output_queue_.empty(); });
        if (output_stopping_) {
          return;
        }
        cloud = std::move(output_queue_.front());
        output_queue_.pop_front();
      }

      dispatchFrame(*cloud);
      recycleCloud(std::move(cloud));
    }
  }

  void dispatchFrame(const InternalPointCloud& cloud) {
    lidar_sdk_frame_callback_t callback = nullptr;
    void* user_data = nullptr;
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      callback = frame_callback_;
      user_data = frame_user_data_;
    }
    if (!callback) {
      return;
    }

    std::vector<lidar_sdk_point_t> points;
    points.reserve(cloud.points.size());
    for (const PointXYZIRT& source : cloud.points) {
      lidar_sdk_point_t point {};
      point.x = source.x;
      point.y = source.y;
      point.z = source.z;
      point.intensity = source.intensity;
      point.ring = static_cast<uint16_t>(source.ring);
      point.time = source.time;
      point.flags = std::isfinite(source.x) && std::isfinite(source.y) && std::isfinite(source.z) ? 0U : 1U;
      points.push_back(point);
    }

    lidar_sdk_frame_t frame {};
    frame.sequence = cloud.seq;
    frame.frame_id = cloud.frame_id;
    frame.point_count = static_cast<uint32_t>(points.size());
    frame.width = static_cast<uint16_t>(cloud.width);
    frame.height = static_cast<uint16_t>(cloud.height);
    frame.is_dense = cloud.is_dense ? 1U : 0U;
    frame.is_dual_return = cloud.is_dual ? 1U : 0U;
    frame.timestamp = cloud.timestamp;
    frame.points = points.data();

    try {
      callback(&frame, user_data);
    } catch (...) {
      reportErrorMessage(LIDAR_SDK_INTERNAL_ERROR, "frame callback threw an exception");
    }
  }

  void reportError(const lidar_sdk::lidar::Error& error) {
    reportErrorMessage(static_cast<int32_t>(error.error_code), error.toString().c_str());
  }

  void reportErrorMessage(int32_t code, const char* message) {
    lidar_sdk_error_callback_t callback = nullptr;
    void* user_data = nullptr;
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      callback = error_callback_;
      user_data = error_user_data_;
    }
    if (callback) {
      callback(code, message, user_data);
    }
  }

  void dispatchImu(const lidar_sdk::lidar::LidarImuData& data) {
    lidar_sdk_imu_callback_t callback = nullptr;
    void* user_data = nullptr;
    {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      callback = imu_callback_;
      user_data = imu_user_data_;
    }
    if (!callback) {
      return;
    }

    lidar_sdk_imu_t imu {};
    imu.timestamp = data.timestamp;
    imu.accel_x = data.accel_x;
    imu.accel_y = data.accel_y;
    imu.accel_z = data.accel_z;
    imu.gyro_x = data.gyro_x;
    imu.gyro_y = data.gyro_y;
    imu.gyro_z = data.gyro_z;
    imu.temperature = data.temperature;

    try {
      callback(&imu, user_data);
    } catch (...) {
      reportErrorMessage(LIDAR_SDK_INTERNAL_ERROR, "IMU callback threw an exception");
    }
  }

  static constexpr size_t kMaxFreeClouds = 8;
  static constexpr size_t kMaxOutputFrames = 4;

  std::mutex state_mutex_;
  std::unique_ptr<lidar_sdk::lidar::LidarDriver<InternalPointCloud>> driver_;
  bool initialized_ = false;
  bool running_ = false;

  std::mutex callback_mutex_;
  lidar_sdk_frame_callback_t frame_callback_ = nullptr;
  void* frame_user_data_ = nullptr;
  lidar_sdk_error_callback_t error_callback_ = nullptr;
  void* error_user_data_ = nullptr;
  lidar_sdk_imu_callback_t imu_callback_ = nullptr;
  void* imu_user_data_ = nullptr;

  std::mutex pool_mutex_;
  std::deque<std::shared_ptr<InternalPointCloud>> free_clouds_;

  std::mutex output_mutex_;
  std::condition_variable output_cv_;
  std::deque<std::shared_ptr<InternalPointCloud>> output_queue_;
  std::thread output_thread_;
  bool output_stopping_ = false;
};

DriverContext* context(lidar_driver_t* driver) {
  return reinterpret_cast<DriverContext*>(driver);
}

}  // namespace

extern "C" {

void lidar_sdk_config_init(lidar_sdk_config_t* config) {
  if (!config) {
    return;
  }
  std::memset(config, 0, sizeof(*config));
  config->model = LIDAR_SDK_MODEL_V192;
  config->input_type = LIDAR_SDK_INPUT_ONLINE;
  config->msop_port = 8899;
  config->difop_port = 0;
  config->pcap_repeat = 1;
  config->pcap_rate = 1.0F;
  config->echo_option = 0;
  std::strncpy(config->host_address, "0.0.0.0", sizeof(config->host_address) - 1U);
  std::strncpy(config->group_address, "0.0.0.0", sizeof(config->group_address) - 1U);
}

const char* lidar_sdk_version(void) {
  return LIDAR_SDK_VERSION_STRING;
}

lidar_driver_t* lidar_driver_create(void) {
  return reinterpret_cast<lidar_driver_t*>(new DriverContext());
}

void lidar_driver_destroy(lidar_driver_t* driver) {
  delete context(driver);
}

lidar_sdk_status_t lidar_driver_set_frame_callback(
    lidar_driver_t* driver, lidar_sdk_frame_callback_t callback, void* user_data) {
  return driver ? context(driver)->setFrameCallback(callback, user_data) : LIDAR_SDK_INVALID_ARGUMENT;
}

lidar_sdk_status_t lidar_driver_set_error_callback(
    lidar_driver_t* driver, lidar_sdk_error_callback_t callback, void* user_data) {
  return driver ? context(driver)->setErrorCallback(callback, user_data) : LIDAR_SDK_INVALID_ARGUMENT;
}

lidar_sdk_status_t lidar_driver_set_imu_callback(
    lidar_driver_t* driver, lidar_sdk_imu_callback_t callback, void* user_data) {
  return driver ? context(driver)->setImuCallback(callback, user_data) : LIDAR_SDK_INVALID_ARGUMENT;
}

lidar_sdk_status_t lidar_driver_init(
    lidar_driver_t* driver, const lidar_sdk_config_t* config) {
  if (!driver || !config) {
    return LIDAR_SDK_INVALID_ARGUMENT;
  }
  if (config->model != LIDAR_SDK_MODEL_P60 && config->model != LIDAR_SDK_MODEL_V192 &&
      config->model != LIDAR_SDK_MODEL_P60_IMU) {
    context(driver)->reportConfigError(
        "invalid config: model must be V192, P60, or P60_IMU");
    return LIDAR_SDK_INVALID_ARGUMENT;
  }
  if (config->input_type != LIDAR_SDK_INPUT_ONLINE && config->input_type != LIDAR_SDK_INPUT_PCAP) {
    context(driver)->reportConfigError(
        "invalid config: input_type must be LIDAR_SDK_INPUT_ONLINE or LIDAR_SDK_INPUT_PCAP");
    return LIDAR_SDK_INVALID_ARGUMENT;
  }
  if (config->echo_option > 3U) {
    context(driver)->reportConfigError(
        "invalid config: echo_option must be 0 (merge), 1 (first), 2 (second), or 3 (dual)");
    return LIDAR_SDK_INVALID_ARGUMENT;
  }
  if (const char* error = validateDistanceRangeConfig(*config)) {
    context(driver)->reportConfigError(error);
    return LIDAR_SDK_INVALID_ARGUMENT;
  }
  return context(driver)->init(*config);
}

lidar_sdk_status_t lidar_driver_start(lidar_driver_t* driver) {
  return driver ? context(driver)->start() : LIDAR_SDK_INVALID_ARGUMENT;
}

lidar_sdk_status_t lidar_driver_stop(lidar_driver_t* driver) {
  return driver ? context(driver)->stop() : LIDAR_SDK_INVALID_ARGUMENT;
}

}  // extern "C"
