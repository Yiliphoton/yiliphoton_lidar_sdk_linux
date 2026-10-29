#include "lidar_sdk/lidar_sdk.h"

#include <stdio.h>

static void on_frame(const lidar_sdk_frame_t* frame, void* user_data) {
  (void)user_data;
  printf("frame=%llu points=%u timestamp=%.6f\n",
      (unsigned long long)frame->sequence, frame->point_count, frame->timestamp);
  fflush(stdout);
}

static void on_error(int32_t error_code, const char* message, void* user_data) {
  (void)user_data;
  fprintf(stderr, "driver error %d: %s\n", error_code, message);
}

int main(void) {
  lidar_sdk_config_t config;
  lidar_sdk_config_init(&config);
  config.model = LIDAR_SDK_MODEL_P60_IMU;
  config.msop_port = 8899;
  config.difop_port = 0;
  snprintf(config.host_address, sizeof(config.host_address), "%s", "0.0.0.0");
  snprintf(config.group_address, sizeof(config.group_address), "%s", "0.0.0.0");

  lidar_driver_t* driver = lidar_driver_create();
  lidar_driver_set_frame_callback(driver, on_frame, NULL);
  lidar_driver_set_error_callback(driver, on_error, NULL);
  if (lidar_driver_init(driver, &config) != LIDAR_SDK_SUCCESS ||
      lidar_driver_start(driver) != LIDAR_SDK_SUCCESS) {
    lidar_driver_destroy(driver);
    return 1;
  }

  printf("Receiving point clouds at %s:%u. Press Enter to stop.\n",
      config.host_address, (unsigned int)config.msop_port);
  (void)getchar();
  lidar_driver_stop(driver);
  lidar_driver_destroy(driver);
  return 0;
}
