#include <lidar_sdk/lidar_sdk.h>

int main(void) {
  lidar_sdk_config_t config;
  lidar_sdk_config_init(&config);

  return lidar_sdk_version() == 0 ? 1 : 0;
}
