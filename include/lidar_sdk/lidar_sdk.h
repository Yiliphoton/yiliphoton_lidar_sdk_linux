#ifndef LIDAR_SDK_H_
#define LIDAR_SDK_H_

#include <stdint.h>

#if defined(_WIN32)
#if defined(LIDAR_SDK_BUILDING_LIBRARY)
#define LIDAR_SDK_API __declspec(dllexport)
#else
#define LIDAR_SDK_API __declspec(dllimport)
#endif
#elif defined(__GNUC__) || defined(__clang__)
#define LIDAR_SDK_API __attribute__((visibility("default")))
#else
#define LIDAR_SDK_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define LIDAR_SDK_STRING_CAPACITY 256U

typedef struct lidar_driver lidar_driver_t;

typedef enum {
  LIDAR_SDK_SUCCESS = 0,
  LIDAR_SDK_INVALID_ARGUMENT = -1,
  LIDAR_SDK_INVALID_STATE = -2,
  LIDAR_SDK_INITIALIZATION_FAILED = -3,
  LIDAR_SDK_START_FAILED = -4,
  LIDAR_SDK_INTERNAL_ERROR = -5
} lidar_sdk_status_t;

typedef enum {
  LIDAR_SDK_MODEL_V192 = 1,
  LIDAR_SDK_MODEL_P60 = 2,
  LIDAR_SDK_MODEL_P60_IMU = 3
} lidar_sdk_model_t;

typedef enum {
  LIDAR_SDK_INPUT_ONLINE = 1,
  LIDAR_SDK_INPUT_PCAP = 2
} lidar_sdk_input_type_t;

typedef struct {
  float x;
  float y;
  float z;
  float intensity;
  uint16_t ring;
  float time;
  uint32_t flags;
} lidar_sdk_point_t;

typedef struct {
  uint64_t sequence;
  uint32_t frame_id;
  uint32_t point_count;
  uint16_t width;
  uint16_t height;
  uint8_t is_dense;
  uint8_t is_dual_return;
  double timestamp;
  const lidar_sdk_point_t* points;
} lidar_sdk_frame_t;

typedef struct {
  lidar_sdk_model_t model;
  lidar_sdk_input_type_t input_type;
  uint16_t msop_port;
  uint16_t difop_port;
  uint8_t use_lidar_clock;
  uint8_t dense_points;
  uint8_t pcap_repeat;
  float pcap_rate;
  /* Distance clipping range in metres. Defaults select 0.0~300.0 m.
     For a custom range, values must be finite/non-negative and max_distance_m > min_distance_m. */
  float min_distance_m;
  float max_distance_m;
  char host_address[LIDAR_SDK_STRING_CAPACITY];
  char group_address[LIDAR_SDK_STRING_CAPACITY];
  char pcap_path[LIDAR_SDK_STRING_CAPACITY];
  char calibration_path[LIDAR_SDK_STRING_CAPACITY];
  /* Echo selection: 0=merge, 1=first echo, 2=second echo, 3=dual echo. */
  uint8_t echo_option;
} lidar_sdk_config_t;

typedef void (*lidar_sdk_frame_callback_t)(
    const lidar_sdk_frame_t* frame, void* user_data);
typedef void (*lidar_sdk_error_callback_t)(
    int32_t error_code, const char* message, void* user_data);

typedef struct {
  double timestamp;
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float temperature;
} lidar_sdk_imu_t;

typedef void (*lidar_sdk_imu_callback_t)(
    const lidar_sdk_imu_t* imu, void* user_data);

LIDAR_SDK_API void lidar_sdk_config_init(lidar_sdk_config_t* config);
LIDAR_SDK_API const char* lidar_sdk_version(void);

LIDAR_SDK_API lidar_driver_t* lidar_driver_create(void);
LIDAR_SDK_API void lidar_driver_destroy(lidar_driver_t* driver);

LIDAR_SDK_API lidar_sdk_status_t lidar_driver_set_frame_callback(
    lidar_driver_t* driver, lidar_sdk_frame_callback_t callback, void* user_data);
LIDAR_SDK_API lidar_sdk_status_t lidar_driver_set_error_callback(
    lidar_driver_t* driver, lidar_sdk_error_callback_t callback, void* user_data);
LIDAR_SDK_API lidar_sdk_status_t lidar_driver_set_imu_callback(
    lidar_driver_t* driver, lidar_sdk_imu_callback_t callback, void* user_data);
LIDAR_SDK_API lidar_sdk_status_t lidar_driver_init(
    lidar_driver_t* driver, const lidar_sdk_config_t* config);
LIDAR_SDK_API lidar_sdk_status_t lidar_driver_start(lidar_driver_t* driver);
LIDAR_SDK_API lidar_sdk_status_t lidar_driver_stop(lidar_driver_t* driver);

#ifdef __cplusplus
}
#endif

#endif
