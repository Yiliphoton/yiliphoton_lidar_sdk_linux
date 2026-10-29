#include <lidar_sdk/lidar_sdk.h>

#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

struct test_state {
  atomic_uint frames;
  atomic_uint saved_frames;
  atomic_uint imus;
  atomic_uint invalid_points;
  atomic_uint residual_points;
  atomic_int eof;
  atomic_int fatal_error;
  unsigned int target_frames;
  unsigned int max_saved_frames;
  char model_name[32];
  char output_dir[512];
  FILE* imu_file;
  pthread_mutex_t output_mutex;
};

static int has_valid_point_cloud(const lidar_sdk_frame_t* frame) {
  for (uint32_t i = 0; i < frame->point_count; ++i) {
    const lidar_sdk_point_t* point = &frame->points[i];
    if ((point->flags & 1U) == 0U && isfinite(point->x) &&
        isfinite(point->y) && isfinite(point->z)) {
      return 1;
    }
  }
  return 0;
}

static int reserve_saved_frame(struct test_state* state,
                               unsigned int* frame_number) {
  unsigned int saved = atomic_load(&state->saved_frames);
  while (saved < state->max_saved_frames) {
    if (atomic_compare_exchange_weak(&state->saved_frames, &saved, saved + 1U)) {
      *frame_number = saved + 1U;
      return 1;
    }
  }
  return 0;
}

static void save_frame(const lidar_sdk_frame_t* frame,
                       struct test_state* state,
                       unsigned int frame_number) {
  if (!state->output_dir[0] || frame_number > state->max_saved_frames) {
    return;
  }

  char path[1024];
  snprintf(path, sizeof(path), "%s/%s_frame_%03u.txt",
           state->output_dir, state->model_name, frame_number);
  FILE* file = fopen(path, "w");
  if (!file) {
    fprintf(stderr, "[sdk-test] cannot save point cloud: %s\n", path);
    return;
  }

  fprintf(file, "# frame_timestamp=%.9f sequence=%llu points=%u\n",
          frame->timestamp, (unsigned long long)frame->sequence, frame->point_count);
  for (uint32_t i = 0; i < frame->point_count; ++i) {
    const lidar_sdk_point_t* point = &frame->points[i];
    fprintf(file, "%.9f %.9f %.9f %.3f %u %.9f %u\n",
            point->x, point->y, point->z, point->intensity,
            (unsigned int)point->ring, point->time, (unsigned int)point->flags);
  }
  fclose(file);
}

static void on_frame(const lidar_sdk_frame_t* frame, void* user_data) {
  struct test_state* state = (struct test_state*)user_data;
  unsigned int invalid = 0;
  unsigned int residual = 0;

  if (!frame || !frame->points || frame->point_count == 0) {
    atomic_store(&state->fatal_error, 1);
    return;
  }

  for (uint32_t i = 0; i < frame->point_count; ++i) {
    const lidar_sdk_point_t* point = &frame->points[i];
    if ((point->flags & 1U) != 0U) {
      ++invalid;
      if (isfinite(point->x) || isfinite(point->y) || isfinite(point->z) ||
          point->time != 0.0F) {
        ++residual;
      }
    }
  }

  atomic_fetch_add(&state->invalid_points, invalid);
  atomic_fetch_add(&state->residual_points, residual);
  atomic_fetch_add(&state->frames, 1U);
  if (has_valid_point_cloud(frame)) {
    unsigned int saved_number = 0U;
    if (reserve_saved_frame(state, &saved_number))
      save_frame(frame, state, saved_number);
  }
}

static void on_imu(const lidar_sdk_imu_t* imu, void* user_data) {
  struct test_state* state = (struct test_state*)user_data;
  if (!imu) {
    atomic_store(&state->fatal_error, 1);
    return;
  }

  atomic_fetch_add(&state->imus, 1U);
  if (state->imu_file) {
    pthread_mutex_lock(&state->output_mutex);
    fprintf(state->imu_file, "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
            imu->timestamp, imu->accel_x, imu->accel_y, imu->accel_z,
            imu->gyro_x, imu->gyro_y, imu->gyro_z, imu->temperature);
    fflush(state->imu_file);
    pthread_mutex_unlock(&state->output_mutex);
  }
}

static void on_error(int32_t error_code, const char* message, void* user_data) {
  struct test_state* state = (struct test_state*)user_data;
  fprintf(stderr, "[sdk-test] error=%d %s\n", (int)error_code,
          message ? message : "");
  if (error_code == 2) { /* ERRCODE_PCAPEXIT */
    atomic_store(&state->eof, 1);
  } else if (error_code >= 0x80) {
    atomic_store(&state->fatal_error, 1);
  }
}

static int parse_model(const char* value, lidar_sdk_model_t* model) {
  if (strcmp(value, "V192") == 0 || strcmp(value, "v192") == 0) {
    *model = LIDAR_SDK_MODEL_V192;
    return 1;
  }
  if (strcmp(value, "P60") == 0 || strcmp(value, "p60") == 0) {
    *model = LIDAR_SDK_MODEL_P60;
    return 1;
  }
  if (strcmp(value, "P60_IMU") == 0 || strcmp(value, "p60_imu") == 0) {
    *model = LIDAR_SDK_MODEL_P60_IMU;
    return 1;
  }
  return 0;
}

static unsigned long long monotonic_ms(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (unsigned long long)ts.tv_sec * 1000ULL +
         (unsigned long long)ts.tv_nsec / 1000000ULL;
}

static void close_output(struct test_state* state) {
  pthread_mutex_lock(&state->output_mutex);
  if (state->imu_file) {
    fclose(state->imu_file);
    state->imu_file = NULL;
  }
  pthread_mutex_unlock(&state->output_mutex);
}

int main(int argc, char** argv) {
  if (argc < 3 || argc > 4) {
    fprintf(stderr, "usage: %s <V192|P60|P60_IMU> <pcap> [frames]\n", argv[0]);
    return 2;
  }

  struct test_state state;
  atomic_init(&state.frames, 0U);
  atomic_init(&state.saved_frames, 0U);
  atomic_init(&state.imus, 0U);
  atomic_init(&state.invalid_points, 0U);
  atomic_init(&state.residual_points, 0U);
  atomic_init(&state.eof, 0);
  atomic_init(&state.fatal_error, 0);
  state.target_frames = argc == 4 ? (unsigned int)strtoul(argv[3], NULL, 10) : 20U;
  state.max_saved_frames = 3U;
  state.output_dir[0] = '\0';
  state.imu_file = NULL;
  pthread_mutex_init(&state.output_mutex, NULL);
  if (state.target_frames == 0U) state.target_frames = 1U;

  lidar_sdk_model_t model;
  if (!parse_model(argv[1], &model)) {
    fprintf(stderr, "unsupported model: %s\n", argv[1]);
    pthread_mutex_destroy(&state.output_mutex);
    return 2;
  }
  snprintf(state.model_name, sizeof(state.model_name), "%s", argv[1]);

  const char* output_dir = getenv("LIDAR_SDK_TEST_OUTPUT_DIR");
  if (output_dir && output_dir[0] != '\0') {
    snprintf(state.output_dir, sizeof(state.output_dir), "%s", output_dir);
    const char* save_frames = getenv("LIDAR_SDK_TEST_SAVE_FRAMES");
    if (save_frames && save_frames[0] != '\0') {
      state.max_saved_frames = (unsigned int)strtoul(save_frames, NULL, 10);
    }
    if (model == LIDAR_SDK_MODEL_P60_IMU) {
      char imu_path[1024];
      snprintf(imu_path, sizeof(imu_path), "%s/%s_imu.csv",
               state.output_dir, state.model_name);
      state.imu_file = fopen(imu_path, "w");
      if (state.imu_file) {
        fprintf(state.imu_file, "timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temperature\n");
      } else {
        fprintf(stderr, "[sdk-test] cannot save IMU data: %s\n", imu_path);
      }
    }
  }

  lidar_sdk_config_t config;
  lidar_sdk_config_init(&config);
  config.model = model;
  config.input_type = LIDAR_SDK_INPUT_PCAP;
  config.msop_port = 8899;
  config.difop_port = 0;
  config.pcap_repeat = 0;
  config.pcap_rate = 1.0F;
  config.min_distance_m = 0.01F;
  config.max_distance_m = 300.0F;

  const char* calib_dir = getenv("LIDAR_SDK_TEST_CALIB_DIR");
  if (calib_dir && calib_dir[0] != '\0') {
    snprintf(config.calibration_path, sizeof(config.calibration_path),
             "%s/%s-Correction.json", calib_dir,
             model == LIDAR_SDK_MODEL_V192 ? "V192" : "P60");
  }
  strncpy(config.pcap_path, argv[2], sizeof(config.pcap_path) - 1U);

  lidar_driver_t* driver = lidar_driver_create();
  if (!driver) {
    close_output(&state);
    pthread_mutex_destroy(&state.output_mutex);
    return 1;
  }
  lidar_driver_set_frame_callback(driver, on_frame, &state);
  lidar_driver_set_imu_callback(driver, on_imu, &state);
  lidar_driver_set_error_callback(driver, on_error, &state);

  if (lidar_driver_init(driver, &config) != LIDAR_SDK_SUCCESS ||
      lidar_driver_start(driver) != LIDAR_SDK_SUCCESS) {
    fprintf(stderr, "[sdk-test] init/start failed\n");
    lidar_driver_destroy(driver);
    close_output(&state);
    pthread_mutex_destroy(&state.output_mutex);
    return 1;
  }

  const unsigned long long deadline = monotonic_ms() + 120000ULL;
  while (monotonic_ms() < deadline &&
         atomic_load(&state.frames) < state.target_frames &&
         !atomic_load(&state.eof) && !atomic_load(&state.fatal_error)) {
    struct timespec delay = {0, 10000000L};
    nanosleep(&delay, NULL);
  }

  lidar_driver_stop(driver);
  lidar_driver_destroy(driver);
  close_output(&state);

  const unsigned int frames = atomic_load(&state.frames);
  const unsigned int imus = atomic_load(&state.imus);
  const unsigned int invalid = atomic_load(&state.invalid_points);
  const unsigned int residual = atomic_load(&state.residual_points);
  printf("SDK_PCAP_RESULT model=%s frames=%u target=%u imu=%u invalid=%u residual=%u eof=%d saved_frames=%u\n",
         argv[1], frames, state.target_frames, imus, invalid, residual,
         atomic_load(&state.eof),
         atomic_load(&state.saved_frames) < state.max_saved_frames
             ? atomic_load(&state.saved_frames) : state.max_saved_frames);

  int result = 0;
  if (atomic_load(&state.fatal_error) || frames < state.target_frames || residual != 0U) {
    result = 1;
  }
  if (model == LIDAR_SDK_MODEL_P60_IMU && imus == 0U) {
    fprintf(stderr, "[sdk-test] P60_IMU produced no IMU callbacks\n");
    result = 1;
  }
  pthread_mutex_destroy(&state.output_mutex);
  return result;
}
