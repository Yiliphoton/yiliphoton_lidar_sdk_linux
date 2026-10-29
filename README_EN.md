# LiDAR SDK

This SDK supports P60, P60_IMU, and V192 LiDARs. It provides architecture-specific binary packages and a complete source package for native builds on the target device. UDP reception, protocol decoding, point-cloud filtering, internal queues, and worker threads are managed by the SDK.

## Binary package layout

```text
include/lidar_sdk/lidar_sdk.h
lib/liblidar_sdk.so
lib/liblidar_algo.so
share/lidar_sdk/config/
lib/cmake/lidar_sdk/
```

Use a binary package only on the architecture named in its archive. For example, a `linux-x86_64` package cannot be used on ARM64. Keep `liblidar_sdk.so` and `liblidar_algo.so` in the same dynamic-library search path; placing both files in one `lib/` directory is recommended.

`share/lidar_sdk/config/` is only the default calibration location when no calibration path is configured. Applications can place calibration JSON files in any configuration directory by setting `calibration_path`. The binary SDK also requires target-system runtime libraries including `libpcap` and `libstdc++`.

## Source package and native builds

```text
include/lidar_sdk/       # Public ABI: the only header delivered to applications
src/api/                    # C ABI adapter, configuration translation, callback dispatch
src/core/                   # Private engine: UDP, protocol decoding, filters, queues, threads
resources/calibration/      # P60/P60_IMU/V192 calibration files installed with the SDK
demo/                       # C example using only the public API
lib/                        # Platform-specific private algorithm shared libraries
cmake/                      # find_package configuration
```

The source package is a complete project root and does not contain a prebuilt `sdk/` subdirectory for another architecture. Build from the extracted source root on the target device. Native x86-64 builds automatically use `lib/x86-64/liblidar_algo.so`; native ARM64 builds automatically use `lib/arm64/liblidar_algo.so`. CMake fails during configuration for unsupported target architectures.

For source-package users, read [QUICKSTART_EN.md](QUICKSTART_EN.md) first and follow the sequence: build the SDK, run the demo, verify a client project, then integrate point-cloud callbacks.

## Build and install

Dependencies for building the SDK from source: a C/C++ compiler, CMake, `libeigen3-dev`, and `libpcap-dev`. Applications using the installed binary SDK do not need these build dependencies. Ubuntu/Debian example:

```bash
sudo apt update
sudo apt install -y build-essential cmake libeigen3-dev libpcap-dev
```

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
cmake --install build --prefix /opt/lidar_sdk
```

`examples/consumer_cmake/` is an independent client example that verifies `find_package(lidar_sdk)` against the installed SDK.

## Application integration

Application `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
project(lidar_application LANGUAGES C CXX)

find_package(lidar_sdk CONFIG REQUIRED)

add_executable(lidar_application main.c)
target_link_libraries(lidar_application PRIVATE lidar_sdk::lidar_sdk)
```

Assuming the SDK is installed in `/opt/lidar_sdk`, configure and build the application with:

```bash
cmake -S . -B build -DCMAKE_PREFIX_PATH=/opt/lidar_sdk
cmake --build build -j
```

At runtime, ensure `liblidar_sdk.so` and `liblidar_algo.so` are in the same dynamic-library search path. If the application does not set an RPATH, run it with:

```bash
export LD_LIBRARY_PATH=/opt/lidar_sdk/lib:${LD_LIBRARY_PATH}
./build/lidar_application
```

```c
#include <lidar_sdk/lidar_sdk.h>

#include <stddef.h>
#include <string.h>

static void on_frame(const lidar_sdk_frame_t* frame, void* user_data) {
  (void)user_data;
  /* Convert frame->points to an application, PCL, or ROS point-cloud type. */
}

lidar_sdk_config_t config;
lidar_sdk_config_init(&config);
config.model = LIDAR_SDK_MODEL_P60;
config.msop_port = 8899;

lidar_driver_t* driver = lidar_driver_create();
if (driver == NULL) {
  return 1;
}
lidar_driver_set_frame_callback(driver, on_frame, NULL);
if (lidar_driver_init(driver, &config) != LIDAR_SDK_SUCCESS ||
    lidar_driver_start(driver) != LIDAR_SDK_SUCCESS) {
  lidar_driver_destroy(driver);
  return 1;
}

/* ... */

lidar_driver_stop(driver);
lidar_driver_destroy(driver);
```

## Configuration

Always call `lidar_sdk_config_init()` before changing fields. Current P60, P60_IMU, and V192 deployment examples both use MSOP port `8899`, which is also the SDK default. Applications should still set the port explicitly to avoid relying on historical LiDAR configuration.

| Field | Description and recommended value |
| --- | --- |
| `model` | `LIDAR_SDK_MODEL_P60`, `LIDAR_SDK_MODEL_P60_IMU`, or `LIDAR_SDK_MODEL_V192`. |
| `input_type` | `LIDAR_SDK_INPUT_ONLINE` for a live LiDAR; `LIDAR_SDK_INPUT_PCAP` for a PCAP file. |
| `msop_port` | Local UDP MSOP receive port; current P60/P60_IMU/V192 examples use `8899`. |
| `difop_port` | P60/P60_IMU/V192 do not currently use DIFOP; keep this as `0` to avoid binding an additional UDP port. |
| `host_address` | Local bind address. Use `0.0.0.0` for broadcast; for unicast, use the IP of the local receive NIC, for example `192.168.1.7`. This controls local binding only and does not register with or configure the LiDAR. |
| `group_address` | Multicast group address only. Keep `0.0.0.0` for broadcast and unicast. |
| `calibration_path` | Leave empty to use the bundled calibration JSON; set an absolute path to override it. |
| `min_distance_m` / `max_distance_m` | Distance clipping range in metres; default range is `0.0~300.0`. Usually no need to set it; for a custom range, `max_distance_m` must be greater than `min_distance_m`. |
| `pcap_path` | Absolute PCAP path; used only with `input_type=PCAP`. |
| `pcap_repeat` / `pcap_rate` | Whether playback loops and its rate multiplier. |
| `echo_option` | Echo mode: `0` dual-echo merge (default), `1` first echo only, `2` second echo only, or `3` dual-echo output. |

### Echo Selection

Set `lidar_sdk_config_t.echo_option` to select the point-cloud echo output mode:

```c
config.echo_option = 0;  /* 0=merge, 1=first, 2=second, 3=dual */
```

- `0`: Process both echoes, merge them, and output one echo.
- `1`: Process and output only the first echo.
- `2`: Process and output only the second echo.
- `3`: Process both echoes and output both; `is_dual_return` is non-zero in the callback frame.

The default is `0`. If the value is greater than `3`, `lidar_driver_init()` returns `LIDAR_SDK_INVALID_ARGUMENT`. This setting is controlled through the SDK API and is not read from the legacy `crosstalk_filter.echo_option` field in the calibration JSON.

Calibration path example. When it is set, the default `share/lidar_sdk/config/` location is not required:

```c
snprintf(config.calibration_path,
         sizeof(config.calibration_path),
         "/opt/lidar_application/config/P60-Correction.json");
```

Broadcast example:

```c
config.model = LIDAR_SDK_MODEL_V192;
config.msop_port = 8899;
config.difop_port = 0;
strncpy(config.host_address, "0.0.0.0", sizeof(config.host_address) - 1U);
strncpy(config.group_address, "0.0.0.0", sizeof(config.group_address) - 1U);
```

Unicast example (the LiDAR must already send data to this IP and port):

```c
config.model = LIDAR_SDK_MODEL_P60;
config.msop_port = 8899;
config.difop_port = 0;
strncpy(config.host_address, "192.168.1.7", sizeof(config.host_address) - 1U);
```

PCAP example:

```c
config.model = LIDAR_SDK_MODEL_P60;
config.input_type = LIDAR_SDK_INPUT_PCAP;
config.msop_port = 8899;
config.pcap_repeat = 0;
config.pcap_rate = 1.0F;
strncpy(config.pcap_path, "/data/p60.pcap", sizeof(config.pcap_path) - 1U);
```

## Point-cloud callback

The SDK provides a `lidar_sdk_frame_t` and its point array during the callback. Frame and point-array memory is owned by the SDK and is valid only during that callback. Copy required data into an application-owned queue before returning when processing asynchronously.

Coordinates in `lidar_sdk_point_t` are metres. `intensity` is the reflectivity/intensity value, `ring` is the laser channel index, and `time` is the per-point offset in seconds from the current frame start. `frame.timestamp` is the absolute timestamp of the first point in the frame (filled from the host clock when the device does not provide a valid timestamp).

## IMU callback (P60_IMU)

The P60_IMU model has an embedded IMU sensor. The SDK parses IMU data from each packet and delivers it through a separate callback. Register it with `lidar_driver_set_imu_callback()`:

```c
static void on_imu(const lidar_sdk_imu_t* imu, void* user_data) {
  (void)user_data;
  printf("IMU ts=%.6f accel=(%.3f, %.3f, %.3f) gyro=(%.3f, %.3f, %.3f) temp=%.1f\n",
         imu->timestamp,
         imu->accel_x, imu->accel_y, imu->accel_z,
         imu->gyro_x, imu->gyro_y, imu->gyro_z,
         imu->temperature);
}

config.model = LIDAR_SDK_MODEL_P60_IMU;
lidar_driver_set_imu_callback(driver, on_imu, NULL);
```

`lidar_sdk_imu_t` contains timestamp, three-axis acceleration (`accel_x/y/z`), three-axis angular velocity (`gyro_x/y/z`), and temperature. The IMU callback is independent of the point-cloud callback; P60_IMU outputs both point clouds and IMU data simultaneously. Registering an IMU callback for V192/P60 models is harmless but will never fire.

## Threading model

The SDK manages packet reception, decoding, frame processing, filtering, and callback dispatch internally. Applications do not include or configure internal queues, template point-cloud types, or worker threads. The point-cloud callback runs on an SDK dispatch thread and should return quickly; dispatch time-consuming work to an application thread.

The call order is `create → set callbacks → init → start → stop → destroy`. `start` and `stop` control only the local SDK receive/processing lifecycle; they do not send LiDAR stream-control commands.

## Scope

- Online UDP and PCAP input
- P60, P60_IMU, and V192 decoding with bundled calibration files
- Bundled software-filter library
- Linux x86-64 and ARM64 prebuilt algorithm libraries

LiDAR-side start/stop-stream commands will be added as separate asynchronous control APIs once the device protocol is finalized. `lidar_driver_start` and `lidar_driver_stop` currently control only the local SDK receive and processing lifecycle.

## LiDAR-IMU Coordinate Frames and Extrinsic Parameters

For products with an integrated IMU, such as P60_IMU, use the following convention:

- LiDAR frame: `lidar`
- IMU frame: `imu_link`
- Both frames are right-handed.
- IMU origin in the LiDAR frame: `[0.0000, 0.0183, -0.0345] m`.
- Rotation quaternion: `[0.0, 0.0, 0.0, 1.0]`, in `x, y, z, w` order.

The extrinsic transform is defined as:

```text
p_lidar = R_lidar_imu · p_imu + t_lidar_imu
```

These values are the common default extrinsic parameters. The SDK core library only handles sensor data reception, decoding, and callbacks; it does not publish ROS messages or coordinate-frame transforms.

