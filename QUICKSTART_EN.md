# LiDAR SDK Source Package Quick Start

This guide is for application developers who received only `lidar_sdk-<version>-source.tar.gz`. Build the SDK from source on the target Linux device; do not copy a shared library from a binary package built for another architecture.

## 1. Extract and Build

Install dependencies on Ubuntu/Debian:

```bash
sudo apt update
sudo apt install -y build-essential cmake libeigen3-dev libpcap-dev
```

Extract, build, and install the SDK:

```bash
tar -xzf lidar_sdk-<version>-source.tar.gz
cd lidar_sdk-<version>-source

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
cmake --install build --prefix /opt/lidar_sdk
```

Native x86-64 builds automatically use `lib/x86-64/liblidar_algo.so`; native ARM64 builds automatically use `lib/arm64/liblidar_algo.so`. Keep these installed files together:

```text
/opt/lidar_sdk/lib/liblidar_sdk.so
/opt/lidar_sdk/lib/liblidar_algo.so
```

## 2. Run the Live Demo

`demo/demo.c` is the smallest live-data example. Its default configuration is:

```text
Model: P60_IMU
MSOP port: 8899
Local bind address: 0.0.0.0 (broadcast reception)
```

Run it after installation:

```bash
/opt/lidar_sdk/bin/demo
```

It prints frame sequence, point count, and timestamp while receiving points. Press Enter to stop.

For V192, change the model in `demo/demo.c` to:

```c
config.model = LIDAR_SDK_MODEL_V192;
```

For unicast, the LiDAR must already send data to the local receive NIC IP and port. Set the local receive NIC address, for example:

```c
snprintf(config.host_address,
         sizeof(config.host_address),
         "192.168.1.7");
```

After changing the demo, rerun `cmake --build build -j` and `cmake --install build --prefix /opt/lidar_sdk`.

## 3. Verify a Client Project

`examples/consumer_cmake/` is an independent client-project example that verifies the public header, `find_package`, and dynamic-library loading:

```bash
cmake -S examples/consumer_cmake -B consumer-build \
  -DCMAKE_PREFIX_PATH=/opt/lidar_sdk
cmake --build consumer-build -j

export LD_LIBRARY_PATH=/opt/lidar_sdk/lib:${LD_LIBRARY_PATH}
./consumer-build/lidar_sdk_consumer
```

This example verifies that a client project can compile, link, and load the SDK. Register callbacks and start a driver as described in the next section to receive point clouds.

## 4. Application Call Order

Applications include only the public header:

```c
#include <lidar_sdk/lidar_sdk.h>
```

Use the following lifecycle:

```text
lidar_sdk_config_init
lidar_driver_create
lidar_driver_set_frame_callback
lidar_driver_set_error_callback
lidar_driver_init
lidar_driver_start
lidar_driver_stop
lidar_driver_destroy
```

See “Application integration” in [README_EN.md](README_EN.md) for the complete C example. Point clouds arrive through `lidar_sdk_frame_t` callbacks. Callbacks run on an SDK thread, so return quickly and hand off expensive work to an application-owned thread.

## 5. Common Configuration

```c
lidar_sdk_config_t config;
lidar_sdk_config_init(&config);

config.model = LIDAR_SDK_MODEL_P60_IMU;
config.msop_port = 8899;
config.difop_port = 0;
config.echo_option = 0;  /* 0=merge, 1=first, 2=second, 3=dual */
```

| Scenario | Required fields |
| --- | --- |
| Broadcast live data | `host_address = "0.0.0.0"`, `group_address = "0.0.0.0"`. |
| Unicast live data | Set `host_address` to the local receive NIC IP. The LiDAR must already send to that IP and `msop_port`. |
| V192 | Set `model = LIDAR_SDK_MODEL_V192`. |
| PCAP playback | Set `input_type = LIDAR_SDK_INPUT_PCAP` and an absolute `pcap_path`. |
| Custom calibration | Set `calibration_path` to the absolute P60 or V192 JSON path. |
| Distance filter | Default range is `0.0~300.0m`; usually no need to set it. For a custom range, `max_distance_m` must be greater than `min_distance_m`. |
| Echo selection | Set `echo_option`: `0` dual-echo merge (default), `1` first echo, `2` second echo, or `3` dual-echo output. |

`echo_option` is configured through the SDK API and applies to P60, P60_IMU, and V192. The driver must be reinitialized after changing it. Modes `0` and `3` process both echoes; modes `1` and `2` process only the selected echo. See the “Echo Selection” section in [README_EN.md](README_EN.md) for details.

For example, when calibration JSON is in the application configuration directory:

```c
snprintf(config.calibration_path,
         sizeof(config.calibration_path),
         "/opt/lidar_application/config/P60-Correction.json");
```

When `calibration_path` is empty, the SDK uses the default JSON from the installed SDK directory.

## 6. Troubleshooting

| Symptom | Check |
| --- | --- |
| `undefined reference to lidar_driver_*` | Link the final executable with `target_link_libraries(... lidar_sdk::lidar_sdk)`; including the header alone is insufficient. |
| `liblidar_algo.so` is missing at startup | Keep it in the same library directory as `liblidar_sdk.so`, or set `LD_LIBRARY_PATH=/opt/lidar_sdk/lib`. |
| The demo receives no point clouds | Check model, `msop_port`, local IP, LiDAR destination, and firewall rules. Use `0.0.0.0` for broadcast. |
| Calibration file cannot be opened | Set `calibration_path` to an existing absolute path, or retain the default files under `/opt/lidar_sdk/share/lidar_sdk/config/`. |

See [README_EN.md](README_EN.md) for all API fields, PCAP examples, IMU callbacks, and threading details.
