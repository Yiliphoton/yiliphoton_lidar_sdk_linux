# LiDAR SDK Binary Package Quick Start

This guide is for application developers using `lidar_sdk-<version>-linux-<arch>.tar.gz`. A binary package must only be used on the architecture named in the archive, such as `x86_64` or `aarch64`.

## 1. Extract and Check

```bash
tar -xzf lidar_sdk-<version>-linux-<arch>.tar.gz
cd lidar_sdk-<version>-linux-<arch>
uname -m
```

Main package contents:

```text
include/lidar_sdk/lidar_sdk.h
lib/liblidar_sdk.so
lib/liblidar_algo.so
share/lidar_sdk/config/
lib/cmake/lidar_sdk/
share/lidar_sdk/examples/consumer_cmake/
bin/demo
```

Keep `liblidar_sdk.so` and `liblidar_algo.so` in the same library search path. When `calibration_path` is empty, the SDK uses the calibration JSON files under `share/lidar_sdk/config/`.

## 2. Run the Demo

```bash
export LD_LIBRARY_PATH=$PWD/lib:${LD_LIBRARY_PATH}
./bin/demo
```

The default demo uses P60_IMU, port `8899`, and `host_address=0.0.0.0`. To change the model or IP, rebuild `demo/demo.c` from the source package or set the fields in your own application as described in README.

## 3. Verify a Client Project

A minimal CMake client example is included:

```bash
cmake -S share/lidar_sdk/examples/consumer_cmake -B consumer-build \
  -DCMAKE_PREFIX_PATH=$PWD
cmake --build consumer-build -j

export LD_LIBRARY_PATH=$PWD/lib:${LD_LIBRARY_PATH}
./consumer-build/lidar_sdk_consumer
```

This example verifies the public header, `find_package(lidar_sdk)`, and dynamic-library loading. To receive point clouds, register callbacks and call `lidar_driver_start()` as shown in README.

## 4. Common Configuration Notes

- Unicast: set `host_address` to the local receive NIC IP; keep `group_address` as `0.0.0.0`.
- Broadcast: set both `host_address` and `group_address` to `0.0.0.0`.
- Multicast: set `host_address` to the local receive NIC IP and `group_address` to the multicast group address.
- Custom calibration: set `calibration_path` to the absolute path of the matching JSON file.
- Distance filter: default range is `0.0~300.0m`; usually no need to set it. For a custom range, `max_distance_m` must be greater than `min_distance_m`.

See `share/doc/lidar_sdk/README_EN.md` for API and field details.
