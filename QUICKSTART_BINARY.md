# LiDAR SDK 二进制包快速开始

本文档面向拿到 `lidar_sdk-<版本>-linux-<架构>.tar.gz` 的应用开发者。二进制包只能用于包名标注的架构，例如 `x86_64` 或 `aarch64`。

## 1. 解压与确认

```bash
tar -xzf lidar_sdk-<版本>-linux-<架构>.tar.gz
cd lidar_sdk-<版本>-linux-<架构>
uname -m
```

包内主要文件：

```text
include/lidar_sdk/lidar_sdk.h
lib/liblidar_sdk.so
lib/liblidar_algo.so
share/lidar_sdk/config/
lib/cmake/lidar_sdk/
share/lidar_sdk/examples/consumer_cmake/
bin/demo
```

`liblidar_sdk.so` 与 `liblidar_algo.so` 必须保留在同一个库搜索路径。未设置 `calibration_path` 时，SDK 默认使用 `share/lidar_sdk/config/` 下的标定 JSON。

## 2. 运行 demo

```bash
export LD_LIBRARY_PATH=$PWD/lib:${LD_LIBRARY_PATH}
./bin/demo
```

默认 demo 使用 P60_IMU、端口 `8899`、`host_address=0.0.0.0`。如需改型号或 IP，请改源码包中的 `demo/demo.c` 后重新编译，或在客户程序中设置参数。

## 3. 验证客户工程

包内带有最小 CMake 示例：

```bash
cmake -S share/lidar_sdk/examples/consumer_cmake -B consumer-build \
  -DCMAKE_PREFIX_PATH=$PWD
cmake --build consumer-build -j

export LD_LIBRARY_PATH=$PWD/lib:${LD_LIBRARY_PATH}
./consumer-build/lidar_sdk_consumer
```

该示例用于验证头文件、`find_package(lidar_sdk)` 和动态库加载。实际收点云时，请按 README 注册点云回调并调用 `lidar_driver_start()`。

## 4. 常用配置提醒

- 单播：`host_address` 填本机接收网卡 IP，`group_address` 保持 `0.0.0.0`。
- 广播：`host_address = "0.0.0.0"`，`group_address = "0.0.0.0"`。
- 组播：`host_address` 填本机接收网卡 IP，`group_address` 填组播地址。
- 自定义标定文件：设置 `calibration_path` 为对应 JSON 的绝对路径。
- 距离过滤：默认范围 `0.0~300.0m`，一般无需设置；自定义时 `max_distance_m` 必须大于 `min_distance_m`。

更多 API 和字段说明见 `share/doc/lidar_sdk/README.md`。
